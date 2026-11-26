/* s47: the command code moved out of ground_brain.c, in its original order (s46 lines 4522-4548, 4981-6245,
 * ground_task_command.inc, 6247-6597). Compiled only through ground_command.c. */
/* Capability P (class part) and default role of a slot. S2: from cls.<seat>
 * (vehicle_classes.json): SPAA 0.5, HE-only 0.6, light / wheeled 0.7, TD /
 * missile 0.9, else 1; role = its default role. Without a cls line the S1
 * approximation: fast (>= 1.2 x team median hp/t) plain tank 0.8 / flank, TD
 * 0.9 / overwatch, SPAA 0.5 / escort, else 1 / breach. */
float strat_class(int s, float hp_med, int *role)
{
    const vcls_t *v = s > 0 && s < MAX_SEATS ? active_cls(s, slot_unit[s]) : NULL;
    if (v && v->ok) {
        uint32_t f = v->flags;
        *role = v->role;
        return f & VC_SPAA ? 0.5f : f & VC_HE_ONLY ? 0.6f : f & (VC_LIGHT | VC_WHEELED) ? 0.7f
               : f & (VC_TD | VC_MISSILE) ? 0.9f : 1.0f;
    }
    int r = s >= 0 && s < MAX_SEATS ? cfg.roles[s] : ROLE_TANK;
    int fast = r == ROLE_TANK && s >= 0 && s < MAX_SEATS && seat_hp[s] > 0.0f && hp_med > 0.0f && seat_hp[s] >= 1.2f * hp_med;
    *role = r == ROLE_SPAA ? SR_ESCORT : r == ROLE_TD ? SR_OVERWATCH : fast ? SR_FLANK : SR_BREACH;
    return r == ROLE_SPAA ? 0.5f : r == ROLE_TD ? 0.9f : fast ? 0.8f : 1.0f;
}

int strat_spaa(int s)
{
    const vcls_t *v = s > 0 && s < MAX_SEATS ? active_cls(s, slot_unit[s]) : NULL;
    if (v && v->ok)
        return (v->flags & VC_SPAA) != 0;
    return s >= 0 && s < MAX_SEATS && cfg.roles[s] == ROLE_SPAA;
}


/* ---- S2 team commander (2026-11-11; the strategy design
 * section 3, review_5003 section 4) ----------------------------------------
 * Every CMD_MS per army, right after that army's strat_update, from the same
 * team knowledge (sview: band fronts and ratios, known enemies, EN + hidden per
 * G32 cell). Objectives:
 *   CAPTURE p   point not ours; need 2 + known enemies within 300 m (<= 5);
 *               value 100 (neutral) / 110 (the enemy's)
 *   HOLD p      our point with known enemies within 300 m; need their count
 *               (1-3); value 90
 *   SUPPORT h   the human: more than CMD_SUP_AHEAD_M ahead of the team's median
 *               depth, or in contact in a band with ratio < CMD_SUP_RATIO.
 *               Hysteresis (5003: band ratios 0.60-0.68 sat on the thresholds):
 *               on for >= CMD_SUP_MIN_MS, off only when < CMD_SUP_BEHIND_M
 *               ahead and (no contact or ratio > CMD_SUP_RATIO_OFF). Need 3,
 *               value 105; destination 60 / 60 / 90 m behind it (team axis),
 *               50 / 50 / 75 m to alternating sides; wingmen do not overtake.
 *   OVERWATCH b one per band with known or likely enemies (f_en + hid >= 0.5)
 *               and an open firing spot: the best G10 cell 40-300 m behind the
 *               band's front, score (0.35 + 0.65 hull-down quality towards the
 *               enemy) x mid + far fire viewshed (+-22.5 deg) x nearness to the
 *               front; turret-blocked cells and open share < CMD_OW_OPEN are
 *               skipped (no G10: the best-covered G32 cell). Need 1, value 65.
 *   PUSH b      band ratio >= 1.3 (off below 1.15) and its front past half way:
 *               350 m ahead of the front; need 2; value 60 + 15 min(ratio, 2)
 *   PRESS b     every point ours: 375 m short of the enemy spawn on the band's
 *               centre line; centre band need 3 / value 88, the others 1 / 70.
 * Assignment, greedy by marginal utility:
 *   U = V * dec * fit * w - T / 2 s + H - risk - role change
 *   dec 1 below need, else 0.25 * 0.5^excess; fit: breach x1.2 for CAPTURE /
 *   HOLD / centre PUSH / PRESS, flank x1.2 for SUPPORT and wing PUSH / PRESS,
 *   overwatch x1.5 for OVERWATCH and x0.7 for CAPTURE / HOLD; w = the
 *   vehicle's weight for the role the objective wants (role_weights,
 *   floored at CMD_W_FLOOR so single-role vehicles can still be sent; SPAA:
 *   SUPPORT 1, points 0.8); switching to a role weighted under CMD_ROLE_FREE
 *   costs CMD_ROLE_COST (>= 0.7: free, design rule); T: TRAVEL field for points,
 *   else straight line / 8 m/s x 1.3, scaled by the vehicle's hp/t; H = +15
 *   for the current objective; risk = 5 x sum of EN + hidden on 5 samples of
 *   the straight line, per 100 m, <= CMD_RISK_MAX.
 * Rules: an order stays CMD_KEEP_MS unless its objective is gone; at most
 * CMD_MAX_SWITCH reassignments per tick (a life's first order is free); never
 * reassigned: engaged within CMD_LOCK_M, close fight, inside a point that is
 * not ours, on a team_plan task, an order still pending; orders take effect
 * after 1-3 s (cmd_apply in lanes_tick). The commander reserves attackers and
 * limits discretionary stationary orders. CAPTURE / HOLD use the lane executor on that
 * point's lane (a fresh life near its spawn takes a tactical route, risk
 * weighted; others go straight like lane_advance did); the others drive an A*
 * route on G32 (cmd_astar) and park at the destination (cmd_drive).
 * Logs: `cmd:` per order (CMD_LOGS_PER_MIN per army, far ones always), a
 * census per army every CMD_CENSUS_MS (advance, physical point presence,
 * effective fire and stationary orders), a minute timing line, and `o` rows in
 * strat_snap.csv. Fault-guarded by the adapter: 3 faults switch the commander off (lanes as
 * before). */
#define CMD_KEEP_MS 20000
#define CMD_LOCK_M 300.0f
#define CMD_W_FLOOR 0.4f
#define CMD_RISK_MAX 40.0f
#define CMD_SUP_AHEAD_M 150.0f
#define CMD_SUP_BEHIND_M 100.0f
#define CMD_SUP_RATIO 0.8f
#define CMD_SUP_RATIO_OFF 0.95f
#define CMD_SUP_MIN_MS 20000
#define CMD_OW_MIN_M 40.0f
#define CMD_OW_MAX_M 300.0f
#define CMD_OW_OPEN 0.2f
#define CMD_OW_MS 20000
#define CMD_OW_LIKELY 0.5f      /* f_en + hid of the band: overwatch on ... */
#define CMD_OW_LIKELY_OFF 0.3f  /* ... and off again */
#define CMD_PUSH_RATIO 1.3f
#define CMD_PUSH_OFF 1.15f
#define CMD_PUSH_AHEAD_M 350.0f
#define CMD_PRESS_M 375.0f
#define CMD_CROSS_M 400.0f      /* a far reassignment moving this much sideways (the cbd7 pattern) */
#define CMD_SHARP_DEG 100.0f
/* Stage 2 (2026-11-15). A point we own is garrisoned, not passive duty: value
 * below a contested capture (100+) and a HOLD under known attack (90), above a
 * push at a moderate force ratio and the old calm hold (45). The owned point
 * deepest towards the enemy is where the next attack lands: two holders. */
#define CMD_GARRISON_VALUE 85.0f
#define CMD_GARRISON_FORWARD 2
/* A reassignment at negative utility is sensible only if the drive is no
 * longer than a freshly spawned tank's drive to the same place plus this
 * (spawn spread); otherwise the slot stays open for someone nearer. */
#define CMD_REACH_SLACK_M 250.0f
const char *const cmd_name[OB_N] = {"none", "capture", "hold", "support", "overwatch", "push", "press", "stay"};
#define CMD_POST_GATES 8        /* gate posts (cmd_build_posts), ids CMD_POST_ID_BASE + 0..7 as before v13 */
#define CMD_POST_GATE_GAP_M 90.0f
#define CMD_POST_ROAD_GAP_M 24.0f
static int tp_watch_offered(int army, const void *post);  /* ground_teamplan.inc */
cmd_post_t cmd_posts[3][CMD_POST_MAX];
int cmd_nposts[3];
static int cmd_posts_ready[3];
#define ROADS_BOOST_MS 90000            /* choke value x ROADS_BOOST in the first 90 s */
#define ROADS_BOOST 1.5f
#define ROADS_LINE_MS 10000             /* the majority (or its loss) must hold this long to switch lines */
#define ROADS_PAIR_VALUE 0.6f           /* a choke this valuable with two spots gets two posts */
roads_state_t roads;
cmd_state_t cmd;

int cmd_on(void)
{
    return cfg.commander && cfg.lanes && strat.state == 1 && !cmd.off;
}

float strat_depth(int army, float x, float z)
{
    const strat_meta_t *m = strat.m;
    float d1 = (x - m->s1x) * m->axx + (z - m->s1z) * m->axz;
    return army == 1 ? d1 : m->L - d1;
}

float strat_lat(float x, float z)
{
    const strat_meta_t *m = strat.m;
    return (x - m->s1x) * m->latx + (z - m->s1z) * m->latz;
}

static void strat_world(int army, float depth, float lat, float *x, float *z)
{
    const strat_meta_t *m = strat.m;
    float d1 = army == 1 ? depth : m->L - depth;
    *x = m->s1x + m->axx * d1 + m->latx * lat;
    *z = m->s1z + m->axz * d1 + m->latz * lat;
}

/* The inside + passable G32 cell nearest (x, z) within 6 cells: index and its
 * representative point, -1 when there is none. */
int cmd_snap(float x, float z, float *ox, float *oz)
{
    const strat_meta_t *m = strat.m;
    int c0 = (int)floorf((x - m->g32_x0) / m->g32_cell), r0 = (int)floorf((z - m->g32_z0) / m->g32_cell);
    c0 = c0 < 0 ? 0 : c0 >= strat.nx ? strat.nx - 1 : c0;
    r0 = r0 < 0 ? 0 : r0 >= strat.nz ? strat.nz - 1 : r0;
    int best = -1;
    float bd = 1e18f;
    for (int dr = -6; dr <= 6; ++dr)
        for (int dc = -6; dc <= 6; ++dc) {
            int r = r0 + dr, c = c0 + dc;
            if (r < 0 || r >= strat.nz || c < 0 || c >= strat.nx)
                continue;
            const strat_cell_t *cl = &strat.cell[r * strat.nx + c];
            if ((cl->flags & 3) != 3)
                continue;
            float d = dist2d(cl->x, cl->z, x, z);
            if (d < bd) {
                bd = d;
                best = r * strat.nx + c;
            }
        }
    if (best >= 0) {
        *ox = strat.cell[best].x;
        *oz = strat.cell[best].z;
    }
    return best;
}

/* The target lane (kind 0) of this army whose point is lane's point, or -1. */
int cmd_lane_point(int lane)
{
    if (lane < 0 || lane >= nlanes)
        return -1;
    const lane_t *l = &lanes[lane];
    for (int i = 0; i < nlanes; ++i)
        if (lanes[i].army == l->army && lanes[i].kind == 0 && dist2d(lanes[i].zx, lanes[i].zz, l->zx, l->zz) < 40.0f)
            return i;
    return -1;
}

/* Best firing spot for band b's overwatch, 40-300 m behind its front (see above). */
static int cmd_ow_spot(int army, int b, float front, float *ox, float *oz, float *score)
{
    const strat_meta_t *m = strat.m;
    float lo = front - CMD_OW_MAX_M, hi = front - CMD_OW_MIN_M, l0 = m->edge[b], l1 = m->edge[b + 1];
    lo = lo < 0.0f ? 0.0f : lo;
    if (hi <= lo + 20.0f)
        return 0;
    float best = 0.0f;
    if (strat.g10) {
        float ang = atan2f(army == 1 ? m->axz : -m->axz, army == 1 ? m->axx : -m->axx);
        int k = (((int)floorf(ang / 0.39269908f + 0.5f)) % 16 + 16) % 16;
        float xs[4], zs[4], x0 = 1e18f, x1 = -1e18f, z0 = 1e18f, z1 = -1e18f;
        strat_world(army, lo, l0, &xs[0], &zs[0]);
        strat_world(army, lo, l1, &xs[1], &zs[1]);
        strat_world(army, hi, l0, &xs[2], &zs[2]);
        strat_world(army, hi, l1, &xs[3], &zs[3]);
        for (int i = 0; i < 4; ++i) {
            x0 = fminf(x0, xs[i]);
            x1 = fmaxf(x1, xs[i]);
            z0 = fminf(z0, zs[i]);
            z1 = fmaxf(z1, zs[i]);
        }
        float cs = m->g10_cell;
        int c0 = (int)floorf((x0 - m->g10_x0) / cs), c1 = (int)floorf((x1 - m->g10_x0) / cs);
        int r0 = (int)floorf((z0 - m->g10_z0) / cs), r1 = (int)floorf((z1 - m->g10_z0) / cs);
        c0 = c0 < 0 ? 0 : c0;
        r0 = r0 < 0 ? 0 : r0;
        c1 = c1 >= strat.n10x ? strat.n10x - 1 : c1;
        r1 = r1 >= strat.n10z ? strat.n10z - 1 : r1;
        for (int r = r0; r <= r1; r += 2)
            for (int c = c0; c <= c1; c += 2) {
                int i = r * strat.n10x + c;
                u8 fl = strat.g10[(size_t)i * 8 + 4];
                if ((fl & 3) != 3 || (fl & 4))
                    continue;
                float px = m->g10_x0 + ((float)c + 0.5f) * cs, pz = m->g10_z0 + ((float)r + 0.5f) * cs;
                float dp = strat_depth(army, px, pz), la = strat_lat(px, pz);
                if (dp < lo || dp > hi || la < l0 || la >= l1)
                    continue;
                u8 h = strat.g10hull[(size_t)i * 16 + k];
                if (h & 0x80)
                    continue;
                int vis = 0;
                for (int dk = -1; dk <= 1; ++dk) {
                    const u8 *vw = strat.g10view + ((size_t)i * 16 + (size_t)((k + dk + 16) % 16)) * 6;
                    vis += (vw[2] >> 4) + (vw[4] >> 4);
                }
                float open = (float)vis / 90.0f;
                if (open < CMD_OW_OPEN)
                    continue;
                float sc = (0.35f + 0.65f * (float)(h & 15) / 15.0f) * open * (1.0f - 0.3f * (front - dp) / CMD_OW_MAX_M);
                if (sc > best) {
                    best = sc;
                    *ox = px;
                    *oz = pz;
                }
            }
    } else {
        for (int i = 0; i < strat.n; ++i) {
            const strat_cell_t *cl = &strat.cell[i];
            if ((cl->flags & 3) != 3 || cl->band != b)
                continue;
            float dp = strat_depth(army, cl->x, cl->z), cv = (float)cl->cover[army - 1] / 255.0f;
            if (dp < lo || dp > hi || cv < 0.3f)
                continue;
            float sc = 0.5f * cv * (1.0f - 0.3f * (front - dp) / CMD_OW_MAX_M);
            if (sc > best) {
                best = sc;
                *ox = cl->x;
                *oz = cl->z;
            }
        }
    }
    *score = best;
    return best > 0.0f;
}

int cmd_known_near(const strat_view_t *v, float x, float z, float r, float max_age)
{
    int n = 0;
    for (int k = 0; k < v->ne; ++k)
        n += v->en[k].age <= max_age && dist2d(v->en[k].x, v->en[k].z, x, z) < r;
    return n;
}

int cmd_cap_index(const lane_t *l)
{
    for (uint32_t c = 0; c < strat.m->ncap && c < 8; ++c)
        if (dist2d(strat.m->cap[c][0], strat.m->cap[c][1], l->zx, l->zz) < 40.0f)
            return (int)c;
    return -1;
}

/* Samples ordered from the capture boundary out along a mapped approach. */
static int cmd_post_gate_at(const troute_t *r, float start, float gap, float *gx, float *gz)
{
    int sample = 0;
    float path = 0.0f;
    for (int i = r->n - 1; i > 0 && sample < CMD_POST_SAMPLES; --i) {
        float d = dist2d(r->x[i], r->z[i], r->x[i - 1], r->z[i - 1]);
        while (d > 0.01f && sample < CMD_POST_SAMPLES) {
            float at = start + gap * (float)sample;
            if (path + d < at)
                break;
            float f = (at - path) / d;
            gx[sample] = r->x[i] + f * (r->x[i - 1] - r->x[i]);
            gz[sample] = r->z[i] + f * (r->z[i - 1] - r->z[i]);
            ++sample;
        }
        path += d;
    }
    return sample == CMD_POST_SAMPLES && dist2d(gx[0], gz[0], gx[2], gz[2]) >= 1.25f * gap;
}

static int cmd_post_gate(const troute_t *r, const lane_t *l, float *gx, float *gz)
{
    return dist2d(r->x[r->n - 1], r->z[r->n - 1], l->zx, l->zz) <= l->zr
           && cmd_post_gate_at(r, l->zr + 40.0f, 55.0f, gx, gz);
}

static int cover_cell_of(float x, float z, float *px, float *pz);
static int position_direction(float x, float z, float tx, float tz);
static float position_leg_distance(float x, float z, float sx, float sz, float gx, float gz);

static int cmd_post_off_road(float x, float z)
{
    for (int i = 0; i < ntroutes; ++i) {
        const troute_t *r = &troutes[i];
        for (int j = 1; j < r->n; ++j) {
            if (x < fminf(r->x[j - 1], r->x[j]) - CMD_POST_ROAD_GAP_M
                || x > fmaxf(r->x[j - 1], r->x[j]) + CMD_POST_ROAD_GAP_M
                || z < fminf(r->z[j - 1], r->z[j]) - CMD_POST_ROAD_GAP_M
                || z > fmaxf(r->z[j - 1], r->z[j]) + CMD_POST_ROAD_GAP_M)
                continue;
            if (position_leg_distance(x, z, r->x[j - 1], r->z[j - 1], r->x[j], r->z[j]) < CMD_POST_ROAD_GAP_M)
                return 0;
        }
    }
    for (int i = 0; i < nlanes; ++i) {
        const lane_t *l = &lanes[i];
        int n = l->kind ? l->n : l->n < 4 ? l->n : 4;
        for (int j = 1; j < n; ++j)
            if (position_leg_distance(x, z, l->x[j - 1], l->z[j - 1], l->x[j], l->z[j]) < CMD_POST_ROAD_GAP_M)
                return 0;
    }
    return 1;
}

static int cmd_post_passable(float x, float z)
{
    int c32 = strat_cellof(x, z);
    if (c32 < 0 || (strat.cell[c32].flags & 3) != 3)
        return 0;
    if (!strat.g10)
        return 1;
    int c10 = cover_cell_of(x, z, NULL, NULL);
    return c10 >= 0 && (strat.g10[(size_t)c10 * 8 + 4] & 7) == 3 && strat.g10[(size_t)c10 * 8 + 5] <= 4;
}

int cmd_post_standable(float x, float z, float gx, float gz)
{
    if (!cmd_post_passable(x, z))
        return 0;
    float dx, dz;
    unit_dir(x, z, gx, gz, &dx, &dz);
    for (int front = -1; front <= 1; front += 2)
        for (int side = -1; side <= 1; side += 2)
            if (!cmd_post_passable(x + 4.5f * (float)front * dx - 2.25f * (float)side * dz,
                                   z + 4.5f * (float)front * dz + 2.25f * (float)side * dx))
                return 0;
    return 1;
}

static float cmd_post_quality(int army, const cmd_post_t *p, const int *cells, float x, float z, float prior)
{
    const lane_t *l = &lanes[p->lane];
    float reach = dist2d(x, z, p->gx[1], p->gz[1]);
    float cap_dist = dist2d(x, z, l->zx, l->zz);
    if (reach < 65.0f || reach > 450.0f || cap_dist < l->zr + 30.0f || cap_dist > l->zr + 460.0f
        || strat_depth(army, x, z) > strat_depth(army, p->gx[1], p->gz[1]) + 45.0f
        || !cmd_post_standable(x, z, p->gx[1], p->gz[1]))
        return 0.0f;
    int c32 = strat_cellof(x, z), visible = 0;
    int src = strat.cap_src[p->cap];
    if (strat.travel && src >= 0 && strat.travel[(size_t)src * (size_t)strat.n + (size_t)c32] == 0xffff)
        return 0.0f;
    for (int i = 0; i < CMD_POST_SAMPLES; ++i)
        visible += cells[i] >= 0 && (i == 1 || cells[i] != cells[1]) && strat_sees(strat.visf, c32, cells[i]);
    if (visible < 2 || cells[1] < 0 || !strat_sees(strat.visf, c32, cells[1]))
        return 0.0f;
    float hard = (float)strat.cell[c32].cover[army - 1] / 255.0f, opening = 0.0f;
    if (strat.g10hull && strat.g10view) {
        int c10 = cover_cell_of(x, z, NULL, NULL);
        if (c10 < 0)
            return 0.0f;
        int k = position_direction(x, z, p->gx[1], p->gz[1]);
        u8 h = strat.g10hull[(size_t)c10 * 16 + k];
        if (h & 0x80)
            return 0.0f;
        float hull = (float)(h & 15) / 15.0f;
        hard = fmaxf(0.5f * hard, ((h >> 4) & 3) == 3 ? 0.35f * hull : hull);
        int band = reach < strat.m->vband[1] ? 0 : reach < strat.m->vband[2] ? 1 : 2;
        opening = (float)(strat.g10view[((size_t)c10 * 16 + k) * 6 + band * 2] >> 4) / 15.0f;
        if (opening < 0.13f)
            return 0.0f;
    }
    if (hard < 0.22f && prior < 0.5f)
        return 0.0f;
    return 0.45f * (float)visible + 1.05f * hard + 0.45f * opening + 0.28f * prior
           - 0.0013f * fabsf(reach - 170.0f);
}

static void cmd_post_candidate(int army, cmd_post_t *p, const int *cells, float x, float z, float prior)
{
    float quality = cmd_post_quality(army, p, cells, x, z, prior);
    if (quality <= 0.0f)
        return;
    int at = p->npos;
    for (int k = 0; k < p->npos; ++k) {
        if (dist2d(x, z, p->pos[k].x, p->pos[k].z) < 20.0f) {
            if (quality <= p->pos[k].quality || at != p->npos)
                return;
            at = k;
        }
    }
    if (at == CMD_POST_OPTIONS) {
        at = CMD_POST_OPTIONS - 1;
        if (quality <= p->pos[at].quality)
            return;
    }
    for (int i = 0; i < cmd_nposts[army]; ++i)
        for (int j = 0; j < cmd_posts[army][i].npos; ++j)
            if (dist2d(x, z, cmd_posts[army][i].pos[j].x, cmd_posts[army][i].pos[j].z) < 42.0f)
                return;
    if (!cmd_post_off_road(x, z))
        return;
    p->npos += at == p->npos;
    while (at > 0 && quality > p->pos[at - 1].quality) {
        p->pos[at] = p->pos[at - 1];
        --at;
    }
    p->pos[at] = (cmd_post_pos_t){x, z, quality, 0};
}

static void cmd_post_sites(int army, cmd_post_t *p)
{
    int cells[CMD_POST_SAMPLES];
    for (int j = 0; j < CMD_POST_SAMPLES; ++j)
        cells[j] = strat_cellof(p->gx[j], p->gz[j]);
    const lane_t *l = &lanes[p->lane];
    for (int i = 0; i < ntspots; ++i) {
        const tspot_t *sp = &tspots[i];
        if (sp->army == army && dist2d(sp->zx, sp->zz, l->zx, l->zz) < 40.0f)
            cmd_post_candidate(army, p, cells, sp->x, sp->z, sp->score * (sp->kind == 2 ? 0.4f : 1.0f));
    }
    /* Search around the mapped gate using its existing passability and visibility layers. */
    const int stride = strat.g10 ? 10 : 32;
    for (int dz = -360; dz <= 360; dz += stride)
        for (int dx = -360; dx <= 360; dx += stride) {
            float x = p->gx[1] + (float)dx, z = p->gz[1] + (float)dz;
            if (strat.g10) {
                if (cover_cell_of(x, z, &x, &z) < 0)
                    continue;
            } else {
                int c32 = strat_cellof(x, z);
                if (c32 < 0)
                    continue;
                x = strat.cell[c32].x;
                z = strat.cell[c32].z;
            }
            cmd_post_candidate(army, p, cells, x, z, 0.0f);
        }
}

static int cmd_post_overlaps(int army, const cmd_post_t *p)
{
    for (int i = 0; i < cmd_nposts[army]; ++i) {
        const cmd_post_t *q = &cmd_posts[army][i];
        if (dist2d(p->gx[1], p->gz[1], q->gx[1], q->gz[1]) < CMD_POST_GATE_GAP_M
            || position_leg_distance(p->gx[1], p->gz[1], q->gx[0], q->gz[0], q->gx[2], q->gz[2]) < 48.0f
            || position_leg_distance(q->gx[1], q->gz[1], p->gx[0], p->gz[0], p->gx[2], p->gz[2]) < 48.0f)
            return 1;
    }
    return 0;
}

/* ---- v13 road network in the commander (SPEC-v13 sections 7, 8, 8a, 8b; stage s10) ----------
 * Choke ambushes (tchoke / tchokespot) and precomputed overwatch positions (tow) become posts
 * beside the gate posts: OB_OVERWATCH with ids CMD_POST_ID_BASE + index, so they enter the same
 * post phase of the assignment, share its stationary budget (live / PASSIVE_DIV minus the posts,
 * nothing added), and use the same retention, release, driving and facing. Only the posts of the
 * army's active line are offered (roads_line_update). Without the config lines nothing here runs. */
static int roads_lane_of_cap(int army, float zx, float zz)
{
    for (int i = 0; i < nlanes; ++i)
        if (lanes[i].army == army && lanes[i].kind == 0 && dist2d(lanes[i].zx, lanes[i].zz, zx, zz) < 40.0f
            && cmd_cap_index(&lanes[i]) >= 0)
            return i;
    return -1;
}

static int roads_nearest_lane(int army, float x, float z)
{
    int best = -1;
    float bd = 1e18f;
    for (int i = 0; i < nlanes; ++i)
        if (lanes[i].army == army && lanes[i].kind == 0 && cmd_cap_index(&lanes[i]) >= 0
            && dist2d(lanes[i].zx, lanes[i].zz, x, z) < bd) {
            bd = dist2d(lanes[i].zx, lanes[i].zz, x, z);
            best = i;
        }
    return best;
}

/* Not on top of a position some post already has (two crews would share it). */
static int roads_spot_free(int army, float x, float z)
{
    for (int i = 0; i < cmd_nposts[army]; ++i)
        for (int j = 0; j < cmd_posts[army][i].npos; ++j)
            if (dist2d(x, z, cmd_posts[army][i].pos[j].x, cmd_posts[army][i].pos[j].z) < 15.0f)
                return 0;
    return 1;
}

static int roads_add_post(int army, int kind, int ref, int line, int pair, int lane, float gx, float gz,
                          const cmd_post_pos_t *pos, int npos)
{
    if (cmd_nposts[army] >= CMD_POST_MAX || npos <= 0)
        return -1;
    cmd_post_t *p = &cmd_posts[army][cmd_nposts[army]];
    *p = (cmd_post_t){0};
    p->kind = kind;
    p->ref = ref;
    p->line = line;
    p->pair = pair;
    p->lane = lane;
    p->cap = cmd_cap_index(&lanes[lane]);
    p->route = -1;
    for (int j = 0; j < CMD_POST_SAMPLES; ++j) {
        p->gx[j] = gx;
        p->gz[j] = gz;
    }
    for (int i = 0; i < npos && i < CMD_POST_OPTIONS; ++i)
        p->pos[p->npos++] = pos[i];
    ++roads.posts[army];
    return cmd_nposts[army]++;
}

static void roads_build_posts(void *host, int army)
{
    static const char *const kinds[] = {"junction", "bridge", "street", "grid"};
    u8 done_choke[MAX_RCHOKES] = {0}, done_ow[MAX_ROWS] = {0};
    /* Chokes, most valuable first, then overwatch positions, best first (CMD_POST_MAX bounds both). */
    for (;;) {
        int c = -1;
        for (int i = 0; i < nrchokes; ++i)
            if (!done_choke[i] && rchokes[i].army == army && (c < 0 || rchokes[i].value > rchokes[c].value))
                c = i;
        if (c < 0)
            break;
        done_choke[c] = 1;
        const rchoke_t *ch = &rchokes[c];
        const int lane = roads_lane_of_cap(army, ch->zx, ch->zz);
        if (lane < 0) {
            ++roads.stat[army][ch->line - 1][RS_CHOKE_NOLANE];
            event(host, "roads: army %d choke %d skipped: no point of ours at (%.0f,%.0f)", army, ch->id, ch->zx, ch->zz);
            continue;
        }
        cmd_post_pos_t spot[CMD_POST_OPTIONS];
        u8 taken[MAX_RCHOKE_SPOTS] = {0};
        int ns = 0, dropped = 0, *st = roads.stat[army][ch->line - 1];
        while (ns < CMD_POST_OPTIONS) {
            int best = -1;
            for (int i = 0; i < nrchoke_spots; ++i) {
                const rchoke_spot_t *sp = &rchoke_spots[i];
                if (!taken[i] && sp->army == army && sp->choke == ch->id && (best < 0 || sp->score > rchoke_spots[best].score))
                    best = i;
            }
            if (best < 0)
                break;
            taken[best] = 1;
            const rchoke_spot_t *sp = &rchoke_spots[best];
            const int stand = cmd_post_standable(sp->x, sp->z, sp->fx, sp->fz), free_ = stand && roads_spot_free(army, sp->x, sp->z);
            if (!stand || !free_) {
                ++dropped;
                ++st[stand ? RS_SPOT_SPACING : RS_SPOT_STAND];
                continue;
            }
            spot[ns++] = (cmd_post_pos_t){sp->x, sp->z, sp->score, 0, sp->fx, sp->fz};
        }
        st[RS_SPOT_KEPT] += ns;
        if (!ns) {
            ++st[RS_CHOKE_NOSPOT];
            event(host, "roads: army %d choke %d skipped: no standable spot (%d dropped)", army, ch->id, dropped);
            continue;
        }
        const float value = fminf(1.0f, fmaxf(0.0f, ch->value));
        const int two = value >= ROADS_PAIR_VALUE && ns >= 2;
        cmd_post_pos_t first[CMD_POST_OPTIONS] = {spot[0]};
        int nfirst = 1;
        for (int i = 1; i < ns; ++i)
            if (!two || i != 1)
                first[nfirst++] = spot[i];
        const int id = roads_add_post(army, PK_CHOKE, c, ch->line, 0, lane, ch->x, ch->z, first, nfirst);
        const int id2 = two && id >= 0 ? roads_add_post(army, PK_CHOKE, c, ch->line, 1, lane, ch->x, ch->z, &spot[1], 1) : -1;
        st[RS_FULL] += (id < 0) + (two && id2 < 0);
        st[RS_CHOKE_POSTS] += (id >= 0) + (id2 >= 0);
        if (id < 0)
            continue;
        event(host, "roads: army %d post %d choke %d (%s, line %d) for cap %d lane %d at (%.0f,%.0f), value %.2f, "
              "%d spot%s (%d dropped)%s", army, CMD_POST_ID_BASE + id, ch->id, kinds[ch->kind & 3], ch->line,
              cmd_posts[army][id].cap, lane, ch->x, ch->z, value, nfirst, nfirst > 1 ? "s" : "", dropped,
              id2 >= 0 ? ", needs two: second post follows" : "");
        if (id2 >= 0)
            event(host, "roads: army %d post %d choke %d second crew, spot (%.0f,%.0f)", army, CMD_POST_ID_BASE + id2,
                  ch->id, spot[1].x, spot[1].z);
    }
    for (;;) {
        int w = -1;
        for (int i = 0; i < nrows; ++i)
            if (!done_ow[i] && rows[i].army == army && (w < 0 || rows[i].score > rows[w].score))
                w = i;
        if (w < 0)
            break;
        done_ow[w] = 1;
        const rows_t *o = &rows[w];
        int *st = roads.stat[army][o->line - 1];
        const int lane = roads_nearest_lane(army, o->x, o->z);
        const int stand = lane >= 0 && cmd_post_standable(o->x, o->z, o->fx, o->fz);
        const int why = lane < 0 ? RS_OW_NOLANE : !stand ? RS_OW_STAND : !roads_spot_free(army, o->x, o->z) ? RS_OW_SPACING : -1;
        if (why >= 0) {
            ++st[why];
            event(host, "roads: army %d overwatch %d (line %d) skipped: %s", army, o->id, o->line,
                  why == RS_OW_NOLANE ? "no point of ours" : why == RS_OW_STAND ? "not standable" : "within 15 m of another post");
            continue;
        }
        const cmd_post_pos_t pos = {o->x, o->z, o->score, 0, o->fx, o->fz};
        const int id = roads_add_post(army, PK_OVERWATCH, w, o->line, 0, lane, o->fx, o->fz, &pos, 1);
        ++st[id < 0 ? RS_FULL : RS_OW_POSTS];
        if (id < 0)
            continue;
        event(host, "roads: army %d post %d overwatch %d (line %d) at (%.0f,%.0f) faces (%.0f,%.0f), score %.2f, arc %.0f deg, "
              "cover %.2f, falls back to lane %d", army, CMD_POST_ID_BASE + id, o->id, o->line, o->x, o->z, o->fx, o->fz,
              o->score, o->arc, o->cover, lane);
    }
    for (int line = 0; line < 2; ++line) {
        const int *st = roads.stat[army][line];
        if (nrchokes || nrows)
            event(host, "roads: army %d line %d built: choke posts %d (chokes without a point of ours %d, without a spot %d; "
                  "spots kept %d, dropped not standable %d, within 15 m %d) | overwatch posts %d (dropped: no point %d, "
                  "not standable %d, within 15 m %d) | post table full %d", army, line + 1, st[RS_CHOKE_POSTS],
                  st[RS_CHOKE_NOLANE], st[RS_CHOKE_NOSPOT], st[RS_SPOT_KEPT], st[RS_SPOT_STAND], st[RS_SPOT_SPACING],
                  st[RS_OW_POSTS], st[RS_OW_NOLANE], st[RS_OW_STAND], st[RS_OW_SPACING], st[RS_FULL]);
    }
}

/* The line whose posts of this kind are offered: line 2 only while the army is on it and has some. */
static int roads_line_of(int army, int kind)
{
    if (roads.line[army] != 2)
        return 1;
    for (int i = 0; i < cmd_nposts[army]; ++i)
        if (cmd_posts[army][i].kind == kind && cmd_posts[army][i].line == 2)
            return 2;
    return 1;
}

int tp_active(int army);         /* ground_teamplan.inc */
/* s41: while the army has an active format-2 plan (a lane pack), the places to stand come from the pack only: a post
 * that is not a plan post (command gate posts; roads choke, second-crew and overwatch posts) is off: never offered,
 * never handed out. Format 1 and battles without a pack: as before. */
static int tp2_old_post_off(int army, const cmd_post_t *p)
{
    return tp_format == 2 && tp_active(army) && !PK_PLAN(p->kind);
}

static int roads_post_offered(int army, const cmd_post_t *p)
{
    if (tp2_old_post_off(army, p))
        return 0;                       /* s41 */
    if (PK_PLAN(p->kind))
        return tp_watch_offered(army, p);  /* s32 / s33: the plan's watch and support areas */
    return p->kind == PK_GATE || p->line == roads_line_of(army, p->kind);
}

static float roads_post_value(int army, const cmd_post_t *p, uint64_t ms)
{
    if (p->kind == PK_WATCH)
        return TP_WATCH_VALUE;
    if (p->kind == PK_SUPPORT)
        return TP_SUPPORT_VALUE;
    if (p->kind == PK_CHOKE) {
        float v = (55.0f + 40.0f * fminf(1.0f, fmaxf(0.0f, rchokes[p->ref].value))) * (p->pair ? 0.85f : 1.0f);
        if (roads.start_ms[army] && ms - roads.start_ms[army] < ROADS_BOOST_MS)
            v *= ROADS_BOOST;          /* the enemy is still on its way: chokes matter most */
        return v;
    }
    return 55.0f + 40.0f * fminf(1.0f, fmaxf(0.0f, rows[p->ref].score));
}

/* Push line (spec section 10 step D): line 2 while the army owns most of its
 * points, line 1 otherwise; either way the new state must hold ROADS_LINE_MS without a break. */
static void roads_line_update(void *host, int army, int owned, int total, uint64_t ms)
{
    if (!roads.start_ms[army])
        roads.start_ms[army] = ms;
    if (!roads.line[army])
        roads.line[army] = 1;
    const int want = total > 0 && 2 * owned > total ? 2 : 1;
    if (want == roads.line[army]) {
        roads.cand_ms[army] = 0;
        return;
    }
    if (!roads.cand_ms[army]) {
        roads.cand_ms[army] = ms;
        return;
    }
    if (ms - roads.cand_ms[army] < ROADS_LINE_MS)
        return;
    int manned = 0;
    for (int i = 1; i < MAX_SEATS; ++i)
        manned += lseat[i].army == army && cmd_is_post(&lseat[i]) && lseat[i].cmd_id - CMD_POST_ID_BASE < cmd_nposts[army]
                  && cmd_posts[army][lseat[i].cmd_id - CMD_POST_ID_BASE].kind != PK_GATE;
    if (roads.posts[army])
        event(host, "roads: army %d line %d -> %d: owns %d of %d points for %.0f s; %d crews on choke/overwatch posts "
              "re-evaluated", army, roads.line[army], want, owned, total, (double)(ms - roads.cand_ms[army]) / 1000.0, manned);
    roads.line[army] = want;
    roads.cand_ms[army] = 0;
    roads.switched[army] = 1;           /* cmd_tick: roads_line_release */
}

/* Per-task vehicle fit (spec 8b), replacing the overwatch role weight for these tasks. Each term
 * maps a vehicle value onto 0.05..1 between lo and hi (lo > hi: smaller is better); a missing
 * value is a neutral 0.5. The raw fit is the geometric mean of its terms, normalised over the
 * army's lineup to 0..1 (0.5 when every vehicle scores the same). SPAA: none of these tasks. */
static float roads_term(int have, float v, float lo, float hi)
{
    if (!have || !isfinite(v))
        return 0.5f;
    const float t = (v - lo) / (hi - lo);
    return t < 0.05f ? 0.05f : t > 1.0f ? 1.0f : t;
}

const vfit_t *roads_vfit(int seat)
{
    return active_cls(seat, slot_unit[seat]) == &reserve_cls[seat] ? &reserve_vfit[seat] : &seat_vfit[seat];
}

static float roads_fit_raw(int seat, int task)
{
    const vcls_t *c = active_cls(seat, slot_unit[seat]);
    const vfit_t *f = roads_vfit(seat);
    float t;
    if (task == RT_OVERWATCH) {
        /* gun depression (hull down behind a crest) x muzzle velocity x reverse speed x stabiliser */
        const float stab = !c->ok ? 0.5f : (c->flags & VC_STAB_V) ? ((c->flags & VC_STAB_H) ? 1.0f : 0.75f)
                           : (c->flags & VC_STAB_H) ? 0.45f : 0.3f;
        t = roads_term(f->ok && f->dep >= 0.0f, f->dep, 3.0f, 15.0f) * roads_term(c->ok && c->speed > 0.0f, c->speed, 600.0f, 1800.0f)
            * roads_term(f->ok && f->rev >= 0.0f, f->rev, 4.0f, 30.0f) * stab;
        return powf(t, 0.25f);
    }
    if (task == RT_CHOKE) {
        /* penetration x reload x frontal armour (close and medium range, mostly urban) */
        t = roads_term(c->ok && c->pen > 0.0f, c->pen, 50.0f, 500.0f) * roads_term(f->ok && f->reload > 0.0f, f->reload, 12.0f, 3.0f)
            * roads_term(c->ok && c->front > 0.0f, c->front, 20.0f, 400.0f);
        return powf(t, 1.0f / 3.0f);
    }
    /* street flank: power-to-weight x small hull x reverse speed */
    t = roads_term(c->ok && c->hp > 0.0f, c->hp, 10.0f, 35.0f)
        * roads_term(f->ok && f->len > 0.0f && f->wid > 0.0f, f->len * f->wid, 40.0f, 15.0f)
        * roads_term(f->ok && f->rev >= 0.0f, f->rev, 4.0f, 30.0f);
    return powf(t, 1.0f / 3.0f);
}

static int roads_seat_army(int seat)
{
    return lseat[seat].unit ? lseat[seat].army : authored_spawn_seat_army[seat];
}

float roads_fit(int army, int seat, int task)
{
    if (strat_spaa(seat))
        return 0.0f;
    const float raw = roads_fit_raw(seat, task);
    float lo = raw, hi = raw;
    for (int i = 1; i < MAX_SEATS; ++i) {
        if (i == seat || roads_seat_army(i) != army || !(seat_cls[i].ok || lseat[i].unit) || strat_spaa(i))
            continue;
        const float r = roads_fit_raw(i, task);
        lo = fminf(lo, r);
        hi = fmaxf(hi, r);
    }
    return hi - lo > 0.02f ? (raw - lo) / (hi - lo) : 0.5f;
}

/* Turretless: a position whose field of fire holds 80 % of its weight within +-30 deg costs
 * nothing, +-60 deg and wider halves the fit (linear between). */
static float roads_arc_factor(float arc_deg)
{
    if (!isfinite(arc_deg) || arc_deg < 0.0f)
        return 0.75f;           /* unknown arc (-1): halfway */
    if (arc_deg <= 30.0f)
        return 1.0f;
    return arc_deg >= 60.0f ? 0.5f : 1.0f - 0.5f * (arc_deg - 30.0f) / 30.0f;
}

int roads_turretless(int seat)
{
    const vfit_t *f = roads_vfit(seat);
    return f->ok && f->turret == 0.0f;
}

/* The fit x role weight that cmd_fitw returns for a choke or overwatch task: 0.4 (CMD_W_FLOOR)
 * for the worst vehicle of the lineup up to 1.5 (an overwatch vehicle on an overwatch post today). */
float roads_fitw(int army, int seat, int kind, int ref)
{
    if (strat_spaa(seat))
        return 0.0f;
    float w = 0.4f + 1.1f * roads_fit(army, seat, kind == PK_CHOKE ? RT_CHOKE : RT_OVERWATCH);
    if (kind == PK_OVERWATCH && roads_turretless(seat))
        w *= roads_arc_factor(rows[ref].arc);
    return w;
}

/* The roads post behind objective o, or NULL (a gate post or no post). */
static const cmd_post_t *roads_post_of(int army, const cmd_obj_t *o)
{
    if (army < 1 || army > 2 || o->type != OB_OVERWATCH || o->id < CMD_POST_ID_BASE
        || o->id - CMD_POST_ID_BASE >= cmd_nposts[army])
        return NULL;
    const cmd_post_t *p = &cmd_posts[army][o->id - CMD_POST_ID_BASE];
    return p->kind != PK_GATE ? p : NULL;
}

/* Band overwatch on a map without points of ours: the best precomputed position of the active
 * line in band b, not ahead of the band's front, at most 2 x CMD_OW_MAX_M behind it; -1 none. */
static int roads_band_row(int army, int b, float front)
{
    const strat_meta_t *m = strat.m;
    const int line = roads.line[army] == 2 ? 2 : 1;
    int best = -1;
    for (int pass = 0; pass < 2 && best < 0; ++pass)
        for (int i = 0; i < nrows; ++i) {
            const rows_t *o = &rows[i];
            const float la = strat_lat(o->x, o->z), dp = strat_depth(army, o->x, o->z);
            if (o->army != army || o->line != (pass ? 1 : line) || la < m->edge[b] || la >= m->edge[b + 1]
                || dp > front || dp < front - 2.0f * CMD_OW_MAX_M)
                continue;
            if (best < 0 || o->score > rows[best].score)
                best = i;
        }
    return best;
}

static void cmd_build_posts(void *host, int army)
{
    if (cmd_posts_ready[army] || cfg.objective_mode == OBJECTIVE_KILL)
        return;
    cmd_posts_ready[army] = 1;
    cmd_post_t gate[MAX_TROUTES];
    int ngate = 0;
    for (int lane = 0; lane < nlanes; ++lane) {
        const lane_t *l = &lanes[lane];
        if (l->army != army || l->kind != 0)
            continue;
        int cap = cmd_cap_index(l);
        if (cap < 0)
            continue;
        for (int i = 0; i < ntroutes; ++i) {
            const troute_t *r = &troutes[i];
            if (r->army != 3 - army || dist2d(r->zx, r->zz, l->zx, l->zz) >= 40.0f)
                continue;
            cmd_post_t g = {0};
            if (!cmd_post_gate(r, l, g.gx, g.gz))
                continue;
            float weight = r->variant == TV_FAST ? 1.6f : r->variant == TV_FLANK ? 0.7f : 1.0f;
            int k = 0;
            while (k < ngate && (gate[k].cap != cap
                   || dist2d(g.gx[1], g.gz[1], gate[k].gx[1], gate[k].gz[1]) >= CMD_POST_GATE_GAP_M))
                ++k;
            if (k < ngate) {
                gate[k].value += weight;
                continue;
            }
            if (ngate == MAX_TROUTES)
                continue;
            g.cap = cap;
            g.lane = lane;
            g.route = i;
            g.value = weight;
            gate[ngate++] = g;
        }
    }
    for (int round = 0; round < ngate && cmd_nposts[army] < CMD_POST_GATES; ++round) {
        int best = -1;
        for (int i = 0; i < ngate; ++i)
            if (gate[i].value > 0.0f && (best < 0 || gate[i].value > gate[best].value))
                best = i;
        if (best < 0)
            break;
        cmd_post_t g = gate[best];
        float weight = g.value;
        gate[best].value = 0.0f;
        int same = 0;
        for (int i = 0; i < cmd_nposts[army]; ++i)
            same += g.cap == cmd_posts[army][i].cap;
        if (same >= 2 || cmd_post_overlaps(army, &g))
            continue;
        cmd_post_sites(army, &g);
        /* A bend can put the wider sample window behind buildings. Look for a
         * shorter exposed section on the same approach before dropping it. */
        static const float offsets[] = {30.0f, 95.0f, 155.0f, 215.0f};
        for (int i = 0; i < 4 && g.npos < 2; ++i) {
            cmd_post_t alternate = g;
            alternate.npos = 0;
            if (!cmd_post_gate_at(&troutes[g.route], lanes[g.lane].zr + offsets[i], 32.0f, alternate.gx, alternate.gz)
                || cmd_post_overlaps(army, &alternate))
                continue;
            cmd_post_sites(army, &alternate);
            if (alternate.npos > g.npos)
                g = alternate;
        }
        if (g.npos < 2)
            continue;
        g.value = 55.0f + 7.0f * g.pos[0].quality + 4.0f * fminf(4.0f, weight);
        int id = cmd_nposts[army]++;
        cmd_posts[army][id] = g;
        event(host, "cmd: army %d post %d cap %d lane %d covers route %d gate (%.0f,%.0f), %d positions",
              army, CMD_POST_ID_BASE + id, g.cap, g.lane, g.route, g.gx[1], g.gz[1], g.npos);
    }
    roads_build_posts(host, army);
}

int cmd_post_option(const cmd_post_t *p, uint64_t ms)
{
    for (int i = 0; i < p->npos; ++i)
        if (ms >= p->pos[i].blocked_until)
            return i;
    return -1;
}


static int cmd_objectives(void *host, int army, uint64_t ms, cmd_obj_t *ob, int *all_ours)
{
    const strat_meta_t *m = strat.m;
    const strat_view_t *v = &sview[army];
    int n = 0, npts = 0, owned = 0;
    *all_ours = 1;
    /* Stage 2 garrison: the owned point deepest towards the enemy. */
    int forward = -1;
    float forward_depth = -1e18f;
    for (int k = 0; cfg.objective_mode != OBJECTIVE_KILL && k < nlanes; ++k) {
        const lane_t *l = &lanes[k];
        if (l->army == army && l->kind == 0 && zone_owner(host, l->zx, l->zz) == army
            && strat_depth(army, l->zx, l->zz) > forward_depth) {
            forward_depth = strat_depth(army, l->zx, l->zz);
            forward = k;
        }
    }
    for (int k = 0; cfg.objective_mode != OBJECTIVE_KILL && k < nlanes && n < CMD_MAX_OBJ; ++k) {
        const lane_t *l = &lanes[k];
        if (l->army != army || l->kind != 0)
            continue;
        ++npts;
        int owner = zone_owner(host, l->zx, l->zz), ne = cmd_known_near(v, l->zx, l->zz, CMD_ENEMY_M, CMD_ENEMY_AGE_S);
        int cap = cmd_cap_index(l);
        cmd_obj_t o = {0, k, 0, strat_band(strat_lat(l->zx, l->zz)), k, 0, cap, l->zx, l->zz, 0.0f, 0};
        owned += owner == army;
        if (owner != army) {
            *all_ours = 0;
            o.type = OB_CAPTURE;
            int local = 0;
            for (int j = 0; j < v->no; ++j)
                local += dist2d(v->own[j].x, v->own[j].z, l->zx, l->zz) < CMD_ENEMY_M;
            int contest = ne > 3 ? 3 : ne;
            int relief = v->point_pressure >= 0.65f && local <= ne;
            o.need = 2 + contest + relief;
            o.value = (owner > 0 ? 110.0f : 100.0f) + v->point_pressure * (60.0f + 4.0f * contest);
        } else {
            /* Stage 2: every point we own always has a HOLD, at least its
             * garrison, not charged to the passive budget and independent of
             * where the front is (every loss in the v11 logs happened with
             * nobody in the circle). A known enemy within 300 m raises it to
             * the defence of design row 3. */
            const int garrison = k == forward ? CMD_GARRISON_FORWARD : 1;
            o.type = OB_HOLD;
            o.need = ne > 3 ? 3 : ne;
            o.need = o.need < garrison ? garrison : o.need;
            o.value = ne > 0 ? 90.0f : CMD_GARRISON_VALUE;
        }
        ob[n++] = o;
    }
    if (!npts)
        *all_ours = 0;
    if (npts) {
        cmd_build_posts(host, army);
        roads_line_update(host, army, owned, npts, ms);
        for (int j = 0; j < cmd_nposts[army] && n < CMD_MAX_OBJ; ++j) {
            const cmd_post_t *p = &cmd_posts[army][j];
            int option = cmd_post_option(p, ms);
            if (option < 0 || PK_PLAN(p->kind) || !roads_post_offered(army, p))
                continue;               /* s32: watch slots are offered by the plan (tp_apply) */
            int known = cmd_known_near(v, p->gx[1], p->gz[1], 165.0f, CMD_ENEMY_AGE_S);
            const cmd_post_pos_t *pos = &p->pos[option];
            int band = strat_band(strat_lat(p->gx[1], p->gz[1]));
            ob[n++] = (cmd_obj_t){OB_OVERWATCH, CMD_POST_ID_BASE + j, 1, band, p->lane, 0, p->cap, pos->x, pos->z,
                                  p->kind ? roads_post_value(army, p, ms) : p->value + 5.0f * fminf(3.0f, (float)known), 0};
        }
    }
    /* SUPPORT the human (off since stage 2, see cmd.support_on) */
    int h = human_slot;
    if (cmd.support_on && h >= 0 && h < 64 && slot_alive[h] && slot_army[h] == army && slot_unit[h]) {
        if (cmd.sup_life[army] != slot_life[h]) {
            cmd.sup_life[army] = slot_life[h];
            cmd.sup_on[army] = 0;
        }
        const float *p = gh_unit_pos(slot_unit[h]);
        float hd = strat_depth(army, p[0], p[2]), ahead = hd - v->dmed;
        int hb = strat_band(strat_lat(p[0], p[2]));
        int in_contact = ring_count(hits_dealt_ms[h], ms, 10000) || ring_count(hits_taken_ms[h], ms, 10000)
                      || cmd_known_near(v, p[0], p[2], CMD_CONTACT_M, 10.0f);
        if (!cmd.sup_on[army] && (ahead > CMD_SUP_AHEAD_M || (in_contact && v->ratio[hb] < CMD_SUP_RATIO))) {
            cmd.sup_on[army] = 1;
            cmd.sup_since[army] = ms;
            event(host, "cmd: army %d supports the human (slot %d): %.0f m ahead of the median, band %d ratio %.2f%s", army,
                  h, ahead, hb, v->ratio[hb], in_contact ? ", in contact" : "");
        } else if (cmd.sup_on[army] && ms - cmd.sup_since[army] > CMD_SUP_MIN_MS && ahead < CMD_SUP_BEHIND_M
                   && (!in_contact || v->ratio[hb] > CMD_SUP_RATIO_OFF)) {
            cmd.sup_on[army] = 0;
            event(host, "cmd: army %d support of the human over (%.0f m ahead, band %d ratio %.2f)", army, ahead, hb,
                  v->ratio[hb]);
        }
        if (cmd.sup_on[army] && n < CMD_MAX_OBJ)
            ob[n++] = (cmd_obj_t){OB_SUPPORT, h, v->point_pressure > 0.0f ? 1 : 3, hb, -1, 0, -1,
                                   p[0], p[2], 105.0f - 32.0f * v->point_pressure, 1};
    } else {
        cmd.sup_on[army] = 0;
    }
    /* per band: OVERWATCH, PUSH, PRESS */
    int centre = v->nb / 2;
    for (int b = 0; b < v->nb && b < STRAT_MAXB; ++b) {
        float lat = 0.5f * (m->edge[b] + m->edge[b + 1]), x, z;
        float likely = v->fen[b] + v->fhid[b];
        cmd.ow_on[army][b] = likely >= CMD_OW_LIKELY || (cmd.ow_on[army][b] && likely >= CMD_OW_LIKELY_OFF);
        if (!npts && cmd.ow_on[army][b] && n < CMD_MAX_OBJ) {
            /* v13: a precomputed overwatch position of the active line replaces the search (fallback). */
            const int refresh = ms >= cmd.ow_ms[army][b] || fabsf(v->front[b] - cmd.ow_front[army][b]) > 40.0f;
            const int row = refresh ? roads_band_row(army, b, v->front[b]) : roads.band_ow[army][b] - 1;
            if (row >= 0 && refresh) {
                if (roads.band_ow[army][b] != row + 1)
                    event(host, "roads: army %d band %d overwatch from position %d (%.0f,%.0f), score %.2f", army, b,
                          rows[row].id, rows[row].x, rows[row].z, rows[row].score);
                cmd.ow_x[army][b] = rows[row].x;
                cmd.ow_z[army][b] = rows[row].z;
                cmd.ow_score[army][b] = rows[row].score;
                cmd.ow_ok[army][b] = 1;
                cmd.ow_ms[army][b] = ms + CMD_OW_MS;
                cmd.ow_front[army][b] = v->front[b];
            } else if (row < 0 && roads.band_ow[army][b]) {
                cmd.ow_ok[army][b] = 0;         /* the precomputed position no longer fits: search as before */
                cmd.ow_ms[army][b] = 0;
            }
            roads.band_ow[army][b] = row + 1;
            if (row < 0 && (ms >= cmd.ow_ms[army][b] || fabsf(v->front[b] - cmd.ow_front[army][b]) > 40.0f)) {
                float sx = 0, sz = 0, sc = 0;
                int ok = cmd_ow_spot(army, b, v->front[b], &sx, &sz, &sc);
                cmd.ow_ms[army][b] = ms + CMD_OW_MS;
                cmd.ow_front[army][b] = v->front[b];
                /* keep the old spot unless the new one is clearly better */
                if (ok && (!cmd.ow_ok[army][b] || sc > 1.2f * cmd.ow_score[army][b]
                           || fabsf(strat_depth(army, cmd.ow_x[army][b], cmd.ow_z[army][b]) - v->front[b]) > CMD_OW_MAX_M)) {
                    cmd.ow_x[army][b] = sx;
                    cmd.ow_z[army][b] = sz;
                    cmd.ow_score[army][b] = sc;
                }
                cmd.ow_ok[army][b] = ok || cmd.ow_ok[army][b];
                if (!ok && fabsf(strat_depth(army, cmd.ow_x[army][b], cmd.ow_z[army][b]) - v->front[b]) > CMD_OW_MAX_M)
                    cmd.ow_ok[army][b] = 0;
            }
            if (cmd.ow_ok[army][b])
                ob[n++] = (cmd_obj_t){OB_OVERWATCH, b, 1, b, -1, b != centre, -1,
                                       cmd.ow_x[army][b], cmd.ow_z[army][b], 65.0f - 22.0f * v->point_pressure, 1};
        }
        float r = v->ratio[b];
        cmd.push_on[army][b] = r >= CMD_PUSH_RATIO || (cmd.push_on[army][b] && r >= CMD_PUSH_OFF);
        if (cmd.push_on[army][b] && !*all_ours && v->front[b] > 0.5f * m->L && n < CMD_MAX_OBJ
            && v->point_pressure < 0.65f) {
            float d = v->front[b] + CMD_PUSH_AHEAD_M;
            d = d > m->L - CMD_PRESS_M ? m->L - CMD_PRESS_M : d;
            strat_world(army, d, lat, &x, &z);
            if (cmd_snap(x, z, &x, &z) >= 0)
                ob[n++] = (cmd_obj_t){OB_PUSH, b, 2, b, -1, b != centre, -1, x, z, 60.0f + 15.0f * (r < 2.0f ? r : 2.0f)};
        }
        if (*all_ours && n < CMD_MAX_OBJ) {
            strat_world(army, m->L - CMD_PRESS_M, lat, &x, &z);
            if (cmd_snap(x, z, &x, &z) >= 0)
                ob[n++] = (cmd_obj_t){OB_PRESS, b, b == centre ? 3 : 1, b, -1, b != centre, -1, x, z, b == centre ? 88.0f : 70.0f};
        }
    }
    /* Stage 3: tanks of ours at a point are fired at from outside it. */
    return npts ? flank_objectives(host, army, ms, ob, n) : n;
}

/* 0 = closed: one overwatch per band, three wingmen at most. */
static float cmd_dec(int count, const cmd_obj_t *o)
{
    if (count >= o->need && o->passive)
        return 0.0f;
    return count < o->need ? 1.0f : 0.25f * powf(0.5f, (float)(count - o->need));
}

static int cmd_want(const cmd_obj_t *o)
{
    switch (o->type) {
    case OB_SUPPORT: return SR_FLANK;
    case OB_OVERWATCH: return SR_OVERWATCH;
    case OB_PUSH:
    case OB_PRESS: return o->wing ? SR_FLANK : SR_BREACH;
    default: return SR_BREACH;
    }
}

static float cmd_weight(int seat, int role, int want)
{
    const vcls_t *c = active_cls(seat, slot_unit[seat]);
    float w = c->ok ? c->w[want] : want == role ? 1.0f : 0.0f;
    if (c->ok && want == role && w < 1.0f)
        w = 1.0f;               /* the default role always counts fully */
    return w > CMD_W_FLOOR ? w : CMD_W_FLOOR;
}

/* fit x role weight of a vehicle for an objective (design section 0 / 3). */
static float cmd_fitw(int seat, int role, const cmd_obj_t *o)
{
    /* v13: choke and overwatch tasks use the per-task vehicle fit (roads_fitw) instead. */
    const int army = lseat[seat].army;
    const cmd_post_t *rp = roads_post_of(army, o);
    if (rp)
        return roads_fitw(army, seat, rp->kind, rp->ref);
    if (o->type == OB_OVERWATCH && o->id >= 0 && o->id < STRAT_MAXB && army >= 1 && army <= 2 && roads.band_ow[army][o->id])
        return roads_fitw(army, seat, PK_OVERWATCH, roads.band_ow[army][o->id] - 1);
    int pt = o->type == OB_CAPTURE || o->type == OB_HOLD, big = o->type == OB_PUSH || o->type == OB_PRESS;
    if (role == SR_ESCORT)      /* SPAA follows the main force */
        return o->type == OB_SUPPORT ? 1.0f : pt ? 0.8f : CMD_W_FLOOR;
    float fit = 1.0f;
    if (role == SR_BREACH && (pt || (big && !o->wing)))
        fit = 1.2f;
    else if (role == SR_FLANK && (o->type == OB_SUPPORT || (big && o->wing)))
        fit = 1.2f;
    else if (role == SR_OVERWATCH)
        fit = o->type == OB_OVERWATCH ? 1.5f : pt ? 0.7f : 1.0f;
    return fit * cmd_weight(seat, role, cmd_want(o));
}

static float cmd_risk(int army, float x0, float z0, float x1, float z1)
{
    float d = dist2d(x0, z0, x1, z1), sum = 0.0f;
    for (int k = 1; k <= 5; ++k) {
        float f = (float)k / 5.0f;
        int c = strat_cellof(x0 + (x1 - x0) * f, z0 + (z1 - z0) * f);
        if (c >= 0)
            sum += strat.enc[army][c];
    }
    float r = 5.0f * sum * (d / 5.0f) / 100.0f;
    return r < CMD_RISK_MAX ? r : CMD_RISK_MAX;
}

static float cmd_travel(int seat, const lane_seat_t *s, const cmd_obj_t *o)
{
    float t = dist2d(s->x, s->z, o->x, o->z) / 8.0f * 1.3f;
    int cell = strat_cellof(s->x, s->z);
    if ((o->type == OB_CAPTURE || o->type == OB_HOLD) && o->cap >= 0 && strat.cap_src[o->cap] >= 0 && cell >= 0) {
        uint16_t tt = strat.travel[(size_t)strat.cap_src[o->cap] * (size_t)strat.n + (size_t)cell];
        if (tt != 0xffff)
            t = (float)tt;
    }
    float hp = seat_hp[seat];
    if (hp > 0.0f) {
        float k = 16.0f / hp;
        t *= k < 0.75f ? 0.75f : k > 1.3f ? 1.3f : k;
    }
    return t;
}

void cmd_log(void *host, int army, int force, const char *fmt, ...)
{
    if (!force && cmd.logs[army] >= CMD_LOGS_PER_MIN) {
        ++cmd.dropped[army];
        return;
    }
    ++cmd.logs[army];
    char msg[400];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    event(host, "%s", msg);
}

#include "ground_task_command.inc"

/* v13: after a line switch the crews on posts of the line left (no longer offered) are free at
 * once: their orders are invalidated (pending ones cancelled), so the same assignment counts them
 * as available and the post budget is not held by posts that are gone. */
static void roads_line_release(void *host, int army, uint64_t ms)
{
    if (!roads.switched[army])
        return;
    roads.switched[army] = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *s = &lseat[i];
        cmd_task_state_t *st = &cmd_tasks[i];
        if (s->army != army)
            continue;
        const ground_task_order_t *orders[2] = {&st->active, &st->pending};
        for (int k = 0; k < 2; ++k) {
            const ground_task_order_t *o = orders[k];
            const int j = o->objective - CMD_POST_ID_BASE;
            if (!o->task_id || o->type != OB_OVERWATCH || j < 0 || j >= cmd_nposts[army]
                || cmd_posts[army][j].kind == PK_GATE || roads_post_offered(army, &cmd_posts[army][j]))
                continue;
            cmd_log(host, army, 1, "roads: army %d seat %d %s %s post %d of line %d: the army is on line %d now", army, i,
                    k ? "cancels its order to" : "leaves", cmd_posts[army][j].kind == PK_CHOKE ? "choke" : "overwatch",
                    o->objective, cmd_posts[army][j].line, roads.line[army]);
            if (k) {
                cmd_task_event(host, "pending_cancelled", &st->pending, ms, GTR_OBJECTIVE_LOST);
                cmd_clear_pending(s, st);
            } else if (!st->invalid_reason) {
                cmd_invalidate(host, st, ms, GTR_OBJECTIVE_LOST);
            }
        }
    }
}

/* first: 0 reassignment, 1 first order of a life, 2 continuation (stage 2):
 * the point changed hands under the order and the same tank keeps the same
 * point under the other duty (garrison after a capture, recapture after a loss). */
static int tp_take_variant(int seat, const cmd_obj_t *o);    /* ground_teamplan.inc */
static void tp_note_first(void *host, int army, int seat, const cmd_obj_t *o);
static int tp_take_target(int seat);
static int tp2_lane_slot_target(int army, int seat, const cmd_obj_t *o);

void cmd_issue(void *host, int army, int seat, const cmd_obj_t *o, float u, int first, uint64_t ms)
{
    lane_seat_t *s = &lseat[seat];
    int from = s->cmd_type;
    ground_task_order_t order = cmd_make_order(army, seat, o, ms, cmd_tasks[seat].invalid_reason);
    order.route_variant = tp_take_variant(seat, o);     /* s32: this tick's flank pick, else 0 */
    order.goal_target1 = tp_take_target(seat);          /* s35: the lane slot of this tick's lane pick, else 0 */
    if (!order.goal_target1)    /* s37 G14: the zone objective of its own circle lane keeps that lane's slot */
        order.goal_target1 = tp2_lane_slot_target(army, seat, o);
    if (!cmd_publish(host, s, seat, order, ms))
        return;
    float d = dist2d(s->x, s->z, o->x, o->z);
    if (first == 2) {
        cmd_log(host, army, 0, "cmd: army %d seat %d keeps point %d as %s (garrison/recapture), %.0f m, U %.0f, was %s",
                army, seat, o->id, cmd_name[o->type], d, u, cmd_name[from]);
        return;
    }
    cmd.far_negative[army] += !first && d > CMD_FAR_M && u < 0.0f;     /* reassignments only */
    cmd.beyond_reach[army] += u < 0.0f && d > cmd_spawn_reach(army, o);
    int is_far = !first && d > CMD_FAR_M;
    if (first)
        ++cmd.firsts[army];
    else
        ++cmd.switches[army];
    cmd.far_switches[army] += is_far;
    cmd.cross_switches[army] += is_far && fabsf(strat_lat(o->x, o->z) - strat_lat(s->x, s->z)) > CMD_CROSS_M;
    static const char *const role_name[SR_N] = {"flank", "breach", "ow", "escort"};
    int role = sview[army].role[seat & 63];
    cmd_log(host, army, is_far, "cmd: army %d seat %d (%s) %s %s %d at (%.0f,%.0f), %.0f m, U %.0f%s%s%s", army, seat,
            role_name[role & 3], first ? "first order:" : "->", cmd_name[o->type], o->id, o->x, o->z, d, u,
            first ? "" : ", was ", first ? "" : cmd_name[from], is_far ? " (far)" : "");
    if (first == 1)
        tp_note_first(host, army, seat, o);    /* s33: the plan item this first order fills */
}

int motion_combat_override(const lane_seat_t *s, uint64_t ms)
{
    return control_engagement_active(s, ms)
           || (nav_recent(s->last_ms, ms) && (s->nav.applied_source == NAV_REACTION || s->nav.applied_source == NAV_FIGHT));
}

int motion_committed(const lane_seat_t *s, uint64_t ms)
{
    return ms < s->l1_until || ms < s->direct_off_until || ms < s->route_turn_until || ms < s->motion_handoff_until
           || fight_committed(s, ms);
}

int cmd_combat_busy(const lane_seat_t *s, uint64_t ms)
{
    if (motion_combat_override(s, ms))
        return 1;
    if (s->mode != M_ENGAGE || s->fight_yield)
        return 0;
    const float d = dist2d(s->x, s->z, s->ex, s->ez);
    const int seen = s->eng_own && ms - s->eng_seen_ms < 1000;
    if (cmd_is_post(s))
        return cmd_post_self_defense(s, ms, d, seen);
    return fight_immediate(s, ms, d, seen)
           || (s->fight_since && ms - s->fight_since < FIGHT_CLOSE_MS
               && d < CMD_LOCK_M);
}


static int cmd_sharp_change(const lane_seat_t *s, const cmd_obj_t *o)
{
    if (!s->wants_move || s->holding || s->parked || s->cmd_parked
        || dist2d(s->x, s->z, s->move_target_x, s->move_target_z) < 10.0f
        || dist2d(s->x, s->z, o->x, o->z) < 10.0f)
        return 0;
    const float turn = movement_turn_error(s, o->x, o->z);
    return turn > CMD_SHARP_DEG
           && (turn > 130.0f || turn > movement_turn_error(s, s->move_target_x, s->move_target_z) + 25.0f);
}

static int cmd_objective_index(const cmd_obj_t *ob, int nob, int type, int id, int lane_point)
{
    for (int k = 0; k < nob; ++k)
        if (type == OB_NONE ? ((ob[k].type == OB_CAPTURE || ob[k].type == OB_HOLD) && ob[k].id == lane_point)
                            : (ob[k].type == type && ob[k].id == id))
            return k;
    return -1;
}

/* The army's census line every CMD_CENSUS_MS and the S2 metrics. */
int cmd_live(const lane_seat_t *s, int seat, int army, uint64_t ms)
{
    return s->n > 0 && s->scene_epoch == ground_scene_epoch && s->army == army
           && s->pose_observed_ms && ms >= s->pose_observed_ms
           && ms - s->pose_observed_ms <= 2000 && cmd_task_unit_alive(s, seat) && !cmd_seat_silent(s, ms);
}

/* Stage 2 helpers. The capture or hold objective whose circle the hull is in, or -1. */
static int cmd_point_at(const lane_seat_t *s, const cmd_obj_t *ob, int nob)
{
    for (int k = 0; k < nob; ++k)
        if ((ob[k].type == OB_CAPTURE || ob[k].type == OB_HOLD) && ob[k].lane >= 0
            && dist2d(s->x, s->z, ob[k].x, ob[k].z) < lanes[ob[k].lane].zr)
            return k;
    return -1;
}

/* Share of the present order done, 0..1: there (in the circle, at the goal) is
 * 1, en route the share of the straight distance from where it was accepted. */
static float cmd_progress(const lane_seat_t *s, int seat, const cmd_obj_t *o, uint64_t ms)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    const float r = (o->type == OB_CAPTURE || o->type == OB_HOLD) && o->lane >= 0 ? lanes[o->lane].zr
                    : st->active.task_id ? st->active.radius : CMD_ARRIVE_M;
    const float d = dist2d(s->x, s->z, o->x, o->z);
    if (d <= r || (cmd_report_fresh(st, ms) && st->report.state == GNR_ARRIVED))
        return 1.0f;
    const float d0 = st->active.task_id ? dist2d(st->start_x, st->start_z, o->x, o->z) : 0.0f;
    return d0 > d ? 1.0f - d / d0 : 0.0f;
}

/* A tank immobilised by damage (cmd_seat_disabled, not a stalled one): stay
 * and fight where it stands. Any pending order is withdrawn, a disabled hull
 * cannot carry it out. */
static void cmd_stay(void *host, int army, lane_seat_t *s, int seat, int at, uint64_t ms)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if ((st->active.type == OB_STAY && !st->invalid_reason) || (st->pending.task_id && st->pending.type == OB_STAY))
        return;
    if (st->pending.task_id) {
        cmd_task_event(host, "pending_cancelled", &st->pending, ms, GTR_IMMOBILIZED);
        cmd_clear_pending(s, st);
    }
    const cmd_obj_t o = {.type = OB_STAY, .need = 1, .band = strat_band(strat_lat(s->x, s->z)), .lane = s->lane,
                         .cap = -1, .x = s->x, .z = s->z};
    const int from = s->cmd_type, from_id = s->cmd_id;
    if (cmd_publish(host, s, seat, cmd_make_order(army, seat, &o, ms, GTR_IMMOBILIZED), ms))
        cmd_log(host, army, 1, "cmd: army %d seat %d -> stay at (%.0f,%.0f): immobilised, was %s %d%s", army, seat,
                s->x, s->z, cmd_name[from], from_id, at >= 0 ? ", still counts for the point it stands in" : "");
}

/* Stage 2 log: per point, time owned and time owned with one of ours in the
 * circle, and each change of owner with who was there. */
static void cmd_watch_points(void *host, int army, uint64_t ms)
{
    const float dt = cmd.pt_ms[army] && ms > cmd.pt_ms[army] ? fminf(5.0f, (float)(ms - cmd.pt_ms[army]) / 1000.0f) : 0.0f;
    cmd.pt_ms[army] = ms;
    for (int k = 0; k < nlanes; ++k) {
        const lane_t *l = &lanes[k];
        const int cap = l->army == army && l->kind == 0 ? cmd_cap_index(l) : -1;
        if (cap < 0)
            continue;
        const int owner = zone_owner(host, l->zx, l->zz);
        int inside = 0, around = 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            if (!cmd_live(o, i, army, ms))
                continue;
            const float d = dist2d(o->x, o->z, l->zx, l->zz);
            inside += d < l->zr;
            around += d < CMD_ENEMY_M;
        }
        if (owner == army) {
            cmd.pt_owned_s[army][cap] += dt;
            cmd.pt_manned_s[army][cap] += inside ? dt : 0.0f;
        }
        if (cmd.pt_seen[army][cap] && owner != cmd.pt_owner[army][cap]) {
            ++cmd.pt_changes[army][cap];
            event(host, "cmd: army %d point %c owner %d -> %d | ours in the circle %d, within %.0f m %d | known enemies "
                  "within %.0f m %d", army, 'A' + cap, cmd.pt_owner[army][cap], owner, inside, CMD_ENEMY_M, around,
                  CMD_ENEMY_M, cmd_known_near(&sview[army], l->zx, l->zz, CMD_ENEMY_M, CMD_ENEMY_AGE_S));
        }
        cmd.pt_seen[army][cap] = 1;
        cmd.pt_owner[army][cap] = owner;
    }
}

static int cmd_capture_active(const lane_seat_t *s, int seat, const cmd_obj_t *o, uint64_t ms)
{
    if (dist2d(s->x, s->z, o->x, o->z) < lanes[o->lane].zr)
        return 1;
    const cmd_task_state_t *st = &cmd_tasks[seat];
    return st->active.type == OB_CAPTURE && st->active.objective == o->id && !st->invalid_reason
           && (st->report.state != GNR_CANCELLED || !cmd_report_fresh(st, ms));
}

static int cmd_guarding(void *host, const lane_seat_t *s, const strat_view_t *v, uint64_t ms)
{
    int contested = 0, point = fight_in_point(host, s, &contested);
    if (point >= 0 && (contested || cmd_known_near(v, lanes[point].zx, lanes[point].zz, CMD_ENEMY_M, CMD_ENEMY_AGE_S)))
        return 1;
    return (s->task && s->task_point) || seat_useful_fire(s, ms);
}

static int cmd_order_parks(int type)
{
    return type == OB_SUPPORT || type == OB_OVERWATCH;
}

static int cmd_post_objective(const cmd_obj_t *o)
{
    return o->type == OB_OVERWATCH && o->id >= CMD_POST_ID_BASE;
}

static int cmd_post_budget(int live, int capture, float pressure)
{
    int slots = live / PASSIVE_DIV;
    if (pressure >= 0.85f)
        slots = 0;
    else if (pressure >= 0.65f)
        slots /= 2;
    return slots < live - capture ? slots : live - capture;
}

static int cmd_retain_posts(int army, uint64_t ms, const cmd_obj_t *ob, int nob, int limit,
                            int *keep, int *claimed)
{
    float rank[MAX_SEATS] = {0};
    int objective[MAX_SEATS] = {0};
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *s = &lseat[i];
        if (!cmd_live(s, i, army, ms) || cmd_seat_immobile(s))
            continue;
        const cmd_task_state_t *st = &cmd_tasks[i];
        int current = cmd_is_post(s) ? cmd_objective_index(ob, nob, OB_OVERWATCH, s->cmd_id, -1) : -1;
        int pending = cmd_pending_reserves(st) && s->cmd_ntype == OB_OVERWATCH && s->cmd_nid >= CMD_POST_ID_BASE
                      ? cmd_objective_index(ob, nob, OB_OVERWATCH, s->cmd_nid, -1) : -1;
        if (current >= 0)
            claimed[current] = 1;       /* even a recalled crew holds the claim until departure */
        if (pending >= 0)
            claimed[pending] = 1;
        int k = pending >= 0 ? pending : cmd_accepted_reserves(st, ms) ? current : -1;
        if (k >= 0 && PK_PLAN(cmd_posts[army][ob[k].id - CMD_POST_ID_BASE].kind))
            k = -1;                     /* s32: watch slots are the plan's, outside the post budget */
        if (k >= 0) {
            objective[i] = k;
            rank[i] = ob[k].value + (current == k ? 6.0f : 0.0f) + (seat_useful_fire(s, ms) ? 8.0f : 0.0f);
        }
    }
    int retained[CMD_MAX_OBJ] = {0}, n = 0;
    while (n < limit) {
        int best = 0;
        for (int i = 1; i < MAX_SEATS; ++i)
            if (rank[i] > rank[best] && !retained[objective[i]])
                best = i;
        if (!best)
            break;
        retained[objective[best]] = 1;
        keep[best] = 1;
        rank[best] = 0.0f;
        ++n;
    }
    return n;
}

static int cmd_stationary_budget(void *host, int army, uint64_t ms, int live, int post_slots)
{
    const strat_view_t *v = &sview[army];
    float rank[MAX_SEATS] = {0};
    int budget = live / PASSIVE_DIV;
    if (v->point_pressure >= 0.65f)
        budget /= 2;
    budget -= post_slots;
    int mandatory = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms))
            continue;
        o->cmd_release = 0;
        if (cmd_is_post(o) || (o->cmd_apply_ms && o->cmd_ntype == OB_OVERWATCH && o->cmd_nid >= CMD_POST_ID_BASE))
            continue;
        /* Stage 2: a garrison is not passive duty, an immobilised tank is counted apart. */
        if (cmd_seat_immobile(o) || o->cmd_type == OB_HOLD || (o->cmd_apply_ms && o->cmd_ntype == OB_HOLD))
            continue;
        /* Stage 3: a stop in cover before a stretch under fire is part of the
         * approach and bounded in time (ground_approach.inc), not parking. */
        if (o->ap_state == AP_TO_COVER || o->ap_state == AP_OBSERVE)
            continue;
        if (cmd_guarding(host, o, v, ms))
            continue;
        if (cmd_combat_busy(o, ms)) {
            mandatory += !seat_advancing(o, ms);
            continue;
        }
        int point_route = o->cmd_type < OB_SUPPORT && !o->task
                          && zone_owner(host, lanes[o->lane].zx, lanes[o->lane].zz) != army;
        int staging = o->holding && o->route[o->step].pt != SLOT_PT;
        int parked = o->cmd_parked || o->holding || o->parked || !o->wants_move;
        int pending_park = o->cmd_apply_ms && cmd_order_parks(o->cmd_ntype);
        if (!parked && !cmd_order_parks(o->cmd_type) && !pending_park && (!o->task || o->task_point))
            continue;
        if ((o->cmd_type == OB_HOLD || cmd_order_parks(o->cmd_type)) && cmd_order_blocks_reassignment(&cmd_tasks[i], ms)) {
            ++mandatory;
            continue;
        }
        o->cmd_release = 1;
        if (point_route && staging)
            continue;
        rank[i] = (o->cmd_type == OB_HOLD ? 3.0f : o->cmd_type == OB_SUPPORT ? 2.0f : 1.0f)
                  + (parked ? 0.25f : 0.0f);
    }
    for (int n = mandatory; n < budget; ++n) {
        int best = 0;
        for (int i = 1; i < MAX_SEATS; ++i)
            if (rank[i] > rank[best])
                best = i;
        if (!best)
            break;
        rank[best] = 0.0f;
        lseat[best].cmd_release = 0;
    }
    return budget > mandatory ? budget - mandatory : 0;
}
