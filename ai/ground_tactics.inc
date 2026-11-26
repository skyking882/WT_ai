/* s28 2026-11-17 tactics (see docs/ground_ai_algorithm_20261120.md, section 7), included by ground_brain.c right above lanes_tick.
 * Four parts, each behind its own switch (config sense= / alert= / takeover= / waitpeek=, env WT_GROUND_SENSE /
 * _ALERT / _TAKEOVER / _WAITPEEK, default on; 0 = s27 for that part):
 *
 *  1. sense: presence within sense_m (25 m) is a feeling, seeing is the game's sight table (the vision hook no
 *     longer answers "visible" inside vis_near). Per observing tank and enemy tank a presence hint: range band
 *     (<= 5 m: 16 sectors, <= 15 m: 8, <= sense_m: 4) and a sector of the bearing relative to the hull, both
 *     debounced (band: 1 m past the boundary; sector: a quarter sector past its edge; hull heading reference with a
 *     3 deg dead band, so a hull standing still keeps its direction). Outside sense_m the hint stops updating at
 *     once and ends 2 s later. It feeds only the turret's idle gaze (tac_gaze, through gaze_source) and the exit a
 *     reaction picks (part 4); it never writes a contact, never picks a gunner target and never moves the hull.
 *  2. alert: a team record per army (ALERT_SLOTS): area (last confirmed position while seen, frozen when sight is
 *     lost), time of the last trusted evidence, responder (seat and life), the enemy slot and life it started from.
 *     It starts only on own-eyes sightings in our rear (s39: team_rear_class, "behind our line" and not at a zone we
 *     do not own) seen by two tanks at once, or seen
 *     again at least ALERT_SECOND_MS after the first sighting; only new own-eyes sightings in the area refresh the
 *     evidence; it lasts ALERT_HOLD_MS after the last of them, and survives the death of the enemy it started from while
 *     a joined enemy is left. One responder (s39: always one) within the team task limit (team_plan
 *     hands its interception over to this record: no withdraw / re-send while the record lives).
 *  3. takeover: what this life can do (tac_ready: unknown / loaded / reloading / empty / no count / no ground gun)
 *     and the gate for leaving cover to fight (tac_advance_ok / tac_advance_abort: sees it now, can engage, the
 *     game's gunner on the same target, the task allows it, and it cannot already fire from where it stands);
 *     target choice keeps its inertia and is re-evaluated once on a new own sighting, a hit, or the target lost.
 *  4. waitpeek: one reaction record per tank (continue / wait / peek; "come out" is the engagement itself under
 *     part 3's gate): reason, exit, wait / peek positions, end condition, the order it returns to. */

/* s29: threat facing (threatface=1, section 5 below): 1 while the decision moves to cover or turns the hull */
static int tac_face_busy(const lane_seat_t *s, int seat);

/* ---- 1. presence hints ------------------------------------------------------------------------------------- */
#define SENSE_EXPIRE_MS 2000
#define SENSE_HREF_DEG 3.0f
typedef struct {
    uint64_t life;              /* the observer's life (born_ms) */
    uint64_t target_life;       /* the enemy slot's life */
    int on;                     /* 0 none, 1 inside sense_m, 2 outside it (frozen until last_in + SENSE_EXPIRE_MS) */
    int band, sector;
    uint64_t start_ms, last_in_ms;
} tac_hint_t;
static tac_hint_t tac_hint[MAX_SEATS][64];
static float tac_href[MAX_SEATS];
static uint64_t tac_href_life[MAX_SEATS];
static const char *const sense_band_name[3] = {"<=5 m", "5-15 m", "15-25 m"};

static int sense_sectors(int band)
{
    return band == 0 ? 16 : band == 1 ? 8 : 4;
}

static int sense_band(int cur, float d)
{
    const int plain = d <= 5.0f ? 0 : d <= 15.0f ? 1 : 2;
    if (cur < 0 || plain == cur)
        return plain;
    if (plain > cur)
        return d > (cur == 0 ? 5.0f : 15.0f) + 1.0f ? plain : cur;
    return d < (cur == 2 ? 15.0f : 5.0f) - 1.0f ? plain : cur;
}

static int sense_sector(int band, int cur, float rel)
{
    const int n = sense_sectors(band);
    const float w = 360.0f / (float)n;
    if (cur >= 0 && cur < n && fabsf(wrap180(rel - (float)cur * w)) <= 0.75f * w)
        return cur;
    return (((int)floorf(rel / w + 0.5f)) % n + n) % n;
}

static float sense_world(int seat, int band, int sector)
{
    return wrap180(tac_href[seat] + (float)sector * 360.0f / (float)sense_sectors(band));
}

static void tac_sense_tick(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (s->fx * s->fx + s->fz * s->fz < 0.25f)
        return;                         /* heading unknown */
    const float hull = atan2f(s->fz, s->fx) * 57.29578f;
    if (tac_href_life[seat] != s->born_ms || fabsf(wrap180(hull - tac_href[seat])) > SENSE_HREF_DEG) {
        tac_href_life[seat] = s->born_ms;
        tac_href[seat] = hull;
    }
    const float range = cfg.sense_m;
    for (int k = 0; k < 64; ++k) {
        tac_hint_t *h = &tac_hint[seat][k];
        if (h->life != s->born_ms) {
            memset(h, 0, sizeof *h);
            h->life = s->born_ms;
        }
        const int enemy = slot_alive[k] && slot_unit[k] && slot_army[k] && slot_army[k] != s->army;
        if (h->on && (!enemy || h->target_life != slot_life[k])) {
            event(host, "sense: seat %d life %llu slot %d hint ends (enemy gone) band %s sector %d/%d bearing %.0f", seat,
                  (unsigned long long)s->born_ms, k, sense_band_name[h->band], h->sector, sense_sectors(h->band),
                  sense_world(seat, h->band, h->sector));
            h->on = 0;
        }
        float d = 1e9f, brg = 0.0f;
        if (enemy) {
            const float *p = gh_unit_pos(slot_unit[k]);
            if (isfinite(p[0]) && isfinite(p[2])) {
                d = dist2d(s->x, s->z, p[0], p[2]);
                brg = atan2f(p[2] - s->z, p[0] - s->x) * 57.29578f;
            }
        }
        if (enemy && d <= range) {
            const float rel = wrap180(brg - tac_href[seat]);
            if (h->on) {
                const int band = sense_band(h->band, d);
                const int sector = sense_sector(band, band == h->band ? h->sector : -1, rel);
                if (band != h->band || sector != h->sector)
                    event(host, "sense: seat %d life %llu slot %d hint changes band %s sector %d/%d -> band %s sector %d/%d "
                          "bearing %.0f", seat, (unsigned long long)s->born_ms, k, sense_band_name[h->band], h->sector,
                          sense_sectors(h->band), sense_band_name[band], sector, sense_sectors(band),
                          sense_world(seat, band, sector));
                h->band = band;
                h->sector = sector;
            } else {
                h->band = sense_band(-1, d);
                h->sector = sense_sector(h->band, -1, rel);
                h->start_ms = ms;
                h->target_life = slot_life[k];
                event(host, "sense: seat %d life %llu slot %d hint appears band %s sector %d/%d bearing %.0f", seat,
                      (unsigned long long)s->born_ms, k, sense_band_name[h->band], h->sector, sense_sectors(h->band),
                      sense_world(seat, h->band, h->sector));
            }
            h->on = 1;
            h->last_in_ms = ms;
        } else if (h->on) {
            h->on = 2;                  /* outside: no more updates */
            if (ms - h->last_in_ms >= SENSE_EXPIRE_MS) {
                event(host, "sense: seat %d life %llu slot %d hint ends (outside %.0f m for %.1f s) band %s sector %d/%d "
                      "bearing %.0f", seat, (unsigned long long)s->born_ms, k, range,
                      (double)(ms - h->last_in_ms) / 1000.0, sense_band_name[h->band], h->sector, sense_sectors(h->band),
                      sense_world(seat, h->band, h->sector));
                h->on = 0;
            }
        }
    }
}

/* The hint that matters most for this tank: nearest band, then the newest. -1 none. */
static int tac_best_hint(const lane_seat_t *s, int seat)
{
    int best = -1;
    for (int k = 0; k < 64; ++k) {
        const tac_hint_t *h = &tac_hint[seat][k];
        if (h->life != s->born_ms || !h->on)
            continue;
        if (best < 0 || h->band < tac_hint[seat][best].band
            || (h->band == tac_hint[seat][best].band && h->last_in_ms > tac_hint[seat][best].last_in_ms))
            best = k;
    }
    return best;
}

/* ---- 3. what this life can do ------------------------------------------------------------------------------ */
enum { RDY_UNKNOWN, RDY_LOADED, RDY_RELOADING, RDY_EMPTY, RDY_NOCOUNT, RDY_NOGROUND };
static const char *const rdy_name[] = {"unknown", "loaded", "reloading", "empty", "no_count", "no_ground_gun"};
static int tac_rdy_cat[MAX_SEATS];
static uint64_t tac_rdy_life[MAX_SEATS];

/* From this life's own gun reading (shots_poll resets it per life) and this life's vehicle class (active_cls tells
 * the reserve vehicle from the seat's first one). unknown: the gun has not been read this life (or record 0 is not
 * a gun); empty: the counter says 0 rounds (seat_ready_in's 99); no_count: a gun without a readable count. */
static int tac_ready(const lane_seat_t *s, int seat, uint64_t ms)
{
    const shot_seat_t *st = &shot_seat[seat];
    if (!s->unit || st->unit != s->unit || st->life != slot_life[seat] || st->base < 0)
        return RDY_UNKNOWN;
    const vcls_t *vc = active_cls(seat, s->unit);
    if (vc->ok && (vc->flags & VC_SPAA))
        return RDY_NOGROUND;
    if (st->left == 0)
        return RDY_EMPTY;
    if (st->left < 0)
        return RDY_NOCOUNT;
    return st->ready_ms > ms ? RDY_RELOADING : RDY_LOADED;
}

static int tac_able(int rdy)
{
    return rdy == RDY_LOADED || rdy == RDY_RELOADING;
}

static void tac_ready_log(void *host, const lane_seat_t *s, int seat, uint64_t ms)
{
    const int rdy = tac_ready(s, seat, ms);
    const int cat = tac_able(rdy) ? RDY_LOADED : rdy;
    if (tac_rdy_life[seat] == s->born_ms && tac_rdy_cat[seat] == cat)
        return;
    const int first = tac_rdy_life[seat] != s->born_ms;
    tac_rdy_life[seat] = s->born_ms;
    tac_rdy_cat[seat] = cat;
    if (first && cat == RDY_UNKNOWN)
        return;                         /* every life starts unknown */
    const char *name = gh_unit_name(s->unit) ? (const char *)gh_unit_name(s->unit) : "?";
    event(host, "ready: seat %d life %llu %s: %s (rounds %d, ready in %.1f s)", seat, (unsigned long long)s->born_ms, name,
          tac_able(rdy) ? "able to engage" : rdy_name[rdy], shot_seat[seat].left, seat_ready_in(seat, ms));
}

static int tac_sees(const lane_seat_t *s, int seat, int k, uint64_t ms)
{
    float x, z, age;
    return k >= 0 && k < 64 && contact_get(s->army, seat, k, ms, &x, &z, &age) == 2;
}

/* Leaving cover to fight is allowed only when all hold (part 3). why: the first that does not. */
static int tac_advance_ok(lane_seat_t *s, int seat, uint64_t ms, const char **why)
{
    const char *w = NULL;
    const int rdy = tac_ready(s, seat, ms);
    if (s->mode != M_ENGAGE || s->eng < 0)
        w = "no target";
    else if (!s->eng_own || ms - s->eng_seen_ms >= 1000 || !tac_sees(s, seat, s->eng, ms))
        w = "does not see it now";
    else if (!tac_able(rdy))
        w = rdy == RDY_EMPTY ? "this life cannot engage (empty)" : rdy == RDY_NOGROUND ? "this life cannot engage (no ground gun)"
            : rdy == RDY_NOCOUNT ? "this life cannot engage (no round count)" : "this life's readiness unknown";
    else if (s->gt_slot != s->eng)
        w = s->gt_slot < 0 ? "the gunner has not taken the target" : "the gunner holds another target";
    else if (!(cmd_motion_policy(s, seat, ms).allowed & GFM_ADVANCE))
        w = "the task does not allow it";
    else if (cfg.threatface && tac_face_busy(s, seat))
        w = "the hull is going to cover or turning to the threat";
    else {
        const shot_opportunity_t shot = shot_opportunity(seat, s, s->eng, ms);
        const int exhausted = s->fight_exhausted_ms && ms - s->fight_exhausted_ms < FIGHT_ROUTE_MS;
        if (shot.aim_ms && !shot.reposition && !exhausted
            && (shot.opportunity == SHOT_KILL || shot.opportunity == SHOT_FIRE || shot.opportunity == SHOT_MOBILITY))
            w = "it can fire from where it stands";
    }
    if (why)
        *why = w;
    return w == NULL;
}

/* While advancing: the reason the advance must end now, NULL to go on. */
static const char *tac_advance_abort(lane_seat_t *s, int seat, uint64_t ms)
{
    if (s->mode != M_ENGAGE || s->eng < 0)
        return "target gone";
    if (!s->eng_own || ms - s->eng_seen_ms >= 1000)
        return "sight lost";
    if (s->gt_slot != s->eng)
        return s->gt_slot >= 0 ? "the gunner switched to another target" : "the gunner dropped the target";
    const int rdy = tac_ready(s, seat, ms);
    if (!tac_able(rdy))
        return rdy == RDY_EMPTY ? "cannot fire now (empty)" : rdy == RDY_NOGROUND ? "cannot fire now (no ground gun)"
               : rdy == RDY_NOCOUNT ? "cannot fire now (no round count)" : "cannot fire now (readiness unknown)";
    return NULL;
}

/* Target choice with inertia (react_update, takeover=1). Re-evaluated once on: a new own sighting, a hit, the
 * present target lost. A target that can still be engaged is kept; a newly seen enemy that directly threatens this
 * tank (it hit us, or it is within FIGHT_IMMEDIATE_M) takes over from one that does not. The bearing of a hit alone
 * changes nothing here (react_update turns the hull / gaze to it). Slot identity plays no part (human or AI). */
typedef struct {
    uint64_t life, hit_ms, match_ms;
    uint64_t seen[64];
    int last, match_slot;
} tac_tgt_t;
static tac_tgt_t tac_tgt[MAX_SEATS];

static int tac_threatens(int seat, int k, float d, uint64_t ms)
{
    const hitdir_t *h = &hit_dir[seat & 63];
    return (h->ms && ms - h->ms < 10000 && h->from == k) || d < FIGHT_IMMEDIATE_M;
}

static int tac_pick_target(void *host, lane_seat_t *s, int seat, uint64_t ms, float *tx, float *tz, int *by_gunner, int *own)
{
    tac_tgt_t *T = &tac_tgt[seat];
    if (T->life != s->born_ms) {
        memset(T, 0, sizeof *T);
        T->life = s->born_ms;
        T->last = T->match_slot = -1;
        T->hit_ms = hit_dir[seat & 63].ms;
    }
    float px[64], pz[64], pd[64];
    int lvl[64], fresh = -1;
    for (int k = 0; k < 64; ++k) {
        float age;
        lvl[k] = contact_get(s->army, seat, k, ms, &px[k], &pz[k], &age) == 2;
        pd[k] = lvl[k] ? dist2d(s->x, s->z, px[k], pz[k]) : 1e9f;
        if (lvl[k] && pd[k] > ENG_MAX_M)
            lvl[k] = 0;
        if (lvl[k]) {
            if ((!T->seen[k] || ms - T->seen[k] > 1500) && (fresh < 0 || pd[k] < pd[fresh]))
                fresh = k;
            T->seen[k] = ms;
        }
    }
    const int gt = s->gt_slot >= 0 && slot_alive[s->gt_slot] && slot_unit[s->gt_slot] ? s->gt_slot : -1;
    const int hit = hit_dir[seat & 63].ms > T->hit_ms;
    if (hit)
        T->hit_ms = hit_dir[seat & 63].ms;
    const int cur = s->mode == M_ENGAGE && s->eng >= 0 && slot_alive[s->eng] && s->fight_life == slot_life[s->eng] ? s->eng : -1;
    const int cur_ok = cur >= 0 && (lvl[cur] || gt == cur || ms - s->eng_seen_ms < STICKY_LOST_MS);
    int pick = cur_ok ? cur : -1;
    const char *why = NULL;
    if (!cur_ok) {
        /* nothing (left) to keep: the game's gunner's target, else the nearest enemy in own sight */
        if (gt >= 0) {
            pick = gt;
        } else {
            for (int k = 0; k < 64; ++k)
                if (lvl[k] && (pick < 0 || pd[k] < pd[pick]))
                    pick = k;
        }
        why = cur >= 0 || (s->mode == M_ENGAGE && s->eng >= 0) ? "target lost" : fresh >= 0 ? "new own sighting"
              : hit ? "hit" : "gunner's target";
    } else if (fresh >= 0 || hit) {
        why = fresh >= 0 ? "new own sighting" : "hit";
        const float dcur = lvl[cur] ? pd[cur] : dist2d(s->x, s->z, s->ex, s->ez);
        if (!tac_threatens(seat, cur, dcur, ms)) {
            int best = -1;
            for (int k = 0; k < 64; ++k)
                if (k != cur && lvl[k] && tac_threatens(seat, k, pd[k], ms) && (best < 0 || pd[k] < pd[best]))
                    best = k;
            if (best >= 0)
                pick = best;
        }
    }
    if (why && (pick != T->last || (cur_ok && (fresh >= 0 || hit))))
        event(host, "takeover: seat %d re-evaluates (%s): %s slot %d (eng %d, gunner %d)", seat, why,
              pick < 0 ? "no target" : pick == cur ? "keeps" : "takes", pick, cur, s->gt_slot);
    T->last = pick;
    if (pick >= 0 && pick == s->gt_slot && T->match_slot != pick) {
        T->match_slot = pick;
        event(host, "takeover: seat %d slot %d held by the gunner too (eng = gunner)", seat, pick);
    } else if (pick != s->gt_slot) {
        T->match_slot = -1;
    }
    if (pick < 0)
        return -1;
    if (pick == gt) {
        const float *p = gh_unit_pos(slot_unit[gt]);
        *tx = p[0];
        *tz = p[2];
    } else if (lvl[pick]) {
        *tx = px[pick];
        *tz = pz[pick];
    } else {
        *tx = s->ex;
        *tz = s->ez;
    }
    *by_gunner = pick == s->gt_slot;
    *own = lvl[pick] || pick == gt;
    return pick;
}

/* ---- exits and positions ----------------------------------------------------------------------------------- */
#define TAC_EXIT_M 80.0f            /* exits within this of where the target vanished / was reported */
#define TAC_HINT_EXIT_M 60.0f       /* exits within this of the tank, inside the hint's sector */
#define TAC_TURN_M 4.3f             /* clearance a hull needs to turn round on the spot */
#define TAC_PEEK_CLEAR_M 3.0f
#define TAC_CHOKE_M 6.0f
#define TAC_EXIT_MIN_M 15.0f        /* a wait / peek point never this close to the exit */
#define TAC_VANISH_MIN_M 20.0f      /* a peek point never this close to where the enemy vanished */
#define TAC_WAIT_ALONG_M 30.0f
#define TAC_PEEK_M 40.0f
#define TAC_FAIL_M 5.0f
#define TAC_MAX_FAILS 8

enum { TB_NONE, TB_CONTINUE, TB_WAIT, TB_PEEK };
enum { TC_NONE, TC_LOST, TC_HINT, TC_REPORT, TC_ALERT };
typedef struct { int ok, ref; float x, z; const char *kind; } tac_exit_t;
typedef struct {
    uint64_t life;
    int beh, clue, slot, guard, band, sector;
    uint64_t clue_ms, clue_key, peeked_key;
    float ox, oz, vx, vz;
    tac_exit_t exit;
    int wait_ok, peek_ok;
    float wait_x, wait_z, peek_x, peek_z;
    uint64_t since, beh_since, arrived_ms, enc_last, enc_spent, tick_ms, move_prog_ms, blocked_ms;
    float move_best;
    uint64_t task_id;
    unsigned revision;
    int order, nfail;
    float fail_x[TAC_MAX_FAILS], fail_z[TAC_MAX_FAILS];
    const char *hull;
    uint64_t own_ms[64], lost_done[64], report_done[64], hint_done, alert_done;
    float own_x[64], own_z[64], own_vx[64], own_vz[64];
} tac_rx_t;
static tac_rx_t tac_rx[MAX_SEATS];

static const gr_lib_t *tac_lib(void)
{
    return routes_lib_state > 0 ? &routes_lib : NULL;
}

static void tac_exit_try(tac_exit_t *e, float *best, float score, float x, float z, const char *kind, int ref)
{
    if (score < *best && isfinite(x) && isfinite(z)) {
        *best = score;
        *e = (tac_exit_t){1, ref, x, z, kind};
    }
}

/* One existing map node: the commander's choke posts, post gate points, lane points, library targets. sector = 0:
 * nodes within rmax of (ox, oz), the one the target went towards (last velocity), else the nearest; sector = 1:
 * nodes 8 m .. rmax from the tank (sx, sz) within half deg of the world bearing centre, the nearest. */
static float tac_exit_score(float x, float z, float ox, float oz, float vx, float vz, float rmax, int sector, float sx,
                            float sz, float centre, float half)
{
    if (sector) {
        const float d = dist2d(sx, sz, x, z);
        if (d < 8.0f || d > rmax || fabsf(wrap180(atan2f(z - sz, x - sx) * 57.29578f - centre)) > half)
            return 1e9f;
        return d;
    }
    const float d = dist2d(ox, oz, x, z), sp = hypotf(vx, vz);
    if (d > rmax)
        return 1e9f;
    if (sp < 1.5f || d < 3.0f)
        return d;
    const float c = ((x - ox) * vx + (z - oz) * vz) / (d * sp);
    return c < 0.2f ? 1e9f : d * (1.6f - c);
}

static tac_exit_t tac_exit_pick(int army, float ox, float oz, float vx, float vz, float rmax, int sector, float sx, float sz,
                                float centre, float half)
{
    tac_exit_t e = {0};
    float best = 1e8f;
    for (int i = 0; i < nrchokes; ++i)
        tac_exit_try(&e, &best, tac_exit_score(rchokes[i].x, rchokes[i].z, ox, oz, vx, vz, rmax, sector, sx, sz, centre, half),
                     rchokes[i].x, rchokes[i].z, "choke", rchokes[i].id);
    for (int i = 0; army >= 1 && army <= 2 && i < cmd_nposts[army] && i < CMD_POST_MAX; ++i) {
        const cmd_post_t *p = &cmd_posts[army][i];
        tac_exit_try(&e, &best, tac_exit_score(p->gx[1], p->gz[1], ox, oz, vx, vz, rmax, sector, sx, sz, centre, half),
                     p->gx[1], p->gz[1], "post", CMD_POST_ID_BASE + i);
    }
    for (int i = 0; i < nlanes; ++i)
        for (int j = 0; j < lanes[i].n; ++j)
            tac_exit_try(&e, &best, tac_exit_score(lanes[i].x[j], lanes[i].z[j], ox, oz, vx, vz, rmax, sector, sx, sz, centre, half),
                         lanes[i].x[j], lanes[i].z[j], "lane", i * 10 + j);
    const gr_lib_t *lib = tac_lib();
    for (uint32_t i = 0; lib && i < lib->n_targets; ++i)
        tac_exit_try(&e, &best, tac_exit_score(lib->targets[i].x, lib->targets[i].z, ox, oz, vx, vz, rmax, sector, sx, sz,
                                               centre, half),
                     lib->targets[i].x, lib->targets[i].z, lib->targets[i].kind ? "lib_post" : "lib_zone", (int)i);
    return e;
}

static int tac_failed(const tac_rx_t *r, float x, float z)
{
    for (int i = 0; r && i < r->nfail; ++i)
        if (dist2d(x, z, r->fail_x[i], r->fail_z[i]) < TAC_FAIL_M)
            return 1;
    return 0;
}

static void tac_fail(tac_rx_t *r, float x, float z)
{
    const int i = r->nfail < TAC_MAX_FAILS ? r->nfail++ : TAC_MAX_FAILS - 1;
    r->fail_x[i] = x;
    r->fail_z[i] = z;
}

/* A place a hull can stand and (need = TAC_TURN_M) turn round on: library clearance when the library is loaded,
 * else the cover layer's own passable test; never within TAC_CHOKE_M of a commander choke, never a failed one. */
static int tac_spot_ok(const tac_rx_t *r, float x, float z, float need)
{
    const gr_lib_t *lib = tac_lib();
    if (lib) {
        if (gr_clearance(lib, x, z) < need)
            return 0;
    } else if (cover_ok()) {
        const int i = cover_cell_of(x, z, NULL, NULL);
        const u8 *g = i >= 0 ? strat.g10 + (size_t)i * 8 : NULL;
        if (!g || (g[4] & 3) != 3 || g[5] > COV_HARD_MAX)
            return 0;
    }
    for (int i = 0; i < nrchokes; ++i)
        if (dist2d(x, z, rchokes[i].x, rchokes[i].z) < TAC_CHOKE_M)
            return 0;
    return !tac_failed(r, x, z);
}

/* A straight drivable line (sufficient, not necessary): hull clearance every metre with the library, else the
 * cover layer's straight-line test and the static connector. The driving side still finds its own way. */
static int tac_reach(float x0, float z0, float x1, float z1)
{
    const float d = dist2d(x0, z0, x1, z1);
    if (d < 2.0f)
        return 1;
    const gr_lib_t *lib = tac_lib();
    if (lib) {
        for (float t = 2.0f; t < d; t += 1.0f)
            if (gr_clearance(lib, x0 + (x1 - x0) * t / d, z0 + (z1 - z0) * t / d) < GR_CLEAR_MIN)
                return 0;
        return 1;
    }
    if (cover_ok() && !cover_reach(x0, z0, x1, z1))
        return 0;
    return control_static_connector(NULL, x0, z0, x1, z1) != NAV_BLOCKED;
}

/* Something between (x, z) and the exit (static layers only): turret-blocked or hard hull cover that way, or the
 * 32 m spotting table has no line between the two cells. */
static int tac_hidden(float x, float z, float ex, float ez)
{
    if (!cover_ok())
        return 0;
    const int c10 = cover_cell_of(x, z, NULL, NULL);
    if (c10 >= 0) {
        const u8 h = strat.g10hull[(size_t)c10 * 16 + (size_t)position_direction(x, z, ex, ez)];
        if ((h & 0x80) || position_hard_cover(h) >= 0.5f)
            return 1;
    }
    const int a = strat_cellof(x, z), b = strat_cellof(ex, ez);
    return a >= 0 && b >= 0 && a != b && position_los_sampled(a, b) && !strat_sees(strat.viss, b, a);
}

/* The exit can be seen from (x, z) by the static line-of-sight data the cover code uses. */
static int tac_sees_exit(float x, float z, float ex, float ez)
{
    if (strat.state != 1 || !strat.viss)
        return 0;
    const int a = strat_cellof(x, z), b = strat_cellof(ex, ez);
    if (a < 0 || b < 0 || (a != b && (!position_los_sampled(a, b) || !strat_sees(strat.viss, a, b))))
        return 0;
    if (cover_ok()) {
        const int c10 = cover_cell_of(x, z, NULL, NULL);
        if (c10 >= 0 && (strat.g10hull[(size_t)c10 * 16 + (size_t)position_direction(x, z, ex, ez)] & 0x80))
            return 0;
    }
    return 1;
}

/* Wait position: no line of sight to the enemy needed; standable and turnable; reachable; behind something as seen
 * from the exit when possible. along = 1: the tank's own position or a point on its way within TAC_WAIT_ALONG_M;
 * along = 0: (ox, oz) or a ring around it (a guard spot). */
static int tac_wait_spot(const lane_seat_t *s, const tac_rx_t *r, float ox, float oz, int along, float ex, float ez,
                         float *wx, float *wz)
{
    float cx[48], cz[48];
    int n = 0;
    cx[n] = ox;
    cz[n++] = oz;
    if (along) {
        if (nav_route_active(s) && s->tr >= 0) {
            const troute_t *t = &troutes[s->tr];
            float px = s->x, pz = s->z, run = 0.0f;
            for (int i = s->wpi; i < t->n && n < 48 && run < TAC_WAIT_ALONG_M; ++i) {
                const float seg = dist2d(px, pz, t->x[i], t->z[i]);
                for (float u = 5.0f; seg > 0.1f && u <= seg && run + u <= TAC_WAIT_ALONG_M && n < 48; u += 5.0f) {
                    cx[n] = px + (t->x[i] - px) * u / seg;
                    cz[n++] = pz + (t->z[i] - pz) * u / seg;
                }
                run += seg;
                px = t->x[i];
                pz = t->z[i];
            }
        } else {
            const float d = dist2d(s->x, s->z, s->move_target_x, s->move_target_z);
            for (float u = 5.0f; d > 0.1f && u <= d && u <= TAC_WAIT_ALONG_M && n < 48; u += 5.0f) {
                cx[n] = s->x + (s->move_target_x - s->x) * u / d;
                cz[n++] = s->z + (s->move_target_z - s->z) * u / d;
            }
        }
    } else {
        for (int ring = 1; ring <= 3; ++ring)
            for (int a = 0; a < 12 && n < 48; ++a) {
                const float ang = (float)a * 0.5235988f;
                cx[n] = ox + 10.0f * (float)ring * cosf(ang);
                cz[n++] = oz + 10.0f * (float)ring * sinf(ang);
            }
    }
    /* along: reachable from the tank; a guard ring: from its centre (the driving side finds the way there) */
    const float rx = along ? s->x : ox, rz = along ? s->z : oz;
    float best = 1e9f;
    int found = 0;
    for (int i = 0; i < n; ++i) {
        if (dist2d(cx[i], cz[i], ex, ez) < TAC_EXIT_MIN_M || !tac_spot_ok(r, cx[i], cz[i], TAC_TURN_M)
            || !tac_reach(rx, rz, cx[i], cz[i]))
            continue;
        const float score = 0.05f * dist2d(s->x, s->z, cx[i], cz[i]) + (tac_hidden(cx[i], cz[i], ex, ez) ? 0.0f : 1.0f);
        if (score < best) {
            best = score;
            *wx = cx[i];
            *wz = cz[i];
            found = 1;
        }
    }
    return found;
}

/* Peek position: near the wait position, within TAC_PEEK_M of the tank, from where the exit can be seen (and the
 * wait position cannot see it), never close to the exit or to where the enemy vanished. */
static int tac_peek_spot(const lane_seat_t *s, const tac_rx_t *r, float *px, float *pz)
{
    if (!r->exit.ok)
        return 0;
    const float bx = r->wait_ok ? r->wait_x : s->x, bz = r->wait_ok ? r->wait_z : s->z;
    if (tac_sees_exit(bx, bz, r->exit.x, r->exit.z))
        return 0;                       /* it already watches the exit from where it waits */
    static const float rings[4] = {6.0f, 12.0f, 20.0f, 30.0f};
    float best = 1e9f;
    int found = 0;
    for (int ring = 0; ring < 4; ++ring)
        for (int a = 0; a < 12; ++a) {
            const float ang = (float)a * 0.5235988f;
            const float x = bx + rings[ring] * cosf(ang), z = bz + rings[ring] * sinf(ang);
            if (dist2d(s->x, s->z, x, z) > TAC_PEEK_M || dist2d(x, z, r->exit.x, r->exit.z) < TAC_EXIT_MIN_M
                || (r->clue != TC_HINT && dist2d(x, z, r->ox, r->oz) < TAC_VANISH_MIN_M)
                || !tac_spot_ok(r, x, z, TAC_PEEK_CLEAR_M) || !tac_reach(s->x, s->z, x, z)
                || !tac_sees_exit(x, z, r->exit.x, r->exit.z))
                continue;
            const float score = dist2d(bx, bz, x, z);
            if (score < best) {
                best = score;
                *px = x;
                *pz = z;
                found = 1;
            }
        }
    return found;
}

/* ---- 2. rear-area alert ------------------------------------------------------------------------------------ */
#define ALERT_HOLD_MS 30000         /* alert lasts this long after the last trusted evidence */
#define ALERT_SLOTS 4
#define ALERT_SECOND_MS 5000        /* a second own-eyes sighting at least this long after the first */
#define ALERT_SEEN_MS 1500          /* own eyes: the observer's line of sight is this fresh */
/* ALERT_R_M (40 m, the area radius) is defined in ground_brain.c next to team_rear_class (s39), which uses it */
#define ALERT_JOIN_M 80.0f          /* a sighting this far past the area joins the record */
#define ALERT_RESP 1                /* s39: one responder per alert (the class "at our point" with its 2 is gone) */
typedef struct {
    int on;                     /* 0 free, 1 candidate (first own-eyes sighting only), 2 alert */
    int enemy, first_by, seen;  /* enemy: the member seen last (the one the record started from until then) */
    uint64_t first_ms, evidence_ms, start_ms;
    float cx, cz, vx, vz;
    const char *why;
    tac_exit_t exit;
    /* members: the enemy it started from and every one that joined; per member its life, last own-eyes evidence
     * and last confirmed position. The record lives while one member is left. */
    uint64_t members, mlife[64], mseen[64];
    float mx[64], mz[64], mvx[64], mvz[64];
    int resp[ALERT_RESP], guard_ok[ALERT_RESP];
    uint64_t resp_life[ALERT_RESP], guard_for[ALERT_RESP], guard_resp_life[ALERT_RESP];
    float guard_x[ALERT_RESP], guard_z[ALERT_RESP];
} tac_alert_t;
static tac_alert_t tac_alert[3][ALERT_SLOTS];

/* team_plan's classification of a contact position: s39 the one rule, team_rear_class (ground_brain.c). */
static const char *tac_rear_class(void *host, int army, float x, float z, float osx, float osz, float ux, float uz, float front,
                                  int np)
{
    return team_rear_class(host, army, x, z, osx, osz, ux, uz, front, np);
}

/* task_point's firing spot, without writing it. */
static void tac_standoff(const lane_seat_t *o, float cx, float cz, float side, float *x, float *z)
{
    const float d = dist2d(o->x, o->z, cx, cz);
    if (d <= INTERCEPT_STANDOFF_M) {
        *x = o->x;
        *z = o->z;
        return;
    }
    const float ux = (o->x - cx) / d, uz = (o->z - cz) / d;
    *x = cx + ux * INTERCEPT_STANDOFF_M - uz * 40.0f * side;
    *z = cz + uz * INTERCEPT_STANDOFF_M + ux * 40.0f * side;
}

/* Responder slot j of the record that seat belongs to, -1 none. */
static int tac_alert_resp_of(const tac_alert_t *a, int seat)
{
    for (int j = 0; j < ALERT_RESP; ++j)
        if (a->resp[j] > 0 && a->resp[j] == seat)
            return j;
    return -1;
}

/* waitpeek=1, the area frozen: the exit the enemy went towards (nodes within TAC_EXIT_M of the last confirmed
 * position, its last velocity), the firing spot team_plan would give responder j, and a wait position around that
 * spot (hidden from the exit when possible). Kept until new evidence or another responder. */
static void tac_alert_guard(void *host, int army, int idx, tac_alert_t *a, int j)
{
    const lane_seat_t *o = &lseat[a->resp[j]];
    a->exit = tac_exit_pick(army, a->cx, a->cz, a->vx, a->vz, TAC_EXIT_M, 0, 0.0f, 0.0f, 0.0f, 0.0f);
    float sx, sz;
    tac_standoff(o, a->cx, a->cz, o->task_side != 0.0f ? o->task_side : 1.0f, &sx, &sz);
    const float ex = a->exit.ok ? a->exit.x : a->cx, ez = a->exit.ok ? a->exit.z : a->cz;
    float wx = sx, wz = sz;
    const tac_rx_t *r = tac_rx[a->resp[j]].life == o->born_ms ? &tac_rx[a->resp[j]] : NULL;
    const int spot = tac_wait_spot(o, r, sx, sz, 0, ex, ez, &wx, &wz);
    if (!spot) {
        wx = sx;
        wz = sz;
    }
    a->guard_ok[j] = 1;
    a->guard_x[j] = wx;
    a->guard_z[j] = wz;
    a->guard_for[j] = a->evidence_ms;
    a->guard_resp_life[j] = o->born_ms;
    event(host, "alert: army %d #%d guard spot for seat %d life %llu: (%.0f,%.0f) %s | exit %s#%d (%.0f,%.0f)%s", army, idx,
          a->resp[j], (unsigned long long)o->born_ms, wx, wz,
          !spot ? "the firing spot (no wait position)" : tac_hidden(wx, wz, ex, ez) ? "wait position, hidden from the exit"
          : "wait position", a->exit.ok ? a->exit.kind : "none", a->exit.ref, ex, ez, a->exit.ok ? "" : " (area centre)");
}

static void tac_alert_end(void *host, int army, int idx, tac_alert_t *a, const char *why, uint64_t ms)
{
    if (a->on == 2)
        event(host, "alert: army %d #%d for slot %d ends after %.1f s (%.1f s after the last evidence): %s", army, idx,
              a->enemy, (double)(ms - a->start_ms) / 1000.0, (double)(ms - a->evidence_ms) / 1000.0, why);
    else
        event(host, "alert: army %d #%d candidate for slot %d dropped: %s", army, idx, a->enemy, why);
    for (int j = 0; j < ALERT_RESP; ++j) {
        if (a->resp[j] <= 0)
            continue;
        lane_seat_t *o = &lseat[a->resp[j]];
        if (o->born_ms == a->resp_life[j] && o->task && o->task_alert == idx + 1) {
            o->task = 0;
            o->task_alert = 0;
            event(host, "plan: army %d seat %d stops intercepting slot %d (alert #%d ended: %s)", army, a->resp[j], a->enemy,
                  idx, why);
        }
    }
    memset(a, 0, sizeof *a);
}

/* A member leaves (dead / respawned / seen out of our rear). The record continues for the rest: evidence time is
 * the newest of theirs, the area the last confirmed position of the one seen last. 1 when members are left. */
static int tac_alert_leave(void *host, int army, int idx, tac_alert_t *a, int t, const char *why)
{
    a->members &= ~(1ull << t);
    int last = -1;
    for (int k = 0; k < 64; ++k)
        if ((a->members >> k) & 1 && (last < 0 || a->mseen[k] > a->mseen[last]))
            last = k;
    if (last < 0)
        return 0;
    a->enemy = last;
    a->evidence_ms = a->mseen[last];
    a->cx = a->mx[last];
    a->cz = a->mz[last];
    a->vx = a->mvx[last];
    a->vz = a->mvz[last];
    for (int j = 0; j < ALERT_RESP; ++j)
        a->guard_ok[j] = 0;
    int left = 0;
    for (int k = 0; k < 64; ++k)
        left += (int)((a->members >> k) & 1);
    event(host, "alert: army %d #%d slot %d left the record (%s); it continues for %d joined enem%s, now slot %d, area "
          "(%.0f,%.0f), last evidence as recorded for it", army, idx, t, why, left, left == 1 ? "y" : "ies", last, a->cx, a->cz);
    return 1;
}

/* Called by team_plan (1 Hz per army, alert=1) instead of its own interception: evidence, the record's life, the
 * responder (one per alert, s39) within the team task limit and the passive budget, and its spot. */
void tac_alert_plan(void *host, int army, uint64_t ms, float osx, float osz, float ux, float uz, float front, int np,
                           int live, int passive, int task_max)
{
    tac_alert_t *al = tac_alert[army];
    int seen_now[ALERT_SLOTS] = {0};
    uint64_t out_mask[ALERT_SLOTS] = {0};
    /* 1. own-eyes evidence of this second */
    for (int t = 0; t < 64; ++t) {
        if (!slot_alive[t] || !slot_army[t] || slot_army[t] == army)
            continue;
        int n = 0, by = -1, by2 = -1;
        gh_lock(GH_LOCK_KNOW);
        const contact_t c = contact[army][t];
        for (int o = 1; o < MAX_SEATS; ++o) {
            const lane_seat_t *q = &lseat[o];
            if (!q->unit || q->army != army || q->scene_epoch != ground_scene_epoch || ms - q->last_ms > 2000
                || !sight_ms[o][t] || ms - sight_ms[o][t] >= ALERT_SEEN_MS)
                continue;
            if (!n)
                by = o;
            else if (n == 1)
                by2 = o;
            ++n;
        }
        gh_unlock(GH_LOCK_KNOW);
        if (!n || !c.last_ms)
            continue;
        const char *why = tac_rear_class(host, army, c.x, c.z, osx, osz, ux, uz, front, np);
        int idx = -1;
        for (int i = 0; i < ALERT_SLOTS && idx < 0; ++i)
            if (al[i].on && ((al[i].members >> t) & 1) && al[i].mlife[t] == slot_life[t])
                idx = i;
        for (int i = 0; i < ALERT_SLOTS && idx < 0 && why; ++i)
            if (al[i].on == 2 && dist2d(c.x, c.z, al[i].cx, al[i].cz) <= ALERT_R_M + ALERT_JOIN_M)
                idx = i;
        if (idx >= 0 && !why) {
            if ((al[idx].members >> t) & 1)
                out_mask[idx] |= 1ull << t;     /* a member was seen out of our rear */
            continue;
        }
        if (idx < 0) {
            if (!why)
                continue;
            for (int i = 0; i < ALERT_SLOTS && idx < 0; ++i)
                if (!al[i].on)
                    idx = i;
            if (idx < 0)
                continue;               /* all records busy */
            tac_alert_t *a = &al[idx];
            memset(a, 0, sizeof *a);
            a->on = 1;
            a->enemy = t;
            a->members = 1ull << t;
            a->mlife[t] = slot_life[t];
            a->first_ms = ms;
            a->first_by = by;
            a->why = why;
            event(host, "alert: army %d #%d candidate: slot %d seen by seat %d in our rear (%s) at (%.0f,%.0f)", army, idx, t,
                  by, why, c.x, c.z);
        }
        tac_alert_t *a = &al[idx];
        if (a->on == 1 && (n >= 2 || ms - a->first_ms >= ALERT_SECOND_MS)) {
            a->on = 2;
            a->start_ms = ms;
            if (n >= 2)
                event(host, "alert: army %d #%d started for slot %d (%s) at (%.0f,%.0f): own eyes of seats %d and %d at once",
                      army, idx, t, a->why, c.x, c.z, by, by2);
            else
                event(host, "alert: army %d #%d started for slot %d (%s) at (%.0f,%.0f): seen by seat %d %.1f s after seat %d's "
                      "first sighting", army, idx, t, a->why, c.x, c.z, by, (double)(ms - a->first_ms) / 1000.0, a->first_by);
        } else if (a->on == 2 && !a->seen && !seen_now[idx]) {
            event(host, "alert: army %d #%d evidence refreshed: slot %d seen by seat %d after %.1f s without sight", army, idx,
                  t, by, (double)(ms - a->evidence_ms) / 1000.0);
        }
        if (a->on == 2 && !((a->members >> t) & 1 && a->mlife[t] == slot_life[t])) {
            event(host, "alert: army %d #%d slot %d seen in the area by seat %d: joins the record", army, idx, t, by);
        }
        a->members |= 1ull << t;
        a->mlife[t] = slot_life[t];
        a->mseen[t] = ms;
        a->mx[t] = c.x;
        a->mz[t] = c.z;
        a->mvx[t] = c.vx;
        a->mvz[t] = c.vz;
        a->enemy = t;
        a->cx = c.x;
        a->cz = c.z;
        a->vx = c.vx;
        a->vz = c.vz;
        a->evidence_ms = ms;
        seen_now[idx] = 1;
    }
    /* 2. the record's life: members leave on death / respawn / seen out of the rear; it ends when none is left */
    for (int i = 0; i < ALERT_SLOTS; ++i) {
        tac_alert_t *a = &al[i];
        if (!a->on)
            continue;
        const char *end = NULL;
        char buf[96];
        for (int t = 0; t < 64 && !end; ++t) {
            if (!((a->members >> t) & 1))
                continue;
            const char *left = !slot_alive[t] || slot_life[t] != a->mlife[t] ? "dead or respawned"
                               : ((out_mask[i] >> t) & 1) ? "seen out of our rear" : NULL;
            if (left && !tac_alert_leave(host, army, i, a, t, left)) {
                snprintf(buf, sizeof buf, "the enemy is out (%s), no joined enemy left", left);
                end = buf;
            }
        }
        if (!end && ms - a->evidence_ms > ALERT_HOLD_MS)
            end = a->on == 2 ? "expired: no trusted evidence for 30 s" : "never confirmed within 30 s";
        if (end) {
            tac_alert_end(host, army, i, a, end, ms);
            continue;
        }
        if (a->on == 2 && a->seen && !seen_now[i]) {
            if (a->resp[0] > 0)
                event(host, "alert: army %d #%d sight of slot %d lost: area frozen at (%.0f,%.0f), responder seat %d keeps it",
                      army, i, a->enemy, a->cx, a->cz, a->resp[0]);
            else
                event(host, "alert: army %d #%d sight of slot %d lost: area frozen at (%.0f,%.0f), no responder yet", army, i,
                      a->enemy, a->cx, a->cz);
        }
        a->seen = seen_now[i];
    }
    /* 3. the responder: one per alert (s39) */
    int tasks = 0;
    for (int j = 1; j < MAX_SEATS; ++j) {
        const lane_seat_t *q = &lseat[j];
        tasks += q->unit && q->n && q->scene_epoch == ground_scene_epoch && q->army == army && q->task;
    }
    for (int i = 0; i < ALERT_SLOTS; ++i) {
        tac_alert_t *a = &al[i];
        if (a->on != 2)
            continue;
        for (int j = 0; j < ALERT_RESP; ++j) {
            lane_seat_t *o = a->resp[j] > 0 ? &lseat[a->resp[j]] : NULL;
            if (o) {
                const char *rel = NULL;
                if (o->born_ms != a->resp_life[j] || o->scene_epoch != ground_scene_epoch || ms - o->last_ms > 2000)
                    rel = "the responder died";
                else if (!o->task || o->task_alert != i + 1)
                    rel = "recalled by a new order";
                else if (o->cmd_release)
                    rel = "recalled: point pressure";
                if (rel) {
                    if (o->born_ms == a->resp_life[j] && o->task && o->task_alert == i + 1) {
                        o->task = 0;
                        o->task_alert = 0;
                        --tasks;
                    }
                    event(host, "alert: army %d #%d responder %d seat %d life %llu released: %s", army, i, j + 1, a->resp[j],
                          (unsigned long long)a->resp_life[j], rel);
                    a->resp[j] = 0;
                    a->guard_ok[j] = 0;
                    o = NULL;
                }
            }
            int fresh = 0;
            if (!o) {
                if (tasks >= task_max || passive >= live / PASSIVE_DIV)
                    continue;
                /* idle or guarding tanks first; capturing tanks are kept on their point; never a tank known unable */
                int best = 0;
                float bd = 1e18f;
                for (int q_i = 1; q_i < MAX_SEATS; ++q_i) {
                    const lane_seat_t *q = &lseat[q_i];
                    if (!q->unit || !q->n || q->scene_epoch != ground_scene_epoch || q->army != army || q->task
                        || ms - q->last_ms > 1500 || ms - q->born_ms < SPAWN_CALM_MS
                        || q->mode == M_ENGAGE || q->mode == M_RETREAT || q->pusher || q->cmd_release
                        || (q->cmd_apply_ms && q->cmd_ntype == OB_CAPTURE)
                        || (q->cmd_apply_ms && q->cmd_ntype == OB_OVERWATCH && q->cmd_nid >= CMD_POST_ID_BASE)
                        || (q->holding && q->route[q->step].pt == SLOT_PT)
                        || q->cmd_type == OB_SUPPORT || q->cmd_type == OB_OVERWATCH)
                        continue;
                    if (q->cmd_type == OB_CAPTURE && q->lane >= 0 && q->lane < nlanes
                        && zone_owner(host, lanes[q->lane].zx, lanes[q->lane].zz) != army)
                        continue;
                    const int rdy = tac_ready(q, q_i, ms);
                    if (rdy == RDY_EMPTY || rdy == RDY_NOGROUND)
                        continue;
                    const float d = dist2d(q->x, q->z, a->cx, a->cz);
                    if (d > INTERCEPT_MAX_M)
                        continue;
                    const int cls = seat_class(host, q, ms);
                    const float cost = d + (cls == C_IDLE || cls == C_HOLD || q->cmd_type == OB_HOLD ? 0.0f : 400.0f);
                    if (cost < bd) {
                        bd = cost;
                        best = q_i;
                    }
                }
                if (best <= 0)
                    continue;
                o = &lseat[best];
                o->task = 1;
                o->task_tgt = a->enemy;
                o->task_since = ms;
                o->task_why = a->why;
                o->task_point = 0;                  /* s39: no task defends a point of ours any more */
                o->task_side = j ? -1.0f : 1.0f;
                o->task_alert = i + 1;
                a->resp[j] = best;
                a->resp_life[j] = o->born_ms;
                a->guard_ok[j] = 0;
                ++tasks;
                ++passive;
                fresh = 1;
                event(host, "alert: army %d #%d responder %d seat %d life %llu assigned (%s, %.0f m from the area, class %d)%s",
                      army, i, j + 1, best, (unsigned long long)o->born_ms, a->why, dist2d(o->x, o->z, a->cx, a->cz),
                      seat_class(host, o, ms), a->seen ? "" : ", area frozen");
            }
            o->task_tgt = a->enemy;
            if (a->seen || (fresh && !cfg.waitpeek)) {
                task_point(o, a->cx, a->cz);    /* while seen: today's firing spot on the live area */
            } else if (cfg.waitpeek) {
                if (!a->guard_ok[j] || a->guard_for[j] != a->evidence_ms || a->guard_resp_life[j] != o->born_ms)
                    tac_alert_guard(host, army, i, a, j);
                o->task_x = a->guard_x[j];
                o->task_z = a->guard_z[j];
            }                                   /* waitpeek=0, frozen: the firing spot it was given last stays */
            if (fresh)
                event(host, "plan: army %d sends seat %d to intercept slot %d (%s, alert #%d) at (%.0f,%.0f); spot (%.0f,%.0f)",
                      army, a->resp[j], a->enemy, a->why, i, a->cx, a->cz, o->task_x, o->task_z);
        }
    }
}

/* ---- 6. intel through the command layer (s34, intel=1) ------------------------------------------------------ *
 * The command layer knows both sides. Twice a second per army it looks at every living enemy tank (true position) and
 * decides per own tank within IN_RANGE_M what that tank is told:
 *   C "camera": the straight line from a camera point (the hull raised IN_CAM_UP m and moved IN_CAM_BACK m back along
 *     its axis: the third-person camera) to the enemy's turret height (IN_TURRET_H) is clear of terrain and solid
 *     objects (in_line_clear: the collision ray (gh_ray) with land + static + object flags; an object hit that is
 *     not on a hard cell of the route library's map.bin (trees, bushes, hedges, fences) is passed through, at most 3
 *     times; a hit on a hard cell (walls, houses), on terrain or on static collision blocks): the tank is told where it is;
 *   R "report": not camera-visible but within IN_REPORT_M: a clue like a team-mate's fresh report (wait / peek), nothing
 *     more;
 *   else nothing.
 * At most IN_RAYS_PER_TICK rays per call, round robin over the pairs within range; a ray's answer is used for
 * IN_RAY_TTL_MS. What a tank does with it: C points its idle turret there (tac_gaze); a tank standing on an order that
 * holds it turns its hull to an enemy within IN_FACE_M (threat facing, third source "anticipated": no cover, never
 * stops a moving tank); a watch / support crew may move once per IN_HOP_MS to another free slot of its own area with a
 * clear tank-height line to the enemy. Never: pursuit, intercepts, alerts, other places, a gunner target. Nothing is
 * written into the team's contact records. */
#define IN_PERIOD_MS 500u
#define IN_RANGE_M 600.0f
#define IN_REPORT_M 300.0f
#define IN_FACE_M 400.0f
#define IN_RAYS_PER_TICK 40
#define IN_RAY_TTL_MS 3000u
#define IN_FRESH_MS 1500u
#define IN_HOP_MS 20000u
#define IN_CAM_UP 5.0f
#define IN_CAM_BACK 7.0f
#define IN_TURRET_H 2.2f
#define IN_SIGHT_H 2.0f
#define IN_RAY_FLAGS GH_RAY_GROUND_AND_OBJECTS   /* land / heightmap, static collision, placed objects */
enum { IN_NONE, IN_REPORT, IN_CAMERA };
static const char *const in_grade_name[3] = {"none", "report", "camera"};
typedef struct { uint64_t ms; int clear; } in_ray_t;
static in_ray_t in_pair[MAX_SEATS][64];
typedef struct {
    uint64_t life, ms, since, hop_ms, report_used;
    int grade, slot;
    float x, z, vx, vz, d;
} in_seat_t;
static in_seat_t in_seat[MAX_SEATS];
static uint64_t in_next[3], in_minute_ms;
static int in_cursor[3];
static double in_us_sum, in_us_max;
static long in_rays, in_passes, in_lines;

static int in_line_clear(void *host, const float *p0, const float *p1)
{
    float dir[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    const float len = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (!(len > 0.5f) || !isfinite(len))
        return 1;
    for (int k = 0; k < 3; ++k)
        dir[k] /= len;
    const gr_lib_t *lib = tac_lib();
    float from[3] = {p0[0], p0[1], p0[2]}, done = 0.0f;
    const double t0us = strat_now_us();
    int result = 0;
    for (int pass = 0; pass < 4; ++pass) {
        float t = len - done;
        int kind = 0;
        const int hit = trace_with_flags(host, from, dir, &t, &kind, IN_RAY_FLAGS);
        ++in_rays;
        if (hit < 0)
            break;                      /* a fault: blocked */
        if (!hit) {
            result = 1;
            break;
        }
        const float hx = from[0] + dir[0] * t, hz = from[2] + dir[2] * t;
        if (kind != 4 || !lib || gr_hard(lib, hx, hz))
            break;                      /* terrain, static collision, or a wall / house of the map */
        ++in_passes;                    /* vegetation, a hedge, a fence: the camera sees past it */
        done += t + 0.5f;
        if (done >= len) {
            result = 1;
            break;
        }
        for (int k = 0; k < 3; ++k)
            from[k] = p0[k] + dir[k] * done;
    }
    const double us = strat_now_us() - t0us;
    ++in_lines;
    in_us_sum += us;
    if (us > in_us_max)
        in_us_max = us;
    return result;
}

/* An order that holds the tank where it stands (watch / support / post, hold, parked, a guard spot, waiting). */
static int in_standing_hold(const lane_seat_t *s, uint64_t ms)
{
    (void)ms;
    return s->cmd_parked || s->parked || s->holding || !s->wants_move
           || (s->task && dist2d(s->x, s->z, s->task_x, s->task_z) <= INTERCEPT_HOLD_M);
}

/* The camera-grade enemy this tank is told of (fresh), 0 none. */
static int in_anticipate(const lane_seat_t *s, int seat, uint64_t ms, int *slot, float *x, float *z)
{
    const in_seat_t *r = &in_seat[seat];
    if (!cfg.intel || r->life != s->born_ms || r->grade != IN_CAMERA || ms - r->ms > IN_FRESH_MS || !slot_alive[r->slot])
        return 0;
    *slot = r->slot;
    *x = r->x;
    *z = r->z;
    return 1;
}

static const char *in_action(const lane_seat_t *s, const in_seat_t *r, uint64_t ms)
{
    if (r->grade == IN_REPORT)
        return in_standing_hold(s, ms) ? "report (a clue for wait / peek)" : "nothing (moving)";
    if (in_standing_hold(s, ms)) {
        const float off = fabsf(wrap180(atan2f(r->z - s->z, r->x - s->x) * 57.29578f - atan2f(s->fz, s->fx) * 57.29578f));
        return r->d <= IN_FACE_M && off > 35.0f && cfg.threatface ? "turns the hull" : "turret";
    }
    return "turret";
}

/* A watch / support crew in place that cannot see the enemy it is told of: another free slot of its own area with a
 * clear tank-height line to it (once per IN_HOP_MS). Rays counted in the tick's budget. */
static void in_hop(void *host, lane_seat_t *s, int seat, uint64_t ms, in_seat_t *r, const float *enemy, float sight_y, int *budget)
{
    const int j = s->cmd_id - CMD_POST_ID_BASE, army = s->army;
    if (!cmd_is_post(s) || !s->cmd_parked || j < 0 || j >= cmd_nposts[army] || !PK_PLAN(cmd_posts[army][j].kind)
        || cmd_tasks[seat].pending.task_id || s->cmd_apply_ms || ms - r->hop_ms < IN_HOP_MS || *budget < 2)
        return;
    r->hop_ms = ms;
    const cmd_post_t *p = &cmd_posts[army][j];
    const float here[3] = {s->x, sight_y, s->z};
    --*budget;
    if (in_line_clear(host, here, enemy))
        return;                         /* it can see from where it stands */
    int ah[TP_MAX_ZONES];               /* s40: no hop into another area at its cap */
    tp2_area_count(army, ms, ah);
    for (int q = 0; q < cmd_nposts[army] && *budget > 0; ++q) {
        const cmd_post_t *o = &cmd_posts[army][q];
        /* s37 G12: a lane slot must be free by the shared-slot rule (tp2_slot_used, the hopping tank left out) */
        const int lane_post = o->kind == PK_LANE && o->ref >= 0 && o->ref < ntp_lanes && o->line >= 0
                              && o->line < tp_lanes[o->ref].nslot;
        if (q == j || o->kind != p->kind || o->ref != p->ref || (o->kind == PK_LANE && !lane_post)
            || tp_post_held(army, CMD_POST_ID_BASE + q) || cmd_post_option(o, ms) < 0
            || (lane_post && tp2_slot_used(army, o->ref, o->line, ms, seat))
            || !tp2_area_open(army, seat, tp2_area_at(o->pos[0].x, o->pos[0].z), ah))
            continue;
        const float from[3] = {o->pos[0].x, sight_y, o->pos[0].z};
        --*budget;
        if (!in_line_clear(host, from, enemy))
            continue;
        const cmd_obj_t ob = {OB_OVERWATCH, CMD_POST_ID_BASE + q, 1, strat_band(strat_lat(o->pos[0].x, o->pos[0].z)), o->lane, 0,
                              o->cap, o->pos[0].x, o->pos[0].z, o->kind == PK_WATCH ? TP_WATCH_VALUE : TP_SUPPORT_VALUE, 0};
        event(host, "intel: army %d seat %d moves to slot %d of %s%s (post %d): a clear line to slot %d from there, none from "
              "post %d", army, seat, o->line, lane_post ? "lane " : "", lane_post ? tp_lanes[o->ref].id : tp_areas[o->ref].id,
              CMD_POST_ID_BASE + q, r->slot, CMD_POST_ID_BASE + j);
        if (lane_post)      /* s37 G12: the order's slot target is the new slot's (read by cmd_issue) */
            tp_seat_target[seat] = tp_lanes[o->ref].slot[o->line].target + 1;
        cmd_issue(host, army, seat, &ob, 0.0f, 0, ms);
        if (lane_post && s->cmd_apply_ms && s->cmd_ntype == OB_OVERWATCH && s->cmd_nid == CMD_POST_ID_BASE + q) {
            s->plan_lane1 = o->ref + 1;     /* the same lane; its slot follows the hop */
            s->plan_slot = o->line;
            s->plan_lane_ms = ms;
        }
        return;
    }
}

static void in_tick(void *host, int army, uint64_t ms)
{
    if (!cfg.intel || army < 1 || army > 2 || ms < in_next[army])
        return;
    in_next[army] = ms + IN_PERIOD_MS;
    int budget = IN_RAYS_PER_TICK;
    /* the pairs within range, in a fixed order; the round robin starts at the cursor */
    int ps[MAX_SEATS * 64 / 2], pe[MAX_SEATS * 64 / 2], np = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *s = &lseat[i];
        if (!s->unit || s->army != army || s->scene_epoch != ground_scene_epoch || ms - s->last_ms > 2000)
            continue;
        for (int k = 0; k < 64 && np < MAX_SEATS * 64 / 2; ++k) {
            if (!slot_alive[k] || !slot_unit[k] || !slot_army[k] || slot_army[k] == army)
                continue;
            const float *e = gh_unit_pos(slot_unit[k]);
            if (dist2d(s->x, s->z, e[0], e[2]) > IN_RANGE_M)
                continue;
            ps[np] = i;
            pe[np++] = k;
        }
    }
    for (int n = 0; n < np && budget > 0; ++n) {
        const int q = (in_cursor[army] + n) % np, i = ps[q], k = pe[q];
        const lane_seat_t *s = &lseat[i];
        if (ms - in_pair[i][k].ms < IN_RAY_TTL_MS / 2)
            continue;
        const float *u = gh_unit_pos(s->unit), *e = gh_unit_pos(slot_unit[k]);
        const float cam[3] = {u[0] - s->fx * IN_CAM_BACK, u[1] + IN_CAM_UP, u[2] - s->fz * IN_CAM_BACK};
        const float tgt[3] = {e[0], e[1] + IN_TURRET_H, e[2]};
        in_pair[i][k] = (in_ray_t){ms, in_line_clear(host, cam, tgt)};
        --budget;
        in_cursor[army] = (q + 1) % np;
    }
    /* per own tank: the nearest camera-grade enemy, else the nearest within report range */
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *s = &lseat[i];
        if (!s->unit || s->army != army || s->scene_epoch != ground_scene_epoch || ms - s->last_ms > 2000)
            continue;
        in_seat_t *r = &in_seat[i];
        if (r->life != s->born_ms) {
            memset(r, 0, sizeof *r);
            r->life = s->born_ms;
            r->slot = -1;
        }
        int grade = IN_NONE, slot = -1;
        float bd = 1e9f;
        for (int k = 0; k < 64; ++k) {
            if (!slot_alive[k] || !slot_unit[k] || !slot_army[k] || slot_army[k] == army)
                continue;
            const float *e = gh_unit_pos(slot_unit[k]);
            const float d = dist2d(s->x, s->z, e[0], e[2]);
            if (d > IN_RANGE_M)
                continue;
            const int cam = in_pair[i][k].ms && ms - in_pair[i][k].ms < IN_RAY_TTL_MS && in_pair[i][k].clear;
            const int g = cam ? IN_CAMERA : d <= IN_REPORT_M ? IN_REPORT : IN_NONE;
            if (g > grade || (g == grade && g && d < bd)) {
                grade = g;
                slot = k;
                bd = d;
            }
        }
        if (grade != r->grade || slot != r->slot) {
            if (r->grade)
                event(host, "intel: army %d seat %d episode slot %d (%s) over after %.1f s", army, i, r->slot,
                      in_grade_name[r->grade], (double)(ms - r->since) / 1000.0);
            r->since = ms;
            r->report_used = 0;
        }
        const int started = grade && (grade != r->grade || slot != r->slot);
        if (grade) {
            const float *e = gh_unit_pos(slot_unit[slot]);
            const float dt = r->slot == slot && r->ms && ms > r->ms ? (float)(ms - r->ms) / 1000.0f : 0.0f;
            r->vx = dt > 0.05f ? (e[0] - r->x) / dt : 0.0f;
            r->vz = dt > 0.05f ? (e[2] - r->z) / dt : 0.0f;
            r->x = e[0];
            r->z = e[2];
            r->d = bd;
            r->ms = ms;
        }
        r->grade = grade;
        r->slot = slot;
        if (started)
            event(host, "intel: army %d seat %d told of slot %d (%s) at %.0f m: %s", army, i, slot, in_grade_name[grade], bd,
                  in_action(s, r, ms));
        if (grade == IN_CAMERA) {
            const float *e = gh_unit_pos(slot_unit[slot]);
            const float tgt[3] = {e[0], e[1] + IN_TURRET_H, e[2]};
            in_hop(host, s, i, ms, r, tgt, (gh_unit_pos(s->unit))[1] + IN_SIGHT_H, &budget);
        }
    }
    if (ms >= in_minute_ms) {
        if (in_minute_ms && in_rays)
            event(host, "intel: last 60 s %ld rays (%ld passed through vegetation / hedges), %.1f us per line on average, "
                  "%.1f us max (%ld lines)", in_rays, in_passes, in_us_sum / (double)(in_lines ? in_lines : 1), in_us_max, in_lines);
        in_minute_ms = ms + 60000;
        in_rays = in_passes = in_lines = 0;
        in_us_sum = in_us_max = 0.0;
    }
}

/* ---- 5. threat facing (s29, threatface=1) ------------------------------------------------------------------- *
 * One decision owns the hull while a threat is off the nose: the taken-over target this tank sees now and the game's
 * gunner holds (eng == gunner), or, with no such target, a hit of the last TF_HIT_MS (then only its bearing). Reports
 * and presence hints never turn the hull. It starts when the hull is more than TF_START_DEG off, and is kept until the
 * threat is dead or unseen for TF_LOST_MS (a hit-only threat: TF_LOST_MS after the last hit); its target does not
 * change while kept (a hit-only threat takes the target that appears). It ends with TF_LINGER_MS of standing still,
 * then the order. When it starts it decides once: cover (library loaded, threat position known: the nearest
 * reachable point whose hull is hidden from the threat by map.bin hard cells, path <= cover_m, first direction within
 * TF_COVER_FWD_DEG of the nose) -> a reaction move there, then the turn; else (or that move failing) the turn in place
 * to within TF_DONE_DEG, then the hold (turned again when the threat gets TF_START_DEG off). The hull request
 * (control_prepare_engagement, tac_face_hull) is not charged to any budget. */
#define TF_START_DEG 35.0f
#define TF_DONE_DEG 15.0f
#define TF_LOST_MS 4000
#define TF_HIT_MS 3000
#define TF_LINGER_MS 3000
#define TF_COVER_FWD_DEG 60.0f
#define TF_LATTICE_M 2.0f
#define TF_ARRIVE_M 3.0f
#define TF_STALL_MS 4000
#define TF_BLOCKED_MS 1500
#define TF_MAX_CANDS 1024
#define TF_COVER_REACH_M 300.0f     /* the hidden test marches each line this far from the candidate towards the threat */
enum { TF_NONE, TF_COVER, TF_TURN, TF_HOLD, TF_LINGER };
static const char *const tf_name[] = {"none", "cover", "turn", "hold", "linger"};
typedef struct {
    uint64_t life, slot_life, since, last_seen, last_hit, linger_until, move_prog_ms, blocked_ms;
    int state, slot, by_cover;
    int anticipated;            /* s34: the third threat source (intel, grade C): no cover, never stops a moving tank */
    float tx, tz, cx, cz, cover_len, move_best;
} tac_face_t;
static tac_face_t tac_face[MAX_SEATS];
typedef struct { float cost, x, z; } tf_cand_t;
static tf_cand_t tf_cands[TF_MAX_CANDS];
static gr_pt_t tf_path[1024];
static double tf_search_us_max;

static int tac_face_busy(const lane_seat_t *s, int seat)
{
    const tac_face_t *f = &tac_face[seat];
    return f->life == s->born_ms && (f->state == TF_COVER || f->state == TF_TURN);
}

static int tf_cand_cmp(const void *a, const void *b)
{
    const float x = ((const tf_cand_t *)a)->cost, y = ((const tf_cand_t *)b)->cost;
    return x < y ? -1 : x > y;
}

/* The hull at (x, z), pointing at the threat (tx, tz), hidden by hard cells: the centre first, then the four corners;
 * all five lines blocked. */
/* One line from (x, z) towards the threat, at most TF_COVER_REACH_M of it: cover stands near the tank. */
static int tf_line_blocked(const gr_lib_t *lib, float x, float z, float tx, float tz)
{
    const float d = dist2d(x, z, tx, tz);
    const float k = d > TF_COVER_REACH_M ? TF_COVER_REACH_M / d : 1.0f;
    return gr_line_hard(lib, x, z, x + (tx - x) * k, z + (tz - z) * k);
}

static int tf_hidden(const gr_lib_t *lib, float x, float z, float tx, float tz, float hl, float hw)
{
    const float d = dist2d(x, z, tx, tz);
    if (d < 0.5f || !tf_line_blocked(lib, x, z, tx, tz))
        return 0;
    const float fx = (tx - x) / d, fz = (tz - z) / d;
    for (int k = 0; k < 4; ++k) {
        const float a = (k & 1) ? hl : -hl, b = (k & 2) ? hw : -hw;
        if (!tf_line_blocked(lib, x + fx * a - fz * b, z + fz * a + fx * b, tx, tz))
            return 0;
    }
    return 1;
}

/* Cover search (library only): gr_dijkstra from the hull, path length <= cover_m over cells with GR_CLEAR_MIN; the 2 m
 * lattice points it reached, in order of path cost; the first one hidden is the nearest cover. 1 when it is near
 * enough (first direction of its path within TF_COVER_FWD_DEG of the nose): *x, *z, *len. *nearest: path length of
 * the nearest hidden point (-1 none); *why: what made it not near enough. */
static int tf_cover_search(const lane_seat_t *s, int seat, float tx, float tz, float *x, float *z, float *len, float *nearest,
                           const char **why)
{
    const gr_lib_t *lib = tac_lib();
    *nearest = -1.0f;
    *why = "none";
    if (!lib)
        return 0;
    const double t0us = strat_now_us();
    const float range = cfg.cover_m;
    gr_info_t info;
    if (gr_dijkstra(lib, s->x, s->z, range, 2.0f, NULL, 0, &info) != GR_OK) {
        *why = "no search from here";
        return 0;
    }
    int n = 0;
    const int k = (int)(range / TF_LATTICE_M);
    for (int i = -k; i <= k; ++i)
        for (int j = -k; j <= k && n < TF_MAX_CANDS; ++j) {
            const float px = s->x + (float)i * TF_LATTICE_M, pz = s->z + (float)j * TF_LATTICE_M;
            if (dist2d(s->x, s->z, px, pz) > range)
                continue;
            const float cost = gr_reach_cost(lib, px, pz);
            if (isfinite(cost))
                tf_cands[n++] = (tf_cand_t){cost, px, pz};
        }
    qsort(tf_cands, (size_t)n, sizeof tf_cands[0], tf_cand_cmp);
    const vfit_t *vf = roads_vfit(seat);
    const float hl = vf && vf->ok && vf->len > 1.0f ? 0.5f * vf->len : 3.1f;
    const float hw = vf && vf->ok && vf->wid > 0.5f ? 0.5f * vf->wid : 1.65f;
    /* in order of path cost: the first point that is hidden AND whose path starts within TF_COVER_FWD_DEG of the nose;
     * a hidden point behind / beside is skipped (the nearest skipped one is reported in *nearest / *why) */
    int ok = 0;
    for (int k2 = 0; k2 < n && !ok; ++k2) {
        if (!tf_hidden(lib, tf_cands[k2].x, tf_cands[k2].z, tx, tz, hl, hw))
            continue;
        const int m = gr_reach_path(lib, s->x, s->z, tf_cands[k2].x, tf_cands[k2].z, NULL, 0, tf_path,
                                    (int)(sizeof tf_path / sizeof tf_path[0]));
        float length = 0.0f;
        for (int q = 1; q < m; ++q)
            length += hypotf(tf_path[q].x - tf_path[q - 1].x, tf_path[q].z - tf_path[q - 1].z);
        if (m <= 0)
            length = dist2d(s->x, s->z, tf_cands[k2].x, tf_cands[k2].z);
        if (m <= 1 || length < 1.0f) {
            ok = 1;                     /* it stands in cover already */
        } else {
            const int a = m - 1 < 6 ? m - 1 : 6;    /* the direction of the first 3 m */
            const float dx = tf_path[a].x - s->x, dz = tf_path[a].z - s->z, dl = hypotf(dx, dz);
            const float c = dl > 0.1f ? (dx * s->fx + dz * s->fz) / dl : 1.0f;
            if (c >= cosf(TF_COVER_FWD_DEG / 57.29578f))
                ok = 1;
            else if (*nearest < 0.0f) {
                *nearest = length;      /* the nearest hidden point, skipped */
                *why = c <= -cosf(30.0f / 57.29578f) ? "not ahead: behind the tail (reverse to cover not offered)"
                                                     : "not ahead";
            }
        }
        if (ok) {
            *x = tf_cands[k2].x;
            *z = tf_cands[k2].z;
            *len = length;
        }
    }
    const double us = strat_now_us() - t0us;
    if (us > tf_search_us_max)
        tf_search_us_max = us;
    return ok;
}

static void tac_face_end(void *host, const lane_seat_t *s, int seat, uint64_t ms, tac_face_t *f, const char *why)
{
    const float hull = atan2f(s->fz, s->fx) * 57.29578f;
    const float off = wrap180(atan2f(f->tz - s->z, f->tx - s->x) * 57.29578f - hull);
    event(host, "face: seat %d life %llu threat slot %d over after %.1f s: %s; hull %.0f deg off at the end (%s); stands %.0f s, "
          "then back to order %s %llu/%u", seat, (unsigned long long)s->born_ms, f->slot, (double)(ms - f->since) / 1000.0,
          why, fabsf(off), tf_name[f->state], 0.001 * TF_LINGER_MS, s->cmd_type >= 0 && s->cmd_type < OB_N ? cmd_name[s->cmd_type] : "?",
          (unsigned long long)s->nav.task_id, s->nav.revision);
    f->state = TF_LINGER;
    f->linger_until = ms + TF_LINGER_MS;
}

static void tac_face_tick(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    tac_face_t *f = &tac_face[seat];
    if (f->life != s->born_ms) {
        memset(f, 0, sizeof *f);
        f->life = s->born_ms;
        f->slot = -1;
    }
    if (s->fx * s->fx + s->fz * s->fz < 0.25f)
        return;
    const float hull = atan2f(s->fz, s->fx) * 57.29578f;
    /* the threat now: the taken-over target seen with own eyes and held by the game's gunner */
    int tgt = -1;
    float px = 0.0f, pz = 0.0f, age;
    if (s->mode == M_ENGAGE && s->eng >= 0 && s->eng == s->gt_slot && slot_alive[s->eng]
        && contact_get(s->army, seat, s->eng, ms, &px, &pz, &age) == 2)
        tgt = s->eng;
    const hitdir_t h = hit_dir[seat & 63];
    const int hit = h.ms && ms >= h.ms && ms - h.ms < TF_HIT_MS;
    if (f->state == TF_COVER || f->state == TF_TURN || f->state == TF_HOLD) {
        const char *end = NULL;
        int aslot = -1;
        float ax = 0.0f, az = 0.0f;
        if (s->mode == M_RETREAT) {
            end = "backs off under fire (retreat rule)";
        } else if (f->anticipated && (tgt >= 0 || hit)) {
            /* s34: a real threat takes over the anticipated one */
            f->anticipated = 0;
            f->slot = tgt;
            f->slot_life = tgt >= 0 ? slot_life[tgt] : 0;
            f->last_seen = ms;
            f->last_hit = hit ? h.ms : f->last_hit;
            f->tx = tgt >= 0 ? px : s->x + h.dx * 100.0f;
            f->tz = tgt >= 0 ? pz : s->z + h.dz * 100.0f;
            event(host, "face: seat %d life %llu anticipated threat replaced by %s", seat, (unsigned long long)s->born_ms,
                  tgt >= 0 ? "a seen target held by the gunner" : "a hit");
        } else if (f->anticipated) {
            if (!in_standing_hold(s, ms))
                end = "the tank moves (anticipation never stops it)";
            else if (!slot_alive[f->slot] || slot_life[f->slot] != f->slot_life)
                end = "threat dead";
            else if (in_anticipate(s, seat, ms, &aslot, &ax, &az) && aslot == f->slot) {
                f->last_seen = ms;
                f->tx = ax;
                f->tz = az;
            } else if (ms - f->last_seen > TF_LOST_MS)
                end = "anticipated enemy not camera-visible for 4 s";
        } else if (f->slot >= 0) {
            if (!slot_alive[f->slot] || slot_life[f->slot] != f->slot_life)
                end = "threat dead";
            else if (tgt == f->slot) {
                f->last_seen = ms;
                f->tx = px;
                f->tz = pz;
            } else if (ms - f->last_seen > TF_LOST_MS)
                end = "unseen 4 s";
        } else if (tgt >= 0) {
            f->slot = tgt;
            f->slot_life = slot_life[tgt];
            f->last_seen = ms;
            f->tx = px;
            f->tz = pz;
            event(host, "face: seat %d life %llu hit-only threat is now slot %d (seen and gunner)", seat,
                  (unsigned long long)s->born_ms, tgt);
        } else {
            if (h.ms > f->last_hit) {
                f->last_hit = h.ms;
                f->tx = s->x + h.dx * 100.0f;
                f->tz = s->z + h.dz * 100.0f;
            }
            if (ms - f->last_hit > TF_LOST_MS)
                end = "no hit for 4 s";
        }
        if (end) {
            tac_face_end(host, s, seat, ms, f, end);
            return;
        }
        const float off = fabsf(wrap180(atan2f(f->tz - s->z, f->tx - s->x) * 57.29578f - hull));
        if (f->state == TF_COVER) {
            const float d = dist2d(s->x, s->z, f->cx, f->cz);
            if (d <= TF_ARRIVE_M) {
                event(host, "face: seat %d life %llu in cover at (%.0f,%.0f) after %.1f s: turns to the threat (%.0f deg off)",
                      seat, (unsigned long long)s->born_ms, f->cx, f->cz, (double)(ms - f->since) / 1000.0, off);
                f->state = TF_TURN;
                return;
            }
            if (d + 1.5f < f->move_best) {
                f->move_best = d;
                f->move_prog_ms = ms;
            }
            if (nav_recent(s->nav.feedback_ms, ms) && s->nav.blocked) {
                if (!f->blocked_ms)
                    f->blocked_ms = ms;
            } else {
                f->blocked_ms = 0;
            }
            const char *fail = f->blocked_ms && ms - f->blocked_ms >= TF_BLOCKED_MS ? "blocked on the driving side"
                               : ms - f->move_prog_ms >= TF_STALL_MS ? (s->motion_state == GM_TURNING ? "cannot turn there" : "no progress")
                               : NULL;
            if (fail) {
                event(host, "face: seat %d life %llu cover move to (%.0f,%.0f) failed (%s, %.0f m left): turns in place", seat,
                      (unsigned long long)s->born_ms, f->cx, f->cz, fail, d);
                f->state = TF_TURN;
            }
            return;
        }
        if (f->state == TF_TURN && off <= TF_DONE_DEG)
            f->state = TF_HOLD;
        else if (f->state == TF_HOLD && off > TF_START_DEG)
            f->state = TF_TURN;
        return;
    }
    if (f->state == TF_LINGER && ms >= f->linger_until)
        f->state = TF_NONE;
    /* a new decision: a seen target first, else a fresh hit (only its bearing) */
    float tx, tz;
    int slot = -1, antic = 0;
    if (tgt >= 0) {
        slot = tgt;
        tx = px;
        tz = pz;
    } else if (hit && h.ms > f->last_hit) {
        tx = s->x + h.dx * 100.0f;
        tz = s->z + h.dz * 100.0f;
    } else if (cfg.intel && in_standing_hold(s, ms) && in_anticipate(s, seat, ms, &slot, &tx, &tz)
               && dist2d(s->x, s->z, tx, tz) <= IN_FACE_M) {
        antic = 1;                      /* s34: the third source, below the two above */
    } else {
        return;
    }
    const float off = fabsf(wrap180(atan2f(tz - s->z, tx - s->x) * 57.29578f - hull));
    if (off <= TF_START_DEG)
        return;                         /* within 35 deg of the nose: route and stop-to-fire as they are */
    f->anticipated = antic;
    if (antic) {
        f->slot = slot;
        f->slot_life = slot_life[slot];
        f->since = f->last_seen = ms;
        f->tx = tx;
        f->tz = tz;
        f->state = TF_TURN;
        event(host, "face: seat %d life %llu threat slot %d (anticipated: intel camera) at %.0f m, hull %.0f deg off | standing "
              "on its order -> turns in place (never to cover)", seat, (unsigned long long)s->born_ms, slot,
              dist2d(s->x, s->z, tx, tz), off);
        return;
    }
    f->slot = slot;
    f->slot_life = slot >= 0 ? slot_life[slot] : 0;
    f->since = f->last_seen = ms;
    f->last_hit = h.ms;
    f->tx = tx;
    f->tz = tz;
    f->move_best = 1e9f;
    f->move_prog_ms = ms;
    f->blocked_ms = 0;
    char head[200];
    if (slot >= 0)
        snprintf(head, sizeof head, "face: seat %d life %llu threat slot %d (seen and gunner) at %.0f m, hull %.0f deg off", seat,
                 (unsigned long long)s->born_ms, slot, dist2d(s->x, s->z, tx, tz), off);
    else
        snprintf(head, sizeof head, "face: seat %d life %llu threat hit (bearing %.0f, from slot %d), hull %.0f deg off", seat,
                 (unsigned long long)s->born_ms, atan2f(h.dz, h.dx) * 57.29578f, h.from, off);
    float cx = 0.0f, cz = 0.0f, len = 0.0f, nearest = -1.0f;
    const char *why = "none";
    /* s35: on a plan lane, the nearest of its bounds within 60 m is the cover; the 30 m search is the fallback */
    int lane_bound = -1;
    const int lane = cfg.teamplan && tp_format == 2 ? tp2_lane_of(s->army, s) : -1;
    if (lane >= 0) {
        float bd = 60.0f;
        for (int q = 0; q < tp_lanes[lane].nbound; ++q) {
            const float d = dist2d(s->x, s->z, tp_lanes[lane].bound[q].x, tp_lanes[lane].bound[q].z);
            if (d < bd) {
                bd = d;
                lane_bound = q;
            }
        }
        if (lane_bound >= 0) {
            cx = tp_lanes[lane].bound[lane_bound].x;
            cz = tp_lanes[lane].bound[lane_bound].z;
            len = bd;
        }
    }
    const int cover = lane_bound >= 0 || (slot >= 0 && tac_lib() && tf_cover_search(s, seat, tx, tz, &cx, &cz, &len, &nearest, &why));
    if (lane_bound >= 0) {
        f->state = TF_COVER;
        f->cx = cx;
        f->cz = cz;
        f->cover_len = len;
        event(host, "%s | bound %d of lane %s at (%.0f,%.0f), %.0f m -> goes to cover (lane bound)", head, lane_bound,
              tp_lanes[lane].id, cx, cz, len);
        return;
    }
    if (cover) {
        f->state = TF_COVER;
        f->cx = cx;
        f->cz = cz;
        f->cover_len = len;
        char skipped[96] = "";
        if (nearest >= 0.0f)
            snprintf(skipped, sizeof skipped, "; nearer hidden point %.0f m skipped (%s)", nearest, why);
        event(host, "%s | cover at (%.0f,%.0f) %.0f m forward%s -> goes to cover (search max %.1f ms)", head, cx, cz, len,
              skipped, tf_search_us_max / 1000.0);
    } else {
        f->state = TF_TURN;
        char nb[96];
        if (slot < 0)
            snprintf(nb, sizeof nb, "no position known (hit only)");
        else if (!tac_lib())
            snprintf(nb, sizeof nb, "no route library");
        else if (nearest >= 0.0f)
            snprintf(nb, sizeof nb, "nearest %.0f m, %s", nearest, why);
        else
            snprintf(nb, sizeof nb, "none");
        event(host, "%s | no cover within %.0f m (%s) -> turns in place", head, cfg.cover_m, nb);
    }
}

/* react_propose: the move to cover (a reaction goal; the driving side finds its way). */
static int tac_face_propose(lane_seat_t *s, int seat)
{
    const tac_face_t *f = &tac_face[seat];
    if (!cfg.threatface || f->life != s->born_ms || f->state != TF_COVER)
        return 0;
    nav_propose(&s->nav, NAV_REACTION, f->cx, f->cz, NAV_NATIVE, 1, 0, 0);
    return 1;
}

/* control_prepare_engagement: 0 no request (route, today's stop-to-fire), 1 turn to the threat (align and its stop),
 * 2 hold (stop). A hold yields to an advance the takeover gate allowed (come-out from there). */
static int tac_face_hull(lane_seat_t *s, int seat, uint64_t ms)
{
    const tac_face_t *f = &tac_face[seat];
    if (!cfg.threatface || f->life != s->born_ms)
        return 0;
    switch (f->state) {
    case TF_TURN:
        s->face_x = f->tx;
        s->face_z = f->tz;
        return 1;
    case TF_HOLD:
        s->face_x = f->tx;
        s->face_z = f->tz;
        return s->fight_move == FIGHT_LOCAL ? 0 : 2;
    case TF_LINGER:
        return ms < f->linger_until ? 2 : 0;
    default:
        return 0;
    }
}

/* ---- 4. the reaction record -------------------------------------------------------------------------------- */
#define TAC_REPORT_M 150.0f         /* a team report this close to the tank is a clue for it */
#define TAC_REPORT_AGE_MS 5000
#define TAC_LOST_MIN_MS 1000        /* own sight gone this long ... */
#define TAC_LOST_MAX_MS 10000       /* ... and not longer: "just saw it vanish" */
#define TAC_LOST_M 500.0f           /* a vanish further than this from the tank starts no reaction */
#define TAC_FRESH_MS 12000          /* a clue this old can still start a peek (an alert: TAC_GUARD_HOLD_MS) */
#define TAC_GUARD_HOLD_MS 20000     /* a guard holds a wait this long after the clue (an alert: while it lives) */
#define TAC_CONT_LOOK_MS 8000       /* continue: the turret watches the exit this long */
#define TAC_ENC_GAP_MS 10000        /* a new clue within this of the last wait / peek is the same encounter */
#define TAC_ARRIVE_M 4.0f
#define TAC_PEEK_AFTER_MS 8000      /* a guard peeks only after waiting this long in place */
#define TAC_PEEK_LOOK_MS 1500       /* at the peek point: time for the game's sight check */
#define TAC_MOVE_STALL_MS 4000
#define TAC_BLOCKED_MS 1500
static const char *const tb_name[] = {"none", "continue", "wait", "peek"};
static const char *const tc_name[] = {"none", "lost", "hint", "report", "alert"};
static const char *const tc_long[] = {"none", "own sight lost", "presence hint", "fresh team report", "rear alert"};

typedef struct { int type, slot, band, sector; uint64_t ms, key; float ox, oz, vx, vz; } tac_clue_t;

static const char *tac_order_name(const lane_seat_t *s)
{
    return s->cmd_type >= 0 && s->cmd_type < OB_N ? cmd_name[s->cmd_type] : "?";
}

static void tac_move_reset(tac_rx_t *r, uint64_t ms)
{
    r->move_best = 1e9f;
    r->move_prog_ms = ms;
    r->blocked_ms = 0;
}

/* A move to a wait / peek point that the driving side does not complete (blocked feedback, no progress: a wall it
 * cannot turn at, stuck, no way found) is a failure of that position. */
static int tac_move_failed(const lane_seat_t *s, tac_rx_t *r, float d, uint64_t ms, const char **why)
{
    if (d + 1.5f < r->move_best) {
        r->move_best = d;
        r->move_prog_ms = ms;
    }
    if (nav_recent(s->nav.feedback_ms, ms) && s->nav.blocked) {
        if (!r->blocked_ms)
            r->blocked_ms = ms;
    } else {
        r->blocked_ms = 0;
    }
    if (r->blocked_ms && ms - r->blocked_ms >= TAC_BLOCKED_MS) {
        *why = "blocked on the driving side";
        return 1;
    }
    if (ms - r->move_prog_ms >= TAC_MOVE_STALL_MS) {
        *why = s->motion_state == GM_TURNING ? "cannot turn there" : "no progress";
        return 1;
    }
    return 0;
}

static void tac_log_decision(void *host, const lane_seat_t *s, int seat, uint64_t ms, const tac_rx_t *r, const char *no,
                             uint64_t budget)
{
    const float x = r->beh == TB_PEEK ? r->peek_x : r->beh == TB_WAIT ? r->wait_x : s->x;
    const float z = r->beh == TB_PEEK ? r->peek_z : r->beh == TB_WAIT ? r->wait_z : s->z;
    event(host, "tac: seat %d life %llu order %s %llu/%u | clue %s slot %d age %.1f s | exit %s#%d (%.0f,%.0f) | -> %s%s%s%s "
          "at (%.0f,%.0f) | eng %d gunner %d | budget %.1f/%.1f s%s", seat, (unsigned long long)s->born_ms,
          tac_order_name(s), (unsigned long long)s->nav.task_id, s->nav.revision, tc_long[r->clue], r->slot,
          (double)(ms - r->clue_ms) / 1000.0, r->exit.ok ? r->exit.kind : "none", r->exit.ref, r->exit.x, r->exit.z,
          tb_name[r->beh], no ? " (" : "", no ? no : "", no ? ")" : "", x, z, s->mode == M_ENGAGE ? s->eng : -1, s->gt_slot,
          (double)r->enc_spent / 1000.0, (double)budget / 1000.0, r->guard ? " | guard task: its own term" : "");
}

static void tac_end(void *host, const lane_seat_t *s, int seat, uint64_t ms, tac_rx_t *r, const char *why)
{
    event(host, "tac: seat %d life %llu %s over after %.1f s: %s | hull %s | clue %s slot %d exit %s#%d | eng %d gunner %d | "
          "budget used %.1f s | back to order %s %llu/%u", seat, (unsigned long long)s->born_ms, tb_name[r->beh],
          (double)(ms - r->since) / 1000.0, why, r->hull ? r->hull : "-", tc_name[r->clue], r->slot,
          r->exit.ok ? r->exit.kind : "none", r->exit.ref, s->mode == M_ENGAGE ? s->eng : -1, s->gt_slot,
          (double)r->enc_spent / 1000.0, tac_order_name(s), (unsigned long long)s->nav.task_id, s->nav.revision);
    r->beh = TB_NONE;
}

/* Peek once per clue: fresh credible clue, this life able to fight, a move allowed, a reachable peek point within
 * TAC_PEEK_M, and (en route) the time for it left in the stop budget. */
static int tac_peek_allowed(const lane_seat_t *s, int seat, uint64_t ms, ground_engagement_policy_t pol, tac_rx_t *r,
                            const char **why)
{
    const char *w = NULL;
    if (r->peeked_key == r->clue_key)
        w = "the clue was already used for a peek";
    else if (ms - r->clue_ms > (uint64_t)(r->clue == TC_ALERT ? TAC_GUARD_HOLD_MS : TAC_FRESH_MS))
        w = "the clue is not fresh";
    else if (!tac_able(tac_ready(s, seat, ms)))
        w = "this life cannot fight";
    else if (!(pol.allowed & GFM_ADVANCE))
        w = "the task allows no move";
    else if (!tac_peek_spot(s, r, &r->peek_x, &r->peek_z))
        w = "no peek point";
    else if (!r->guard) {
        const uint64_t need = (uint64_t)(dist2d(s->x, s->z, r->peek_x, r->peek_z) / 4.0f * 1000.0f) + TAC_PEEK_LOOK_MS;
        if (r->enc_spent + need > pol.short_stop_budget_ms)
            w = "no time left in the stop budget";
    }
    r->peek_ok = w == NULL;
    if (why)
        *why = w;
    return r->peek_ok;
}

static int tac_new_clue(lane_seat_t *s, int seat, uint64_t ms, tac_rx_t *r, tac_clue_t *c)
{
    memset(c, 0, sizeof *c);
    c->slot = -1;
    /* own sight lost: the newest vanish not used yet */
    int best = -1;
    for (int k = 0; k < 64; ++k)
        if (r->own_ms[k] && ms - r->own_ms[k] >= TAC_LOST_MIN_MS && ms - r->own_ms[k] <= TAC_LOST_MAX_MS
            && r->lost_done[k] != r->own_ms[k] && slot_alive[k] && dist2d(s->x, s->z, r->own_x[k], r->own_z[k]) <= TAC_LOST_M
            && (best < 0 || r->own_ms[k] > r->own_ms[best]))
            best = k;
    if (best >= 0) {
        r->lost_done[best] = r->own_ms[best];
        *c = (tac_clue_t){TC_LOST, best, 0, 0, r->own_ms[best], ((uint64_t)TC_LOST << 58) ^ ((uint64_t)best << 50) ^ r->own_ms[best],
                          r->own_x[best], r->own_z[best], r->own_vx[best], r->own_vz[best]};
        return 1;
    }
    /* the responder of a frozen rear alert */
    if (cfg.alert && s->task && s->task_alert > 0 && s->task_alert <= ALERT_SLOTS && s->army >= 1 && s->army <= 2) {
        const tac_alert_t *a = &tac_alert[s->army][s->task_alert - 1];
        if (a->on == 2 && !a->seen && tac_alert_resp_of(a, seat) >= 0 && r->alert_done != a->evidence_ms) {
            r->alert_done = a->evidence_ms;
            *c = (tac_clue_t){TC_ALERT, a->enemy, 0, 0, a->evidence_ms,
                              ((uint64_t)TC_ALERT << 58) ^ ((uint64_t)a->enemy << 50) ^ a->evidence_ms, a->cx, a->cz, a->vx, a->vz};
            return 1;
        }
    }
    /* a fresh team report close to this tank */
    float bd = TAC_REPORT_M;
    for (int k = 0; k < 64; ++k) {
        float x, z, age;
        if (contact_get(s->army, seat, k, ms, &x, &z, &age) != 1 || age * 1000.0f > (float)TAC_REPORT_AGE_MS)
            continue;
        const float d = dist2d(s->x, s->z, x, z);
        gh_lock(GH_LOCK_KNOW);
        const contact_t ct = contact[s->army][k];
        gh_unlock(GH_LOCK_KNOW);
        if (d > bd || r->report_done[k] == ct.first_ms)
            continue;
        bd = d;
        best = k;
        *c = (tac_clue_t){TC_REPORT, k, 0, 0, ct.last_ms, ((uint64_t)TC_REPORT << 58) ^ ((uint64_t)k << 50) ^ ct.first_ms,
                          ct.x, ct.z, ct.vx, ct.vz};
    }
    if (best >= 0) {
        gh_lock(GH_LOCK_KNOW);
        r->report_done[best] = contact[s->army][best].first_ms;
        gh_unlock(GH_LOCK_KNOW);
        return 1;
    }
    /* s34: what the command layer told this tank (grade R or C), as a fresh report clue, once per episode */
    if (cfg.intel && s->army >= 1 && s->army <= 2) {
        in_seat_t *in = &in_seat[seat];
        if (in->life == s->born_ms && in->grade && ms - in->ms <= IN_FRESH_MS && in->report_used != in->since && in->slot >= 0
            && dist2d(s->x, s->z, in->x, in->z) <= TAC_REPORT_M + (in->grade == IN_CAMERA ? 250.0f : 0.0f)) {
            in->report_used = in->since;
            *c = (tac_clue_t){TC_REPORT, in->slot, 0, 0, in->ms, ((uint64_t)TC_REPORT << 58) ^ ((uint64_t)in->slot << 50) ^ in->since,
                              in->x, in->z, in->vx, in->vz};
            return 1;
        }
    }
    /* a new presence hint */
    if (cfg.sense) {
        const int k = tac_best_hint(s, seat);
        if (k >= 0 && tac_hint[seat][k].on == 1 && r->hint_done != tac_hint[seat][k].start_ms) {
            const tac_hint_t *h = &tac_hint[seat][k];
            r->hint_done = h->start_ms;
            *c = (tac_clue_t){TC_HINT, k, h->band, h->sector, h->start_ms, ((uint64_t)TC_HINT << 58) ^ ((uint64_t)k << 50) ^ h->start_ms,
                              s->x, s->z, 0.0f, 0.0f};
            return 1;
        }
    }
    return 0;
}

static void tac_start(void *host, lane_seat_t *s, int seat, uint64_t ms, ground_engagement_policy_t pol, tac_rx_t *r,
                      const tac_clue_t *c)
{
    r->clue = c->type;
    r->slot = c->slot;
    r->clue_ms = c->ms;
    r->clue_key = c->key;
    r->ox = c->ox;
    r->oz = c->oz;
    r->vx = c->vx;
    r->vz = c->vz;
    r->band = c->band;
    r->sector = c->sector;
    if (c->type == TC_HINT) {
        r->exit = tac_exit_pick(s->army, 0.0f, 0.0f, 0.0f, 0.0f, TAC_HINT_EXIT_M, 1, s->x, s->z,
                                sense_world(seat, c->band, c->sector), 180.0f / (float)sense_sectors(c->band));
    } else if (c->type == TC_ALERT) {
        const tac_alert_t *a = &tac_alert[s->army][s->task_alert - 1];
        r->exit = a->exit.ok ? a->exit : tac_exit_pick(s->army, c->ox, c->oz, c->vx, c->vz, TAC_EXIT_M, 0, 0.0f, 0.0f, 0.0f, 0.0f);
    } else {
        r->exit = tac_exit_pick(s->army, c->ox, c->oz, c->vx, c->vz, TAC_EXIT_M, 0, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    /* a tank formally holding a guard / rear-alert task (or a hold, a post) stays for that task's term */
    r->guard = s->task || cmd_is_post(s) || s->cmd_type == OB_HOLD || s->holding;
    if (ms - r->enc_last > TAC_ENC_GAP_MS)
        r->enc_spent = 0;               /* a new encounter: a fresh stop budget */
    r->enc_last = ms;
    const uint64_t budget = pol.short_stop_budget_ms;
    const char *no = NULL;
    r->wait_ok = r->peek_ok = 0;
    if (!r->exit.ok)
        no = c->type == TC_REPORT ? "only a team-mate's report, no exit fits it"
             : c->type == TC_HINT ? "only a direction, no exit in that sector" : "no exit near where it vanished";
    else if (!r->guard && (seat_rush(s) || pol.urgency == GTO_TIME_CRITICAL))
        no = "it must enter the circle now";
    else if (!(pol.allowed & GFM_HOLD))
        no = "the task allows no stop";
    else if (pol.deadline_ms && ms >= pol.deadline_ms)
        no = "the task's deadline is past";
    else if (!r->guard && (!budget || r->enc_spent >= budget))
        no = "the stop budget of this encounter is spent";
    else if (s->task) {
        r->wait_ok = 1;                 /* the task's spot (an alert: its guard spot) */
        r->wait_x = s->task_x;
        r->wait_z = s->task_z;
    } else if (r->guard) {
        r->wait_ok = 1;                 /* a hold or post waits where its order holds it */
        r->wait_x = s->x;
        r->wait_z = s->z;
    } else {
        r->wait_ok = tac_wait_spot(s, r, s->x, s->z, 1, r->exit.x, r->exit.z, &r->wait_x, &r->wait_z);
    }
    if (!no && !r->wait_ok)
        no = "no reachable wait position on its way";
    r->task_id = s->nav.task_id;
    r->revision = s->nav.revision;
    r->order = s->cmd_type;
    r->since = r->beh_since = ms;
    r->arrived_ms = 0;
    tac_move_reset(r, ms);
    if (no) {
        r->beh = TB_CONTINUE;
        r->hull = "kept its order";
        tac_log_decision(host, s, seat, ms, r, no, budget);
        return;
    }
    r->beh = TB_WAIT;
    r->hull = r->guard ? "held its task's spot" : dist2d(s->x, s->z, r->wait_x, r->wait_z) > TAC_ARRIVE_M
              ? "drove to a wait position on its way" : "stopped where it was";
    if (!r->guard && tac_peek_allowed(s, seat, ms, pol, r, NULL)) {
        r->beh = TB_PEEK;
        r->hull = "drove to a peek point";
    }
    tac_log_decision(host, s, seat, ms, r, NULL, budget);
}

static void tac_run(void *host, lane_seat_t *s, int seat, uint64_t ms, ground_engagement_policy_t pol, tac_rx_t *r, uint64_t dt)
{
    if (!r->guard)
        r->enc_spent += dt;             /* wait and peek of one encounter share the budget; no restart */
    r->enc_last = ms;
    const uint64_t budget = pol.short_stop_budget_ms;
    const tac_alert_t *a = cfg.alert && s->task_alert > 0 && s->task_alert <= ALERT_SLOTS && s->army >= 1 && s->army <= 2
                           ? &tac_alert[s->army][s->task_alert - 1] : NULL;
    const char *end = NULL;
    if (s->mode == M_ENGAGE)
        end = r->beh == TB_PEEK ? "seen from the peek point: comes out (native engagement, takeover gate)"
              : "sees an enemy: native engagement";
    else if (s->mode == M_RETREAT)
        end = "backs off under fire";
    else if (s->mode == M_PURSUE)
        end = "pursuit";
    else if (s->nav.task_id != r->task_id || s->nav.revision != r->revision)
        end = "the order changed";
    else if (pol.deadline_ms && ms >= pol.deadline_ms)
        end = "the task's deadline";
    else if (!r->guard && r->enc_spent >= budget)
        end = "the stop budget is spent";
    else if (r->guard && r->clue == TC_ALERT && (!s->task || !a || a->on != 2))
        end = "the alert task ended";
    else if (r->guard && r->clue == TC_ALERT && a->seen)
        end = "the alert area is seen again";
    else if (r->guard && r->clue != TC_ALERT && ms - r->clue_ms > TAC_GUARD_HOLD_MS)
        end = "the clue expired";
    if (end) {
        tac_end(host, s, seat, ms, r, end);
        return;
    }
    const char *why = NULL;
    if (r->beh == TB_WAIT) {
        if (s->task) {
            r->wait_x = s->task_x;      /* the task branch drives to its spot; the record follows it */
            r->wait_z = s->task_z;
        }
        const float d = dist2d(s->x, s->z, r->wait_x, r->wait_z);
        const float arrive = s->task ? INTERCEPT_HOLD_M : TAC_ARRIVE_M;
        if (d > arrive) {
            r->arrived_ms = 0;
            if (!r->guard && tac_move_failed(s, r, d, ms, &why)) {
                float wx, wz;
                tac_fail(r, r->wait_x, r->wait_z);
                if (tac_wait_spot(s, r, s->x, s->z, 1, r->exit.x, r->exit.z, &wx, &wz)) {
                    event(host, "tac: seat %d life %llu wait position (%.0f,%.0f) failed (%s): takes (%.0f,%.0f)", seat,
                          (unsigned long long)s->born_ms, r->wait_x, r->wait_z, why, wx, wz);
                    r->wait_x = wx;
                    r->wait_z = wz;
                    tac_move_reset(r, ms);
                } else {
                    tac_end(host, s, seat, ms, r, "the wait position failed and none is left");
                }
            }
            return;
        }
        if (!r->arrived_ms)
            r->arrived_ms = ms;
        if (r->guard && ms - r->arrived_ms >= TAC_PEEK_AFTER_MS && r->peeked_key != r->clue_key) {
            if (tac_peek_allowed(s, seat, ms, pol, r, &why)) {
                r->beh = TB_PEEK;
                r->beh_since = ms;
                r->arrived_ms = 0;
                r->hull = "waited, then drove to a peek point";
                tac_move_reset(r, ms);
                event(host, "tac: seat %d life %llu peeks once at (%.0f,%.0f) after %.1f s of waiting | exit %s#%d (%.0f,%.0f) | "
                      "clue %s age %.1f s", seat, (unsigned long long)s->born_ms, r->peek_x, r->peek_z,
                      (double)(ms - r->since) / 1000.0, r->exit.kind, r->exit.ref, r->exit.x, r->exit.z, tc_long[r->clue],
                      (double)(ms - r->clue_ms) / 1000.0);
            } else {
                r->peeked_key = r->clue_key;    /* asked once for this clue */
                event(host, "tac: seat %d life %llu keeps waiting, no peek: %s", seat, (unsigned long long)s->born_ms, why);
            }
        }
        return;
    }
    /* TB_PEEK */
    const float d = dist2d(s->x, s->z, r->peek_x, r->peek_z);
    if (!r->arrived_ms && d > TAC_ARRIVE_M) {
        if (tac_move_failed(s, r, d, ms, &why)) {
            tac_fail(r, r->peek_x, r->peek_z);
            r->peeked_key = r->clue_key;
            event(host, "tac: seat %d life %llu peek point (%.0f,%.0f) failed (%s)", seat, (unsigned long long)s->born_ms,
                  r->peek_x, r->peek_z, why);
            if (r->wait_ok && (r->guard || r->enc_spent < budget)) {
                r->beh = TB_WAIT;
                r->beh_since = ms;
                r->arrived_ms = 0;
                tac_move_reset(r, ms);
            } else {
                tac_end(host, s, seat, ms, r, "the peek point failed");
            }
        }
        return;
    }
    if (!r->arrived_ms)
        r->arrived_ms = ms;
    if (ms - r->arrived_ms < TAC_PEEK_LOOK_MS)
        return;                         /* the game's sight check gets its time; a sighting ends it above */
    r->peeked_key = r->clue_key;
    if (r->wait_ok && (r->guard || r->enc_spent < budget)) {
        event(host, "tac: seat %d life %llu nothing seen from the peek point: back to the wait position (%.0f,%.0f)", seat,
              (unsigned long long)s->born_ms, r->wait_x, r->wait_z);
        r->beh = TB_WAIT;
        r->beh_since = ms;
        r->arrived_ms = 0;
        tac_move_reset(r, ms);
    } else {
        tac_end(host, s, seat, ms, r, "nothing seen from the peek point");
    }
}

/* A hint clue whose sector changed is new evidence: the exit follows it (if one fits). */
static void tac_hint_follow(void *host, const lane_seat_t *s, int seat, tac_rx_t *r)
{
    const int k = tac_best_hint(s, seat);
    if (k < 0)
        return;
    const tac_hint_t *h = &tac_hint[seat][k];
    if (h->band == r->band && h->sector == r->sector)
        return;
    r->band = h->band;
    r->sector = h->sector;
    const tac_exit_t e = tac_exit_pick(s->army, 0.0f, 0.0f, 0.0f, 0.0f, TAC_HINT_EXIT_M, 1, s->x, s->z,
                                       sense_world(seat, h->band, h->sector), 180.0f / (float)sense_sectors(h->band));
    if (e.ok && (!r->exit.ok || dist2d(e.x, e.z, r->exit.x, r->exit.z) > 1.0f)) {
        r->exit = e;
        event(host, "tac: seat %d life %llu hint now sector %d/%d: watches exit %s#%d (%.0f,%.0f)", seat,
              (unsigned long long)s->born_ms, h->sector, sense_sectors(h->band), e.kind, e.ref, e.x, e.z);
    }
}

static void tac_react_tick(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    tac_rx_t *r = &tac_rx[seat];
    if (r->life != s->born_ms) {
        memset(r, 0, sizeof *r);
        r->life = s->born_ms;
        r->slot = -1;
    }
    const uint64_t dt = r->tick_ms && ms > r->tick_ms && ms - r->tick_ms < 1000 ? ms - r->tick_ms : 0;
    r->tick_ms = ms;
    /* own sightings of this life (the vanish point and last velocity of each enemy) */
    if (s->army >= 1 && s->army <= 2) {
        gh_lock(GH_LOCK_KNOW);
        for (int k = 0; k < 64; ++k) {
            const uint64_t own = sight_ms[seat & 63][k];
            if (own && ms >= own && ms - own < 1500 && slot_alive[k] && slot_army[k] != s->army && contact[s->army][k].last_ms) {
                r->own_ms[k] = ms;
                r->own_x[k] = contact[s->army][k].x;
                r->own_z[k] = contact[s->army][k].z;
                r->own_vx[k] = contact[s->army][k].vx;
                r->own_vz[k] = contact[s->army][k].vz;
            }
        }
        gh_unlock(GH_LOCK_KNOW);
    }
    const ground_engagement_policy_t pol = cmd_motion_policy(s, seat, ms);
    if (r->beh == TB_CONTINUE && ms - r->beh_since > TAC_CONT_LOOK_MS)
        r->beh = TB_NONE;
    if (r->clue == TC_HINT && r->beh != TB_NONE)
        tac_hint_follow(host, s, seat, r);
    if (r->beh == TB_WAIT || r->beh == TB_PEEK) {
        tac_run(host, s, seat, ms, pol, r, dt);
        return;
    }
    if (s->mode != M_MOVE && s->mode != M_INTERCEPT)
        return;                         /* fighting, backing off or pursuing: no reaction record */
    if (cfg.threatface && tac_face[seat].life == s->born_ms && tac_face[seat].state != TF_NONE)
        return;                         /* s29: the threat facing holds the hull */
    tac_clue_t c;
    if (tac_new_clue(s, seat, ms, r, &c))
        tac_start(host, s, seat, ms, pol, r, &c);
}

/* ---- 7. cover to cover on a plan lane (s35, WTPLAN002) ---------------------------------------------------------- *
 * A tank driving its lane does not stop at its bounds by default. It stops at the NEXT bound ahead when the command
 * layer's intel tells it of an enemy (s36: camera grade only) within BD_RANGE_M in the half-plane ahead, or it was hit
 * in the last BD_HIT_MS; it leaves when the intel episode is over (and no fresh hit) or after BD_HOLD_MS there, unless
 * its gunner is engaging. Never the only tank on its zone's circle lanes while the zone is not ours and no enemy is known
 * near it. A reaction like the wait: it holds the hull only while no fight, threat facing or wait / peek does. */
#define BD_RANGE_M 600.0f
#define BD_HIT_MS 5000u
#define BD_HOLD_MS 12000u
#define BD_AHEAD_M 150.0f
#define BD_ARRIVE_M 4.0f
#define BD_GO_MS 20000u
#define BD_AGAIN_MS 5000u
typedef struct { uint64_t life, since, arrived, last_end; int state, lane, bound, declined; char why[96]; } tac_bound_t;
static tac_bound_t tac_bound[MAX_SEATS];

static int bd_threat(const lane_seat_t *s, int seat, uint64_t ms, char *why, size_t n)
{
    const in_seat_t *in = &in_seat[seat];
    /* s36: camera grade only (a report is a clue for the turret, never a stop) */
    if (cfg.intel && in->life == s->born_ms && in->grade == IN_CAMERA && ms - in->ms <= IN_FRESH_MS && in->d <= BD_RANGE_M
        && (in->x - s->x) * s->fx + (in->z - s->z) * s->fz > 0.0f) {
        if (why)
            snprintf(why, n, "intel %s of slot %d at %.0f m ahead", in_grade_name[in->grade], in->slot, in->d);
        return 1;
    }
    const hitdir_t *h = &hit_dir[seat & 63];
    if (h->ms && ms >= h->ms && ms - h->ms < BD_HIT_MS) {
        if (why)
            snprintf(why, n, "hit %.1f s ago", (double)(ms - h->ms) / 1000.0);
        return 1;
    }
    return 0;
}

/* the only tank on its zone's circle lanes, the zone not ours and no enemy known near it: capture must not stall */
static int bd_sole_capturer(void *host, const lane_seat_t *s, int lane, uint64_t ms)
{
    const tp_lane_t *ln = &tp_lanes[lane];
    if (ln->end != TLE_CIRCLE || ln->serves < 0)
        return 0;
    const int zl = tp_state[s->army].lane[ln->serves];
    if (zl < 0 || zone_owner(host, lanes[zl].zx, lanes[zl].zz) == s->army
        || cmd_known_near(&sview[s->army], lanes[zl].zx, lanes[zl].zz, CMD_ENEMY_M, CMD_ENEMY_AGE_S))
        return 0;
    int others = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (o == s || !cmd_live(o, i, s->army, ms))
            continue;
        const int l = tp2_lane_of(s->army, o);
        others += l >= 0 && tp_lanes[l].serves == ln->serves && tp_lanes[l].end == TLE_CIRCLE;
    }
    return !others;
}

/* s36: a bound point (x, z) another live tank of the army stands at or has been sent to */
#define BD_TAKEN_M 8.0f
static int bd_taken(const lane_seat_t *s, int seat, float x, float z, uint64_t ms)
{
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (i == seat || !cmd_live(o, i, s->army, ms))
            continue;
        const tac_bound_t *ob = &tac_bound[i];
        if (ob->life == o->born_ms && ob->state && ob->lane >= 0 && ob->lane < ntp_lanes
            && dist2d(tp_lanes[ob->lane].bound[ob->bound].x, tp_lanes[ob->lane].bound[ob->bound].z, x, z) < BD_TAKEN_M)
            return 1;
        if (hypotf(o->vx, o->vz) < 0.5f && dist2d(o->x, o->z, x, z) < BD_TAKEN_M)
            return 1;
    }
    return 0;
}

static void bd_end(void *host, const lane_seat_t *s, int seat, uint64_t ms, tac_bound_t *b, const char *why)
{
    event(host, "lane: seat %d life %llu stop at bound %d of lane %s over after %.1f s (%.1f s there): %s", seat,
          (unsigned long long)s->born_ms, b->bound, tp_lanes[b->lane].id, (double)(ms - b->since) / 1000.0,
          b->arrived ? (double)(ms - b->arrived) / 1000.0 : 0.0, why);
    b->state = 0;
    b->last_end = ms;
}

static void tac_bound_tick(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    tac_bound_t *b = &tac_bound[seat];
    if (b->life != s->born_ms) {
        memset(b, 0, sizeof *b);
        b->life = s->born_ms;
    }
    if (!cfg.teamplan || tp_format != 2 || !tp_active(s->army))
        return;
    const int lane = tp2_lane_of(s->army, s);
    if (b->state) {
        const char *end = NULL;
        if (lane != b->lane)
            end = "its lane changed";
        else if (s->mode == M_RETREAT)
            end = "backs off under fire";
        else if (cfg.threatface && tac_face[seat].life == s->born_ms && tac_face[seat].state == TF_COVER)
            end = "threat facing goes to cover";    /* a turn in place at the bound belongs to the stop */
        else if (b->state == 1) {
            const tp_bound_t *q = &tp_lanes[lane].bound[b->bound];
            if (dist2d(s->x, s->z, q->x, q->z) <= BD_ARRIVE_M) {
                b->state = 2;
                b->arrived = ms;
            } else if (ms - b->since > BD_GO_MS) {
                end = "did not get there";
            }
        } else {
            const int engaging = s->gt_slot >= 0;
            if (!engaging && !bd_threat(s, seat, ms, NULL, 0))
                end = "intel episode over";
            else if (!engaging && ms - b->arrived >= BD_HOLD_MS)
                end = "12 s at the bound";
        }
        if (end)
            bd_end(host, s, seat, ms, b, end);
        return;
    }
    if (lane < 0 || s->mode == M_ENGAGE || !s->wants_move || ms - b->last_end < BD_AGAIN_MS
        || (cfg.threatface && tac_face[seat].life == s->born_ms && tac_face[seat].state != TF_NONE)
        || (cfg.waitpeek && tac_rx[seat].life == s->born_ms && (tac_rx[seat].beh == TB_WAIT || tac_rx[seat].beh == TB_PEEK)))
        return;
    char why[96];
    if (!bd_threat(s, seat, ms, why, sizeof why)) {
        b->declined = 0;    /* s36: the episode is over */
        return;
    }
    if (b->declined || bd_sole_capturer(host, s, lane, ms))
        return;
    const tp_lane_t *ln = &tp_lanes[lane];
    /* s36: one tank per bound. The bounds ahead, nearest first; a bound another live tank of the army stands at or has
     * been sent to (within BD_TAKEN_M) is taken; the nearest free one before the first taken one, else no stop. */
    int next = -1, taken = -1;
    float bd = BD_AHEAD_M, taken_d = BD_AHEAD_M;
    for (int q = 0; q < ln->nbound; ++q) {
        const float dx = ln->bound[q].x - s->x, dz = ln->bound[q].z - s->z, d = hypotf(dx, dz);
        if (d <= BD_ARRIVE_M || d >= BD_AHEAD_M || dx * s->fx + dz * s->fz <= 0.0f)
            continue;
        if (bd_taken(s, seat, ln->bound[q].x, ln->bound[q].z, ms)) {
            if (d < taken_d) {
                taken_d = d;
                taken = q;
            }
            continue;
        }
        if (d < bd) {
            bd = d;
            next = q;
        }
    }
    if (taken >= 0 && (next < 0 || bd >= taken_d)) {
        b->declined = 1;
        event(host, "lane: seat %d life %llu no stop this episode: bound %d of lane %s, %.0f m ahead, is taken and no free bound "
              "lies before it (%s)", seat, (unsigned long long)s->born_ms, taken, ln->id, taken_d, why);
        return;
    }
    if (next < 0)
        return;
    b->state = 1;
    b->lane = lane;
    b->bound = next;
    b->since = ms;
    b->arrived = 0;
    snprintf(b->why, sizeof b->why, "%s", why);
    event(host, "lane: seat %d life %llu stops at bound %d of lane %s at (%.0f,%.0f), %.0f m ahead (%.0f m along): %s", seat,
          (unsigned long long)s->born_ms, next, ln->id, ln->bound[next].x, ln->bound[next].z, bd, ln->bound[next].at, why);
}

/* tac_propose: the bound stop holds the hull (the driving side finds its way there) */
static int tac_bound_propose(lane_seat_t *s, int seat)
{
    const tac_bound_t *b = &tac_bound[seat];
    if (b->life != s->born_ms || !b->state || b->lane < 0 || b->lane >= ntp_lanes)
        return 0;
    const tp_bound_t *q = &tp_lanes[b->lane].bound[b->bound];
    const int move = dist2d(s->x, s->z, q->x, q->z) > BD_ARRIVE_M;
    nav_propose(&s->nav, NAV_REACTION, q->x, q->z, move ? NAV_NATIVE : NAV_PARK, move, !move, 0);
    return 1;
}

/* ---- the hooks ----------------------------------------------------------------------------------------------- */
/* Turret attention while idle (gaze_source): the one exit a reaction watches, else the nearest hint's sector centre.
 * Only the turret: gaze_source feeds preaim's idle gaze, which never turns the hull. */
static int tac_gaze(const lane_seat_t *s, int seat, uint64_t ms, float *brg)
{
    if (cfg.waitpeek) {
        const tac_rx_t *r = &tac_rx[seat];
        if (r->life == s->born_ms && r->exit.ok && r->beh != TB_NONE
            && (r->beh != TB_CONTINUE || ms - r->beh_since <= TAC_CONT_LOOK_MS)
            && dist2d(s->x, s->z, r->exit.x, r->exit.z) > 5.0f) {
            *brg = atan2f(r->exit.z - s->z, r->exit.x - s->x) * 57.29578f;
            return GZ_ATTN * 100;
        }
    }
    {
        /* s34: the enemy the command layer told this tank of (camera grade) */
        int slot;
        float x, z;
        if (in_anticipate(s, seat, ms, &slot, &x, &z) && dist2d(s->x, s->z, x, z) > 5.0f) {
            *brg = atan2f(z - s->z, x - s->x) * 57.29578f;
            return GZ_ATTN * 100;
        }
    }
    if (cfg.sense) {
        const int k = tac_best_hint(s, seat);
        if (k >= 0) {
            *brg = sense_world(seat, tac_hint[seat][k].band, tac_hint[seat][k].sector);
            return GZ_ATTN * 100;
        }
    }
    return 0;
}

/* react_propose: the reaction holds the hull while it waits on its way (a guard's wait is its task's own spot) or
 * peeks. NAV_REACTION goal; the driving side finds its way there. */
static int tac_propose(void *host, lane_seat_t *s, int seat, uint64_t ms, ground_engagement_policy_t policy)
{
    (void)host;
    (void)ms;
    (void)policy;
    if (tac_bound_propose(s, seat))
        return 1;                       /* s35: a stop at the next bound of its lane */
    const tac_rx_t *r = &tac_rx[seat];
    if (r->life != s->born_ms)
        return 0;
    float x, z;
    if (r->beh == TB_PEEK) {
        x = r->peek_x;
        z = r->peek_z;
    } else if (r->beh == TB_WAIT && !r->guard) {
        x = r->wait_x;
        z = r->wait_z;
    } else {
        return 0;
    }
    const int move = dist2d(s->x, s->z, x, z) > TAC_ARRIVE_M;
    nav_propose(&s->nav, NAV_REACTION, x, z, move ? NAV_NATIVE : NAV_PARK, move, !move, 0);
    return 1;
}

static void tac_gunner_cols(const lane_seat_t *s, int seat, uint64_t ms, char *out, size_t n)
{
    const tac_rx_t *r = &tac_rx[seat];
    const int live = cfg.waitpeek && r->life == s->born_ms;
    const int h = cfg.sense ? tac_best_hint(s, seat) : -1;
    const tac_face_t *f = &tac_face[seat];
    const int fl = cfg.threatface && f->life == s->born_ms && f->state != TF_NONE && s->fx * s->fx + s->fz * s->fz > 0.25f;
    char off[16] = "";
    if (fl)
        snprintf(off, sizeof off, "%.0f",
                 fabsf(wrap180(atan2f(f->tz - s->z, f->tx - s->x) * 57.29578f - atan2f(s->fz, s->fx) * 57.29578f)));
    const in_seat_t *in = &in_seat[seat];
    const int il = cfg.intel && in->life == s->born_ms && in->grade && ms - in->ms <= IN_FRESH_MS;
    snprintf(out, n, "%s,%d,%s,%s,%d,%d,%s,%s,%s,%d", rdy_name[tac_ready(s, seat, ms)],
             s->mode == M_ENGAGE && s->eng >= 0 && s->eng == s->gt_slot, live ? tb_name[r->beh] : "none",
             live ? tc_name[r->clue] : "none", h >= 0 ? tac_hint[seat][h].band : -1, h >= 0 ? tac_hint[seat][h].sector : -1,
             fl ? (f->anticipated ? "anticipated" : tf_name[f->state]) : "none", off, il ? in_grade_name[in->grade] : "none",
             il ? in->slot : -1);
}

static void tac_update(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    static int said;
    if (!said) {
        said = 1;
        event(host, "tactics: sense %d (%.0f m; vision inside vis_near %.0f m by the sight table: %s) alert %d takeover %d "
              "waitpeek %d threatface %d (cover within %.0f m)", cfg.sense, cfg.sense_m, cfg.vis_near,
              cfg.sense ? "yes" : "no (hearing)", cfg.alert, cfg.takeover, cfg.waitpeek, cfg.threatface, cfg.cover_m);
    }
    if (cfg.sense)
        tac_sense_tick(host, s, seat, ms);
    if (cfg.takeover)
        tac_ready_log(host, s, seat, ms);
    if (cfg.intel)
        in_tick(host, s->army, ms);    /* s34: the command layer's intel, twice a second per army */
    if (cfg.threatface)
        tac_face_tick(host, s, seat, ms);
    tac_bound_tick(host, s, seat, ms);     /* s35: cover to cover on a plan lane */
    if (cfg.waitpeek)
        tac_react_tick(host, s, seat, ms);
}
