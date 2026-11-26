/* s32 / s33 team plan (teamplan=1 and a plan pack for the mission; else exactly s29). Owned by the command layer, one
 * plan per army, drawn ONCE by weight at the army's first commander tick (seed from that moment, logged). Later events
 * (a zone changes owner; the army's live tanks fall below half of its roster) keep the plan; only the rule-based parts
 * follow (capture or hold, the support offers, move-ups). The plan:
 *  - team size: the army's ROSTER (its seats in this battle, spawn_seat lines), not who is alive: a zone's force is
 *    round(share x roster), fixed at the draw; of it at most the zone's capacity get the capture (or hold) objective,
 *    the rest are support slots of the zone's fire-support areas (PK_SUPPORT); dead tanks leave a need unfilled;
 *  - "skip": no objective of any kind into that zone (capture, hold, posts of its lane, anything within its circle +
 *    TP_SKIP_M), no lane default towards it (tp_apply re-points the lane of a tank without an order, roads_post_leave
 *    falls back to another zone), and a tank without its first order holds on its spawn (lanes_propose) instead of
 *    driving its lane default;
 *  - first orders come only from the plan: watch entries first (one tank of the entry's spawn area, at once), then
 *    each plan zone's capture places, then its support slots; what is left falls to the main zones by nearness;
 *  - watch: PK_WATCH slots; a watch tank is released only by its death, an empty gun, or the end of the battle;
 *  - flank: one tank of a zone's first-order wave by a flank route variant (unchanged from s32). Nothing here
 *    touches the hull. */
#define TP_FIRST_WAIT_MS 20000u     /* a tank without its first order holds this long at most, then its lane default */
#define TP_SKIP_M 150.0f            /* no objective within a skipped zone's circle + this */
#define TP_MAX_SUPPORT_SLOTS 12
/* s37 (WTPLAN002): a zone's force is min(round(share x roster), quota), at least 1; the quota is TP2_ZONE_MAX, and
 * TP2_ZONE_MAX_EFFORT for the main zone of a plan of kind main_effort. Lane slots of different lanes closer than
 * TP2_SLOT_SHARE_M are one place (a held post of either lane takes both). */
#define TP2_ZONE_MAX 4
#define TP2_ZONE_MAX_EFFORT 5
#define TP2_SLOT_SHARE_M 3.0f
/* s39 (format 2 only, driven by the owner-change event): a zone we own needs only a garrison of TP2_GARRISON (its force
 * is min(today's force, TP2_GARRISON) while ours, the full force again once lost); at the tick a zone becomes ours the
 * TP2_GARRISON tanks counted for it nearest to its centre keep it and the others are released for another order
 * (tp2_release_surplus); a skipped zone becomes an objective (contest, its quota as force) from the tick every main
 * (non-skip) zone of the plan is ours, latched for the battle (skip_open, tp_entry_role). */
#define TP2_GARRISON 2
#include <limits.h>                 /* s40 spread: INT_MAX */
enum { TPK_NONE, TPK_ZONE, TPK_WATCH, TPK_SUPPORT, TPK_LANE };

tp_army_t tp_state[3];
static int tp_seat_variant[MAX_SEATS];
int tp_seat_target[MAX_SEATS];
static int tp_lane_post_first[TP_MAX_LANES], tp_lane_post_n[TP_MAX_LANES];  /* s35: posts of support / guard lane slots */
static void tp2_area_setup(void);       /* s40, below */

static int tp_has_plans(int army)
{
    for (int i = 0; i < ntp_plans; ++i)
        if (tp_plans[i].army == army)
            return 1;
    return 0;
}

int tp_active(int army)
{
    return cfg.teamplan && army >= 1 && army <= 2 && tp_state[army].started && tp_state[army].plan >= 0;
}

static const tp_plan_t *tp_plan_of(int army)
{
    return tp_active(army) ? &tp_plans[tp_state[army].plan] : NULL;
}

/* s39: the role of plan entry j for behaviour (every behavioural read of a role goes through here): a skipped zone is
 * contested once the army's skip latch is open (format 2); otherwise the plan's role. */
static int tp_entry_role(int army, const tp_plan_t *pl, int j)
{
    const int r = pl->z[j].role;
    return r == TPR_SKIP && tp_format == 2 && army >= 1 && army <= 2 && tp_state[army].skip_open ? TPR_CONTEST : r;
}

/* role of zone z in the army's plan (s39: the effective role, tp_entry_role), -1 not in the plan */
static int tp_role(int army, int z)
{
    const tp_plan_t *pl = tp_plan_of(army);
    for (int j = 0; pl && j < pl->nz; ++j)
        if (pl->z[j].zone == z)
            return tp_entry_role(army, pl, j);
    return -1;
}

static int tp_zone_of_lane(int army, int lane)
{
    for (int z = 0; lane >= 0 && z < ntp_zones; ++z)
        if (tp_state[army].lane[z] == lane)
            return z;
    return -1;
}

static int tp_lane_skipped(int army, int lane)
{
    const int z = tp_active(army) ? tp_zone_of_lane(army, lane) : -1;
    return z >= 0 && tp_role(army, z) == TPR_SKIP;
}

/* a lane the plan does not skip: lane itself, else the main zone's lane nearest to the skipped zone */
static int tp_lane_redirect(int army, int lane)
{
    if (!tp_lane_skipped(army, lane))
        return lane;
    const int from = tp_zone_of_lane(army, lane);
    int best = lane;
    float bd = 1e18f;
    for (int z = 0; z < ntp_zones; ++z) {
        const int l = tp_state[army].lane[z];
        if (l < 0 || tp_role(army, z) != TPR_MAIN)
            continue;
        const float d = dist2d(tp_zones[z].x, tp_zones[z].z, tp_zones[from].x, tp_zones[from].z);
        if (d < bd) {
            bd = d;
            best = l;
        }
    }
    return best;
}

static int tp_watch_obj(int army, int k)
{
    return tp_active(army) && k >= 0 && k < CMD_MAX_OBJ && tp_state[army].obj_kind[k] == TPK_WATCH;
}

static int tp_plan_post(int army, int k)
{
    return tp_active(army) && k >= 0 && k < CMD_MAX_OBJ
           && (tp_state[army].obj_kind[k] == TPK_WATCH || tp_state[army].obj_kind[k] == TPK_SUPPORT
               || tp_state[army].obj_kind[k] == TPK_LANE);
}

static int tp_watch_offered(int army, const void *post)
{
    const cmd_post_t *p = (const cmd_post_t *)post;
    const tp_plan_t *pl = tp_plan_of(army);
    if (!pl || !PK_PLAN(p->kind))
        return 0;
    if (p->kind == PK_LANE)
        return tp_format == 2;      /* s35: lane slots while the plan holds */
    if (p->kind == PK_WATCH) {
        for (int w = 0; w < pl->nw; ++w)
            if (pl->w[w].area == p->ref)
                return 1;
        return 0;
    }
    for (int z = 0; z < ntp_zones; ++z)
        for (int s = 0; s < tp_zones[z].nsup[army]; ++s)
            if (tp_zones[z].sup[army][s] == p->ref && tp_state[army].need_sup[z] > 0)
                return 1;
    return 0;
}

static uint32_t tp_rand(tp_army_t *t)
{
    t->rng ^= t->rng << 13;
    t->rng ^= t->rng >> 17;
    t->rng ^= t->rng << 5;
    return t->rng;
}

static int tp_live(int army, uint64_t ms)
{
    int live = 0;
    for (int i = 1; i < MAX_SEATS; ++i)
        live += cmd_live(&lseat[i], i, army, ms);
    return live;
}

/* The army's seats in this battle (spawn_seat lines, else the lane table's seats), known before anyone is born. */
static int tp_roster(int army, uint64_t ms)
{
    int n = 0;
    for (int i = 1; i < MAX_SEATS; ++i)
        n += authored_spawn_seat_army[i] == army;
    if (!n)
        for (int i = 1; i < MAX_SEATS; ++i)
            n += seat_has_lane[i] && seat_lane[i] >= 0 && seat_lane[i] < nlanes && lanes[seat_lane[i]].army == army;
    return n ? n : tp_live(army, ms);
}

/* s37: the quota of plan entry j (format 2); s39: by its effective role */
static int tp2_quota(int army, const tp_plan_t *pl, int j)
{
    return tp_entry_role(army, pl, j) == TPR_MAIN && !strcmp(pl->kind, "main_effort") ? TP2_ZONE_MAX_EFFORT : TP2_ZONE_MAX;
}

/* s39: a plan zone that is ours needs only its garrison (format 2) */
static int tp2_garrisoned(int army, int z)
{
    return tp_format == 2 && z >= 0 && z < TP_MAX_ZONES && tp_state[army].owner[z] == army;
}

static void tp_compute_needs(int army)
{
    tp_army_t *t = &tp_state[army];
    const tp_plan_t *pl = &tp_plans[t->plan];
    memset(t->need_total, 0, sizeof t->need_total);
    memset(t->need_cap, 0, sizeof t->need_cap);
    memset(t->need_sup, 0, sizeof t->need_sup);
    for (int j = 0; j < pl->nz; ++j) {
        const int z = pl->z[j].zone;
        if (tp_entry_role(army, pl, j) == TPR_SKIP)
            continue;
        int n = (int)lroundf(pl->z[j].share * (float)t->roster);
        if (pl->z[j].role == TPR_SKIP)
            n = tp2_quota(army, pl, j);     /* s39: an opened skip zone (share 0 in the plan): its quota is its force */
        if (tp_format == 2 && n > tp2_quota(army, pl, j))
            n = tp2_quota(army, pl, j);     /* s37 G1: the zone quota */
        if (n < 1)
            n = 1;
        if (tp2_garrisoned(army, z) && n > TP2_GARRISON)
            n = TP2_GARRISON;               /* s39: a zone of ours needs only its garrison */
        const int cap = tp_zones[z].capacity > 0 && tp_zones[z].capacity < n ? tp_zones[z].capacity : n;
        int slots = 0;
        for (int s = 0; s < tp_zones[z].nsup[army]; ++s)
            slots += tp_state[army].sup_n[tp_zones[z].sup[army][s]];
        t->need_total[z] = n;
        t->need_cap[z] = cap;
        t->need_sup[z] = n - cap < slots ? n - cap : slots;     /* no more support than there are slots */
        if (tp_format == 2) {
            /* s35: the zone's force is counted on its circle + support lanes; the capture objective takes what its circle
             * lanes hold; no support-slot overflow (the lanes are the support). s37: and no more than the zone's force */
            int circle = 0;
            for (int l = 0; l < ntp_lanes; ++l)
                circle += tp_lanes[l].army == army && tp_lanes[l].serves == z && tp_lanes[l].end == TLE_CIRCLE ? tp_lanes[l].capacity : 0;
            circle = circle > 0 ? circle : 1;
            t->need_cap[z] = circle < n ? circle : n;
            t->need_sup[z] = 0;
        }
    }
}

static void tp_draw(void *host, int army, uint64_t ms)
{
    tp_army_t *t = &tp_state[army];
    float total = 0.0f;
    for (int i = 0; i < ntp_plans; ++i)
        total += tp_plans[i].army == army ? tp_plans[i].weight : 0.0f;
    const float r = (float)(tp_rand(t) >> 8) / 16777216.0f * total;
    int pick = -1;
    float acc = 0.0f;
    for (int i = 0; i < ntp_plans && pick < 0; ++i) {
        if (tp_plans[i].army != army)
            continue;
        acc += tp_plans[i].weight;
        if (r < acc)
            pick = i;
    }
    for (int i = ntp_plans - 1; i >= 0 && pick < 0; --i)
        if (tp_plans[i].army == army)
            pick = i;                   /* rounding at the top end */
    t->plan = pick;
    t->drawn_ms = ms;
    tp_compute_needs(army);
    const tp_plan_t *pl = &tp_plans[pick];
    char d[420];
    int len = snprintf(d, sizeof d, "zones");
    for (int j = 0; j < pl->nz && len < (int)sizeof d - 60; ++j) {
        const int z = pl->z[j].zone;
        if (tp_format == 2)     /* s37: the zone's force after its quota, and the capture objective's need */
            len += snprintf(d + len, sizeof d - (size_t)len, "%s %s %s %d (share x roster %ld, quota %d, capture %d)",
                            j ? "," : "", tp_zones[z].id, tp_role_name[tp_entry_role(army, pl, j)], t->need_total[z],
                            lroundf(pl->z[j].share * (float)t->roster),
                            tp_entry_role(army, pl, j) == TPR_SKIP ? 0 : tp2_quota(army, pl, j), t->need_cap[z]);
        else
            len += snprintf(d + len, sizeof d - (size_t)len, "%s %s %s %d (capture %d support %d, front corridors %d)",
                            j ? "," : "", tp_zones[z].id, tp_role_name[tp_entry_role(army, pl, j)], t->need_total[z], t->need_cap[z],
                            t->need_sup[z], tp_zones[z].front[army]);
    }
    len += snprintf(d + len, sizeof d - (size_t)len, "; watch");
    for (int w = 0; w < pl->nw && len < (int)sizeof d - 40; ++w)
        len += snprintf(d + len, sizeof d - (size_t)len, " %s<%s x%d", tp_areas[pl->w[w].area].id, tp_spawns[pl->w[w].from].id,
                        pl->w[w].count);
    len += snprintf(d + len, sizeof d - (size_t)len, "; flank");
    for (int z = 0; z < ntp_zones && len < (int)sizeof d - 8; ++z)
        if (pl->flank[z])
            len += snprintf(d + len, sizeof d - (size_t)len, " %s", tp_zones[z].id);
    event(host, "teamplan: army %d plan %s (%s) chosen: roster %d tanks; %s; why \"%s\"; drawn %.2f of %.2f, seed %u; drawn once "
          "for the battle%s", army, pl->id, pl->kind, t->roster, d, pl->why, r, total, t->seed,
          strstr(d, "delay") ? " | delay = capture with its share (a delay order with fall-back positions is a later stage)" : "");
}

static void tp_build_posts(void *host, int army, int kind, int a, int *first, int *count, int lane_hint, int *built, int *skipped)
{
    *first = -1;
    *count = 0;
    for (int s = 0; s < tp_areas[a].nslot; ++s) {
        const tp_slot_t *sl = &tp_areas[a].slot[s];
        const int lane = lane_hint >= 0 ? lane_hint : roads_nearest_lane(army, sl->x, sl->z);
        const float f = sl->face * 0.017453293f;
        const float fx = isfinite(f) ? sl->x + 100.0f * cosf(f) : sl->x, fz = isfinite(f) ? sl->z + 100.0f * sinf(f) : sl->z;
        const cmd_post_pos_t pos = {sl->x, sl->z, 1.0f, 0, fx, fz};
        const int id = lane >= 0 ? roads_add_post(army, kind, a, s, 0, lane, fx, fz, &pos, 1) : -1;
        if (id < 0) {
            ++*skipped;
            continue;
        }
        if (*first < 0)
            *first = id;
        ++*count;   /* consecutive: roads_add_post appends */
        ++*built;
        event(host, "teamplan: army %d %s area %s (cls %c) slot %d -> post %d at (%.0f,%.0f) faces %.0f deg, lane %d", army,
              kind == PK_WATCH ? "watch" : "support", tp_areas[a].id, tp_areas[a].cls, s, CMD_POST_ID_BASE + id, sl->x, sl->z,
              sl->face, lane);
    }
}

/* Once per army: zones to the army's lanes, watch and support slots as posts, roster, seed, the one draw. */
static void tp_start(void *host, int army, uint64_t ms)
{
    tp_army_t *t = &tp_state[army];
    t->started = 1;
    t->plan = -1;
    tp2_area_setup();       /* s40 */
    for (int z = 0; z < TP_MAX_ZONES; ++z) {
        t->lane[z] = -1;
        t->owner[z] = -2;
    }
    for (int z = 0; z < ntp_zones; ++z) {
        float bd = 1e9f;
        for (int k = 0; k < nlanes; ++k) {
            const lane_t *l = &lanes[k];
            const float d = dist2d(l->zx, l->zz, tp_zones[z].x, tp_zones[z].z);
            if (l->army == army && l->kind == 0 && d < fmaxf(l->zr, tp_zones[z].r) + 30.0f && d < bd) {
                bd = d;
                t->lane[z] = k;
            }
        }
    }
    int built = 0, skipped = 0;
    for (int a = 0; a < ntp_areas; ++a) {
        t->watch_first[a] = t->sup_first[a] = -1;
        t->watch_n[a] = t->sup_n[a] = 0;
        if (tp_areas[a].army != army)
            continue;
        int watch = 0, sup_zone = -1;
        for (int i = 0; i < ntp_plans && !watch; ++i)
            for (int w = 0; w < tp_plans[i].nw && !watch; ++w)
                watch = tp_plans[i].army == army && tp_plans[i].w[w].area == a;
        for (int z = 0; z < ntp_zones && sup_zone < 0; ++z)
            for (int s = 0; s < tp_zones[z].nsup[army]; ++s)
                if (tp_zones[z].sup[army][s] == a)
                    sup_zone = z;
        if (watch)
            tp_build_posts(host, army, PK_WATCH, a, &t->watch_first[a], &t->watch_n[a], -1, &built, &skipped);
        if (sup_zone >= 0)  /* its fall-back lane: the zone it supports (first named) */
            tp_build_posts(host, army, PK_SUPPORT, a, &t->sup_first[a], &t->sup_n[a], t->lane[sup_zone], &built, &skipped);
    }
    if (tp_format == 2) {
        /* s35: neighbours by id; a post per end slot of each support / guard lane of this army */
        for (int l = 0; l < ntp_lanes; ++l) {
            tp_lane_t *ln = &tp_lanes[l];
            for (int q = 0; q < ln->nnb; ++q) {
                ln->nb[q] = -1;
                for (int m = 0; m < ntp_lanes && ln->nb[q] < 0; ++m)
                    if (!strcmp(tp_lanes[m].id, ln->nb_id[q]))
                        ln->nb[q] = m;
            }
            for (int q = 0; q < ln->nop; ++q) {        /* s36: opposing lanes by id, same army only */
                ln->op[q] = -1;
                for (int m = 0; m < ntp_lanes && ln->op[q] < 0; ++m)
                    if (m != l && tp_lanes[m].army == ln->army && !strcmp(tp_lanes[m].id, ln->op_id[q]))
                        ln->op[q] = m;
            }
            if (ln->army != army)
                continue;               /* s36: the other army's lanes and their posts are its own (global arrays) */
            tp_lane_post_first[l] = -1;
            tp_lane_post_n[l] = 0;
            if (ln->end == TLE_CIRCLE)
                continue;
            for (int s = 0; s < ln->nslot; ++s) {
                const tp_slot_t *sl = &ln->slot[s];
                const int lane = ln->serves >= 0 && t->lane[ln->serves] >= 0 ? t->lane[ln->serves] : roads_nearest_lane(army, sl->x, sl->z);
                const float f = sl->face * 0.017453293f;
                const float fx = isfinite(f) ? sl->x + 100.0f * cosf(f) : sl->x, fz = isfinite(f) ? sl->z + 100.0f * sinf(f) : sl->z;
                const cmd_post_pos_t pos = {sl->x, sl->z, 1.0f, 0, fx, fz};
                const int id = lane >= 0 ? roads_add_post(army, PK_LANE, l, s, 0, lane, fx, fz, &pos, 1) : -1;
                if (id < 0) {
                    ++skipped;
                    continue;
                }
                if (tp_lane_post_first[l] < 0)
                    tp_lane_post_first[l] = id;
                ++tp_lane_post_n[l];
                ++built;
            }
        }
        int mine = 0, pairs = 0;
        for (int l = 0; l < ntp_lanes; ++l) {
            mine += tp_lanes[l].army == army;
            for (int q = 0; tp_lanes[l].army == army && q < tp_lanes[l].nop; ++q) {
                const int m = tp_lanes[l].op[q];
                int back = 0;           /* a pair named from both sides counts once */
                for (int r = 0; m >= 0 && m < l && r < tp_lanes[m].nop; ++r)
                    back |= tp_lanes[m].op[r] == l;
                pairs += m >= 0 && !back;
            }
        }
        event(host, "teamplan: army %d WTPLAN002: %d lanes of ours (%d sectors, width %.0f m), %d lane slot posts, "
              "%d opposing lane pairs", army, mine, tp_axes[army].sectors, tp_axes[army].width, built, pairs);
    }
    int mapped = 0, plans = 0;
    for (int z = 0; z < ntp_zones; ++z)
        mapped += t->lane[z] >= 0;
    for (int i = 0; i < ntp_plans; ++i)
        plans += tp_plans[i].army == army;
    t->roster = tp_roster(army, ms);
    t->seed = (uint32_t)(ms ^ (ms >> 32)) ^ (0x9e3779b9u * (uint32_t)army);
    t->rng = t->seed ? t->seed : 1u;
    event(host, "teamplan: army %d pack: %d spawn areas, %d zones (%d on our lanes), %d areas, %d plans of ours, %d items "
          "dropped while reading; watch / support posts %d (%d slots without a lane or room); roster %d, %d live now", army,
          ntp_spawns, ntp_zones, mapped, ntp_areas, plans, tp_dropped, built, skipped, t->roster, tp_live(army, ms));
    tp_draw(host, army, ms);
    if (tp_format == 2 && t->plan >= 0) {
        int old = 0;    /* s41: the posts of the older mechanisms (built, logged, never offered while the plan holds) */
        for (int j = 0; j < cmd_nposts[army]; ++j)
            old += !PK_PLAN(cmd_posts[army][j].kind);
        event(host, "teamplan: army %d plan posts only: %d old posts (command and roads) are not offered while the plan "
              "holds", army, old);
    }
}

/* The spawn area of a life (by the pose it stood on), -1 unknown. */
static int tp_spawn_area(const lane_seat_t *s, int army)
{
    const int pose = s->spawn_pose1 - 1;
    for (int a = 0; pose >= 0 && a < ntp_spawns; ++a)
        if (tp_spawns[a].army == army)
            for (int k = 0; k < tp_spawns[a].n; ++k)
                if (tp_spawns[a].pose[k] == pose)
                    return a;
    return -1;
}

int tp_post_held(int army, int id)
{
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *q = &lseat[i];
        if (q->army != army || !q->unit || q->scene_epoch != ground_scene_epoch)
            continue;
        if ((q->cmd_type == OB_OVERWATCH && q->cmd_id == id) || (q->cmd_apply_ms && q->cmd_ntype == OB_OVERWATCH && q->cmd_nid == id))
            return 1;
    }
    return 0;
}

static int tp_first_order_seat(const lane_seat_t *o, int seat)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    return o->cmd_type == OB_NONE && !st->first_accepted_at_ms && !st->pending.task_id;
}

/* lanes_propose: a tank of a plan army without its first order holds where it stands (no lane default) */
int tp_wait_first_order(const lane_seat_t *s, int seat, uint64_t ms)
{
    return cfg.teamplan && cmd_on() && s->army >= 1 && s->army <= 2 && tp_has_plans(s->army)
           && (!tp_state[s->army].started || tp_state[s->army].plan >= 0)
           && tp_first_order_seat(s, seat) && ms - s->born_ms < TP_FIRST_WAIT_MS;
}

/* Offer the free slots (held ones first) of `first..first+n` up to `count`; returns how many were offered. */
static int tp_offer_slots(int army, uint64_t ms, cmd_obj_t *ob, int *n, int first, int count_slots, int want, int kind, int ref,
                          float value)
{
    tp_army_t *t = &tp_state[army];
    int offered = 0;
    for (int pass = 0; pass < 2; ++pass)        /* held slots first, then free ones */
        for (int s = 0; first >= 0 && s < count_slots && (pass == 0 || offered < want) && *n < CMD_MAX_OBJ; ++s) {
            /* s34: a held slot is always offered (a crew moving to another slot of its area holds two for a moment) */
            const int j = first + s, id = CMD_POST_ID_BASE + j;
            const cmd_post_t *p = &cmd_posts[army][j];
            const int held = tp_post_held(army, id), option = cmd_post_option(p, ms);
            if ((pass == 0) != held || (!held && option < 0))
                continue;
            const int opt = option >= 0 ? option : 0;
            t->obj_kind[*n] = kind;
            t->obj_ref[*n] = ref;
            if (kind == TPK_SUPPORT)
                t->post_zone[j] = ref;
            ob[(*n)++] = (cmd_obj_t){OB_OVERWATCH, id, 1, strat_band(strat_lat(p->pos[opt].x, p->pos[opt].z)), p->lane, 0,
                                     p->cap, p->pos[opt].x, p->pos[opt].z, value, 0};
            ++offered;
        }
    return offered;
}

/* ---- s35 WTPLAN002: lanes -------------------------------------------------------------------------------------- */
/* The plan lane a tank is on now: its life was given lane L and its active or pending order is that lane's (the zone's
 * capture / hold for a circle lane, a slot post of L otherwise). -1: none (the lane is gone). */
int tp2_lane_of(int army, const lane_seat_t *o)
{
    if (tp_format != 2 || o->plan_lane1 <= 0 || o->plan_lane1 > ntp_lanes)
        return -1;
    const int l = o->plan_lane1 - 1;
    const tp_lane_t *ln = &tp_lanes[l];
    if (ln->army != army)
        return -1;
    if (ln->end == TLE_CIRCLE) {
        const int zl = ln->serves >= 0 ? tp_state[army].lane[ln->serves] : -1;
        if (zl < 0)
            return -1;
        const int now = (o->cmd_type == OB_CAPTURE || o->cmd_type == OB_HOLD) && o->cmd_id == zl;
        const int next = o->cmd_apply_ms && (o->cmd_ntype == OB_CAPTURE || o->cmd_ntype == OB_HOLD) && o->cmd_nid == zl;
        return now || next ? l : -1;
    }
    for (int s = 0; s < tp_lane_post_n[l]; ++s) {
        const int id = CMD_POST_ID_BASE + tp_lane_post_first[l] + s;
        if ((o->cmd_type == OB_OVERWATCH && o->cmd_id == id) || (o->cmd_apply_ms && o->cmd_ntype == OB_OVERWATCH && o->cmd_nid == id))
            return l;
    }
    return -1;
}

/* s37 G2: the zone an order counts for: the capture / hold of a zone (with or without a plan lane), or a slot post of a
 * support lane that serves the zone. -1 none. */
static int tp2_order_zone(int army, int type, int id)
{
    if (type == OB_CAPTURE || type == OB_HOLD)
        return tp_zone_of_lane(army, id);
    if (type == OB_OVERWATCH && id >= CMD_POST_ID_BASE && id - CMD_POST_ID_BASE < cmd_nposts[army]) {
        const cmd_post_t *p = &cmd_posts[army][id - CMD_POST_ID_BASE];
        if (p->kind == PK_LANE && p->ref >= 0 && p->ref < ntp_lanes && tp_lanes[p->ref].end == TLE_SUPPORT)
            return tp_lanes[p->ref].serves;
    }
    return -1;
}

/* s37 G2: the zone a tank counts for, once: its pending order's when it has one (where it is going), else its active
 * order's. -1 none. */
static int tp2_seat_zone(int army, const lane_seat_t *o)
{
    return o->cmd_apply_ms ? tp2_order_zone(army, o->cmd_ntype, o->cmd_nid) : tp2_order_zone(army, o->cmd_type, o->cmd_id);
}

/* ---- s40 areas (format 2): no crowd at one place ------------------------------------------------------------ *
 * The area of a point: the zone whose centre is nearest, when the point lies within that zone's area radius (half the
 * distance from its centre to the nearest other zone centre; a single zone: its circle radius + 100 m), else none.
 * The area count of an army: its live tanks whose destination (pending order, else active) lies in the area: a capture /
 * hold of zone z counts for z, a post order by the post's standing position (the order's point), a stay order by where
 * the tank stands; other orders not at all. The cap: the zone's quota by its effective role (tp2_quota). A tank not
 * counted in area a gets no destination there while a is at its cap (tp2_area_open). */
static float tp2_area_r[TP_MAX_ZONES];
static int tp2_area_n = -1;             /* the zone count the radii were made for (-1: not yet) */

/* tp_start (each army's start of the battle): the area radii from the pack's zones */
static void tp2_area_setup(void)
{
    for (int z = 0; z < ntp_zones && z < TP_MAX_ZONES; ++z) {
        float nd = 1e18f;
        for (int q = 0; q < ntp_zones; ++q)
            if (q != z)
                nd = fminf(nd, dist2d(tp_zones[z].x, tp_zones[z].z, tp_zones[q].x, tp_zones[q].z));
        tp2_area_r[z] = ntp_zones > 1 ? 0.5f * nd : tp_zones[z].r + 100.0f;
    }
    tp2_area_n = ntp_zones;
}

/* the area of point (x, z), -1 none */
int tp2_area_at(float x, float z)
{
    if (tp_format != 2 || tp2_area_n != ntp_zones)
        return -1;
    int best = -1;
    float bd = 1e18f;
    for (int q = 0; q < ntp_zones && q < TP_MAX_ZONES; ++q) {
        const float d = dist2d(x, z, tp_zones[q].x, tp_zones[q].z);
        if (d < bd) {
            bd = d;
            best = q;
        }
    }
    return best >= 0 && bd <= tp2_area_r[best] ? best : -1;
}

/* the area an order's destination lies in: capture / hold of a lane point (its plan zone, else the point's area), a post
 * by its standing position (x, z), a stay by where the tank stands (sx, sz); -1 any other order */
static int tp2_order_area(int army, int type, int id, float x, float z, float sx, float sz)
{
    if (type == OB_CAPTURE || type == OB_HOLD) {
        const int zone = tp_zone_of_lane(army, id);
        return zone >= 0 ? zone : id >= 0 && id < nlanes ? tp2_area_at(lanes[id].zx, lanes[id].zz) : -1;
    }
    if (type == OB_OVERWATCH && id >= CMD_POST_ID_BASE)
        return tp2_area_at(x, z);
    if (type == OB_STAY)
        return tp2_area_at(sx, sz);
    return -1;
}

/* the area a tank counts for: its pending order's when it has one, else its active order's (as tp2_seat_zone) */
static int tp2_seat_area(int army, const lane_seat_t *o)
{
    return o->cmd_apply_ms ? tp2_order_area(army, o->cmd_ntype, o->cmd_nid, o->cmd_nx, o->cmd_nz, o->x, o->z)
                           : tp2_order_area(army, o->cmd_type, o->cmd_id, o->cmd_x, o->cmd_z, o->x, o->z);
}

/* the cap of area a: its zone's quota by the effective role; a zone the plan does not name: TP2_ZONE_MAX */
static int tp2_area_cap(int army, int a)
{
    const tp_plan_t *pl = tp_plan_of(army);
    for (int j = 0; pl && j < pl->nz; ++j)
        if (pl->z[j].zone == a)
            return tp2_quota(army, pl, j);
    return TP2_ZONE_MAX;
}

/* the area counts of the army now (live tanks) */
void tp2_area_count(int army, uint64_t ms, int *area_have)
{
    memset(area_have, 0, sizeof(int) * TP_MAX_ZONES);
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms))
            continue;
        const int a = tp2_seat_area(army, o);
        if (a >= 0)
            ++area_have[a];
    }
}

/* THE test (s40): may tank `seat` (0: no particular tank, an offer) be given a destination in area a, with the area
 * counts area_have? Yes outside format 2, outside any area, when the tank is counted in a already, or while a is below
 * its cap. */
int tp2_area_open(int army, int seat, int a, const int *area_have)
{
    if (!tp_active(army) || tp_format != 2 || a < 0)
        return 1;
    if (seat > 0 && seat < MAX_SEATS && tp2_seat_area(army, &lseat[seat]) == a)
        return 1;
    return area_have[a] < tp2_area_cap(army, a);
}

/* the area of slot s of lane l: a circle lane's is its zone's area, any other lane's the slot's position */
static int tp2_slot_area(int l, int s)
{
    const tp_lane_t *ln = &tp_lanes[l];
    if (ln->end == TLE_CIRCLE)
        return ln->serves;
    return s >= 0 && s < ln->nslot ? tp2_area_at(ln->slot[s].x, ln->slot[s].z) : -1;
}

/* "not taken: area at its cap": once per tank and commander tick */
static uint64_t tp2_cap_note_ms[MAX_SEATS];
static void tp2_cap_note(void *host, int army, int seat, const char *what, int a, uint64_t ms)
{
    if (seat <= 0 || seat >= MAX_SEATS || tp2_cap_note_ms[seat] == ms)
        return;
    tp2_cap_note_ms[seat] = ms;
    event(host, "teamplan: army %d seat %d %s not taken: area %s at its cap %d", army, seat, what, tp_zones[a].id,
          tp2_area_cap(army, a));
}

/* s37: zone z is at its quota (a plan zone that is not skipped; a zone the plan does not name has none) */
static int tp2_at_quota(int army, int z, const int *zone_have)
{
    const int role = z >= 0 ? tp_role(army, z) : -1;
    return role >= 0 && role != TPR_SKIP && zone_have[z] >= tp_state[army].need_total[z];
}

/* s37 G4: giving this tank the capture / hold of zone z would take the zone over its quota: the zone is at its quota and
 * the tank is not counted for it already (its order is that zone's objective, or a support lane slot of it). */
static int tp2_zone_full(int army, int z, const lane_seat_t *o, const int *zone_have)
{
    if (!tp_active(army) || tp_format != 2 || z < 0 || tp2_seat_zone(army, o) == z)
        return 0;
    return tp2_at_quota(army, z, zone_have);
}

/* s40: lane l serves a plan zone (circle or support lane) whose capture / hold need is still open: the zone is below its
 * quota, or tank `seat` (0: none) is counted for it already. Such a place is not barred by its area's cap (rule (2) of
 * the lane rule: the point comes first). */
static int tp2_zone_first(int army, int l, int seat, const int *zone_have)
{
    const int z = tp_lanes[l].serves;
    if (z < 0 || (tp_lanes[l].end != TLE_CIRCLE && tp_lanes[l].end != TLE_SUPPORT))
        return 0;
    const int role = tp_role(army, z);
    if (role < 0 || role == TPR_SKIP)
        return 0;
    return !tp2_at_quota(army, z, zone_have) || (seat > 0 && tp2_seat_zone(army, &lseat[seat]) == z);
}

/* Tanks on each lane now (alive, ordered to it), and on each zone (s37 G2: tp2_seat_zone), and per sector (lanes). */
static void tp2_count(int army, uint64_t ms, int *occ, int *zone_have, int *sector_have)
{
    memset(occ, 0, sizeof(int) * TP_MAX_LANES);
    memset(zone_have, 0, sizeof(int) * TP_MAX_ZONES);
    memset(sector_have, 0, sizeof(int) * 16);
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms))
            continue;
        const int z = tp2_seat_zone(army, o);
        if (z >= 0)
            ++zone_have[z];
        const int l = tp2_lane_of(army, o);
        if (l < 0)
            continue;
        ++occ[l];
        if (tp_lanes[l].sector >= 0 && tp_lanes[l].sector < 16)
            ++sector_have[tp_lanes[l].sector];
    }
}

/* s37 G4, a path outside the commander's tick (a post crew sent back to its zone): the capture / hold of lane point `lane`
 * would take the zone over its quota for this tank. */
static int tp2_resume_barred(int army, int seat, int lane, uint64_t ms)
{
    if (!tp_active(army) || tp_format != 2)
        return 0;
    const int z = tp_zone_of_lane(army, lane);
    if (z < 0)
        return 0;
    int occ[TP_MAX_LANES], zh[TP_MAX_ZONES], sh[16];
    tp2_count(army, ms, occ, zh, sh);
    return tp2_zone_full(army, z, &lseat[seat], zh);
}

/* s37 G4, cmd_tick: the zone counts the fill tests (tp_fill_excluded): tp2_count's, plus this tick's picks so far (an
 * order to a zone the tank is not counted for adds one; leaving a zone is counted once its order is out). */
static void tp2_quota_take(int army, const cmd_obj_t *g, int seat, int *zq)
{
    const int z = tp_active(army) && tp_format == 2 ? tp2_order_zone(army, g->type, g->id) : -1;
    if (z >= 0 && z != tp2_seat_zone(army, &lseat[seat]))
        ++zq[z];
    /* s40: and the area count (an order into an area the tank is not counted for adds one) */
    const lane_seat_t *o = &lseat[seat];
    const int a = tp_active(army) && tp_format == 2 ? tp2_order_area(army, g->type, g->id, g->x, g->z, o->x, o->z) : -1;
    if (a >= 0 && a != tp2_seat_area(army, o))
        ++tp_state[army].area_run[a];
}

static void tp2_quota_have(int army, uint64_t ms, const cmd_obj_t *ob, int nc, const int *seat_of, const int *pick, int *zq)
{
    memset(zq, 0, sizeof(int) * TP_MAX_ZONES);
    memset(tp_state[army].area_run, 0, sizeof tp_state[army].area_run);
    if (!tp_active(army) || tp_format != 2)
        return;
    int occ[TP_MAX_LANES], sh[16];
    tp2_count(army, ms, occ, zq, sh);
    tp2_area_count(army, ms, tp_state[army].area_run);     /* s40 */
    for (int u = 0; u < nc; ++u)
        if (pick[u] >= 0)
            tp2_quota_take(army, &ob[pick[u]], seat_of[u], zq);
}

static int tp2_lane_usable(int army, int l)
{
    const tp_lane_t *ln = &tp_lanes[l];
    return ln->army == army && (ln->serves < 0 || tp_role(army, ln->serves) != TPR_SKIP)
           && (ln->end != TLE_CIRCLE || (ln->serves >= 0 && tp_state[army].lane[ln->serves] >= 0))
           && (ln->end == TLE_CIRCLE || tp_lane_post_n[l] > 0);
}

/* s37 G5: a lane-slot post (PK_LANE) within TP2_SLOT_SHARE_M of (x, z), of any lane of the army */
static int tp2_lane_post_near(int army, int id, float x, float z)
{
    const int j = id - CMD_POST_ID_BASE;
    if (j < 0 || j >= cmd_nposts[army] || cmd_posts[army][j].kind != PK_LANE)
        return 0;
    const cmd_post_t *p = &cmd_posts[army][j];
    return dist2d(p->pos[0].x, p->pos[0].z, x, z) <= TP2_SLOT_SHARE_M;
}

/* s37 G5, the one free-slot rule: slot s of lane l is used when a tank of the army holds (active or pending overwatch
 * order) a lane-slot post within TP2_SLOT_SHARE_M of it, whichever lane that post belongs to (lanes from different spawn
 * areas may end at the same place), or is on (or picked this tick for) a lane slot within TP2_SLOT_SHARE_M of it (circle
 * lanes have no posts); and, as before, when the lane's own post for it has no usable position. `skip`: a seat left out
 * (in_hop: the tank that moves), 0 none. */
int tp2_slot_used(int army, int l, int s, uint64_t ms, int skip)
{
    const tp_lane_t *ln = &tp_lanes[l];
    const float x = ln->slot[s].x, z = ln->slot[s].z;
    int used = 0;
    for (int i = 1; i < MAX_SEATS && !used; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (i == skip || o->army != army || !o->unit || o->scene_epoch != ground_scene_epoch)
            continue;               /* tp_post_held's holder test */
        used = (o->cmd_type == OB_OVERWATCH && tp2_lane_post_near(army, o->cmd_id, x, z))
               || (o->cmd_apply_ms && o->cmd_ntype == OB_OVERWATCH && tp2_lane_post_near(army, o->cmd_nid, x, z));
        /* s36: also a pick of this tick (its order is not issued yet, so tp2_lane_of does not see it) */
        const int m = o->plan_lane1 - 1;
        if (!used && m >= 0 && m < ntp_lanes && o->plan_slot >= 0 && o->plan_slot < tp_lanes[m].nslot
            && tp_lanes[m].army == army && cmd_live(o, i, army, ms)
            && dist2d(tp_lanes[m].slot[o->plan_slot].x, tp_lanes[m].slot[o->plan_slot].z, x, z) <= TP2_SLOT_SHARE_M)
            used = tp2_lane_of(army, o) == m || o->plan_lane_ms == ms;
    }
    if (ln->end != TLE_CIRCLE && s < tp_lane_post_n[l])
        used = used || cmd_post_option(&cmd_posts[army][tp_lane_post_first[l] + s], ms) < 0;
    return used;
}

/* A free end slot of lane l (tp2_slot_used), -1 none. */
static int tp2_free_slot(int army, int l, uint64_t ms)
{
    for (int s = 0; s < tp_lanes[l].nslot; ++s)
        if (!tp2_slot_used(army, l, s, ms, 0))
            return s;
    return -1;
}

/* s36 opposing lanes: the lane of the army met head-on in the same passage that has a live tank on it now (either side
 * may name the pair), -1 none. */
static int tp2_opposer(int l, const int *occ)
{
    const tp_lane_t *ln = &tp_lanes[l];
    for (int q = 0; q < ln->nop; ++q)
        if (ln->op[q] >= 0 && occ[ln->op[q]] > 0)
            return ln->op[q];
    for (int m = 0; m < ntp_lanes; ++m)
        for (int q = 0; m != l && occ[m] > 0 && q < tp_lanes[m].nop; ++q)
            if (tp_lanes[m].op[q] == l)
                return m;
    return -1;
}

/* s36: THE free-place rule of a lane (tp2_choose and tp2_street): below its capacity, a free end slot, and no lane it
 * opposes in use. The slot, -1 no place, TP2_OPPOSED the place is free but an opposing lane is in use. tp2_no_oppose:
 * the opposing check left out (only to find out whether that check is what excluded a tank's lane, for the log). */
#define TP2_OPPOSED (-2)
static int tp2_no_oppose;
static int tp2_place(int army, int l, const int *occ, uint64_t ms)
{
    if (occ[l] >= tp_lanes[l].capacity)
        return -1;
    const int slot = tp2_free_slot(army, l, ms);
    if (slot < 0)
        return -1;
    return !tp2_no_oppose && tp2_opposer(l, occ) >= 0 ? TP2_OPPOSED : slot;
}

/* s36: "opposed" logged once per tank life and lane */
static uint64_t tp2_opp_life[MAX_SEATS];
static int tp2_opp_lane[MAX_SEATS];
static void tp2_opposed_note(void *host, int army, int seat, int l, const int *occ)
{
    const lane_seat_t *o = &lseat[seat];
    const int by = tp2_opposer(l, occ);
    if (by < 0 || (tp2_opp_life[seat] == o->born_ms && tp2_opp_lane[seat] == l))
        return;
    tp2_opp_life[seat] = o->born_ms;
    tp2_opp_lane[seat] = l;
    event(host, "teamplan: army %d seat %d lane %s not taken: opposed by lane %s in use", army, seat, tp_lanes[l].id,
          tp_lanes[by].id);
}

/* tp_apply: the slots of every usable support / guard lane: those held, and one free slot of a lane with a free place.
 * s37 G5: the free slot is tp2_free_slot's (the one tp2_choose / tp2_street pick, a slot shared with another lane's
 * held post is not free). */
static void tp2_offer_lanes(int army, uint64_t ms, cmd_obj_t *ob, int *n)
{
    int occ[TP_MAX_LANES], zh[TP_MAX_ZONES], sh[16], ah[TP_MAX_ZONES];
    tp2_count(army, ms, occ, zh, sh);
    tp2_area_count(army, ms, ah);       /* s40 */
    for (int l = 0; l < ntp_lanes && *n < CMD_MAX_OBJ; ++l) {
        if (!tp2_lane_usable(army, l) || tp_lanes[l].end == TLE_CIRCLE)
            continue;
        for (int s = 0; s < tp_lane_post_n[l]; ++s)
            if (tp_post_held(army, CMD_POST_ID_BASE + tp_lane_post_first[l] + s))
                tp_offer_slots(army, ms, ob, n, tp_lane_post_first[l] + s, 1, 1, TPK_LANE, l + 1, TP_SUPPORT_VALUE);
        const int fs = occ[l] < tp_lanes[l].capacity ? tp2_free_slot(army, l, ms) : -1;
        /* s40: a free slot in an area at its cap is not offered (held slots are, above); except a support lane's slot
         * of a zone below its quota, which rule (2) of the lane rule may still give (the point comes first) */
        if (fs >= 0 && fs < tp_lane_post_n[l] && (tp2_zone_first(army, l, 0, zh) || tp2_area_open(army, 0, tp2_slot_area(l, fs), ah)))
            tp_offer_slots(army, ms, ob, n, tp_lane_post_first[l] + fs, 1, 1, TPK_LANE, l + 1, TP_SUPPORT_VALUE);
    }
}

/* s39 (format 2), tp_apply at the tick zone z becomes ours: of the live tanks counted for z (tp2_seat_zone: active or
 * pending capture / hold of z, or a slot post of a support lane of z) the TP2_GARRISON nearest to its centre keep it;
 * every other one is released for another order now, by the s37 mechanism: a pending order is cancelled
 * (cmd_clear_pending), the active order is invalidated (capture of z: capture_complete, else superseded) and a release
 * is published (cmd_publish_release); once the release is accepted the tank has no order and no first acceptance
 * (cmd_clear_active), so tp_first_order_seat holds and tp2_assign gives it a lane by the lane rule before the fill. A
 * tank with only a pending order (none active) is a first-order tank at once, in this tick. */
static void tp2_release_surplus(void *host, int army, int z, uint64_t ms)
{
    int seat[MAX_SEATS], n = 0;
    float dist[MAX_SEATS];
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms) || tp2_seat_zone(army, o) != z)
            continue;
        const float d = dist2d(o->x, o->z, tp_zones[z].x, tp_zones[z].z);
        int at = n++;
        while (at > 0 && dist[at - 1] > d) {   /* nearest first; equal distances: the lower seat first */
            seat[at] = seat[at - 1];
            dist[at] = dist[at - 1];
            --at;
        }
        seat[at] = i;
        dist[at] = d;
    }
    if (n <= TP2_GARRISON)
        return;
    char kept[64] = "";
    for (int k = 0, len = 0; k < TP2_GARRISON && len < (int)sizeof kept - 8; ++k)
        len += snprintf(kept + len, sizeof kept - (size_t)len, "%s%d", k ? " " : "", seat[k]);
    for (int k = TP2_GARRISON; k < n; ++k) {
        const int i = seat[k];
        lane_seat_t *o = &lseat[i];
        cmd_task_state_t *st = &cmd_tasks[i];
        if (st->pending.task_id) {
            cmd_task_event(host, "pending_cancelled", &st->pending, ms, GTR_SUPERSEDED);
            cmd_clear_pending(o, st);
        }
        if (st->active.task_id) {
            cmd_invalidate(host, st, ms, st->active.type == OB_CAPTURE && tp_zone_of_lane(army, st->active.objective) == z
                                          ? GTR_CAPTURE_COMPLETE : GTR_SUPERSEDED);
            cmd_publish_release(host, o, i, ms);
        }
        event(host, "teamplan: army %d seat %d released from zone %s (ours now, garrison %d kept: seats %s)", army, i,
              tp_zones[z].id, TP2_GARRISON, kept);
    }
}

/* cmd_tick, right after cmd_objectives: spawn poses, lanes, events, the needs, skip, watch and support offers. */
static void tp_apply(void *host, int army, uint64_t ms, cmd_obj_t *ob, int *nob)
{
    tp_army_t *t = &tp_state[army];
    memset(t->obj_kind, 0, sizeof t->obj_kind);
    memset(t->obj_ref, 0, sizeof t->obj_ref);
    if (!cfg.teamplan || army < 1 || army > 2 || !tp_has_plans(army))
        return;
    const gr_lib_t *lib = routes_library(host);
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!lib || o->spawn_pose1 || o->army != army || !cmd_live(o, i, army, ms) || o->cmd_type != OB_NONE
            || hypotf(o->vx, o->vz) >= 1.0f)
            continue;
        const int pose = gr_pose_of(lib, o->x, o->z, atan2f(o->fz, o->fx), 3.0f, 20.0f);
        if (pose >= 0)
            o->spawn_pose1 = pose + 1;
    }
    if (!t->started)
        tp_start(host, army, ms);
    if (t->plan < 0)
        return;
    const tp_plan_t *pl = &tp_plans[t->plan];
    /* events keep the plan (one draw per battle) */
    int gained[TP_MAX_ZONES] = {0}, changed[TP_MAX_ZONES] = {0}, old_need[TP_MAX_ZONES];
    for (int z = 0; z < ntp_zones; ++z) {
        old_need[z] = t->need_total[z];
        if (t->lane[z] < 0)
            continue;
        const int owner = zone_owner(host, lanes[t->lane[z]].zx, lanes[t->lane[z]].zz);
        if (t->owner[z] != -2 && owner != t->owner[z]) {
            event(host, "teamplan: army %d event: zone %s owner %d -> %d; plan %s kept (capture / hold follow the owner)", army,
                  tp_zones[z].id, t->owner[z], owner, pl->id);
            changed[z] = 1;
            gained[z] = owner == army;
        }
        t->owner[z] = owner;
    }
    if (tp_format == 2) {
        /* s39 c: the skip latch opens the tick every main (non-skip) zone of the plan is ours; it stays open */
        int mains = 0, ours = 0, skips = 0;
        for (int j = 0; j < pl->nz; ++j) {
            const int z = pl->z[j].zone;
            skips += pl->z[j].role == TPR_SKIP;
            if (pl->z[j].role == TPR_SKIP)
                continue;
            ++mains;
            ours += t->owner[z] == army;
        }
        if (!t->skip_open && skips && mains && ours == mains) {
            t->skip_open = 1;
            for (int j = 0; j < pl->nz; ++j)
                if (pl->z[j].role == TPR_SKIP)
                    event(host, "teamplan: army %d event: every main zone is ours; skipped zone %s becomes an objective "
                          "(contest, quota %d)", army, tp_zones[pl->z[j].zone].id, tp2_quota(army, pl, j));
        }
        /* s39 a: the needs follow the owner (garrison while ours) and the latch, every tick */
        tp_compute_needs(army);
        for (int z = 0; z < ntp_zones; ++z)
            if (t->need_total[z] != old_need[z])
                event(host, "teamplan: army %d zone %s force %d -> %d (%s)", army, tp_zones[z].id, old_need[z],
                      t->need_total[z], tp2_garrisoned(army, z) ? "ours: garrison" : changed[z] ? "not ours: full force"
                      : "skipped zone opened");
        /* s39 b: at the capture, the surplus of the zone is released for another order before the plan's assignment */
        for (int z = 0; z < ntp_zones; ++z) {
            const int role = tp_role(army, z);
            if (gained[z] && role >= 0 && role != TPR_SKIP)
                tp2_release_surplus(host, army, z, ms);
        }
    }
    const int live = tp_live(army, ms);
    if (!t->half_fired && t->roster && 2 * live < t->roster) {
        t->half_fired = 1;
        event(host, "teamplan: army %d event: live tanks %d below half of the roster %d; plan %s kept", army, live, t->roster,
              pl->id);
    }
    /* a tank without an order whose lane leads into a skipped zone: another lane (and its route) */
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (o->army != army || !cmd_live(o, i, army, ms) || o->cmd_type != OB_NONE || o->lane < 0 || !tp_lane_skipped(army, o->lane))
            continue;
        const int to = tp_lane_redirect(army, o->lane);
        if (to == o->lane)
            continue;
        event(host, "teamplan: army %d seat %d lane %d leads into a skipped zone: lane %d instead", army, i, o->lane, to);
        o->lane = to;
        lane_route(o, &lanes[to]);
        pick_troute(host, o, i);
        lane_begin_step(o, i, ms);
    }
    /* needs (capacity-bound) of the plan zones; nothing into a skipped zone */
    int n = 0;
    for (int k = 0; k < *nob; ++k) {
        cmd_obj_t o = ob[k];
        int skip = 0, zone = -1;
        if (o.type == OB_CAPTURE || o.type == OB_HOLD)
            zone = tp_zone_of_lane(army, o.id);
        else if (o.type == OB_OVERWATCH && o.id >= CMD_POST_ID_BASE && o.id - CMD_POST_ID_BASE < cmd_nposts[army])
            skip = tp_lane_skipped(army, cmd_posts[army][o.id - CMD_POST_ID_BASE].lane)
                   || tp2_old_post_off(army, &cmd_posts[army][o.id - CMD_POST_ID_BASE]);    /* s41: pack posts only */
        for (int z = 0; z < ntp_zones && !skip; ++z)
            skip = tp_role(army, z) == TPR_SKIP && dist2d(o.x, o.z, tp_zones[z].x, tp_zones[z].z) < tp_zones[z].r + TP_SKIP_M;
        if (zone >= 0 && tp_role(army, zone) == TPR_SKIP)
            skip = 1;
        if (skip)
            continue;
        if (zone >= 0 && tp_role(army, zone) >= 0) {
            o.need = t->need_cap[zone] > 0 ? t->need_cap[zone] : o.need;
            t->obj_kind[n] = TPK_ZONE;
            t->obj_ref[n] = zone + 1;
        }
        ob[n++] = o;
    }
    /* watch entries, then the support slots of each plan zone */
    for (int w = 0; w < pl->nw && n < CMD_MAX_OBJ; ++w)
        tp_offer_slots(army, ms, ob, &n, t->watch_first[pl->w[w].area], t->watch_n[pl->w[w].area], pl->w[w].count, TPK_WATCH,
                       pl->w[w].from + 1, TP_WATCH_VALUE);
    for (int z = 0; z < ntp_zones; ++z) {
        int want = t->need_sup[z];
        for (int s = 0; s < tp_zones[z].nsup[army] && want > 0 && n < CMD_MAX_OBJ; ++s) {
            const int a = tp_zones[z].sup[army][s];
            want -= tp_offer_slots(army, ms, ob, &n, t->sup_first[a], t->sup_n[a], want, TPK_SUPPORT, z + 1, TP_SUPPORT_VALUE);
        }
    }
    if (tp_format == 2)
        tp2_offer_lanes(army, ms, ob, &n);  /* s35: the slots of support / guard lanes */
    *nob = n;
}

/* A seat may take objective k. A watch slot: it holds it, or its life started in the entry's spawn area and it has had
 * no order this life. A first order: only plan objectives (zones, watch, support). Anything else: as before. */
static int tp_allowed(int army, int k, const cmd_obj_t *g, int seat, const lane_seat_t *o)
{
    if (!tp_active(army))
        return 1;
    const tp_army_t *t = &tp_state[army];
    if (t->obj_kind[k] == TPK_WATCH) {
        if ((o->cmd_type == OB_OVERWATCH && o->cmd_id == g->id) || (o->cmd_apply_ms && o->cmd_ntype == OB_OVERWATCH && o->cmd_nid == g->id))
            return 1;
        return tp_first_order_seat(o, seat) && tp_spawn_area(o, army) == t->obj_ref[k] - 1;
    }
    if (tp_first_order_seat(o, seat))
        return t->obj_kind[k] != TPK_NONE;
    return 1;
}

/* The fill after tp_assign: plan posts are tp_assign's; a first order goes beyond a zone's need only to a main zone.
 * s37 G4 (format 2): no capture / hold of a zone at its quota for a tank not counted for that zone (zq: the zone counts
 * with this tick's picks, tp2_quota_have / tp2_quota_take); the fill, its capture reserve and the reassignments are
 * all picks of this loop. */
static int tp_fill_excluded(void *host, int army, int k, int seat, const int *cnt, const cmd_obj_t *ob, const int *zq,
                            uint64_t ms)
{
    if (!tp_active(army))
        return 0;
    if (tp_plan_post(army, k))
        return 1;
    /* s40 (format 2): a post objective (old roads posts included) in an area at its cap, for a tank not counted there;
     * the counts are this tick's running ones (tp2_quota_have / tp2_quota_take) */
    if (tp_format == 2 && cmd_post_objective(&ob[k])) {
        const int a = tp2_area_at(ob[k].x, ob[k].z);
        if (!tp2_area_open(army, seat, a, tp_state[army].area_run)) {
            char what[64];
            snprintf(what, sizeof what, "post %d", ob[k].id);
            tp2_cap_note(host, army, seat, what, a, ms);
            return 1;
        }
    }
    if (tp_format == 2 && (ob[k].type == OB_CAPTURE || ob[k].type == OB_HOLD)
        && tp2_zone_full(army, tp_zone_of_lane(army, ob[k].id), &lseat[seat], zq))
        return 1;
    const tp_army_t *t = &tp_state[army];
    return t->obj_kind[k] == TPK_ZONE && tp_first_order_seat(&lseat[seat], seat) && cnt[k] >= ob[k].need
           && tp_role(army, t->obj_ref[k] - 1) != TPR_MAIN;
}

/* cmd_tick after cmd_retain_posts: crews keep their offered plan slots (outside the post budget); a watch tank whose
 * gun is empty is let go (the only release besides its death and the battle's end). */
static void tp_keep_plan(int army, const cmd_obj_t *ob, int nob, int *keep, int *claimed, uint64_t ms)
{
    for (int k = 0; k < nob; ++k) {
        if (!tp_plan_post(army, k))
            continue;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *q = &lseat[i];
            if (q->army != army || !q->unit || q->scene_epoch != ground_scene_epoch
                || !((q->cmd_type == OB_OVERWATCH && q->cmd_id == ob[k].id)
                     || (q->cmd_apply_ms && q->cmd_ntype == OB_OVERWATCH && q->cmd_nid == ob[k].id)))
                continue;
            claimed[k] = 1;
            if (!(tp_state[army].obj_kind[k] == TPK_WATCH && seat_ready_in(i, ms) > 90.0f))
                keep[i] = 1;
        }
    }
}

/* s35: the spawn area of a life; unknown pose: the army's spawn area nearest to where the life started. */
static int tp2_area_of(const lane_seat_t *o, int army)
{
    int a = tp_spawn_area(o, army);
    float bd = 1e18f;
    if (a >= 0)
        return a;
    for (int i = 0; i < ntp_spawns; ++i)
        if (tp_spawns[i].army == army) {
            float d = 1e18f;
            for (int k = 0; k < tp_spawns[i].n; ++k) {
                const int pose = tp_spawns[i].pose[k];
                const gr_lib_t *lib = routes_lib_state > 0 ? &routes_lib : NULL;
                if (lib && pose >= 0 && (uint32_t)pose < lib->n_poses)
                    d = fminf(d, dist2d(o->sx, o->sz, lib->poses[pose].x, lib->poses[pose].z));
            }
            if (d < bd) {
                bd = d;
                a = i;
            }
        }
    return a;
}

/* The objective of lane l in this tick's list: the zone's capture / hold for a circle lane, the slot's post otherwise. */
static int tp2_objective(int army, int l, int slot, const cmd_obj_t *ob, int nob)
{
    const tp_lane_t *ln = &tp_lanes[l];
    for (int k = 0; k < nob; ++k) {
        if (ln->end == TLE_CIRCLE) {
            if ((ob[k].type == OB_CAPTURE || ob[k].type == OB_HOLD) && ob[k].id == tp_state[army].lane[ln->serves])
                return k;
        } else if (slot < tp_lane_post_n[l] && ob[k].type == OB_OVERWATCH && ob[k].id == CMD_POST_ID_BASE + tp_lane_post_first[l] + slot) {
            return k;
        }
    }
    return -1;
}

/* s40 spread: the tanks on a position: the live tanks (occ, this tick's picks included) on every lane of the army that
 * shares a slot with lane l (a slot within TP2_SLOT_SHARE_M of one of l's slots, lane l itself included). */
static int tp2_pos_occ(int army, int l, const int *occ)
{
    const tp_lane_t *ln = &tp_lanes[l];
    int n = 0;
    for (int m = 0; m < ntp_lanes; ++m) {
        const tp_lane_t *lm = &tp_lanes[m];
        int shares = m == l;
        for (int a = 0; !shares && lm->army == army && a < ln->nslot; ++a)
            for (int b = 0; !shares && b < lm->nslot; ++b)
                shares = dist2d(ln->slot[a].x, ln->slot[a].z, lm->slot[b].x, lm->slot[b].z) <= TP2_SLOT_SHARE_M;
        n += shares ? occ[m] : 0;
    }
    return n;
}

/* The lane rule (one rule for every tank that needs an order): lanes of its own spawn area with a free place and a free
 * slot; (1) a sector of the width without a tank on any lane, outer sectors first (s37: not a lane of a zone at its
 * quota); (2) the plan zone below its quota this area reaches fastest, its circle lanes first, then its support lanes;
 * (3) s37: the free position (guard) lane with the highest value, equal values the least time ("toll position");
 * (4) s37: the free lane with the least time whatever its zone's count ("overflow", the only way over a quota). -1 none.
 * s40 (areas): rules (1), (3) and (4) skip a lane whose slot lies in an area at its cap for this tank (tp2_area_open with
 * the running area counts `ah`); rule (2) is as before (the point comes first). Last resort (5): nothing at all free:
 * the free lane with the least time whose objective is in this tick's list, caps ignored ("every area at its cap").
 * *cap_lane / *cap_area: the first lane skipped for its area and that area (-1 none), for the log.
 * s40 spread: within one choice (sector coverage, the lanes of the zone rule (2) picked, overflow, the last resort) the
 * lane with fewer live tanks on it (occ, this tick's earlier picks included) comes first, then the least time; rule (2)
 * still picks the zone by its least-time free lane, as before. Toll positions: fewer tanks on the position (every lane
 * sharing its slots, tp2_pos_occ) first, then the higher value, then the least time. */
static int tp2_choose(int army, int area, const int *occ, const int *zone_have, const int *sector_have, uint64_t ms, int *slot,
                      const char **why, int *why_ref, int seat, const int *ah, const cmd_obj_t *ob, int nob, int *cap_lane,
                      int *cap_area)
{
    const tp_army_t *t = &tp_state[army];
    const int S = tp_axes[army].ok ? tp_axes[army].sectors : 0;
    int best = -1, capped = 1, by_occ = 1, bo = INT_MAX;
    float bt = 1e18f;
    *cap_lane = *cap_area = -1;
    /* s40 spread: the lane order of one choice: fewer tanks on the lane, then less time (by_occ 0: time only) */
#define TP2_BEFORE(l, t_s) (by_occ ? (occ[l] < bo || (occ[l] == bo && (t_s) < bt)) : (t_s) < bt)
#define TP2_TRY(cond)                                                                                     \
    for (int l = 0; l < ntp_lanes; ++l) {                                                                \
        const tp_lane_t *ln = &tp_lanes[l];                                                              \
        int sl;                                                                                          \
        if (ln->from != area || !tp2_lane_usable(army, l) || !(cond) || !TP2_BEFORE(l, ln->time_s)       \
            || (sl = tp2_place(army, l, occ, ms)) < 0)                                                   \
            continue;                                                                                    \
        if (capped && !tp2_area_open(army, seat, tp2_slot_area(l, sl), ah)) {                            \
            if (*cap_lane < 0) {                                                                         \
                *cap_lane = l;                                                                           \
                *cap_area = tp2_slot_area(l, sl);                                                        \
            }                                                                                            \
            continue;                                                                                    \
        }                                                                                                \
        bt = ln->time_s;                                                                                 \
        bo = occ[l];                                                                                     \
        best = l;                                                                                        \
        *slot = sl;                                                                                      \
    }
    /* (1) sector coverage, outer sectors first */
    const float mid = 0.5f * (float)(S - 1);
    for (int r = 0; r < S && best < 0; ++r) {
        int sec = -1;
        for (int q = 0; q < S; ++q) {
            /* the r-th sector in the outer-first order (further from the middle first, then the left one) */
            int rank = 0;
            for (int w = 0; w < S; ++w)
                rank += fabsf((float)w - mid) > fabsf((float)q - mid) || (fabsf((float)w - mid) == fabsf((float)q - mid) && w < q);
            if (rank == r)
                sec = q;
        }
        if (sec < 0 || sec >= 16 || sector_have[sec])
            continue;
        TP2_TRY(ln->sector == sec && !tp2_at_quota(army, ln->serves, zone_have))
        if (best >= 0) {
            *why = "sector coverage";
            *why_ref = sec;
            return best;
        }
    }
    /* (2) s36: among the plan zones still below their share, the one this spawn area reaches fastest: each zone's
     * candidate is its least-time free circle lane from this area, else its least-time free support lane; the least
     * candidate time wins (equal times: the larger deficit) */
    const tp_plan_t *pl = &tp_plans[t->plan];
    int zbest = -1, zend = -1, zdef = 0;
    float ztime = 1e18f;
    capped = 0;                     /* s40: rule (2) as before */
    by_occ = 0;                     /* s40 spread: the zone is picked by its least-time free lane, as before */
    for (int j = 0; j < pl->nz; ++j) {
        const int z = pl->z[j].zone, def = t->need_total[z] - zone_have[z];
        if (tp_entry_role(army, pl, j) == TPR_SKIP || def <= 0)
            continue;
        int end = TLE_CIRCLE;
        best = -1;
        bt = 1e18f;
        TP2_TRY(ln->serves == z && ln->end == TLE_CIRCLE)
        if (best < 0) {
            end = TLE_SUPPORT;
            TP2_TRY(ln->serves == z && ln->end == TLE_SUPPORT)
        }
        if (best >= 0 && (bt < ztime || (bt == ztime && def > zdef))) {
            ztime = bt;
            zbest = z;
            zend = end;
            zdef = def;
        }
    }
    best = -1;
    bt = 1e18f;
    bo = INT_MAX;
    by_occ = 1;
    if (zbest >= 0) {
        /* s40 spread: among that zone's lanes of the same end (circle, else support) from this spawn area: fewer tanks
         * on the lane first, then the least time */
        TP2_TRY(ln->serves == zbest && ln->end == zend)
        capped = 1;
        *why = "zone share";
        *why_ref = zbest;
        return best;
    }
    capped = 1;
    /* (3) s37: a toll position: the free position lane of this area with the highest value, equal values the least time */
    /* s40 spread: fewer tanks on the position first (tp2_pos_occ), then the higher value, then the least time */
    float bvalue = -1.0f;
    int bpo = INT_MAX;
    for (int l = 0; l < ntp_lanes; ++l) {
        const tp_lane_t *ln = &tp_lanes[l];
        int sl;
        if (ln->from != area || ln->end != TLE_GUARD || !tp2_lane_usable(army, l))
            continue;
        const int po = tp2_pos_occ(army, l, occ);
        if (po > bpo || (po == bpo && (ln->value < bvalue || (ln->value == bvalue && ln->time_s >= bt)))
            || (sl = tp2_place(army, l, occ, ms)) < 0)
            continue;
        if (!tp2_area_open(army, seat, tp2_slot_area(l, sl), ah)) {     /* s40 */
            if (*cap_lane < 0) {
                *cap_lane = l;
                *cap_area = tp2_slot_area(l, sl);
            }
            continue;
        }
        bpo = po;
        bvalue = ln->value;
        bt = ln->time_s;
        best = l;
        *slot = sl;
    }
    if (best >= 0) {
        *why = "toll position";
        *why_ref = -1;
        return best;
    }
    /* (4) s37 overflow: the free lane with the least time, whatever its zone's count (s40: fewer tanks on it first) */
    bt = 1e18f;
    bo = INT_MAX;
    TP2_TRY(1)
    if (best >= 0) {
        *why = "overflow: no free position";
        *why_ref = -1;
        return best;
    }
    /* (5) s40 last resort, only when nothing was found and a lane was skipped for its area: the old overflow with the
     * caps ignored, among the
     * lanes whose objective is in this tick's list (a slot in an area at its cap is not offered: tp2_offer_lanes) */
    if (*cap_lane < 0)
        return -1;
    capped = 0;
    bt = 1e18f;
    bo = INT_MAX;
    TP2_TRY(tp2_objective(army, l, tp2_free_slot(army, l, ms), ob, nob) >= 0)
#undef TP2_TRY
#undef TP2_BEFORE
    if (best >= 0) {
        *why = "overflow: every area at its cap";
        *why_ref = -1;
    }
    return best;
}

static void tp2_note(void *host, int army, int seat, int l, int slot, const char *why, int ref, const char *change, int on_lane)
{
    const tp_lane_t *ln = &tp_lanes[l];
    char r[32] = "";
    if (!strcmp(why, "sector coverage"))
        snprintf(r, sizeof r, " %d", ref);
    else if (!strcmp(why, "zone share") && ref >= 0)
        snprintf(r, sizeof r, " %s", tp_zones[ref].id);
    char val[32] = "";
    if (ln->end == TLE_GUARD)   /* s37: a position lane's value */
        snprintf(val, sizeof val, ", value %.0f s", ln->value);
    /* s40 spread: the lane's occupancy after the pick */
    event(host, "teamplan: army %d seat %d %slane %s (%s%s%s, sector %d, %s end, slot %d at (%.0f,%.0f), %.0f s, %d bounds%s): %s%s "
          "(%d on this lane)",
          army, seat, change ? change : "", ln->id, ln->serves >= 0 ? "zone " : "position", ln->serves >= 0 ? tp_zones[ln->serves].id : "",
          "", ln->sector, ln->end == TLE_CIRCLE ? "circle" : ln->end == TLE_SUPPORT ? "support" : "guard", slot, ln->slot[slot].x,
          ln->slot[slot].z, ln->time_s, ln->nbound, val, why, r, on_lane);
}

/* cmd_tick before the fill (WTPLAN002): every tank that needs an order (none yet this life, or its lane is gone) gets a
 * lane and one of its free end slots by the lane rule. */
static void tp2_assign(void *host, int army, uint64_t ms, const cmd_obj_t *ob, int nob, int nc, const int *seat_of, int *pick,
                       int *cnt)
{
    if (!tp_active(army))
        return;
    for (int i = 1; i < MAX_SEATS; ++i)
        if (lseat[i].army == army)
            tp_seat_target[i] = 0;      /* s36: a pick of an earlier tick that was not issued must not reach a later order */
    int occ[TP_MAX_LANES], zh[TP_MAX_ZONES], sh[16], ah[TP_MAX_ZONES];
    tp2_count(army, ms, occ, zh, sh);
    tp2_area_count(army, ms, ah);       /* s40: running area counts, this loop's picks added */
    for (int u = 0; u < nc; ++u) {
        const int seat = seat_of[u];
        lane_seat_t *o = &lseat[seat];
        if (pick[u] >= 0 || o->cmd_type != OB_NONE || cmd_tasks[seat].pending.task_id || o->cmd_apply_ms)
            continue;
        const int area = tp2_area_of(o, army);
        int slot = -1, ref = -1, cap_lane = -1, cap_area = -1;
        const char *why = NULL;
        const int l = area >= 0 ? tp2_choose(army, area, occ, zh, sh, ms, &slot, &why, &ref, seat, ah, ob, nob, &cap_lane,
                                             &cap_area) : -1;
        if (area >= 0) {
            /* s36: the lane it would take without the opposing-lane check; when that differs, the check excluded it */
            int s2 = -1, r2 = -1, c2 = -1, a2 = -1;
            const char *w2 = NULL;
            tp2_no_oppose = 1;
            const int l2 = tp2_choose(army, area, occ, zh, sh, ms, &s2, &w2, &r2, seat, ah, ob, nob, &c2, &a2);
            tp2_no_oppose = 0;
            if (l2 >= 0 && l2 != l)
                tp2_opposed_note(host, army, seat, l2, occ);
        }
        if (cap_lane >= 0 && cap_area >= 0) {   /* s40: a place skipped for its area (one line per tank and tick) */
            char what[64];
            snprintf(what, sizeof what, "lane %s", tp_lanes[cap_lane].id);
            tp2_cap_note(host, army, seat, what, cap_area, ms);
        }
        if (l < 0)
            continue;   /* no lane: today's fallback (main zones by nearness) */
        const int k = tp2_objective(army, l, slot, ob, nob);
        if (k < 0)
            continue;
        pick[u] = k;
        ++cnt[k];
        ++occ[l];
        if (tp_lanes[l].serves >= 0)
            ++zh[tp_lanes[l].serves];
        if (tp2_slot_area(l, slot) >= 0)
            ++ah[tp2_slot_area(l, slot)];       /* s40 */
        if (tp_lanes[l].sector >= 0 && tp_lanes[l].sector < 16)
            ++sh[tp_lanes[l].sector];
        o->plan_lane1 = l + 1;
        o->plan_slot = slot;
        o->plan_lane_ms = ms;
        tp_seat_target[seat] = tp_lanes[l].slot[slot].target + 1;
        tp2_note(host, army, seat, l, slot, why, ref, NULL, occ[l]);
    }
}

/* s35: a lane blocked (the driving side's "no other street") or lost: a neighbour lane with a free place, then any lane of
 * the same zone / sector of the same spawn area. */
/* s36: lanes barred for a tank (life) after a closed street: those its way was found blocked to, and the lane it left,
 * until its closing box expires; and the time until which tp2_street leaves the tank alone (one lane change per
 * closed-street event; with no candidate, until the driving side's retry). */
#define TP2_BARS 8
#define TP2_STREET_CANDIDATES 3
#define TP2_STREET_RETRY_MS 20000u      /* the driving side's GL_STREET_RETRY_MS */
#define TP2_STREET_SETTLE_MS 2000u      /* ... and the snapshots its retry and its result take */
static struct {
    uint64_t life, quiet_until;
    int n;
    int lane[TP2_BARS];
    uint64_t until[TP2_BARS];
} tp2_bar[MAX_SEATS];

static int tp2_barred(int seat, int l, uint64_t ms)
{
    for (int k = 0; k < tp2_bar[seat].n; ++k)
        if (tp2_bar[seat].lane[k] == l && ms < tp2_bar[seat].until[k])
            return 1;
    return 0;
}

static void tp2_bar_lane(int seat, int l, uint64_t until)
{
    int k = 0;
    while (k < tp2_bar[seat].n && tp2_bar[seat].lane[k] != l)
        ++k;
    if (k == TP2_BARS) {
        k = 0;  /* full: the one that expires first goes */
        for (int q = 1; q < TP2_BARS; ++q)
            if (tp2_bar[seat].until[q] < tp2_bar[seat].until[k])
                k = q;
    } else if (k == tp2_bar[seat].n) {
        ++tp2_bar[seat].n;
    }
    tp2_bar[seat].lane[k] = l;
    tp2_bar[seat].until[k] = until;
}

/* s35: a lane blocked (the driving side's "no other street") or lost: a neighbour lane with a free place, then any lane of
 * the same zone / sector of the same spawn area. s36: a candidate's way from where the tank stands must not run through
 * a street it has closed (the driving side's own street search with its closed boxes, gm_lib_street_open); at most
 * TP2_STREET_CANDIDATES such checks per event; lanes found blocked and the lane left are barred for the tank until its
 * closing box expires. No candidate: it keeps its lane (the driving side stands short and tries again after 20 s), and
 * tp2_street leaves it alone until that retry. */
static void tp2_street(void *host, int army, uint64_t ms, const cmd_obj_t *ob, int nob)
{
    int occ[TP_MAX_LANES], zh[TP_MAX_ZONES], sh[16], ah[TP_MAX_ZONES];
    tp2_count(army, ms, occ, zh, sh);
    tp2_area_count(army, ms, ah);       /* s40 */
    const gr_lib_t *lib = routes_library(host);
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms) || !o->street_closed_ms || ms - o->street_closed_ms > 1500u || cmd_tasks[i].pending.task_id
            || o->cmd_apply_ms)
            continue;
        if (tp2_bar[i].life != o->born_ms) {
            memset(&tp2_bar[i], 0, sizeof tp2_bar[i]);
            tp2_bar[i].life = o->born_ms;
        }
        if (ms < tp2_bar[i].quiet_until)
            continue;           /* this event was answered (a lane change, or "stands and tries again") */
        const int from = tp2_lane_of(army, o);
        o->street_closed_ms = 0;
        if (from < 0)
            continue;
        const tp_lane_t *fl = &tp_lanes[from];
        uint64_t box_until = 0;
        int to = -1, slot = -1, checked = 0, blocked = 0;
        char bl[96] = "";
        /* candidates in order: the neighbours, then the same zone / sector of the same spawn area */
        for (int pass = 0; pass < 2 && to < 0 && checked < TP2_STREET_CANDIDATES; ++pass)
            for (int q = 0; q < (pass ? ntp_lanes : fl->nnb) && to < 0 && checked < TP2_STREET_CANDIDATES; ++q) {
                const int l = pass ? q : fl->nb[q];
                if (l < 0 || l == from || !tp2_lane_usable(army, l) || tp2_barred(i, l, ms))
                    continue;
                if (pass && !(tp_lanes[l].from == fl->from
                              && ((fl->serves >= 0 && tp_lanes[l].serves == fl->serves) || tp_lanes[l].sector == fl->sector)))
                    continue;
                /* s37 G6: a lane of another zone only while that zone is below its quota (same zone, a position: as before) */
                if (tp_lanes[l].serves >= 0 && tp_lanes[l].serves != fl->serves && tp2_at_quota(army, tp_lanes[l].serves, zh))
                    continue;
                const int sl = tp2_place(army, l, occ, ms);
                if (sl == TP2_OPPOSED)
                    tp2_opposed_note(host, army, i, l, occ);
                if (sl < 0)
                    continue;
                /* s40: not a slot in an area at its cap, unless it is a lane of a zone below its quota (the point first) */
                if (!tp2_zone_first(army, l, i, zh) && !tp2_area_open(army, i, tp2_slot_area(l, sl), ah)) {
                    char what[64];
                    snprintf(what, sizeof what, "lane %s", tp_lanes[l].id);
                    tp2_cap_note(host, army, i, what, tp2_slot_area(l, sl), ms);
                    continue;
                }
                ++checked;
                uint64_t until = 0;
                if (!gm_lib_street_open(lib, (uint64_t)i, o->born_ms, tp_lanes[l].slot[sl].target, o->x, o->z, ms, &until)) {
                    tp2_bar_lane(i, l, until ? until : ms + 60000u);
                    ++blocked;
                    const size_t len = strlen(bl);
                    snprintf(bl + len, sizeof bl - len, "%s%s", len ? ", " : "", tp_lanes[l].id);
                    continue;
                }
                box_until = until;
                to = l;
                slot = sl;
            }
        const int k = to >= 0 ? tp2_objective(army, to, slot, ob, nob) : -1;
        if (k < 0) {
            tp2_bar[i].quiet_until = ms + TP2_STREET_RETRY_MS + TP2_STREET_SETTLE_MS;
            event(host, "teamplan: army %d seat %d lane %s blocked: no lane with a free place and an open way (%d checked, "
                  "%d through its closed street%s%s); it keeps its lane, stands and tries again in 20 s", army, i, fl->id,
                  checked, blocked, blocked ? ": " : "", bl);
            continue;
        }
        tp2_bar_lane(i, from, box_until ? box_until : ms + 60000u);
        tp2_bar[i].quiet_until = ms + TP2_STREET_RETRY_MS + TP2_STREET_SETTLE_MS;
        --occ[from];
        ++occ[to];
        const int z0 = tp2_seat_zone(army, o), z1 = tp_lanes[to].serves;   /* s37: the zone counts follow the change */
        if (z0 != z1) {
            if (z0 >= 0)
                --zh[z0];
            if (z1 >= 0)
                ++zh[z1];
        }
        const int a0 = tp2_seat_area(army, o), a1 = tp2_slot_area(to, slot);  /* s40: and the area counts */
        if (a0 != a1) {
            if (a0 >= 0)
                --ah[a0];
            if (a1 >= 0)
                ++ah[a1];
        }
        o->plan_lane1 = to + 1;
        o->plan_slot = slot;
        o->plan_lane_ms = ms;
        tp_seat_target[i] = tp_lanes[to].slot[slot].target + 1;
        tp2_note(host, army, i, to, slot, "lane blocked (no other street)", -1, "changes to ", occ[to]);
        if (blocked)
            event(host, "teamplan: army %d seat %d lanes %s left out: their way runs through its closed street", army, i, bl);
        cmd_issue(host, army, i, &ob[k], 0.0f, 0, ms);
    }
}

/* Census (WTPLAN002): tanks per sector, per zone (circle / support) against the plan. */
static void tp2_report(void *host, int army, uint64_t ms)
{
    const tp_army_t *t = &tp_state[army];
    const tp_plan_t *pl = &tp_plans[t->plan];
    int occ[TP_MAX_LANES], zh[TP_MAX_ZONES], sh[16], ah[TP_MAX_ZONES];
    tp2_count(army, ms, occ, zh, sh);
    tp2_area_count(army, ms, ah);       /* s40 */
    char line[800];
    int len = snprintf(line, sizeof line, "teamplan: army %d plan %s (%s), roster %d live %d | sectors", army, pl->id, pl->kind,
                       t->roster, tp_live(army, ms));
    for (int s = 0; s < tp_axes[army].sectors && s < 16 && len < (int)sizeof line - 16; ++s)
        len += snprintf(line + len, sizeof line - (size_t)len, " %d:%d", s, sh[s]);
    len += snprintf(line + len, sizeof line - (size_t)len, " | zones");
    for (int j = 0; j < pl->nz && len < (int)sizeof line - 60; ++j) {
        const int z = pl->z[j].zone;
        int circle = 0, support = 0;
        for (int l = 0; l < ntp_lanes; ++l)
            if (tp_lanes[l].army == army && tp_lanes[l].serves == z) {
                circle += tp_lanes[l].end == TLE_CIRCLE ? occ[l] : 0;
                support += tp_lanes[l].end == TLE_SUPPORT ? occ[l] : 0;
            }
        /* s37 G8: have / quota by the zone count (every tank ordered to the zone, lane or not, and its support lanes) */
        /* s39: the effective role ("contest, skip opened"), and " garrison" while the zone is ours */
        const int role = tp_entry_role(army, pl, j);
        len += snprintf(line + len, sizeof line - (size_t)len, " %s %s%s circle %d support %d, have %d/%d%s, area %d/%d",
                        tp_zones[z].id, tp_role_name[role], role != pl->z[j].role ? ", skip opened" : "", circle, support,
                        zh[z], t->need_total[z], tp2_garrisoned(army, z) && role != TPR_SKIP ? " garrison" : "", ah[z],
                        tp2_area_cap(army, z));   /* s40: the area count against its cap */
    }
    int guard = 0;
    for (int l = 0; l < ntp_lanes; ++l)
        guard += tp_lanes[l].army == army && tp_lanes[l].end == TLE_GUARD ? occ[l] : 0;
    snprintf(line + len, sizeof line - (size_t)len, " | positions %d", guard);
    event(host, "%s", line);
}

/* s37 G14 (cmd_issue, no pick of this tick): an order that is the capture / hold of the zone objective of the tank's
 * own circle lane (plan_lane1 / plan_slot, the tp2_lane_of test) names that lane's slot: 1 + its library target; any
 * other order 0 (the goal is mapped as before). */
static int tp2_lane_slot_target(int army, int seat, const cmd_obj_t *o)
{
    if (!tp_active(army) || tp_format != 2 || seat <= 0 || seat >= MAX_SEATS || (o->type != OB_CAPTURE && o->type != OB_HOLD))
        return 0;
    const lane_seat_t *s = &lseat[seat];
    const int l = s->plan_lane1 - 1;
    if (l < 0 || l >= ntp_lanes)
        return 0;
    const tp_lane_t *ln = &tp_lanes[l];
    if (ln->army != army || ln->end != TLE_CIRCLE || ln->serves < 0 || s->plan_slot < 0 || s->plan_slot >= ln->nslot)
        return 0;
    const int zl = tp_state[army].lane[ln->serves];
    return zl >= 0 && o->id == zl ? ln->slot[s->plan_slot].target + 1 : 0;
}

static int tp_take_target(int seat)
{
    if (seat <= 0 || seat >= MAX_SEATS)
        return 0;
    const int v = tp_seat_target[seat];
    tp_seat_target[seat] = 0;
    return v;
}

/* cmd_tick after tp_keep_plan: a zone with fewer capture tanks than its circle takes moves one of its support tanks up. */
static void tp_move_up(void *host, int army, uint64_t ms, const cmd_obj_t *ob, int nob)
{
    if (!tp_active(army) || tp_format == 2)
        return;
    const tp_army_t *t = &tp_state[army];
    for (int k = 0; k < nob; ++k) {
        if (t->obj_kind[k] != TPK_ZONE || ob[k].type != OB_CAPTURE)
            continue;
        const int z = t->obj_ref[k] - 1;
        int have = 0, up = 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *q = &lseat[i];
            if (!cmd_live(q, i, army, ms))
                continue;
            have += (q->cmd_type == OB_CAPTURE && q->cmd_id == ob[k].id) || (q->cmd_apply_ms && q->cmd_ntype == OB_CAPTURE && q->cmd_nid == ob[k].id);
            const int j = q->cmd_id - CMD_POST_ID_BASE;
            if (!up && cmd_is_post(q) && j >= 0 && j < cmd_nposts[army] && cmd_posts[army][j].kind == PK_SUPPORT
                && t->post_zone[j] == z + 1 && !cmd_tasks[i].pending.task_id && !q->cmd_apply_ms)
                up = i;
        }
        if (have >= ob[k].need || !up)
            continue;
        event(host, "teamplan: army %d seat %d moves up from support into zone %s (capture %d of %d)", army, up, tp_zones[z].id, have,
              ob[k].need);
        cmd_issue(host, army, up, &ob[k], 0.0f, 0, ms);
    }
}

/* s34: the driving side found no other street into the zone: a capture tank of a plan zone takes a free support slot of
 * that zone instead (any other order: the driving side stands and tries again). */
#define TP_STREET_FRESH_MS 1500u
static void tp_street(void *host, int army, uint64_t ms, const cmd_obj_t *ob, int nob)
{
    if (!tp_active(army))
        return;
    if (tp_format == 2) {
        tp2_street(host, army, ms, ob, nob);     /* s35: a neighbour lane instead */
        return;
    }
    const tp_army_t *t = &tp_state[army];
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms) || !o->street_closed_ms || ms - o->street_closed_ms > TP_STREET_FRESH_MS
            || o->cmd_type != OB_CAPTURE || cmd_tasks[i].pending.task_id || o->cmd_apply_ms)
            continue;
        const int z = tp_zone_of_lane(army, o->cmd_id);
        if (z < 0)
            continue;
        int pick = -1;
        for (int k = 0; k < nob && pick < 0; ++k)
            if (t->obj_kind[k] == TPK_SUPPORT && t->obj_ref[k] == z + 1 && !tp_post_held(army, ob[k].id))
                pick = k;
        if (pick < 0) {
            event(host, "teamplan: army %d seat %d: no other street into zone %s and no free support slot: it stands and tries "
                  "again", army, i, tp_zones[z].id);
            o->street_closed_ms = 0;
            continue;
        }
        event(host, "teamplan: army %d seat %d: no other street into zone %s: takes support post %d instead", army, i,
              tp_zones[z].id, ob[pick].id);
        o->street_closed_ms = 0;
        cmd_issue(host, army, i, &ob[pick], 0.0f, 0, ms);
    }
}

/* cmd_tick before the fill: first orders from the plan. Watch slots first (one tank of the entry's spawn area), then the
 * capture places of each plan zone, then its support slots; candidates: tanks without an order yet (support also takes
 * a tank whose objective is gone). Utility as the fill computes it. */
static void tp_assign(int army, const cmd_obj_t *ob, int nob, int nc, const int *seat_of, const int *cur, int *pick, int *cnt,
                      float (*base)[CMD_MAX_OBJ], float (*cost)[CMD_MAX_OBJ], const int *claimed)
{
    if (!tp_active(army))
        return;
    const tp_army_t *t = &tp_state[army];
    static const int order[3] = {TPK_WATCH, TPK_ZONE, TPK_SUPPORT};
    for (int pass = 0; pass < 3; ++pass)
        for (int k = 0; k < nob; ++k) {
            if (t->obj_kind[k] != order[pass] || (order[pass] != TPK_ZONE && (cnt[k] || claimed[k])))
                continue;
            while (cnt[k] < (order[pass] == TPK_ZONE ? ob[k].need : 1)) {
                int bu = -1;
                float bv = -1e30f;
                for (int u = 0; u < nc; ++u) {
                    const int seat = seat_of[u];
                    const int ok = tp_first_order_seat(&lseat[seat], seat) || (order[pass] == TPK_SUPPORT && cur[u] < 0);
                    if (pick[u] < 0 && ok && base[u][k] > 0.0f && base[u][k] - cost[u][k] > bv) {
                        bv = base[u][k] - cost[u][k];
                        bu = u;
                    }
                }
                if (bu < 0)
                    break;
                pick[bu] = k;
                ++cnt[k];
            }
        }
}

static int tp_variant_route(const gr_lib_t *lib, int pose, int target, int v)
{
    const int r = gr_route_variant(lib, pose, target, v);
    return r >= 0 && lib->routes[r].fallback == v ? r : -1;
}

/* cmd_tick before the orders go out: the flank of each flank zone of the plan (one tank of this tick's first-order
 * wave to it). tp_seat_variant[seat] is read by cmd_issue. */
static void tp_flank_wave(void *host, int army, uint64_t ms, const cmd_obj_t *ob, int nob, int nc, const int *seat_of,
                          const int *pick)
{
    memset(tp_seat_variant, 0, sizeof tp_seat_variant);
    if (!tp_active(army) || tp_format == 2)
        return;                         /* s35: the lanes carry their own designated routes: no variants */
    const gr_lib_t *lib = routes_library(host);
    if (!lib)
        return;
    tp_army_t *t = &tp_state[army];
    const tp_plan_t *pl = &tp_plans[t->plan];
    for (int z = 0; z < ntp_zones; ++z) {
        if (!pl->flank[z] || t->lane[z] < 0 || t->need_total[z] < 4)
            continue;
        int k = -1;
        for (int q = 0; q < nob && k < 0; ++q)
            if (ob[q].type == OB_CAPTURE && ob[q].id == t->lane[z])
                k = q;
        if (k < 0)
            continue;
        const int fs = t->flank_seat[z];
        if (fs > 0 && lseat[fs].born_ms == t->flank_life[z] && lseat[fs].route_variant > 0 && cmd_live(&lseat[fs], fs, army, ms)
            && lseat[fs].cmd_type == OB_CAPTURE && lseat[fs].cmd_id == t->lane[z])
            continue;                   /* the last flanker of this zone is still on its way */
        const int target = gr_target_of(lib, ob[k].x, ob[k].z, army, 12.0f);
        if (target < 0 || lib->targets[target].kind != 0)
            continue;
        int bs = 0, bv = 0, bfast = 0;
        float bx = 1e30f;
        for (int u = 0; u < nc; ++u) {
            const int seat = seat_of[u];
            const lane_seat_t *o = &lseat[seat];
            if (pick[u] != k || o->cmd_type != OB_NONE || o->spawn_pose1 <= 0)
                continue;
            const int pose = o->spawn_pose1 - 1, d = gr_designated(lib, pose, target);
            const int want = t->flank_prev[z] == 1 ? 2 : 1;
            int v = want, r = tp_variant_route(lib, pose, target, want);
            if (r < 0 && want == 2) {
                v = 1;
                r = tp_variant_route(lib, pose, target, 1);
            }
            if (r < 0 || d < 0)
                continue;
            const vcls_t *c = active_cls(seat, o->unit);
            const int fast = c->ok && (c->flags & (VC_LIGHT | VC_SCOUT | VC_WHEELED)) != 0;
            const float extra = lib->routes[r].total_s - lib->routes[d].total_s;
            if (!bs || fast > bfast || (fast == bfast && extra < bx)) {
                bs = seat;
                bv = v;
                bfast = fast;
                bx = extra;
            }
        }
        if (!bs)
            continue;
        tp_seat_variant[bs] = bv;
        t->flank_prev[z] = bv;
        t->flank_seat[z] = bs;
        t->flank_life[z] = lseat[bs].born_ms;
        event(host, "teamplan: army %d seat %d to zone %s by flank variant %d (+%.0f s over its recommended route)%s", army, bs,
              tp_zones[z].id, bv, bx, bfast ? ", fast class" : "");
    }
}

/* cmd_issue: the route variant of this order (a capture of this tick's flank pick), 0 otherwise. */
static int tp_take_variant(int seat, const cmd_obj_t *o)
{
    if (seat <= 0 || seat >= MAX_SEATS || o->type != OB_CAPTURE)
        return 0;
    const int v = tp_seat_variant[seat];
    tp_seat_variant[seat] = 0;
    return v;
}

/* cmd_issue, first orders: which plan item it fills. */
static void tp_note_first(void *host, int army, int seat, const cmd_obj_t *o)
{
    if (!tp_active(army))
        return;
    const tp_army_t *t = &tp_state[army];
    int z = -1;
    const char *what = "not a plan item";
    char buf[96];
    if (o->type == OB_CAPTURE || o->type == OB_HOLD) {
        z = tp_zone_of_lane(army, o->id);
        if (z >= 0) {
            snprintf(buf, sizeof buf, "zone %s %s (%s)", tp_zones[z].id, o->type == OB_CAPTURE ? "capture" : "hold",
                     tp_role_name[tp_role(army, z) >= 0 ? tp_role(army, z) : 0]);
            what = buf;
        }
    } else if (o->type == OB_OVERWATCH && o->id >= CMD_POST_ID_BASE && o->id - CMD_POST_ID_BASE < cmd_nposts[army]) {
        const int j = o->id - CMD_POST_ID_BASE;
        const cmd_post_t *p = &cmd_posts[army][j];
        if (p->kind == PK_WATCH) {
            snprintf(buf, sizeof buf, "watch %s slot %d", tp_areas[p->ref].id, p->line);
            what = buf;
        } else if (p->kind == PK_LANE) {
            snprintf(buf, sizeof buf, "lane %s slot %d", tp_lanes[p->ref].id, p->line);
            what = buf;
        } else if (p->kind == PK_SUPPORT && t->post_zone[j] > 0) {
            snprintf(buf, sizeof buf, "zone %s support %s slot %d", tp_zones[t->post_zone[j] - 1].id, tp_areas[p->ref].id, p->line);
            what = buf;
        }
    }
    const int pose = lseat[seat].spawn_pose1 - 1, area = tp_spawn_area(&lseat[seat], army);
    event(host, "teamplan: army %d seat %d first order fills %s (spawn pose %d, area %s)", army, seat, what, pose,
          area >= 0 ? tp_spawns[area].id : "?");
}

/* Every census: the plan and how many tanks are where against its numbers. */
static void tp_report(void *host, int army, uint64_t ms)
{
    if (!tp_active(army))
        return;
    if (tp_format == 2) {
        tp2_report(host, army, ms);
        return;
    }
    const tp_army_t *t = &tp_state[army];
    const tp_plan_t *pl = &tp_plans[t->plan];
    char line[480];
    int len = snprintf(line, sizeof line, "teamplan: army %d plan %s (%s), roster %d live %d | zones", army, pl->id, pl->kind,
                       t->roster, tp_live(army, ms));
    for (int j = 0; j < pl->nz && len < (int)sizeof line - 60; ++j) {
        const int z = pl->z[j].zone, lane = t->lane[z];
        int cap = 0, sup = 0, flank = 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            if (!cmd_live(o, i, army, ms))
                continue;
            if (lane >= 0 && (o->cmd_type == OB_CAPTURE || o->cmd_type == OB_HOLD) && o->cmd_id == lane) {
                ++cap;
                flank += o->route_variant > 0;
            }
            const int pj = o->cmd_id - CMD_POST_ID_BASE;
            sup += cmd_is_post(o) && pj >= 0 && pj < cmd_nposts[army] && cmd_posts[army][pj].kind == PK_SUPPORT
                   && t->post_zone[pj] == z + 1;
        }
        len += snprintf(line + len, sizeof line - (size_t)len, " %s %s capture %d/%d support %d/%d%s", tp_zones[z].id,
                        tp_role_name[tp_entry_role(army, pl, j)], cap, t->need_cap[z], sup, t->need_sup[z], flank ? " (flank 1)" : "");
    }
    len += snprintf(line + len, sizeof line - (size_t)len, " | watch");
    for (int w = 0; w < pl->nw && len < (int)sizeof line - 40; ++w) {
        const int a = pl->w[w].area;
        int crews = 0, placed = 0;
        for (int s = 0; t->watch_first[a] >= 0 && s < t->watch_n[a]; ++s)
            for (int i = 1; i < MAX_SEATS; ++i) {
                const lane_seat_t *o = &lseat[i];
                if (cmd_live(o, i, army, ms) && o->cmd_type == OB_OVERWATCH && o->cmd_id == CMD_POST_ID_BASE + t->watch_first[a] + s) {
                    ++crews;
                    placed += o->cmd_parked;
                }
            }
        len += snprintf(line + len, sizeof line - (size_t)len, " %s<%s %d/%d (in place %d)", tp_areas[a].id,
                        tp_spawns[pl->w[w].from].id, crews, pl->w[w].count, placed);
    }
    if (!pl->nw && len < (int)sizeof line - 8)
        snprintf(line + len, sizeof line - (size_t)len, " none");
    event(host, "%s", line);
}
