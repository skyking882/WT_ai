/* s48: the team planner moved out of ground_brain.c, in its original order (s46 lines 3811-4148: the
 * offensive share census, team_census, task_point, team_rear_class, team_plan; the constants the tactics use
 * are in ground_command.h). Compiled only through ground_command.c, before ground_command_core.inc. */
/* ---- Team planner (2026-11-10, plan L1), 1 Hz per army -------------
 * Threats: enemies the team knows about (team_knowledge, < TASK_FORGET_MS old)
 * that are behind our line (projection on the spawn-to-spawn axis below the
 * median of our live tanks), s39: and not at a zone we do not own
 * (team_rear_class; "near our spawn" and "at our point" are gone). Each
 * threat (deepest first, at most 3) gets 1 interceptor: the nearest free
 * tanks within INTERCEPT_MAX_M (not engaged, not retreating, not holding a
 * point, not fresh from the spawn). An interceptor drives to a firing spot
 * INTERCEPT_STANDOFF_M short of the contact (task_side +1 for the first).
 * Tasks last at least TASK_MIN_MS and at most TASK_MAX_MS; they end when the
 * contact is dead or forgotten. */
#define PLAN_MS 1000
#define TASK_MIN_MS 15000
#define TASK_MAX_MS 60000
#define MAX_THREATS 3
static uint64_t plan_next[3];

/* ---- Offensive share (design rule: at least 75% of a team attacks or
 * defends the points instead of hanging back) --------------------------------
 * Every live tank is in one class, once a second:
 *   point-directed: push (driving to a point that is not ours), at_point
 *   (within its radius + POINT_NEAR_M), defend (intercept task for a contact
 *   at a point of ours), reinforce (driving to a point of ours);
 *   passive: hold (staging hold short of a point: overwatch S1/S2, screen F1),
 *   intercept (task away from the points), pursue, retreat.
 * Passive tanks may be at most a quarter of the live team (PASSIVE_DIV): no
 * new intercept task away from the points beyond that, and the oldest staging
 * hold is released. Tasks of any kind: at most a fifth of the team (TASK_DIV,
 * at least 1), 1 per threat (s39).
 * Pushers: per point that is not ours, the nearest tank heading for it, and
 * anyone within PUSH_IN_M of it, make no short stops beyond PUSH_STOP_M, do not
 * pursue or back off and are never taken for a task.
 * `team:` log line every CENSUS_MS. */
#define POINT_NEAR_M 40.0f
#define CMD_START_GRACE_MS 1800
#define PUSH_IN_M 80.0f
#define TASK_DIV 5
#define CENSUS_MS 30000
static uint64_t census_next[3];

int fight_in_point(void *host, const lane_seat_t *s, int *contested)
{
    for (int i = 0; i < nlanes; ++i) {
        const lane_t *l = &lanes[i];
        if (l->army == s->army && l->kind == 0 && dist2d(s->x, s->z, l->zx, l->zz) < l->zr) {
            *contested = zone_owner(host, l->zx, l->zz) != s->army;
            return i;
        }
    }
    return -1;
}

static int seat_advancing(const lane_seat_t *s, uint64_t ms)
{
    return s->wants_move && !s->holding && !s->cmd_parked && !s->parked
           && !control_engagement_active(s, ms)
           && (s->vx * s->vx + s->vz * s->vz > 0.25f
               || (s->motion_cmd_ms && ms - s->motion_cmd_ms < CMD_START_GRACE_MS));
}


int seat_class(void *host, const lane_seat_t *o, uint64_t ms)
{
    if (o->mode == M_RETREAT)
        return C_RETREAT;
    if (o->mode == M_PURSUE)
        return C_PURSUE;
    int contested = 0;
    if (fight_in_point(host, o, &contested) >= 0)
        return C_AT_POINT;
    if (o->task)
        return o->task_point ? C_DEFEND : C_INTERCEPT;
    if (o->cmd_parked || o->parked || o->holding || !o->wants_move)
        return C_HOLD;
    if (!seat_advancing(o, ms))
        return C_IDLE;
    if (o->cmd_type >= OB_SUPPORT)
        return C_PUSH;
    return zone_owner(host, lanes[o->lane].zx, lanes[o->lane].zz) == o->army ? C_REINFORCE : C_PUSH;
}

/* Marks the pushers, releases a staging hold over the passive budget, logs the
 * census. Returns the live count; *passive = passive tanks now. */
static int team_census(void *host, int army, uint64_t ms, int *passive)
{
    int cnt[C_N] = {0}, live = 0, engaged = 0, stops = 0, stalled = 0, native = 0, pushers = 0;
    int oldest = -1;
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!o->unit || !o->n || o->scene_epoch != ground_scene_epoch || o->army != army)
            continue;
        o->pusher = 0;
        if (ms - o->last_ms > 2000)
            continue;
        int c = seat_class(host, o, ms);
        ++live;
        ++cnt[c];
        engaged += o->mode == M_ENGAGE;
        stops += o->stopping;
        native += ms < o->native_until;
        stalled += o->wants_move && o->vx * o->vx + o->vz * o->vz < 0.25f;
        if (c == C_HOLD && o->holding && o->route[o->step].pt != SLOT_PT
            && (oldest < 0 || o->hold_start < lseat[oldest].hold_start))
            oldest = i;
    }
    for (int k = 0; k < nlanes; ++k) {
        const lane_t *l = &lanes[k];
        if (l->army != army || l->kind != 0 || zone_owner(host, l->zx, l->zz) == army)
            continue;
        int best = -1;
        float bd = 1e18f;
        for (int i = 1; i < MAX_SEATS; ++i) {
            lane_seat_t *o = &lseat[i];
            if (!o->unit || !o->n || o->scene_epoch != ground_scene_epoch || o->army != army || ms - o->last_ms > 2000 || o->task || o->mode == M_RETREAT
                || o->cmd_type >= OB_SUPPORT || dist2d(lanes[o->lane].zx, lanes[o->lane].zz, l->zx, l->zz) > 40.0f)
                continue;
            float d = dist2d(o->x, o->z, l->zx, l->zz);
            if (d < l->zr + PUSH_IN_M)
                o->pusher = 1;
            if (d < bd) {
                bd = d;
                best = i;
            }
        }
        if (best >= 0)
            lseat[best].pusher = 1;
    }
    for (int i = 1; i < MAX_SEATS; ++i)
        pushers += lseat[i].unit && lseat[i].n && lseat[i].scene_epoch == ground_scene_epoch && lseat[i].army == army && lseat[i].pusher;
    *passive = cnt[C_HOLD] + cnt[C_INTERCEPT] + cnt[C_PURSUE] + cnt[C_RETREAT];
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (o->unit && o->n && o->scene_epoch == ground_scene_epoch && o->army == army && ms - o->last_ms <= 2000 && seat_useful_fire(o, ms)
            && seat_class(host, o, ms) == C_HOLD)
            --*passive;
    }
    if (*passive > live / PASSIVE_DIV && oldest >= 0) {
        lseat[oldest].hold_start = 1;   /* long ago: the hold is over on its next tick */
        event(host, "team: army %d releases seat %d's staging hold (%d passive of %d live)", army, oldest, *passive,
              live);
        --*passive;
    }
    if (ms >= census_next[army] && live) {
        census_next[army] = ms + CENSUS_MS;
        int directed = cnt[C_PUSH] + cnt[C_AT_POINT] + cnt[C_DEFEND] + cnt[C_REINFORCE];
        event(host, "team: army %d live %d | push %d at_point %d defend %d reinforce %d = %d%% point-directed | "
              "hold %d intercept %d pursue %d retreat %d idle %d | engaged %d short-stop %d stalled %d native %d pushers %d",
              army, live, cnt[C_PUSH], cnt[C_AT_POINT], cnt[C_DEFEND], cnt[C_REINFORCE], directed * 100 / live,
              cnt[C_HOLD], cnt[C_INTERCEPT], cnt[C_PURSUE], cnt[C_RETREAT], cnt[C_IDLE], engaged, stops, stalled, native, pushers);
    }
    return live;
}

void task_point(lane_seat_t *o, float cx, float cz)
{
    float d = dist2d(o->x, o->z, cx, cz);
    if (d <= INTERCEPT_STANDOFF_M) {
        o->task_x = o->x;
        o->task_z = o->z;
        return;
    }
    float ux = (o->x - cx) / d, uz = (o->z - cz) / d;
    o->task_x = cx + ux * INTERCEPT_STANDOFF_M - uz * 40.0f * o->task_side;
    o->task_z = cz + uz * INTERCEPT_STANDOFF_M + ux * 40.0f * o->task_side;
}

/* s39: THE rear classification of a known enemy at (x, z), used by team_plan and by the alert (tac_rear_class): "behind
 * our line" (its projection on the spawn-to-spawn axis below the median of our live tanks minus 30 m), and never at a
 * zone we do not own (within that zone's circle + ALERT_R_M: an enemy at a zone being fought for is the business of the
 * capture / hold orders). The classes "near our spawn" and "at our point" are gone. NULL: not in our rear. */
const char *team_rear_class(void *host, int army, float x, float z, float osx, float osz, float ux, float uz,
                                   float front, int np)
{
    const float depth = (x - osx) * ux + (z - osz) * uz;
    if (!np || depth >= front - 30.0f)
        return NULL;
    for (int i = 0; i < nlanes; ++i)
        if (lanes[i].army == army && lanes[i].kind == 0 && dist2d(x, z, lanes[i].zx, lanes[i].zz) < lanes[i].zr + ALERT_R_M
            && zone_owner(host, lanes[i].zx, lanes[i].zz) != army)
            return NULL;
    return "behind our line";
}

void team_plan(void *host, int army, uint64_t ms)
{
    if (!cfg.team || army < 1 || army > 2 || ms < plan_next[army])
        return;
    plan_next[army] = ms + PLAN_MS;
    int enemy = 3 - army;
    float osx = 0, osz = 0, esx = 0, esz = 0, proj[MAX_SEATS];
    int on = 0, en = 0, np = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (!o->unit || !o->n || o->scene_epoch != ground_scene_epoch)
            continue;
        if (o->army == army) {
            osx += o->sx;
            osz += o->sz;
            ++on;
        } else if (o->army == enemy) {
            esx += o->sx;
            esz += o->sz;
            ++en;
        }
    }
    if (!on || !en)
        return;
    osx /= (float)on;
    osz /= (float)on;
    esx /= (float)en;
    esz /= (float)en;
    float ux, uz;
    unit_dir(osx, osz, esx, esz, &ux, &uz);
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (o->unit && o->n && o->scene_epoch == ground_scene_epoch && o->army == army && ms - o->last_ms < 2000)
            proj[np++] = (o->x - osx) * ux + (o->z - osz) * uz;
    }
    for (int i = 1; i < np; ++i)        /* insertion sort, np <= 31 */
        for (int j = i; j > 0 && proj[j - 1] > proj[j]; --j) {
            float tmp = proj[j];
            proj[j] = proj[j - 1];
            proj[j - 1] = tmp;
        }
    float front = np ? proj[np / 2] : 0.0f;

    typedef struct { int t; float x, z, depth; const char *why; } threat_t;
    threat_t th[MAX_THREATS + 1];
    int nth = 0;
    for (int t = 0; t < 64; ++t) {
        float x, z, age;
        if (!contact_get(army, 0, t, ms, &x, &z, &age) || age * 1000.0f > (float)TASK_FORGET_MS)
            continue;
        uint64_t first = contact[army][t].first_ms;
        if (learnt_logged[army][t] != first) {
            learnt_logged[army][t] = first;
            event(host, "know: army %d learns of slot %d at (%.0f,%.0f), spotted by seat %d", army, t, x, z,
                  contact[army][t].by);
        }
        float depth = (x - osx) * ux + (z - osz) * uz;
        const char *why = team_rear_class(host, army, x, z, osx, osz, ux, uz, front, np);     /* s39: the one rule */
        if (!why)
            continue;
        /* keep the MAX_THREATS deepest */
        int at = nth;
        while (at > 0 && th[at - 1].depth > depth) {
            th[at] = th[at - 1];
            --at;
        }
        th[at] = (threat_t){t, x, z, depth, why};
        if (nth < MAX_THREATS)
            ++nth;
    }

    /* Refresh or end the current tasks. */
    int passive = 0, live = team_census(host, army, ms, &passive), tasks = 0;
    int task_max = live / TASK_DIV > 1 ? live / TASK_DIV : 1;
    int assigned[64] = {0};
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!o->unit || !o->n || o->scene_epoch != ground_scene_epoch || o->army != army || !o->task)
            continue;
        if (cfg.alert && o->task_alert)
            continue;                   /* s28: the alert record keeps or ends it (tac_alert_plan) */
        float x, z, age;
        int known = contact_get(army, i, o->task_tgt, ms, &x, &z, &age) && age * 1000.0f <= (float)TASK_FORGET_MS;
        int threat = 0;
        for (int k = 0; k < nth; ++k)
            threat |= th[k].t == o->task_tgt;
        const char *end = ms - o->last_ms > 2000 ? "dead"
                          : !known ? "contact dead or lost"
                          : ms - o->task_since > TASK_MAX_MS ? "time out"
                          : (!threat && ms - o->task_since > TASK_MIN_MS) ? "no longer a threat" : NULL;
        if (end) {
            o->task = 0;
            event(host, "plan: army %d seat %d stops intercepting slot %d (%s)", army, i, o->task_tgt, end);
            continue;
        }
        task_point(o, x, z);
        ++assigned[o->task_tgt & 63];
        ++tasks;
    }

    if (cfg.alert) {
        /* s28: rear-area interception hangs on the team's alert record (trusted evidence, 30 s hold, one
         * responder per alert), not on a report existing right now (ground_tactics.inc). */
        tac_alert_plan(host, army, ms, osx, osz, ux, uz, front, np, live, passive, task_max);
        return;
    }
    /* Interceptors: 1 per threat (s39: the only class is "behind our line"), within
     * the task and passive budgets (team_census). */
    for (int k = 0; k < nth; ++k) {
        while (assigned[th[k].t] < 1 && tasks < task_max && passive < live / PASSIVE_DIV) {
            int best = -1;
            float bd = INTERCEPT_MAX_M;
            for (int i = 1; i < MAX_SEATS; ++i) {
                const lane_seat_t *o = &lseat[i];
                if (!o->unit || !o->n || o->scene_epoch != ground_scene_epoch || o->army != army || o->task || ms - o->last_ms > 1500 || ms - o->born_ms < SPAWN_CALM_MS
                    || o->mode == M_ENGAGE || o->mode == M_RETREAT || o->pusher || o->cmd_release
                    || (o->cmd_apply_ms && o->cmd_ntype == OB_CAPTURE)
                    || (o->cmd_apply_ms && o->cmd_ntype == OB_OVERWATCH && o->cmd_nid >= CMD_POST_ID_BASE)
                    || (o->holding && o->route[o->step].pt == SLOT_PT)
                    || o->cmd_type == OB_SUPPORT || o->cmd_type == OB_OVERWATCH)    /* S2: wingmen / overwatch stay */
                    continue;
                float d = dist2d(o->x, o->z, th[k].x, th[k].z);
                if (d < bd) {
                    bd = d;
                    best = i;
                }
            }
            if (best < 0)
                break;
            lane_seat_t *o = &lseat[best];
            o->task = 1;
            o->task_tgt = th[k].t;
            o->task_since = ms;
            o->task_why = th[k].why;
            o->task_point = 0;          /* s39: no task defends a point of ours any more */
            o->task_side = assigned[th[k].t] ? -1.0f : 1.0f;
            task_point(o, th[k].x, th[k].z);
            ++assigned[th[k].t];
            ++tasks;
            ++passive;
            event(host, "plan: army %d sends seat %d to intercept slot %d (%s) at (%.0f,%.0f), %.0f m away; firing spot (%.0f,%.0f)",
                  army, best, th[k].t, th[k].why, th[k].x, th[k].z, bd, o->task_x, o->task_z);
        }
    }
}