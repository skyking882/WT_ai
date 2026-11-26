/* ---- Covered approach (stage three, 2026-11-15) --------------------------------
 * What a tank does with the fire-threat field (ground_threat.inc) on its way to
 * an objective. Findings from the battle logs: a team charged a point down the open main
 * road; tanks held at a point by enemies outside it sat there instead of going
 * round them.
 *
 * 1. Route. A fresh life picks the tactical route of lowest cost under the
 *    field (pick_troute). A tank sent to a point from where it stands gets the
 *    commander's A* under the field instead of a straight line (approach_plan).
 *    A route in use is measured again every AP_EVAL_MS; when the fire on what is
 *    left of it has grown by a third (a team-mate was hit ahead, an enemy showed
 *    up), a better way round is taken if there is one (approach_review). The A*
 *    makes the cells of the team-mates' own routes dearer, so that a point is
 *    not approached down one lane only.
 * 2. Bounds. An exposed stretch is route under a threat of AP_TH_HI or more for
 *    at least AP_RUN_MIN_M. When one begins within AP_TRIGGER_M the tank drives
 *    into the best cover cell at its near end (AP_TO_COVER), stops and looks
 *    over it (AP_OBSERVE), then crosses it in one go (AP_DASH): while it dashes
 *    it makes no fire stops beyond close range and starts no cover hop, exactly
 *    like the tank nearest a point (only when the stretch ends in cover within
 *    the AP_LOOK_M looked at, or at the goal: with no cover in sight to run to,
 *    it keeps its fire stops). Events end the look, not a clock alone:
 *      - nothing shows up for AP_OBSERVE_MS: go;
 *      - a team-mate that arrived later is watching the stretch: go after
 *        AP_HURRY_MS (it covers the crossing, then follows);
 *      - a located enemy has a fire line onto the stretch: fight from the cover
 *        up to AP_CONTACT_MS (the route review looks for a way round meanwhile);
 *      - the team is behind on points (pressure 0.65 or more): AP_HURRY_MS.
 *    Without a cover cell, or already standing in the open, it just dashes.
 * 3. Assembly. The stop before a point that is not ours (within AP_ASSEMBLY_M)
 *    waits for the others sent there: it ends as soon as a second tank is at
 *    its own stop or at the point, like any other stop when nobody else is on
 *    the way (AP_OBSERVE_MS), and
 *    after AP_ASSEMBLY_MS at the latest (a design rule: staging
 *    before a point stays short).
 * 4. Flank. Our tanks at a point are fired at by a located enemy outside it:
 *    the commander gets a flank objective beside that enemy (flank_objectives),
 *    an OB_PUSH with ids from FLANK_ID_BASE, driven like every region order on
 *    the A* under the field, which keeps out of that enemy's line of fire.
 * Every state is bounded in time. A stop proposes "not moving" on purpose, so
 * nothing above reads it as a stuck tank. WT_GROUND_APPROACH=0 switches all of
 * it off (routes, bounds, flank, the CVF_AHEAD rule for cover hops) and leaves
 * the layer below as it was; stage two's immobility rules are not part of it.
 * Logs: `approach:` lines (AP_LOGS_PER_MIN) and a minute summary. */
#define AP_LOOK_M 320.0f
#define AP_TRIGGER_M 70.0f
#define AP_RUN_MIN_M 48.0f
#define AP_COVER_R 30.0f        /* the stop lies this close to where the stretch begins ... */
#define AP_COVER_MAX_M 90.0f    /* ... and this close to the tank */
#define AP_OBSERVE_MS 4000u
#define AP_HURRY_MS 1500u
#define AP_CONTACT_MS 20000u
#define AP_ASSEMBLY_MS 15000u
#define AP_ASSEMBLY_M 260.0f
#define AP_MATE_M 250.0f
#define AP_MAX_BOUNDS 4         /* stops on one route */
#define AP_COOL_MS 3000u
#define AP_EVAL_MS 2000u
#define AP_REPLAN_GAP_MS 20000u
#define AP_PLAN_GAP_MS 150u     /* searches of all tanks are this far apart (shared with the flank planner) */
#define AP_PLAN_MIN_M 220.0f
#define AP_LOGS_PER_MIN 40
#define AP_SAMPLES 48

static struct {
    int logs, dropped;
    uint64_t minute_ms;
    int stops, assemblies, dashes, no_cover, unreached, plans, no_route, changes, flanks;
    float watched_s;
} apst;

static void approach_log(void *host, const char *fmt, ...)
{
    if (apst.logs >= AP_LOGS_PER_MIN) {
        ++apst.dropped;
        return;
    }
    ++apst.logs;
    char msg[400];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof msg, fmt, args);
    va_end(args);
    event(host, "%s", msg);
}

static void approach_minute(void *host, uint64_t ms)
{
    if (!approach_on())
        return;
    if (!apst.minute_ms)
        apst.minute_ms = ms + 60000;
    if (ms < apst.minute_ms)
        return;
    apst.minute_ms = ms + 60000;
    event(host, "approach: last 60 s %d stops in cover (%d assemblies, %.1f s each on average), %d dashes, %d stretches "
          "without cover, %d covers not reached | %d routes planned, %d searches without a route, %d routes changed on the "
          "way | %d flank objectives | %d lines dropped", apst.stops, apst.assemblies,
          apst.stops ? apst.watched_s / (float)apst.stops : 0.0f, apst.dashes, apst.no_cover, apst.unreached, apst.plans, apst.no_route,
          apst.changes, apst.flanks, apst.dropped);
    apst.logs = apst.dropped = apst.stops = apst.assemblies = apst.dashes = apst.no_cover = apst.unreached = 0;
    apst.plans = apst.no_route = apst.changes = apst.flanks = 0;
    apst.watched_s = 0.0f;
}

/* Where the order leads: the point of a lane order, the region of the others. */
static void approach_goal(const lane_seat_t *s, float *gx, float *gz)
{
    if (s->cmd_type >= OB_SUPPORT) {
        *gx = s->cmd_x;
        *gz = s->cmd_z;
    } else {
        *gx = lanes[s->lane].zx;
        *gz = lanes[s->lane].zz;
    }
}

typedef struct { float x, z, at, t; } ap_sample_t;      /* at: metres of route from the tank, t: threat there */

static int approach_leg(int army, ap_sample_t *p, int n, float *x, float *z, float *at, float tx, float tz)
{
    const float d = dist2d(*x, *z, tx, tz);
    if (!(d > 1.0f) || !isfinite(d))
        return n;
    const int steps = (int)ceilf(d / AP_SAMPLE_M);
    for (int j = 1; j <= steps && n < AP_SAMPLES; ++j) {
        const float f = (float)j / (float)steps;
        if (*at + d * f > AP_LOOK_M + AP_SAMPLE_M)
            break;
        const float px = *x + (tx - *x) * f, pz = *z + (tz - *z) * f;
        p[n++] = (ap_sample_t){px, pz, *at + d * f, threat_at(army, px, pz)};
    }
    *at += d;
    *x = tx;
    *z = tz;
    return n;
}

/* The tank's position, then its route and the way on to the goal, AP_LOOK_M of it. */
static int approach_samples(const lane_seat_t *s, float gx, float gz, ap_sample_t *p)
{
    const troute_t *r = &troutes[s->tr];
    float x = s->x, z = s->z, at = 0.0f;
    int n = 0;
    p[n++] = (ap_sample_t){x, z, 0.0f, threat_at(s->army, x, z)};
    for (int k = s->wpi; k < s->wpn && k < MAX_TWP && at < AP_LOOK_M; ++k)
        n = approach_leg(s->army, p, n, &x, &z, &at, r->x[k], r->z[k]);
    if (at < AP_LOOK_M)
        n = approach_leg(s->army, p, n, &x, &z, &at, gx, gz);
    return n;
}

/* The first exposed stretch: samples begin .. end, end being the first one back
 * in cover (two calm samples in a row) or the last one looked at. */
static int approach_stretch(const ap_sample_t *p, int n, int *begin, int *end)
{
    int a = -1;
    for (int i = 0; i < n && a < 0; ++i)
        if (p[i].t >= AP_TH_HI)
            a = i;
    if (a < 0)
        return 0;
    int b = n - 1, calm = 0;
    for (int i = a + 1; i < n; ++i) {
        calm = p[i].t < AP_TH_LO ? calm + 1 : 0;
        if (calm >= 2) {
            b = i - 1;
            break;
        }
    }
    *begin = a;
    *end = b;
    return 1;
}

/* Where fire onto the ground around (x, z) is expected from: the located enemy
 * nearest it, else the enemy's side of the spawn axis. */
static void approach_face(int army, float x, float z, float *fx, float *fz)
{
    if (cover_threat(army, x, z, strat.m->vis_range, fx, fz))
        return;
    const strat_meta_t *m = strat.m;
    const float sg = army == 1 ? 1.0f : -1.0f;
    *fx = x + sg * m->axx * 400.0f;
    *fz = z + sg * m->axz * 400.0f;
}

/* A located enemy (seen in the last 8 s) has a fire line onto the stretch. */
static int approach_contact(const lane_seat_t *s)
{
    const strat_view_t *v = &sview[s->army];
    const int cells[3] = {
        strat_cellof(s->ap_run_x, s->ap_run_z),
        strat_cellof(0.5f * (s->ap_run_x + s->ap_end_x), 0.5f * (s->ap_run_z + s->ap_end_z)),
        strat_cellof(s->ap_end_x, s->ap_end_z)
    };
    for (int k = 0; v->ok && k < v->ne && k < 64; ++k) {
        if (v->en[k].age > 8.0f || v->en[k].cell < 0)
            continue;
        for (int j = 0; j < 3; ++j)
            if (cells[j] >= 0 && strat_sees(strat.visf, v->en[k].cell, cells[j]))
                return 1;
    }
    return 0;
}

/* later: team-mates stopped near it that began to watch after it did (they
 * cover its crossing). ready: others sent to its point that are at their own
 * assembly stop or at the point already. coming: others sent there, still
 * driving and close enough to be worth the wait. */
static void approach_mates(const lane_seat_t *s, int seat, uint64_t ms, int *later, int *ready, int *coming)
{
    const lane_t *l = &lanes[s->lane];
    const int point = cmd_lane_point(s->lane);
    *later = *ready = *coming = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (i == seat || !o->unit || o->army != s->army || o->scene_epoch != ground_scene_epoch || ms - o->last_ms > 2000)
            continue;
        if (o->ap_state == AP_OBSERVE && o->ap_since > s->ap_since && dist2d(o->x, o->z, s->x, s->z) < AP_MATE_M)
            ++*later;
        if (o->cmd_type >= OB_SUPPORT || point < 0 || cmd_lane_point(o->lane) != point)
            continue;
        const float d = dist2d(o->x, o->z, l->zx, l->zz);
        if ((o->ap_state == AP_OBSERVE && o->ap_assembly) || d < l->zr + 40.0f
            || (o->ap_state == AP_DASH && d < AP_ASSEMBLY_M + 80.0f))
            ++*ready;
        else if (d < 450.0f && o->wants_move && hypotf(o->vx, o->vz) > 0.5f)
            ++*coming;
    }
}

/* Cells the team-mates' own routes use in their next 400 m. */
static void approach_load(int army, int seat, uint64_t ms)
{
    memset(th.load, 0, (size_t)th.n * sizeof(float));
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (i == seat || !o->unit || o->army != army || o->scene_epoch != ground_scene_epoch || ms - o->last_ms > 2000
            || !nav_route_active(o))
            continue;
        const troute_t *r = &troutes[o->tr];
        float x = o->x, z = o->z, at = 0.0f;
        int last = -1;
        for (int k = o->wpi; k < o->wpn && k < MAX_TWP && at < 400.0f; ++k) {
            const float d = dist2d(x, z, r->x[k], r->z[k]);
            const int steps = d > 1.0f && isfinite(d) ? (int)ceilf(d / AP_SAMPLE_M) : 0;
            for (int j = 1; j <= steps; ++j) {
                const float f = (float)j / (float)steps;
                const int c = strat_cellof(x + (r->x[k] - x) * f, z + (r->z[k] - z) * f);
                if (c >= 0 && c != last) {
                    th.load[c] += 1.0f;
                    last = c;
                }
            }
            if (isfinite(d))
                at += d;
            x = r->x[k];
            z = r->z[k];
        }
    }
}

/* The commander's A* under the field from the tank to (gx, gz).
 * 1 found, 0 no route, -1 another search ran too recently. */
static int approach_search(lane_seat_t *s, int seat, uint64_t ms, float gx, float gz, troute_t *out, threat_measure_t *measure)
{
    if (ms < strat.flank_plan_ms)
        return -1;
    strat.flank_plan_ms = ms + AP_PLAN_GAP_MS;
    approach_load(s->army, seat, ms);
    memset(out, 0, sizeof *out);
    th.load_on = 1;
    out->n = cmd_astar(s->army, s->x, s->z, gx, gz, out->x, out->z);
    th.load_on = 0;
    if (out->n < 1) {
        ++apst.no_route;
        return 0;
    }
    out->army = s->army;
    out->variant = TV_COVER;
    out->sx = s->x;
    out->sz = s->z;
    out->zx = gx;
    out->zz = gz;
    *measure = threat_measure_route(s->army, s->x, s->z, out, 0, out->n);
    return 1;
}

/* A lane order then goes on from the route's end into its slot at the point,
 * not back to an earlier lane point. */
static int approach_install(void *host, lane_seat_t *s, int seat, uint64_t ms, const troute_t *route,
                            const threat_measure_t *m, const char *why)
{
    if (!nav_install_route(s, CMD_TR(seat), route, 0, route->n, ms))
        return 0;
    if (s->cmd_type < OB_SUPPORT && s->route[s->step].pt != SLOT_PT)
        lane_goto_step(host, s, seat, lane_slot_step(s), ms, "approach route");
    s->route_interrupted = 1;
    s->native_tried = s->route_native_only = 0;
    s->ap_replan_ms = ms + AP_REPLAN_GAP_MS;    /* one search per tank in that time, whatever becomes of the route */
    ++apst.plans;
    approach_log(host, "approach: seat %d %s to (%.0f,%.0f): %d waypoints, %.0f m, %.0f m of it under fire, threat up to %.1f",
                 seat, why, route->zx, route->zz, route->n, m->length, m->exposed, m->peak);
    return 1;
}

/* An assault tank that would drive straight at its point from where it stands. */
static void approach_plan(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (ms < s->ap_eval_ms || ms < s->ap_replan_ms || s->cmd_type >= OB_SUPPORT)
        return;                     /* region orders get their route from cmd_route */
    const lane_t *l = &lanes[s->lane];
    if (l->kind != 0 || s->role != LROLE_ASSAULT || s->mode == M_ENGAGE || motion_committed(s, ms))
        return;
    s->ap_eval_ms = ms + AP_EVAL_MS;
    if (dist2d(s->x, s->z, l->zx, l->zz) < fmaxf(AP_PLAN_MIN_M, l->zr + 150.0f))
        return;
    troute_t route;
    threat_measure_t m;
    const int found = approach_search(s, seat, ms, l->zx, l->zz, &route, &m);
    if (found < 0)
        s->ap_eval_ms = ms + 2 * AP_PLAN_GAP_MS;
    else if (!found || !approach_install(host, s, seat, ms, &route, &m, "takes a covered route"))
        s->ap_eval_ms = ms + 10000;
}

/* What is left of the route in use, and the way on from its end to the goal. */
static threat_measure_t approach_remaining(const lane_seat_t *s, float gx, float gz)
{
    const troute_t *r = &troutes[s->tr];
    threat_measure_t m = threat_measure_route(s->army, s->x, s->z, r, s->wpi, s->wpn);
    threat_measure_leg(s->army, &m, r->x[s->wpn - 1], r->z[s->wpn - 1], gx, gz);
    return m;
}

/* The route in use against the field as it is now. Returns 1 when it was replaced. */
static int approach_review(void *host, lane_seat_t *s, int seat, uint64_t ms, float gx, float gz)
{
    if (ms < s->ap_eval_ms)
        return 0;
    s->ap_eval_ms = ms + AP_EVAL_MS;
    if (s->cmd_type < OB_SUPPORT && lanes[s->lane].kind != 0)
        return 0;                   /* flank lanes: flank_route_update */
    if (s->ap_pick_blind && threat_ready(s->army)) {
        /* Chosen in the first second of the battle, before the field existed. */
        s->ap_pick_blind = 0;
        if (s->nav.route_tactical && dist2d(s->x, s->z, s->sx, s->sz) < 80.0f && !motion_committed(s, ms)) {
            pick_troute(host, s, seat);
            return 1;
        }
    }
    const threat_measure_t now = approach_remaining(s, gx, gz);
    if (now.threat < 1.3f * s->ap_route_threat + 6.0f)
        return 0;
    if (ms < s->ap_replan_ms || s->mode == M_ENGAGE || motion_committed(s, ms)
        || threat_at(s->army, s->x, s->z) >= AP_TH_HI || dist2d(s->x, s->z, gx, gz) < AP_PLAN_MIN_M)
        return 0;
    troute_t route;
    threat_measure_t alt;
    const int found = approach_search(s, seat, ms, gx, gz, &route, &alt);
    if (found < 0) {
        s->ap_eval_ms = ms + 2 * AP_PLAN_GAP_MS;
        return 0;
    }
    s->ap_replan_ms = ms + AP_REPLAN_GAP_MS;
    s->ap_route_threat = now.threat;        /* what it is now is the yardstick from here on */
    if (!found || threat_cost(&alt) + 12.0f > threat_cost(&now) || threat_cost(&alt) > 0.85f * threat_cost(&now))
        return 0;
    if (!approach_install(host, s, seat, ms, &route, &alt, "leaves a route that came under fire"))
        return 0;
    ++apst.changes;
    return 1;
}

static void approach_dash(void *host, lane_seat_t *s, int seat, uint64_t ms, const char *why)
{
    s->ap_state = AP_DASH;
    s->ap_since = ms;
    /* From here to the stretch and through it: the crossing is over when that
     * much has been driven, however the route bends. */
    s->ap_plan_m = s->ap_run_m + dist2d(s->x, s->z, s->ap_run_x, s->ap_run_z);
    s->ap_odo = 0.0f;
    s->ap_px = s->x;
    s->ap_pz = s->z;
    s->ap_until = ms + 8000 + (uint64_t)(s->ap_plan_m / 3.0f * 1000.0f);
    s->route_interrupted = 1;
    if (!why)
        return;
    ++apst.dashes;
    approach_log(host, "approach: seat %d crosses %.0f m under fire to (%.0f,%.0f) %s (%s)", seat, s->ap_run_m,
                 s->ap_end_x, s->ap_end_z, s->ap_rush ? "without stopping" : "with no cover in sight, fire stops allowed", why);
}

/* Looks along the route for a stretch under fire that begins near. Returns 1
 * when it changed the state (to AP_TO_COVER or AP_DASH). */
static int approach_look(void *host, lane_seat_t *s, int seat, uint64_t ms, float gx, float gz)
{
    if (ms < s->ap_next_ms)
        return 0;
    s->ap_next_ms = ms + 500;
    if (ms < s->ap_cool_until || s->mode == M_ENGAGE || motion_committed(s, ms))
        return 0;
    ap_sample_t p[AP_SAMPLES];
    int a = 0, b = 0;
    const int n = approach_samples(s, gx, gz, p);
    if (!approach_stretch(p, n, &a, &b) || p[a].at > AP_TRIGGER_M || p[b].at - p[a].at < AP_RUN_MIN_M)
        return 0;
    float peak = 0.0f;
    for (int i = a; i <= b; ++i)
        peak = fmaxf(peak, p[i].t);
    const ap_sample_t *mid = &p[(a + b) / 2];
    s->ap_run_x = p[a].x;
    s->ap_run_z = p[a].z;
    s->ap_end_x = p[b].x;
    s->ap_end_z = p[b].z;
    s->ap_run_m = p[b].at - p[a].at;
    s->ap_final = dist2d(p[b].x, p[b].z, gx, gz) < 40.0f;
    /* Running without a stop to fire makes sense towards cover: the stretch ends
     * within sight, or at the goal. Across open ground with no end in sight the
     * tank keeps its fire stops. */
    s->ap_rush = b < n - 1 || s->ap_final;
    s->ap_assembly = 0;
    approach_face(s->army, mid->x, mid->z, &s->ap_face_x, &s->ap_face_z);
    if (a == 0 || s->ap_bounds >= AP_MAX_BOUNDS) {
        if (!s->ap_rush)
            return 0;
        approach_dash(host, s, seat, ms, NULL);    /* in the open already: no place to stop, on to the cover ahead */
        return 1;
    }
    const ap_sample_t *entry = &p[a - 1];
    cover_t c;
    if (!cover_find(seat, s->army, COV_BOUND, entry->x, entry->z, s->ap_face_x, s->ap_face_z, 0.0f, AP_COVER_R,
                    CVF_VIEW | CVF_CALM, &c)
        || dist2d(s->x, s->z, c.x, c.z) > AP_COVER_MAX_M) {
        ++apst.no_cover;
        approach_dash(host, s, seat, ms, "no cover before it");
        return 1;
    }
    const lane_t *l = &lanes[s->lane];
    s->ap_state = AP_TO_COVER;
    ++s->ap_bounds;         /* counted when chosen: a cover that cannot be reached is not tried for ever */
    s->ap_x = c.x;
    s->ap_z = c.z;
    s->ap_since = ms;
    s->ap_until = ms + 4000 + (uint64_t)(dist2d(s->x, s->z, c.x, c.z) / COV_HOP_SPEED * 1000.0f);
    s->ap_best = dist2d(s->x, s->z, c.x, c.z);
    s->ap_progress_ms = ms;
    s->ap_assembly = s->cmd_type < OB_SUPPORT && l->kind == 0 && zone_owner(host, l->zx, l->zz) != s->army
                     && dist2d(c.x, c.z, l->zx, l->zz) < AP_ASSEMBLY_M;
    cover_used(host, seat, COV_BOUND, &c, cover_here(s->x, s->z, s->ap_face_x, s->ap_face_z), s->ap_face_x, s->ap_face_z,
               entry->x, entry->z);
    approach_log(host, "approach: seat %d: %.0f m under fire begin %.0f m ahead (threat up to %.1f), stops in cover at "
                 "(%.0f,%.0f) first%s", seat, s->ap_run_m, p[a].at, peak, c.x, c.z,
                 s->ap_assembly ? " and waits for the others (assembly)" : "");
    return 1;
}

/* Every tick, before anything reads the state (seat_rush in the reactions, the
 * cover claim, the parking budget, other tanks' assembly): approach_propose is
 * not called on every path (a parked or posted tank, an order that parks on
 * arrival), so a state nobody drives any more ends here, as does one whose
 * time is up or whose order has changed. */
static void approach_expire(lane_seat_t *s, uint64_t ms)
{
    if (s->ap_state == AP_NONE)
        return;
    if (approach_on() && !cmd_is_post(s) && !s->task && !s->holding && !s->cmd_parked && s->ap_task == s->nav.task_id
        && (s->ap_state == AP_OBSERVE ? ms - s->ap_since <= AP_CONTACT_MS + 1000u : ms < s->ap_until))
        return;
    if (s->ap_state == AP_DASH)
        s->ap_cool_until = ms + AP_COOL_MS;
    s->ap_state = AP_NONE;
    s->ap_assembly = 0;
}

/* Called where a route would be followed (lanes_propose, cmd_drive). Returns 1
 * when it proposed the tank's destination for this tick. */
static int approach_propose(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (!approach_on() || cmd_is_post(s) || s->task || s->holding || s->cmd_parked || ms < s->l1_until) {
        s->ap_state = AP_NONE;
        return 0;
    }
    float gx, gz;
    approach_goal(s, &gx, &gz);
    if (s->ap_state == AP_DASH) {
        const float step = dist2d(s->x, s->z, s->ap_px, s->ap_pz);
        if (step < 30.0f)
            s->ap_odo += step;
        s->ap_px = s->x;
        s->ap_pz = s->z;
        /* Over: another order or route, at the goal, the planned distance driven,
         * or at the far end with at least half of it driven (a route that bends
         * back can end near where it began). */
        const int over = s->ap_generation != s->route_generation || s->ap_task != s->nav.task_id
                         || dist2d(s->x, s->z, gx, gz) < 24.0f || s->ap_odo >= s->ap_plan_m - 8.0f
                         || (dist2d(s->x, s->z, s->ap_end_x, s->ap_end_z) < 24.0f && s->ap_odo >= 0.5f * s->ap_plan_m);
        if (!over && ms < s->ap_until)
            return 0;               /* the route goes on underneath */
        s->ap_state = AP_NONE;
        s->ap_assembly = 0;
        s->ap_cool_until = ms + AP_COOL_MS;
    }
    if (!nav_route_active(s)) {
        s->ap_state = AP_NONE;
        approach_plan(host, s, seat, ms);
        if (!nav_route_active(s))
            return 0;
    }
    if (s->ap_generation != s->route_generation || s->ap_task != s->nav.task_id) {
        /* A route this state has not seen: what is left of it now is the yardstick. */
        const threat_measure_t m = approach_remaining(s, gx, gz);
        s->ap_generation = s->route_generation;
        s->ap_task = s->nav.task_id;
        s->ap_state = AP_NONE;
        s->ap_assembly = s->ap_bounds = 0;
        s->ap_route_threat = m.threat;
    }
    if (s->ap_state == AP_NONE) {
        if (approach_review(host, s, seat, ms, gx, gz) || !approach_look(host, s, seat, ms, gx, gz)
            || s->ap_state != AP_TO_COVER)
            return 0;
    }
    if (s->ap_state == AP_TO_COVER) {
        const float left = dist2d(s->x, s->z, s->ap_x, s->ap_z);
        if (left <= s->ap_best - COV_HOP_PROGRESS_M || s->motion_state == GM_TURNING) {
            s->ap_best = fminf(left, s->ap_best);
            s->ap_progress_ms = ms;
        }
        if (left < COV_ARRIVE_M) {
            s->ap_state = AP_OBSERVE;
            s->ap_since = ms;
            ++apst.stops;
            apst.assemblies += s->ap_assembly;
        } else if (ms >= s->ap_until || ms - s->ap_progress_ms >= COV_HOP_STALL_MS) {
            ++apst.unreached;
            approach_dash(host, s, seat, ms, "the cover was not reached");
            return 0;
        } else {
            nav_propose(&s->nav, NAV_LANE, s->ap_x, s->ap_z, NAV_NATIVE, 1, 0, 0);
            return 1;
        }
    }
    /* AP_OBSERVE */
    const uint64_t held = ms - s->ap_since;
    const int hurry = sview[s->army].point_pressure >= 0.65f;
    int later, ready, coming;
    approach_mates(s, seat, ms, &later, &ready, &coming);
    uint64_t need = hurry ? AP_HURRY_MS : AP_OBSERVE_MS;
    const char *why = hurry ? "no time to look longer" : "nothing showed up";
    if (s->ap_assembly) {
        if (ready) {
            need = AP_HURRY_MS;
            why = "the others are there";
        } else if (coming && !hurry) {
            need = AP_ASSEMBLY_MS;
            why = "the others did not come in time";
        } else if (!hurry) {
            why = "nobody else is on the way";
        }
    } else if (later) {
        need = AP_HURRY_MS;
        why = "a team-mate watches the stretch";
    } else if (!hurry && !s->ap_final && approach_contact(s)) {
        need = AP_CONTACT_MS;
        why = "the enemy still covers it";
    }
    if (held >= need) {
        apst.watched_s += (float)held / 1000.0f;
        approach_dash(host, s, seat, ms, why);
        return 0;
    }
    nav_propose(&s->nav, NAV_LANE, s->ap_x, s->ap_z, NAV_PARK, 0, 0, 0);
    if (!s->face) {
        s->face = 1;
        s->face_x = s->ap_face_x;
        s->face_z = s->ap_face_z;
    }
    s->ax = s->x;
    s->az = s->z;
    s->anchor_ms = ms;
    return 1;
}

/* ---- Flank objectives ----------------------------------------------------------
 * A located enemy outside a point (beyond its circle + FLANK_NEAR_M, within
 * FLANK_FAR_M, seen in the last FLANK_AGE_S) has a fire line into it while tanks
 * of ours are there. Once that has lasted FLANK_MIN_MS, or at once when one of
 * them is hit, the commander gets up to two objectives beside that enemy: a
 * passable cell 180-260 m from it, 75-135 degrees off its line of sight to the
 * point, with a fire line to it, preferring hull cover and little other threat.
 * They are OB_PUSH objectives with ids FLANK_ID_BASE + 2 x point + side, need 1,
 * so the whole order machinery applies unchanged: assignment by utility,
 * cmd_route (the A* under the field keeps out of that enemy's fire), a covered
 * stop spot at the end (cover_dest), parking, reactions. A position is kept
 * while the same enemy stays within CMD_MOVE_M of where it was; the objectives
 * outlive the last sighting by FLANK_KEEP_MS so that a flanker under way is not
 * called back by one lost contact. */
#define FLANK_ID_BASE 16
#define FLANK_NEAR_M 80.0f
#define FLANK_FAR_M 800.0f
#define FLANK_AGE_S 20.0f
#define FLANK_MIN_MS 6000u
#define FLANK_KEEP_MS 12000u
#define FLANK_VALUE 96.0f
#define FLANK_MAX 2             /* flank objectives per army at a time */
typedef struct {
    int slot, ok[2];
    float ex, ez, x[2], z[2];
    uint64_t since, last_ms, log_ms;
} flank_state_t;
static flank_state_t flank_state[3][8];

static int flank_position(int army, float px, float pz, float ex, float ez, int side, float *ox, float *oz)
{
    static const float angle[3] = {1.31f, 1.83f, 2.36f}, reach[2] = {180.0f, 260.0f};
    const int ecell = strat_cellof(ex, ez);
    float ux, uz, best = -1e9f;
    unit_dir(ex, ez, px, pz, &ux, &uz);         /* the way that enemy looks: at the point */
    if (ecell < 0 || (ux == 0.0f && uz == 0.0f))
        return 0;
    for (int a = 0; a < 3; ++a)
        for (int k = 0; k < 2; ++k) {
            const float c = cosf(angle[a]), sn = side ? -sinf(angle[a]) : sinf(angle[a]);
            const float fx = ex + reach[k] * (c * ux - sn * uz), fz = ez + reach[k] * (c * uz + sn * ux);
            float cx = 0.0f, cz = 0.0f;
            const int cell = cmd_snap(fx, fz, &cx, &cz);
            if (cell < 0 || dist2d(cx, cz, fx, fz) > 80.0f || !strat_sees(strat.visf, cell, ecell))
                continue;
            const float d = dist2d(cx, cz, ex, ez);
            if (d < 100.0f || d > 420.0f)
                continue;
            const float score = 0.6f * (float)strat.cell[cell].cover[army - 1] / 255.0f
                                - 0.5f * fmaxf(0.0f, threat_cell(army, cell) - 1.0f)
                                - dist2d(cx, cz, px, pz) / 1000.0f + (a ? 0.15f : 0.0f);
            if (score > best) {
                best = score;
                *ox = cx;
                *oz = cz;
            }
        }
    return best > -1e8f;
}

int flank_objectives(void *host, int army, uint64_t ms, cmd_obj_t *ob, int n)
{
    if (!approach_on())
        return n;
    const strat_view_t *v = &sview[army];
    int made = 0;
    for (int k = 0; k < nlanes && n < CMD_MAX_OBJ && made < FLANK_MAX; ++k) {
        const lane_t *l = &lanes[k];
        const int cap = l->army == army && l->kind == 0 ? cmd_cap_index(l) : -1;
        if (cap < 0)
            continue;
        flank_state_t *f = &flank_state[army][cap];
        const int pcell = strat_cellof(l->zx, l->zz);
        int here = 0, hit = 0, e = -1;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            if (!cmd_live(o, i, army, ms) || dist2d(o->x, o->z, l->zx, l->zz) > l->zr + 60.0f)
                continue;
            ++here;
            hit += ring_count(hits_taken_ms[i], ms, 15000) > 0;
        }
        float nearest = FLANK_FAR_M;
        for (int j = 0; here && pcell >= 0 && j < v->ne && j < 64; ++j) {
            const float d = dist2d(v->en[j].x, v->en[j].z, l->zx, l->zz);
            if (v->en[j].age > FLANK_AGE_S || d < l->zr + FLANK_NEAR_M || d >= nearest || v->en[j].cell < 0
                || !strat_sees(strat.visf, v->en[j].cell, pcell))
                continue;
            e = j;
            nearest = d;
        }
        if (e >= 0) {
            if (!f->since || f->slot != v->en[e].slot) {
                f->since = ms;
                f->slot = v->en[e].slot;
                f->ok[0] = f->ok[1] = 0;
            }
            if (dist2d(f->ex, f->ez, v->en[e].x, v->en[e].z) > CMD_MOVE_M) {
                f->ex = v->en[e].x;
                f->ez = v->en[e].z;
                f->ok[0] = f->ok[1] = 0;
            }
            f->last_ms = ms;
        } else if (!f->since || ms - f->last_ms > FLANK_KEEP_MS) {
            f->since = 0;
            continue;
        }
        if (ms - f->since < FLANK_MIN_MS && !hit)
            continue;
        int sides = 0;
        for (int side = 0; side < 2 && n < CMD_MAX_OBJ && made < FLANK_MAX; ++side) {
            if (!f->ok[side])
                f->ok[side] = flank_position(army, l->zx, l->zz, f->ex, f->ez, side, &f->x[side], &f->z[side]) ? 1 : -1;
            if (f->ok[side] < 0)
                continue;
            ob[n++] = (cmd_obj_t){OB_PUSH, FLANK_ID_BASE + 2 * cap + side, 1, strat_band(strat_lat(f->x[side], f->z[side])),
                                  -1, 0, -1, f->x[side], f->z[side], FLANK_VALUE, 0};
            ++made;
            ++sides;
        }
        if (sides && ms >= f->log_ms) {
            f->log_ms = ms + 30000;
            ++apst.flanks;
            event(host, "cmd: army %d flank at point %d: enemy slot %d at (%.0f,%.0f), %.0f m out, fires at %d of ours there "
                  "(%d hit); %d position%s beside it: push %d (%.0f,%.0f)%s", army, cap, f->slot, f->ex, f->ez,
                  dist2d(f->ex, f->ez, l->zx, l->zz), here, hit, sides, sides > 1 ? "s" : "",
                  FLANK_ID_BASE + 2 * cap + (f->ok[0] > 0 ? 0 : 1), f->ok[0] > 0 ? f->x[0] : f->x[1],
                  f->ok[0] > 0 ? f->z[0] : f->z[1], sides > 1 ? " and the other side" : "");
        }
    }
    return n;
}
