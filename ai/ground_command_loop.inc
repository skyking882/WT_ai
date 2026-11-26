/* s48: the commander's tick and the post leave rules moved out of ground_brain.c, in their original order
 * (s46 lines 6600-7169: cmd_census, cmd_near_times, cmd_tick; 9190-9313: roads_post_leave, roads_post_check).
 * Compiled only through ground_command.c, after ground_teamplan.inc. */
static void cmd_census(void *host, int army, uint64_t ms, const cmd_obj_t *ob, int nob, int all_ours)
{
    const strat_meta_t *m = strat.m;
    float pline = 0.0f;
    for (uint32_t k = 0; k < m->ncap && k < 8; ++k)
        pline += strat_depth(army, m->cap[k][0], m->cap[k][1]) / (float)m->ncap;
    int live = 0, off = 0, pas = 0, idle = 0, past = 0, stale = 0, by[OB_N] = {0}, occ[8] = {0};
    int advancing = 0, cap_active = 0, cap_pending = 0, stopped = 0;
    const strat_view_t *v = &sview[army];
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms))
            continue;
        ++live;
        ++by[o->cmd_type];
        int found = !o->cmd_type;
        for (int k = 0; k < nob && !found; ++k)
            found = ob[k].type == o->cmd_type && ob[k].id == o->cmd_id;
        stale += !found;     /* its objective is gone, not reassigned yet */
        int c = seat_class(host, o, ms);
        int advancing_now = o->vx * o->vx + o->vz * o->vz > 0.25f && seat_advancing(o, ms);
        advancing += advancing_now;
        stopped += !seat_advancing(o, ms) && !cmd_guarding(host, o, v, ms);
        int active = cmd_objective_index(ob, nob, o->cmd_type, o->cmd_id, cmd_lane_point(o->lane));
        int contested = 0;
        fight_in_point(host, o, &contested);
        cap_active += contested || (active >= 0 && ob[active].type == OB_CAPTURE && cmd_capture_active(o, i, &ob[active], ms));
        cap_pending += o->cmd_apply_ms && o->cmd_ntype == OB_CAPTURE;
        if (c == C_AT_POINT || c == C_DEFEND || seat_useful_fire(o, ms)
            || (c < C_HOLD && advancing_now))
            ++off;
        else if (c >= C_HOLD && c != C_IDLE)
            ++pas;
        else
            ++idle;
        past += strat_depth(army, o->x, o->z) > pline + 250.0f;
        for (uint32_t k = 0; k < m->ncap && k < 8; ++k)
            occ[k] += dist2d(o->x, o->z, m->cap[k][0], m->cap[k][1]) < m->cap[k][2] + 40.0f;
    }
    /* full capture: how many are past the point line + 250 m 60 s later */
    if (all_ours && !cmd.allmine_ms[army]) {
        cmd.allmine_ms[army] = ms;
        cmd.allmine_logged[army] = 0;
        event(host, "cmd: army %d holds every point (%d bots past the point line + 250 m)", army, past);
    } else if (!all_ours) {
        cmd.allmine_ms[army] = 0;
    }
    if (all_ours && !cmd.allmine_logged[army] && ms - cmd.allmine_ms[army] >= 60000) {
        cmd.allmine_logged[army] = 1;
        event(host, "cmd: army %d 60 s after taking every point: %d of %d bots past the point line + 250 m", army, past, live);
    }
    if (ms < cmd.census_ms[army])
        return;
    int assigned = 0, enroute = 0, arrived = 0, effective = 0, blocked = 0, pending = 0, deferred = 0;
    int missing = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        const cmd_task_state_t *task = &cmd_tasks[i];
        if (!cmd_live(o, i, army, ms))      /* stage 2: wrecks were counted here until they respawned */
            continue;
        pending += task->pending.task_id != 0;
        deferred += task->pending.task_id && task->deferred_reason;
        if (!task->active.task_id)
            continue;
        const int fresh = cmd_report_fresh(task, ms);
        const int nav_state = fresh ? task->report.state : GNR_NEED_ROUTE;
        const int at_goal = nav_state == GNR_ARRIVED;
        const int moving = nav_state == GNR_FOLLOWING || nav_state == GNR_WAIT_TRAFFIC
                           || (nav_state == GNR_PLANNING && task->report.reason == GTR_GEOMETRY_UNKNOWN);
        arrived += at_goal;
        enroute += moving;
        assigned += !at_goal && !moving;
        missing += !fresh;
        blocked += cmd_report_blocked(task, ms);
        if (at_goal && !task->invalid_reason) {
            int contested = 0;
            int lane = fight_in_point(host, o, &contested);
            const int point = task->active.type == OB_CAPTURE || task->active.type == OB_HOLD;
            const int point_contribution = point && contested && lane == task->active.lane
                                           && zone_owner(host, lanes[lane].zx, lanes[lane].zz) >= 0;
            const int useful_fire = seat_useful_fire(o, ms) && (!cmd_is_post(o) || cmd_post_in_arc(o, o->ex, o->ez));
            effective += point_contribution || useful_fire;
        }
    }
    if (!live && !(assigned + enroute + arrived + pending))
        return;
    cmd.census_ms[army] = ms + CMD_CENSUS_MS;
    event(host, "task: census army %d accepted %d assigned %d enroute %d arrived %d effective %d blocked %d pending %d deferred %d nav_missing %d",
          army, assigned + enroute + arrived, assigned, enroute, arrived, effective, blocked, pending, deferred, missing);
    if (!live)
        return;
    char pts[64] = "";
    for (uint32_t k = 0; k < m->ncap && k < 8; ++k)
        snprintf(pts + strlen(pts), sizeof pts - strlen(pts), " %c%d", 'A' + k, occ[k]);
    float share = cmd.contact_s[army] > 0.0f ? 100.0f * cmd.backed_s[army] / cmd.contact_s[army] : -1.0f;
    event(host, "cmd: census army %d live %d | offensive %d%% passive %d%% idle %d%% | capture %d hold %d support %d "
          "overwatch %d push %d press %d none %d stale %d | advancing %d stopped %d capture active %d pending %d "
          "point deficit %d pressure %.2f | at points%s | past line+250 %d | human contact %.0f s, >=2 mates "
          "within 250 m %.0f%% | switches %d (>700 m %d, of them lateral >400 m %d), first orders %d",
          army, live, off * 100 / live, pas * 100 / live, idle * 100 / live, by[OB_CAPTURE], by[OB_HOLD], by[OB_SUPPORT],
          by[OB_OVERWATCH], by[OB_PUSH], by[OB_PRESS], by[OB_NONE], stale, advancing, stopped, cap_active, cap_pending,
          v->point_deficit, v->point_pressure, pts, past, cmd.contact_s[army], share,
          cmd.switches[army], cmd.far_switches[army], cmd.cross_switches[army], cmd.firsts[army]);
    char line[480];
    int len = snprintf(line, sizeof line, "cmd: army %d objectives:", army);
    for (int k = 0; k < nob && len < (int)sizeof line - 48; ++k) {
        int have = 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            have += cmd_live(o, i, army, ms) && cmd_accepted_reserves(&cmd_tasks[i], ms)
                    && o->cmd_type == ob[k].type && o->cmd_id == ob[k].id;
        }
        len += snprintf(line + len, sizeof line - (size_t)len, " %s %d %d/%d v%.0f |", cmd_name[ob[k].type], ob[k].id, have,
                        ob[k].need, ob[k].value);
    }
    event(host, "%s", line);
    /* Stage 2 counters (cumulative since the battle start unless "now"). */
    int silent = 0, disabled = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        silent += o->n > 0 && o->army == army && o->scene_epoch == ground_scene_epoch && cmd_task_unit_alive(o, i)
                  && cmd_seat_silent(o, ms);
        disabled += cmd_live(o, i, army, ms) && cmd_seat_disabled(o);
    }
    /* cmd.immobile counts both kinds (out of the quotas); stalled = the rest. */
    len = snprintf(line, sizeof line, "cmd: stage2 army %d | now: stalled %d immobilised %d (out of quotas in a point %d) "
                   "stay orders %d silent %d, capture slots open %d | reassignments U<0 >%.0f m %d, orders U<0 beyond "
                   "spawn reach %d | points owned/manned s, owner changes:", army, cmd.immobile[army] - disabled, disabled,
                   cmd.immobile_in_point[army], by[OB_STAY], silent, cmd.cap_open[army], CMD_FAR_M,
                   cmd.far_negative[army], cmd.beyond_reach[army]);
    for (uint32_t k = 0; k < m->ncap && k < 8 && len < (int)sizeof line - 40; ++k)
        len += snprintf(line + len, sizeof line - (size_t)len, " %c %.0f/%.0f %d", 'A' + k, cmd.pt_owned_s[army][k],
                        cmd.pt_manned_s[army][k], cmd.pt_changes[army][k]);
    event(host, "%s", line);
    if (roads.posts[army]) {
        /* v13: per kind, posts offered on the active line, crews ordered there and crews in position */
        int offered[PK_N] = {0}, crews[PK_N] = {0}, placed[PK_N] = {0};    /* s32: PK_WATCH counted apart (teamplan: line) */
        for (int j = 0; j < cmd_nposts[army]; ++j)
            offered[cmd_posts[army][j].kind] += roads_post_offered(army, &cmd_posts[army][j]) && cmd_post_option(&cmd_posts[army][j], ms) >= 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            if (!cmd_live(o, i, army, ms) || !cmd_is_post(o) || o->cmd_id - CMD_POST_ID_BASE >= cmd_nposts[army])
                continue;
            const int kind = cmd_posts[army][o->cmd_id - CMD_POST_ID_BASE].kind;
            ++crews[kind];
            placed[kind] += o->cmd_parked;
        }
        event(host, "roads: census army %d line %d | gate posts %d crews %d in place %d | choke posts %d crews %d in place %d | "
              "overwatch posts %d crews %d in place %d | stationary budget %d of %d live | plan watch slots %d crews %d in "
              "place %d", army, roads.line[army] ? roads.line[army] : 1,
              offered[PK_GATE], crews[PK_GATE], placed[PK_GATE], offered[PK_CHOKE], crews[PK_CHOKE], placed[PK_CHOKE],
              offered[PK_OVERWATCH], crews[PK_OVERWATCH], placed[PK_OVERWATCH], live / PASSIVE_DIV, live,
              offered[PK_WATCH] + offered[PK_SUPPORT], crews[PK_WATCH] + crews[PK_SUPPORT],
              placed[PK_WATCH] + placed[PK_SUPPORT]);
    }
    tp_report(host, army, ms);         /* s32: the plan and its numbers (every census) */
}

/* s26 (design rule: "spawn point and target as near as possible"): for a tank's first order of its life, the extra cost
 * CMD_NEAR_W * max(0, T_k - T_min) of each objective, T_min the least T among the objectives of the same type. T: the
 * designated library route's total time when the tank stands on a library spawn pose and the objective maps to a
 * library target; else cmd_travel as the cost uses it. 0 everywhere with near=0 or for a later order. Needs and
 * values are untouched: how many tanks go where stays the commander's decision. */
#define CMD_NEAR_W 3.0f /* points per second (the time cost is 0.5 per second) */
static void cmd_near_times(void *host, int army, int seat, const lane_seat_t *o, const cmd_obj_t *ob, int nob, int c,
                           uint64_t ms, float *extra)
{
    for (int k = 0; k < nob; ++k)
        extra[k] = 0.0f;
    if (!cfg.near_first || o->cmd_type != OB_NONE)
        return;
    const gr_lib_t *lib = routes_library(host);
    const int pose = lib && hypotf(o->vx, o->vz) < 1.0f
        ? gr_pose_of(lib, o->x, o->z, atan2f(o->fz, o->fx), 2.0f, 15.0f) : -1;
    float t[CMD_MAX_OBJ];
    int from_lib[CMD_MAX_OBJ], ok[CMD_MAX_OBJ];
    for (int k = 0; k < nob; ++k) {
        const cmd_obj_t *g = &ob[k];
        ok[k] = !((k != c && !cmd_objective_ready(army, g, ms)) || cmd_offer_unavailable(army, seat, g, ms)
                  || (k == c && cmd_is_post(o) && o->cmd_release));
        from_lib[k] = 0;
        t[k] = cmd_travel(seat, o, g);
        if (ok[k] && pose >= 0) {
            const int target = gr_target_of(lib, g->x, g->z, army, 12.0f);
            const int route = target >= 0 ? gr_designated(lib, pose, target) : -1;
            if (route >= 0) {
                t[k] = lib->routes[route].total_s;
                from_lib[k] = 1;
            }
        }
    }
    char line[360];
    int len = snprintf(line, sizeof line, "cmd: army %d seat %d first-order nearness (pose %d):", army, seat, pose);
    for (int k = 0; k < nob; ++k) {
        if (!ok[k])
            continue;
        float tmin = t[k];
        for (int q = 0; q < nob; ++q)
            if (ok[q] && ob[q].type == ob[k].type && t[q] < tmin)
                tmin = t[q];
        extra[k] = CMD_NEAR_W * fmaxf(0.0f, t[k] - tmin);
        if (len > 0 && len < (int)sizeof line - 24)
            len += snprintf(line + len, sizeof line - (size_t)len, " %d:%.0fs%s+%.0f", ob[k].id, (double)t[k],
                            from_lib[k] ? "L" : "", (double)extra[k]);
    }
    static uint64_t logged_ms[MAX_SEATS];
    if (seat > 0 && seat < MAX_SEATS && (!logged_ms[seat] || ms >= logged_ms[seat] + 10000u)) {
        logged_ms[seat] = ms;
        cmd_log(host, army, 0, "%s", line);
    }
}

void cmd_tick(void *host, int army, uint64_t ms)
{
    const strat_view_t *v = &sview[army];
    if (!v->ok)
        return;
    if (!cmd.rng)
        cmd.rng = (uint32_t)ms | 1u;
    if (ms >= cmd.log_ms[army]) {
        if (cmd.dropped[army])
            event(host, "cmd: army %d: %d order lines not logged in the last minute", army, cmd.dropped[army]);
        cmd.log_ms[army] = ms + 60000;
        cmd.logs[army] = cmd.dropped[army] = 0;
    }
    cmd_obj_t ob[CMD_MAX_OBJ];
    int all_ours = 0, nob = cmd_objectives(host, army, ms, ob, &all_ours);
    tp_apply(host, army, ms, ob, &nob);    /* s32: the plan's needs and watch objectives (teamplan=1 with a pack) */
    roads_line_release(host, army, ms);
    ++cmd_epoch[army];
    cmd_track_objectives(army, ob, nob, ms);

    /* human contact metric (share of contact time with >= 2 bots within 250 m) */
    int h = human_slot;
    if (h >= 0 && h < 64 && slot_alive[h] && slot_army[h] == army && slot_unit[h]) {
        const float *p = gh_unit_pos(slot_unit[h]);
        if (ring_count(hits_dealt_ms[h], ms, 10000) || ring_count(hits_taken_ms[h], ms, 10000)
            || cmd_known_near(v, p[0], p[2], CMD_CONTACT_M, 10.0f)) {
            int mates = 0;
            for (int i = 1; i < MAX_SEATS; ++i) {
                const lane_seat_t *o = &lseat[i];
                mates += cmd_live(o, i, army, ms) && dist2d(o->x, o->z, p[0], p[2]) < 250.0f;
            }
            cmd.contact_s[army] += (float)CMD_MS / 1000.0f;
            cmd.backed_s[army] += mates >= 2 ? (float)CMD_MS / 1000.0f : 0.0f;
        }
    }

    cmd_watch_points(host, army, ms);
    int seat_of[MAX_SEATS], cur[MAX_SEATS], pick[MAX_SEATS], required[MAX_SEATS], cont[MAX_SEATS];
    float stay[MAX_SEATS], reach[CMD_MAX_OBJ];
    int cnt[CMD_MAX_OBJ] = {0}, nc = 0, live = 0, passive = 0, cap_commit = 0, immobile = 0, immobile_in = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (o->army != army || !cmd_task_unit_alive(o, i))
            continue;
        live += cmd_live(o, i, army, ms);
        immobile += cmd_live(o, i, army, ms) && cmd_seat_immobile(o);
        int current = cmd_objective_index(ob, nob, o->cmd_type, o->cmd_id, -1);
        int next = cmd_objective_index(ob, nob, o->cmd_ntype, o->cmd_nid, -1);
        cmd_check_orders(host, o, i, ms, current >= 0, next >= 0);
    }
    int cap_need = 0, post_count = 0;
    for (int k = 0; k < nob; ++k) {
        if (ob[k].type == OB_CAPTURE)
            cap_need += ob[k].need;
        post_count += cmd_post_objective(&ob[k]) && !tp_plan_post(army, k);    /* s32 / s33: plan slots outside the budget */
        reach[k] = cmd_spawn_reach(army, &ob[k]);
    }
    /* Stage 2: quotas and budgets count the tanks that can still go somewhere.
     * A stalled or immobilised tank counts only for the point it stands in. */
    const int able = live - immobile;
    const int cap_floor = able;
    const int cap_reserve = cap_need < cap_floor ? cap_need : cap_floor;
    int post_limit = cmd_post_budget(able, cap_reserve, v->point_pressure);
    if (post_limit > post_count)
        post_limit = post_count;
    const int passive_max = cmd_stationary_budget(host, army, ms, able, post_limit);
    int keep_post[MAX_SEATS] = {0}, claim_post[CMD_MAX_OBJ] = {0};
    int post_commit = cmd_retain_posts(army, ms, ob, nob, post_limit, keep_post, claim_post);
    tp_keep_plan(army, ob, nob, keep_post, claim_post, ms);    /* s33: crews keep their watch / support slots */
    tp_move_up(host, army, ms, ob, nob);                       /* s33: support -> capture when the circle has room */
    tp_street(host, army, ms, ob, nob);                        /* s34: no other street into the zone: a support slot */
    static float base[MAX_SEATS][CMD_MAX_OBJ], cost[MAX_SEATS][CMD_MAX_OBJ];
    for (int i = 1; i < MAX_SEATS; ++i) {
        lane_seat_t *o = &lseat[i];
        if (!cmd_live(o, i, army, ms))
            continue;
        cmd_task_state_t *task = &cmd_tasks[i];
        if (cmd_seat_immobile(o)) {
            /* Stage 2: a stalled or immobilised tank fills a slot only where it
             * physically is (in a capture circle or the hold radius it still
             * does the job) and is not reassigned. Only one immobilised by
             * damage is ordered to stay and fight there; a stalled one keeps
             * its order and its pending order and keeps trying (Poland 01:40:
             * stay orders for stalled healthy tanks parked 27% of all tank time). */
            const int at = cmd_point_at(o, ob, nob);
            if (at >= 0) {
                ++cnt[at];
                cap_commit += ob[at].type == OB_CAPTURE;
                ++immobile_in;
            }
            task->candidate_ms = 0;
            if (cmd_seat_disabled(o))
                cmd_stay(host, army, o, i, at, ms);
            continue;
        }
        const int lp = cmd_lane_point(o->lane);
        int c = cmd_objective_index(ob, nob, o->cmd_type, o->cmd_id, lp), same_point = 0;
        if (c < 0 && (o->cmd_type == OB_CAPTURE || o->cmd_type == OB_HOLD)) {
            /* Stage 2: the point changed hands under this order. The same point
             * under the other duty continues it (the captors are the natural
             * garrison; holders of a lost point retake it). */
            c = cmd_objective_index(ob, nob, OB_NONE, 0, o->cmd_id);
            same_point = c >= 0;
        }
        if (c >= 0 && cmd_offer_unavailable(army, i, &ob[c], ms))
            c = -1;
        same_point = same_point && c >= 0;
        int next = task->pending.task_id && task->pending.type != OB_NONE
                   ? cmd_objective_index(ob, nob, o->cmd_ntype, o->cmd_nid, lp) : -1;
        const int committed = cmd_order_blocks_reassignment(task, ms);
        if (cmd_is_post(o) && !keep_post[i]) {
            if (!task->post_release_since_ms)
                task->post_release_since_ms = ms;
            if (!committed && ms - task->post_release_since_ms >= CMD_SOFT_EXIT_MS)
                o->cmd_release = 1;
        } else {
            task->post_release_since_ms = 0;
        }
        if (cmd_refresh_capture_policy(host, o, i, ms))
            next = c;
        const int guarded = cmd_guarding(host, o, v, ms);
        if (c >= 0 && !task->invalid_reason && !cmd_is_post(o) && !o->cmd_release
            && !task->pending.task_id && o->cmd_type >= OB_SUPPORT
            && ms - task->accepted_at_ms >= CMD_MANEUVER_MS) {
            /* Update a moving goal under the same task identity. The consumer
             * may defer applying this revision until the current handoff is safe. */
            ground_task_order_t updated = cmd_make_order(army, i, &ob[c], ms, GTR_NONE);
            float moved = dist2d(updated.x, updated.z, o->cmd_x, o->cmd_z);
            float threshold = o->cmd_type == OB_SUPPORT ? 25.0f : CMD_MOVE_M;
            if (moved > threshold && cmd_publish(host, o, i, updated, ms))
                next = c;
        }
        int locked = task->pending.task_id || keep_post[i] || committed;
        int occupying = -1;
        for (int k = 0; k < nob && occupying < 0; ++k)
            if (ob[k].type == OB_CAPTURE && dist2d(o->x, o->z, ob[k].x, ob[k].z) < lanes[ob[k].lane].zr)
                occupying = k;
        if (locked || occupying >= 0) {
            task->candidate_ms = 0;
            int reserved = next >= 0 && cmd_pending_reserves(task) ? next : -1;
            if (occupying >= 0)
                reserved = occupying;
            if (reserved >= 0) {
                ++cnt[reserved];
                cap_commit += ob[reserved].type == OB_CAPTURE;
            } else if (c >= 0 && cmd_accepted_reserves(task, ms)
                       && (ob[c].type != OB_CAPTURE || cmd_capture_active(o, i, &ob[c], ms))) {
                ++cnt[c];
                cap_commit += ob[c].type == OB_CAPTURE;
            }
            passive += !keep_post[i] && !guarded && o->cmd_type != OB_HOLD     /* a garrison is not passive (stage 2) */
                       && ((reserved >= 0 && ob[reserved].passive)
                           || (!o->cmd_release && (cmd_order_parks(o->cmd_type) || o->cmd_parked || o->holding)));
            continue;
        }
        seat_of[nc] = i;
        cur[nc] = c;
        cont[nc] = same_point;
        pick[nc] = -1;
        required[nc] = 0;
        int role = v->role[i & 63] & 3, cur_want = c >= 0 ? cmd_want(&ob[c]) : role;
        /* s26 (near=1): the first order of a life prefers the objectives nearest in travel time among those of the
         * same type: + CMD_NEAR_W * max(0, T_k - T_min) (cmd_near_times). */
        float near_extra[CMD_MAX_OBJ];
        cmd_near_times(host, army, i, o, ob, nob, c, ms, near_extra);
        for (int k = 0; k < nob; ++k) {
            const cmd_obj_t *g = &ob[k];
            if ((k != c && !cmd_objective_ready(army, g, ms)) || cmd_offer_unavailable(army, i, g, ms)
                || (k == c && cmd_is_post(o) && o->cmd_release)
                || !tp_allowed(army, k, g, i, o)) {     /* s33: first orders only from the plan; watch: its spawn area */
                base[nc][k] = 0.0f;
                cost[nc][k] = 1e30f;
                continue;
            }
            base[nc][k] = g->value * cmd_fitw(i, role, g);
            float cs = cmd_travel(i, o, g) * (g->type == OB_CAPTURE ? 0.5f - 0.12f * v->point_pressure : 0.5f)
                       + cmd_risk(army, o->x, o->z, g->x, g->z);
            if (g->type == OB_CAPTURE && o->cmd_release)
                cs -= 30.0f + 22.0f * v->point_pressure;
            if (k == c && o->cmd_parked && g->type >= OB_PUSH && !guarded && !all_ours)
                cs += 45.0f;
            int want = cmd_want(g);
            const cmd_post_t *rp = roads_post_of(army, g);      /* v13: its task fit is the role weight */
            if (role != SR_ESCORT && want != cur_want
                && (rp ? roads_fitw(army, i, rp->kind, rp->ref) : cmd_weight(i, role, want)) < CMD_ROLE_FREE)
                cs += CMD_ROLE_COST;
            if (k == c && !o->cmd_release)
                cs -= CMD_H + CMD_STAY_PROGRESS * cmd_progress(o, i, g, ms);  /* stage 2: progress counts */
            else if (o->cmd_type != OB_NONE && cmd_sharp_change(o, g))
                cs += CMD_TURN_GAIN;
            cs += near_extra[k]; /* s26: 0 unless near=1 and this is the first order of the life */
            cost[nc][k] = cs;
        }
        ++nc;
    }
    /* Stage 2: what staying on the present order is worth, against which every
     * reassignment below is measured. Committed tanks on it are counted by now;
     * of several free tanks on the same objective the ones that stay best take
     * its slots first, so the others see the diminished value (two captors of a
     * point that needs one garrison: the second is free to go). */
    for (int u = 0; u < nc; ++u) {
        const int c = cur[u];
        stay[u] = -1e30f;
        if (c < 0 || base[u][c] <= 0.0f)
            continue;
        int ahead = 0;
        for (int w = 0; w < nc; ++w)
            ahead += w != u && cur[w] == c && base[w][c] > 0.0f && lseat[seat_of[w]].cmd_type != OB_NONE
                     && (base[w][c] - cost[w][c] > base[u][c] - cost[u][c]
                         || (base[w][c] - cost[w][c] == base[u][c] - cost[u][c] && w < u));
        stay[u] = base[u][c] * cmd_dec(cnt[c] + ahead, &ob[c]) - cost[u][c];
    }
    /* Fill capture slots, then assign the limited gate posts, then allocate
     * remaining crews by utility. */
    enum { CMD_FILL_CAPTURE, CMD_FILL_POSTS, CMD_FILL_OTHER };
    /* s33: first orders from the plan: watch slots, then each zone's capture places, then its support slots */
    if (tp_format == 2)
        tp2_assign(host, army, ms, ob, nob, nc, seat_of, pick, cnt);    /* s35: the lane rule */
    else
        tp_assign(army, ob, nob, nc, seat_of, cur, pick, cnt, base, cost, claim_post);
    int zq[TP_MAX_ZONES];       /* s37 G4: the plan zones' counts with this tick's picks (format 2), for the quota */
    tp2_quota_have(army, ms, ob, nc, seat_of, pick, zq);
    int phase = cap_commit < cap_reserve ? CMD_FILL_CAPTURE : CMD_FILL_POSTS;
    for (int round = 0; round < nc && nob && phase <= CMD_FILL_OTHER;) {
        if (phase == CMD_FILL_POSTS && (post_commit >= post_limit || cap_commit < cap_reserve))
            phase = CMD_FILL_OTHER;
        int reserve = phase == CMD_FILL_CAPTURE;
        int bu = -1, bo = -1;
        float bv = -1e30f;
        for (int u = 0; u < nc; ++u) {
            if (pick[u] >= 0)
                continue;
            for (int k = 0; k < nob; ++k) {
                float dc = cmd_dec(cnt[k], &ob[k]);
                const lane_seat_t *o = &lseat[seat_of[u]];
                int post = cmd_post_objective(&ob[k]);
                if (base[u][k] <= 0.0f || (phase == CMD_FILL_POSTS) != post || tp_fill_excluded(host, army, k, seat_of[u], cnt, ob, zq, ms)
                    || (post && (claim_post[k] || cnt[k] || v->role[seat_of[u] & 63] == SR_ESCORT))
                    || dc <= 0.0f || (ob[k].passive && passive >= passive_max)
                    || (reserve && (ob[k].type != OB_CAPTURE || cnt[k] >= ob[k].need
                                    || (k == cur[u] && o->cmd_type != OB_NONE && !cont[u]
                                        && !cmd_capture_active(o, seat_of[u], &ob[k], ms)
                                        && !(o->cmd_release && o->holding && o->route[o->step].pt != SLOT_PT)))))
                    continue;
                float val = base[u][k] * dc - cost[u][k];
                if (k != cur[u]) {
                    /* Stage 2, in every phase (the capture reserve included): a
                     * new order must be sensible on its own, not negative or no
                     * longer a drive than a fresh spawn's, else the slot stays
                     * open for someone nearer; and it must beat staying by the
                     * same margin the switch test below applies, so that no pick
                     * is spent on a tank that will not switch. */
                    const float d = dist2d(o->x, o->z, ob[k].x, ob[k].z);
                    const float margin = post ? 0.0f
                                         : (cur[u] >= 0 && d > CMD_FAR_M ? CMD_FAR_GAIN : CMD_SWITCH_GAIN)
                                           - (reserve ? 34.0f + 28.0f * v->point_pressure : 0.0f);
                    if ((val < 0.0f && d > reach[k]) || (o->cmd_type != OB_NONE && val < stay[u] + margin))
                        continue;
                }
                if (reserve)
                    val += 16.0f * (float)(ob[k].need - cnt[k]) / (float)ob[k].need;
                if (val > bv) {
                    bv = val;
                    bu = u;
                    bo = k;
                }
            }
        }
        if (bu < 0) {
            ++phase;
            continue;
        }
        pick[bu] = bo;
        required[bu] = reserve;
        tp2_quota_take(army, &ob[bo], seat_of[bu], zq);     /* s37 G4 */
        ++cnt[bo];
        cap_commit += ob[bo].type == OB_CAPTURE;
        post_commit += cmd_post_objective(&ob[bo]);
        if (reserve && cap_commit >= cap_reserve)
            phase = CMD_FILL_POSTS;
        passive += ob[bo].passive;
        ++round;
    }
    /* s32: the plan's flank pick among this tick's first orders (read by cmd_issue) */
    tp_flank_wave(host, army, ms, ob, nob, nc, seat_of, pick);
    /* first orders (free) and reassignments (best gains, CMD_MAX_SWITCH a tick) */
    int sw[MAX_SEATS], nsw = 0;
    float gain[MAX_SEATS];
    for (int u = 0; u < nc; ++u) {
        int k = pick[u], i = seat_of[u];
        lane_seat_t *o = &lseat[i];
        cmd_task_state_t *task = &cmd_tasks[i];
        if (k < 0) {
            task->candidate_ms = 0;
            continue;
        }
        float up = base[u][k] * cmd_dec(cnt[k] - 1, &ob[k]) - cost[u][k];
        if (o->cmd_type == OB_NONE) {
            cmd_issue(host, army, i, &ob[k], up, 1, ms);
            continue;
        }
        if (k == cur[u]) {
            task->candidate_ms = 0;
            if (cont[u])        /* stays on the point under its new duty: not a switch */
                cmd_issue(host, army, i, &ob[k], up, 2, ms);
            continue;
        }
        /* A seat which is not executing a capture cannot consume its reserve.
         * Treat a required assignment as an advantage, still checking the
         * turn, long drive and confirmation thresholds. */
        float g = cur[u] < 0 ? CMD_GONE_GAIN + up : up - (base[u][cur[u]] * cmd_dec(cnt[cur[u]], &ob[cur[u]]) - cost[u][cur[u]]);
        if (required[u])
            g += 34.0f + 28.0f * v->point_pressure;
        if (cmd_post_objective(&ob[k]) || (cmd_is_post(o) && o->cmd_release))
            g = fmaxf(g, CMD_FAR_GAIN + 1.0f);
        const int sharp = cmd_sharp_change(o, &ob[k]);
        if (g < CMD_SWITCH_GAIN
            || (cur[u] >= 0 && dist2d(o->x, o->z, ob[k].x, ob[k].z) > CMD_FAR_M && g < CMD_FAR_GAIN)) {
            task->candidate_ms = 0;
            continue;
        }
        const int confirmed = task->candidate_ms && ms - task->candidate_ms <= 2 * CMD_MS
                              && task->candidate_type == ob[k].type && task->candidate_objective == ob[k].id;
        task->candidate_ms = ms;
        task->candidate_type = ob[k].type;
        task->candidate_objective = ob[k].id;
        if (sharp && !confirmed)
            continue;
        sw[nsw] = u;
        gain[nsw++] = g;
    }
    int max_sw = all_ours && (!cmd.allmine_ms[army] || ms - cmd.allmine_ms[army] < CMD_ALLMINE_BURST_MS)
                     ? CMD_MAX_SWITCH_ALLMINE : CMD_MAX_SWITCH;
    for (int n = 0; n < max_sw && n < nsw; ++n) {
        int b = n;
        for (int j = n + 1; j < nsw; ++j)
            if (gain[j] > gain[b])
                b = j;
        int tu = sw[n];
        float tg = gain[n];
        sw[n] = sw[b];
        gain[n] = gain[b];
        sw[b] = tu;
        gain[b] = tg;
        int u = sw[n], k = pick[u];
        cmd_issue(host, army, seat_of[u], &ob[k], base[u][k] * cmd_dec(cnt[k] - 1, &ob[k]) - cost[u][k], 0, ms);
    }
    for (int u = 0; u < nc; ++u)
        if (pick[u] < 0)
            cmd_publish_release(host, &lseat[seat_of[u]], seat_of[u], ms);
    int open = 0;
    for (int k = 0; k < nob; ++k)
        open += ob[k].type == OB_CAPTURE && cnt[k] < ob[k].need ? ob[k].need - cnt[k] : 0;
    cmd.cap_open[army] = open;
    cmd.immobile[army] = immobile;
    cmd.immobile_in_point[army] = immobile_in;
    cmd_census(host, army, ms, ob, nob, all_ours);
}

/* v13 leave rules (spec 8 / 8b). hit: the spot is given up (blocked CMD_POST_BLOCK_MS); a choke
 * crew takes the post's next spot, an overwatch crew the best free overwatch post of the active
 * line within ROADS_NEXT_OW_M; failing that, and always after an idle minute at a choke (every
 * spot of that post blocked), the crew goes back to capture / hold duty at the post's point. */
static void roads_post_leave(void *host, lane_seat_t *s, int seat, cmd_post_t *p, uint64_t ms, int hit, int from)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    const ground_task_order_t *active = cmd_active_order(seat);
    if (!active || st->pending.task_id || active->type != OB_OVERWATCH || active->post_option < 0)
        return;
    const int army = s->army, option = active->post_option, reason = hit ? GTR_POST_HIT : GTR_POST_IDLE;
    const float held = (float)(ms - s->post_in_place_ms) / 1000.0f;
    for (int i = 0; i < p->npos; ++i)
        if (i == option || !hit)
            p->pos[i].blocked_until = ms + CMD_POST_BLOCK_MS;
    cmd_invalidate(host, st, ms, reason);
    cmd_obj_t o = {.type = OB_OVERWATCH, .id = active->objective, .need = 1, .lane = p->lane, .cap = p->cap,
                   .value = active->value_rank};
    int ah[TP_MAX_ZONES];       /* s40: area counts (format 2; outside it every place is open) */
    tp2_area_count(army, ms, ah);
    int next = hit && p->kind == PK_CHOKE && !tp2_old_post_off(army, p) ? cmd_post_option(p, ms) : -1;     /* s41 */
    if (next >= 0 && !tp2_area_open(army, seat, tp2_area_at(p->pos[next].x, p->pos[next].z), ah)) {
        cmd_log(host, army, 1, "roads: army %d seat %d post %d spot %d not taken: area %s at its cap %d", army, seat,
                active->objective, next + 1, tp_zones[tp2_area_at(p->pos[next].x, p->pos[next].z)].id,
                tp2_area_cap(army, tp2_area_at(p->pos[next].x, p->pos[next].z)));
        next = -1;              /* s40: the next spot lies in an area at its cap: back to the point */
    }
    if (next >= 0) {
        o.x = p->pos[next].x;
        o.z = p->pos[next].z;
    } else if (hit && p->kind == PK_OVERWATCH) {
        float best = -1.0f;
        for (int j = 0; j < cmd_nposts[army]; ++j) {
            cmd_post_t *q = &cmd_posts[army][j];
            const int qid = CMD_POST_ID_BASE + j, qo = cmd_post_option(q, ms);
            if (q == p || q->kind != PK_OVERWATCH || qo < 0 || !roads_post_offered(army, q)
                || dist2d(s->x, s->z, q->pos[qo].x, q->pos[qo].z) > ROADS_NEXT_OW_M
                || !tp2_area_open(army, seat, tp2_area_at(q->pos[qo].x, q->pos[qo].z), ah))     /* s40 */
                continue;
            int claimed = 0;
            for (int i = 1; i < MAX_SEATS && !claimed; ++i)
                claimed = i != seat && lseat[i].army == army
                          && ((cmd_tasks[i].active.task_id && cmd_tasks[i].active.type == OB_OVERWATCH
                               && cmd_tasks[i].active.objective == qid)
                              || (cmd_tasks[i].pending.task_id && cmd_tasks[i].pending.type == OB_OVERWATCH
                                  && cmd_tasks[i].pending.objective == qid));
            const float value = roads_post_value(army, q, ms);
            if (!claimed && value > best) {
                best = value;
                next = qo;
                o.id = qid;
                o.x = q->pos[qo].x;
                o.z = q->pos[qo].z;
                o.value = value;
            }
        }
    }
    const int back = tp_lane_redirect(army, p->lane);  /* s33: never into a zone the plan skips */
    const lane_t *lane = &lanes[back];
    if (next < 0) {
        if (tp2_resume_barred(army, seat, back, ms)) {
            /* s37 G4: its zone is at its quota: no capture / hold order; its order (invalidated above) is released, the
             * commander gives it another under the quota */
            cmd_log(host, army, 1, "roads: army %d seat %d leaves post %d spot %d after %.0f s in place: %s; zone of lane %d "
                    "at its quota: no capture / hold order, released for another order", army, seat, active->objective,
                    option + 1, held, !hit ? "no enemy detected for 60 s" : "hit", back);
            cmd_publish_release(host, s, seat, ms);
            return;
        }
        o.type = zone_owner(host, lane->zx, lane->zz) == army ? OB_HOLD : OB_CAPTURE;
        o.id = back;
        o.lane = back;
        o.x = lane->zx;
        o.z = lane->zz;
    }
    float kx = 0.0f, kz = 0.0f, kage = -1.0f;
    const int known = from >= 0 ? contact_get(army, seat, from, ms, &kx, &kz, &kage) : 0;
    cmd_log(host, army, 1, "roads: army %d seat %d leaves %s post %d spot %d after %.0f s in place: %s -> %s %d at "
            "(%.0f,%.0f)", army, seat, p->kind == PK_CHOKE ? "choke" : "overwatch", active->objective, option + 1, held,
            !hit ? "no enemy detected for 60 s" : known ? "hit, shooter not in sight" : "hit, shooter position unknown",
            o.type == OB_OVERWATCH ? (o.id == active->objective ? "spot" : "overwatch post") : cmd_name[o.type],
            o.type == OB_OVERWATCH && o.id == active->objective ? next + 1 : o.id, o.x, o.z);
    cmd_publish(host, s, seat, cmd_make_order(army, seat, &o, ms, reason), ms);
}

/* Per tick of a crew in position at a choke / overwatch spot (cmd_post_drive). Detected: own
 * sight of an engaged enemy, or a team-known one within ROADS_SEEN_M of the post's point. Hit:
 * the newest enemy hit since it got there. Choke: leaves when the shooter's position is not
 * known (seen in the last ROADS_HIT_KNOWN_S); overwatch: when the shooter is not in sight now. */
int roads_post_check(void *host, lane_seat_t *s, int seat, cmd_post_t *p, uint64_t ms)
{
    const int slot = seat & 63;
    if (!p->kind || p->kind == PK_WATCH || (p->kind == PK_LANE && tp_lanes[p->ref].end == TLE_GUARD)
        || s->cmd_apply_ms || cmd_tasks[seat].pending.task_id || cmd_tasks[seat].invalid_reason)
        return 0;                       /* s33 / s35: a plan watch / guard tank is not released by the post leave rules */
    if (!s->post_in_place_ms) {
        s->post_in_place_ms = s->post_enemy_ms = ms;
        s->post_hit_ms = last_hitter_ms[slot];
        return 0;
    }
    const strat_view_t *v = &sview[s->army];
    const int sees = s->mode == M_ENGAGE && s->eng_own && ms - s->eng_seen_ms < 1500;
    if (sees || (v->ok && cmd_known_near(v, p->gx[1], p->gz[1], ROADS_SEEN_M, 3.0f)))
        s->post_enemy_ms = ms;
    if (last_hitter_ms[slot] > s->post_hit_ms) {
        s->post_hit_ms = last_hitter_ms[slot];
        const int from = last_hitter[slot];
        if (from >= 0 && from < 64 && slot_army[from] != s->army) {
            float x, z, age = 1e9f;
            const int level = contact_get(s->army, seat, from, ms, &x, &z, &age);
            const int seen = sees && s->eng == from;
            const int known = p->kind == PK_CHOKE ? seen || (level && age <= ROADS_HIT_KNOWN_S) : seen || level == 2;
            if (!known) {
                roads_post_leave(host, s, seat, p, ms, 1, from);
                return 1;
            }
        }
    }
    if (p->kind == PK_CHOKE && ms - s->post_enemy_ms >= ROADS_IDLE_MS) {
        roads_post_leave(host, s, seat, p, ms, 0, -1);
        return 1;
    }
    return 0;
}