/* Command owns TaskOrder, its receipt and the last accepted navigation evidence.
 * The compatibility fields are written only when command publishes or commits an order. */
#define CMD_SOFT_ENTRY_MS 2000
#define CMD_ORDER_RETRY_MS 8000
#define CMD_REPORT_FRESH_MS 2500
#define CMD_STOP_BUDGET_MS 2500u
#define CMD_URGENT_STOP_BUDGET_MS 750u


cmd_task_state_t cmd_tasks[MAX_SEATS];
static uint64_t cmd_task_serial, cmd_epoch[3], cmd_life_scan[3];
static uint64_t cmd_objective_since[3][OB_N][CMD_POST_ID_BASE + CMD_POST_MAX];
static uint64_t cmd_objective_last[3][OB_N][CMD_POST_ID_BASE + CMD_POST_MAX];

static const char *cmd_task_reason(int reason)
{
    static const char *const names[] = {
        "none", "stale", "invalid", "clearance", "combat", "no_route", "geometry_unknown", "traffic_blocked",
        "rejoin_failed", "objective_lost", "point_changed", "post_invalid", "disabled", "unit_lost",
        "immobilized", "expired", "soft_exit", "capture_complete", "superseded", "post_idle", "post_hit"
    };
    return reason >= 0 && reason < (int)(sizeof names / sizeof names[0]) ? names[reason] : "unknown";
}

static int cmd_same_key(const ground_task_order_t *a, const ground_task_order_t *b)
{
    return a->task_id && a->task_id == b->task_id && a->revision == b->revision
           && a->seat == b->seat && a->life_id == b->life_id;
}

static void cmd_task_event(void *host, const char *status, const ground_task_order_t *o, uint64_t ms, int reason)
{
    event(host, "task: %s seat %d life %llu key %llu/%u epoch %llu at %llu %s %d reason %s apply %llu commit %llu",
          status, o->seat, (unsigned long long)o->life_id, (unsigned long long)o->task_id,
          o->revision, (unsigned long long)o->epoch_id, (unsigned long long)ms,
          cmd_name[o->type], o->objective, cmd_task_reason(reason),
          (unsigned long long)o->apply_at_ms, (unsigned long long)o->min_commit_until_ms);
}

static void cmd_clear_pending(lane_seat_t *s, cmd_task_state_t *st)
{
    st->pending = (ground_task_order_t){0};
    st->retry_after_ms = st->pending_missing_ms = 0;
    st->deferred_reason = 0;
    s->cmd_ntype = s->cmd_nid = s->cmd_nlane = 0;
    s->cmd_nx = s->cmd_nz = 0.0f;
    s->cmd_apply_ms = 0;
}

static void cmd_clear_active(lane_seat_t *s, cmd_task_state_t *st)
{
    st->active = (ground_task_order_t){0};
    st->report = (ground_task_report_t){0};
    st->accepted_at_ms = st->first_accepted_at_ms = st->missing_since_ms = 0;
    st->invalid_reason = GTR_NONE;
    st->candidate_ms = st->urgency_since_ms = st->post_release_since_ms = 0;
    st->last_active_revision = 0;
    s->cmd_type = s->cmd_id = s->cmd_lane = s->cmd_side = s->cmd_release = 0;
    s->cmd_x = s->cmd_z = 0.0f;
    s->cmd_since = 0;
}

void cmd_cancel_unit(void *host, lane_seat_t *s, int seat, uint64_t ms, int reason)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if (st->pending.task_id)
        cmd_task_event(host, "pending_cancelled", &st->pending, ms, reason);
    if (st->active.task_id)
        cmd_task_event(host, "cancelled", &st->active, ms, reason);
    cmd_clear_pending(s, st);
    cmd_clear_active(s, st);
    st->rejected = (ground_task_order_t){0};
    st->rejected_until_ms = 0;
}

int cmd_life_replaced(const lane_seat_t *s, int seat)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    return st->life_id == s->born_ms && st->scan_life && slot_alive[seat]
           && slot_unit[seat] == s->unit && slot_life[seat] != st->scan_life;
}

void cmd_unit_observe(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if (st->life_id != s->born_ms) {
        cmd_cancel_unit(host, s, seat, ms, GTR_UNIT_LOST);
        *st = (cmd_task_state_t){.life_id = s->born_ms, .army = s->army, .spawn_ok = 1, .spawn_x = s->x, .spawn_z = s->z};
    }
    if (slot_alive[seat] && slot_unit[seat] == s->unit)
        st->scan_life = slot_life[seat];
}

void cmd_observe_lost_units(void *host, int army, uint64_t ms)
{
    if (cmd_life_scan[army] == last_unit_scan)
        return;
    cmd_life_scan[army] = last_unit_scan;
    for (int seat = 1; seat < MAX_SEATS; ++seat) {
        cmd_task_state_t *st = &cmd_tasks[seat];
        lane_seat_t *s = &lseat[seat];
        if (st->army == army && last_unit_scan >= s->born_ms
            && (!slot_alive[seat] || slot_unit[seat] != s->unit || cmd_life_replaced(s, seat))
            && (st->active.task_id || st->pending.task_id))
            cmd_cancel_unit(host, s, seat, ms, GTR_UNIT_LOST);
    }
}

static int cmd_task_unit_alive(const lane_seat_t *s, int seat)
{
    return s->unit && cmd_tasks[seat].life_id == s->born_ms && !cmd_life_replaced(s, seat)
           && (last_unit_scan < s->last_ms || (slot_alive[seat] && slot_unit[seat] == s->unit));
}

/* Mobility (stage 2, 2026-11-15; split 2026-11-15 after Poland 01:40). The
 * stretch is what the seat already knows: ordered to move (wants_move) and not
 * out of a CMD_IMMOBILE_MOVE_M circle, counted in ordered time only. Time it
 * was not expected to move is not counted: holding, parked, arrived, in a
 * commanded engagement stop, waiting on geometry or a spawn, without a plan, or
 * behind a blocker that is itself moving. The circle is larger than a back-off
 * (3 m) plus pivot creep (p90 5 m), so the liveness manoeuvres of a wedged hull
 * do not reset it. A stretch is one attempt: after CMD_IMMOBILE_MS without an
 * order to move it starts again (Poland seat 25 had sat 64 s arrived at its
 * goal; the seconds of realignment turns it had collected on the way were
 * still counted when it was hit there).
 *
 * Two outcomes, in s->immobile, which the rest of the code reads:
 * - CMD_STALLED: CMD_IMMOBILE_MS of the stretch without damage evidence. The
 *   hull is healthy and its movement is failing (Poland: 12 of the 15 first
 *   flags had taken no damaging hit in that life; turning on the spot,
 *   driving / static_blocked in turns, a speed
 *   under the tracker's dead band; before the stay order most freed
 *   themselves in tens of seconds). The commander stops counting on it for
 *   the moment - out of the quotas, a slot only where it physically is - and
 *   leaves it its order (and pending order) to keep trying; no stay order.
 *   It counts again as soon as it has left a CMD_IMMOBILE_MOVE_M circle round
 *   where it was flagged (the circle that defined the stall: once out of it
 *   the stall is over by its own definition), when it is at its goal, when it
 *   has no order left to carry out (it is stalled on an order, and must be
 *   free for a new one), or after CMD_IMMOBILE_MS not ordered to move.
 * - CMD_IMMOBILISED: damage evidence plus CMD_IMMOBILE_HIT_MS of the stretch
 *   counted after the hit. Evidence is a hit taken in this life after which
 *   the hull has not been CMD_MOBILE_M from where it was hit: one that drove
 *   on after the hit (Poland seat 23, 20 m in 4 s) was not disabled by it.
 *   Hits come from hits_taken_ms (the hit callback writes every hit row on the
 *   victim there, under `lock`): last_hit_ms depends on the shooter's pose
 *   being fresh (hit_dir) and on the threat code having run, and starts a
 *   life with the previous life's hit. Neither carries the damage, so a
 *   zero-damage machine-gun hit counts too; the after-the-hit stall it must be
 *   followed by is what keeps that from mattering. The stay order parks it;
 *   it ends when the hull has moved CMD_MOBILE_M from where it was flagged or,
 *   for a retry, after CMD_IMMOBILE_RETRY_MS, then CMD_IMMOBILE_RETRY_MAX_MS
 *   for every later flag. Not doubled: the game repairs tracks on its own
 *   (Poland seat 25, which may only have had a broken track, moved within
 *   26 s once asked again, but had sat 60 s before that; seat 10, really
 *   stuck, was flagged again 17 s after its retry, so a retry costs little). */
enum { CMD_MOBILE, CMD_STALLED, CMD_IMMOBILISED };
#define CMD_IMMOBILE_MS 20000
#define CMD_IMMOBILE_HIT_MS 8000
#define CMD_IMMOBILE_MOVE_M 10.0f
#define CMD_MOBILE_M 20.0f
#define CMD_IMMOBILE_RETRY_MS 30000
#define CMD_IMMOBILE_RETRY_MAX_MS 60000
/* A wreck keeps the unit's alive flags until the seat respawns, and
 * wt_ground_observe keeps refreshing its pose, so slot_alive and a fresh pose
 * counted dead seats as live (Spaceport: seats 7, 19, 28 ... 16-52 s each). A
 * wreck gets no ticks of the game's own move order and does not move: CMD_SILENT_MS of both. */
#define CMD_SILENT_MS 5000

/* Out of the quotas and budgets: stalled or immobilised. */
int cmd_seat_immobile(const lane_seat_t *s)
{
    return s->immobile != CMD_MOBILE;
}

/* Immobilised by damage: parked by a stay order. */
int cmd_seat_disabled(const lane_seat_t *s)
{
    return s->immobile == CMD_IMMOBILISED;
}

static int cmd_seat_silent(const lane_seat_t *s, uint64_t ms)
{
    return s->imm_tick_ms && ms > s->imm_tick_ms && ms - s->imm_tick_ms > CMD_SILENT_MS
           && dist2d(s->x, s->z, s->imm_tick_x, s->imm_tick_z) < 2.0f;
}

static int cmd_mobility_expected(const lane_seat_t *s, int seat, uint64_t ms)
{
    if (!s->wants_move || s->holding || s->parked || s->cmd_parked || s->nav.state == GNR_ARRIVED
        || control_engagement_active(s, ms) || motion_capture_occupied(s, seat, ms))
        return 0;
    /* Integration (2026-11-15): the queue out of a spawn is not immobility. In
     * the opening of 2026-11-14 five of the seven tanks of one spawn group needed
     * 40-86 s for their first 100 m; a stay order would park them a minute more. */
    const cmd_task_state_t *st = &cmd_tasks[seat];
    if (st->spawn_ok && ms - s->born_ms < 60000 && dist2d(s->x, s->z, st->spawn_x, st->spawn_z) < 120.0f)
        return 0;
    switch (s->motion_state) {
    case GM_OBSERVED:
    case GM_PARKED:
    case GM_ARRIVED:
    case GM_WAIT_GEOMETRY:
    case GM_WAIT_SPAWN:
        return 0;
    case GM_WAIT_TRAFFIC:
        if (s->yield_to > 0 && s->yield_to < MAX_SEATS) {
            const lane_seat_t *b = &lseat[s->yield_to];
            return b->vx * b->vx + b->vz * b->vz < 1.0f;    /* queued behind a standing hull: counts */
        }
        return 1;
    default:
        return 1;
    }
}

/* Called by nav_report once per task-layer tick (the game's own move order). */
/* A new stretch from where the hull is now. */
static void cmd_mobility_anchor(lane_seat_t *s, cmd_task_state_t *st, uint64_t ms)
{
    s->imm_x = s->x;
    s->imm_z = s->z;
    s->imm_ref_ms = ms;
    s->imm_need_ms = st->hit_need_ms = 0;
}

/* A stall is a stall on an order: without one to carry out (none, invalid, or
 * a stay order) the commander must be free to give it a new one. */
static int cmd_mobility_has_order(const cmd_task_state_t *st)
{
    return (st->active.task_id && !st->invalid_reason && st->active.type != OB_STAY)
           || (st->pending.task_id && st->pending.type != OB_NONE && st->pending.type != OB_STAY);
}

static void cmd_stall_end(void *host, lane_seat_t *s, cmd_task_state_t *st, int seat, uint64_t ms, const char *why)
{
    cmd_log(host, s->army, 1, "cmd: army %d seat %d stall ends after %.0f s at (%.0f,%.0f): %s", s->army, seat,
            (double)(ms - s->imm_since) / 1000.0, s->imm_x, s->imm_z, why);
    s->immobile = CMD_MOBILE;
    cmd_mobility_anchor(s, st, ms);
}

/* Called by nav_report once per task-layer tick (the game's own move order). */
void cmd_observe_mobility(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if (st->life_id != s->born_ms)      /* not this life's task state yet (cmd_unit_observe comes first) */
        return;
    const uint64_t dt = s->imm_tick_ms && ms > s->imm_tick_ms ? ms - s->imm_tick_ms : 0;
    const int first = !s->imm_tick_ms;
    s->imm_tick_ms = ms;
    s->imm_tick_x = s->x;
    s->imm_tick_z = s->z;
    if (first) {
        cmd_mobility_anchor(s, st, ms);
        return;
    }
    /* Damage evidence: the newest hit of this life, kept while the hull stays
     * near where it was when the hit was first seen (at most one tick later). */
    const uint64_t hit = hits_taken_ms[seat & 63][0];
    if (hit > s->born_ms && hit > st->hit_ms) {
        st->hit_ms = hit;
        st->hit_x = s->x;
        st->hit_z = s->z;
        st->hit_holds = 1;
        st->hit_need_ms = s->imm_need_ms;   /* only the stall after the hit counts */
    }
    if (st->hit_holds && dist2d(s->x, s->z, st->hit_x, st->hit_z) >= CMD_MOBILE_M)
        st->hit_holds = 0;
    const int expected = dt <= 1000 && cmd_mobility_expected(s, seat, ms);
    const int idle = !expected && st->expect_ms && ms - st->expect_ms > CMD_IMMOBILE_MS;
    if (expected)
        st->expect_ms = ms;
    const float moved = dist2d(s->x, s->z, s->imm_x, s->imm_z);
    if (s->immobile == CMD_IMMOBILISED) {
        if (moved < CMD_MOBILE_M && ms < s->imm_retry_ms)
            return;
        cmd_log(host, s->army, 1, "cmd: army %d seat %d %s after %.0f s immobilised at (%.0f,%.0f)%s", s->army, seat,
                moved >= CMD_MOBILE_M ? "mobile again" : "retries to move", (double)(ms - s->imm_since) / 1000.0,
                s->imm_x, s->imm_z, moved >= CMD_MOBILE_M ? "" : " (no displacement; flagged again sooner)");
        s->immobile = CMD_MOBILE;
        if (moved >= CMD_MOBILE_M) {
            cmd_mobility_anchor(s, st, ms);
        } else {
            /* Same place, same evidence: CMD_IMMOBILE_HIT_MS of ordered time from now flags it again. */
            s->imm_need_ms = st->hit_need_ms = 0;
            s->imm_ref_ms = ms;
        }
        return;
    }
    if (s->immobile == CMD_STALLED) {
        const char *why = moved >= CMD_IMMOBILE_MOVE_M ? "moved" : s->nav.state == GNR_ARRIVED ? "at its goal"
                          : !cmd_mobility_has_order(st) ? "no order left" : idle ? "not ordered to move" : NULL;
        if (why) {
            cmd_stall_end(host, s, st, seat, ms, why);
            return;
        }
    } else if (moved >= CMD_IMMOBILE_MOVE_M || idle) {
        cmd_mobility_anchor(s, st, ms);
        return;
    }
    if (expected)
        s->imm_need_ms += dt;
    const uint64_t after_hit = s->imm_need_ms - st->hit_need_ms;
    if (st->hit_holds && after_hit >= CMD_IMMOBILE_HIT_MS) {
        const uint64_t stalled_ms = s->immobile == CMD_STALLED ? ms - s->imm_since : 0;
        if (s->immobile == CMD_STALLED)
            cmd_stall_end(host, s, st, seat, ms, "immobilised after a hit");
        s->immobile = CMD_IMMOBILISED;
        ++s->imm_flags;
        ++st->damage_flags;
        s->imm_since = ms;
        s->imm_retry_ms = ms + (st->damage_flags == 1 ? CMD_IMMOBILE_RETRY_MS : CMD_IMMOBILE_RETRY_MAX_MS);
        cmd_log(host, s->army, 1, "cmd: army %d seat %d immobilised at (%.0f,%.0f): %.0f s ordered to move within %.0f m "
                "after a hit %.0f s ago (motion %d, nav %d, %s order %d, stalled %.0f s before), damage flag %d this life, "
                "retry in %.0f s", s->army, seat, s->x, s->z, (double)after_hit / 1000.0, CMD_IMMOBILE_MOVE_M,
                (double)(ms - st->hit_ms) / 1000.0, (int)s->motion_state, s->nav.state, cmd_name[s->cmd_type], s->cmd_id,
                (double)stalled_ms / 1000.0, st->damage_flags, (double)(s->imm_retry_ms - ms) / 1000.0);
        s->imm_x = s->x;
        s->imm_z = s->z;
        return;
    }
    if (s->immobile != CMD_MOBILE || s->imm_need_ms < CMD_IMMOBILE_MS)
        return;
    s->immobile = CMD_STALLED;
    ++s->imm_flags;
    s->imm_since = ms;
    cmd_log(host, s->army, 1, "cmd: army %d seat %d stalled at (%.0f,%.0f): %.0f s ordered to move within %.0f m "
            "(motion %d, nav %d, %s order %d, %s), flag %d this life; keeps its order, out of the quotas", s->army, seat,
            s->x, s->z, (double)s->imm_need_ms / 1000.0, CMD_IMMOBILE_MOVE_M, (int)s->motion_state, s->nav.state,
            cmd_name[s->cmd_type], s->cmd_id,
            !st->hit_ms ? "no hit this life" : st->hit_holds ? "hit, stall after it too short" : "drove on after its last hit",
            s->imm_flags);
    s->imm_x = s->x;
    s->imm_z = s->z;
}

/* The drive a freshly spawned tank of this army has to o, plus the spawn spread. */
static float cmd_spawn_reach(int army, const cmd_obj_t *o)
{
    float best = 1e18f;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const cmd_task_state_t *st = &cmd_tasks[i];
        if (st->army == army && st->spawn_ok && st->life_id) {
            const float d = dist2d(st->spawn_x, st->spawn_z, o->x, o->z);
            best = d < best ? d : best;
        }
    }
    return best < 1e17f ? best + CMD_REACH_SLACK_M : best;
}

const ground_task_order_t *cmd_active_order(int seat)
{
    const ground_task_order_t *order = &cmd_tasks[seat].active;
    return order->task_id ? order : NULL;
}

/* Policy revisions leave the live order intact. Objective invalidation and
 * deadline expiry revoke any motion transaction associated with it. */
int cmd_control_task_valid(const lane_seat_t *s, int seat, uint64_t ms)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    const ground_task_order_t *o = &st->active;
    return o->task_id == s->nav.task_id && o->life_id == s->born_ms
           && !st->invalid_reason && (!o->deadline_ms || ms < o->deadline_ms);
}

const ground_task_order_t *cmd_pending_order(int seat, uint64_t ms)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    return st->pending.task_id && ms >= st->pending.apply_at_ms && ms >= st->retry_after_ms
           ? &st->pending : NULL;
}

ground_engagement_policy_t cmd_motion_policy(const lane_seat_t *s, int seat, uint64_t ms)
{
    const ground_task_order_t *o = cmd_active_order(seat);
    if (o && o->life_id == s->born_ms && (!o->deadline_ms || ms < o->deadline_ms))
        return o->engagement;
    return (ground_engagement_policy_t){
        .allowed = GFM_STOP_FIRE | GFM_HULL_ALIGN | GFM_ADVANCE | GFM_RETURN_COVER | GFM_HOLD,
        .short_stop_budget_ms = CMD_STOP_BUDGET_MS, .urgency = GTO_ROUTINE
    };
}

static void cmd_invalidate(void *host, cmd_task_state_t *st, uint64_t ms, int reason)
{
    if (st->invalid_reason != GTR_NONE)
        return;
    st->invalid_reason = reason;
    cmd_task_event(host, reason == GTR_CAPTURE_COMPLETE ? "completed" : "invalidated", &st->active, ms, reason);
}

void cmd_task_receipt(void *host, lane_seat_t *s, int seat, uint64_t ms,
                             const ground_task_order_t *order, int status, int reason, uint64_t retry_after_ms)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if (order->life_id != s->born_ms || !cmd_same_key(order, &st->pending)
        || order->issued_at_ms != st->pending.issued_at_ms)
        return;
    if (status == GTR_DEFERRED) {
        st->retry_after_ms = retry_after_ms > ms ? retry_after_ms : ms + 250;
        if (st->deferred_reason != reason + 1) {
            st->deferred_reason = reason + 1;
            cmd_task_event(host, "deferred", order, ms, reason);
        }
        return;
    }
    if (status == GTR_REJECTED) {
        cmd_task_event(host, "rejected", order, ms, reason);
        st->rejected = *order;
        st->rejected_until_ms = ms + CMD_ORDER_RETRY_MS;
        cmd_clear_pending(s, st);
        return;
    }
    if (status != GTR_ACCEPTED || ms < order->apply_at_ms || ms < st->retry_after_ms)
        return;
    ground_task_order_t accepted = *order;
    cmd_clear_pending(s, st);
    if (accepted.type == OB_NONE) {
        cmd_task_event(host, "released", &accepted, ms, accepted.reason);
        cmd_clear_active(s, st);
        return;
    }
    if (st->active.task_id != accepted.task_id) {
        if (st->active.task_id)
            cmd_task_event(host, "replaced", &st->active, ms, accepted.reason ? accepted.reason : GTR_SUPERSEDED);
        st->first_accepted_at_ms = ms;
        st->last_active_revision = accepted.revision;
        st->post_release_since_ms = 0;
        st->start_x = s->x;
        st->start_z = s->z;
    }
    uint64_t keep = st->first_accepted_at_ms + CMD_KEEP_MS;
    if (accepted.min_commit_until_ms < keep)
        accepted.min_commit_until_ms = keep;
    cmd_task_event(host, "accepted", &accepted, ms, accepted.reason);
    st->active = accepted;
    st->accepted_at_ms = ms;
    st->missing_since_ms = st->urgency_since_ms = 0;
    st->invalid_reason = GTR_NONE;
    st->report = (ground_task_report_t){0};
    s->cmd_type = accepted.type;
    s->cmd_id = accepted.objective;
    s->cmd_lane = accepted.lane;
    s->cmd_x = accepted.x;
    s->cmd_z = accepted.z;
    s->cmd_side = accepted.support_slot;
    s->cmd_since = st->first_accepted_at_ms;
    s->cmd_release = 0;
}

void cmd_navigation_report(void *host, lane_seat_t *s, int seat, uint64_t ms, const ground_task_report_t *r)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    const ground_task_order_t *o = &st->active;
    if (!o->task_id || r->seat != seat || r->life_id != s->born_ms || r->life_id != o->life_id
        || r->task_id != o->task_id || r->revision != o->revision || r->sampled_ms < st->accepted_at_ms
        || r->sampled_ms > ms || ms - r->sampled_ms > CMD_REPORT_FRESH_MS || r->sampled_ms < st->report.sampled_ms
        || (st->report.sampled_ms && r->generation < st->report.generation))
        return;
    if (!st->report.sampled_ms || st->report.state != r->state || st->report.reason != r->reason
        || st->report.generation != r->generation) {
        static const char *const names[] = {
            "need_route", "planning", "ready", "following", "wait_traffic", "recovering",
            "replan_pending", "arrived", "blocked", "unreachable", "cancelled"
        };
        const char *status = r->state >= 0 && r->state < (int)(sizeof names / sizeof names[0])
                             ? names[r->state] : "unknown";
        event(host, "task: nav seat %d life %llu key %llu/%u route %u/%u at %llu %s reason %s",
              seat, (unsigned long long)r->life_id, (unsigned long long)r->task_id, r->revision,
              r->route_id, r->generation, (unsigned long long)r->sampled_ms, status, cmd_task_reason(r->reason));
    }
    st->report = *r;
    if (r->state == GNR_UNREACHABLE && r->reachability == GNR_REACH_NO_ROUTE)
        cmd_invalidate(host, st, ms, GTR_NO_ROUTE);
}

static int cmd_report_fresh(const cmd_task_state_t *st, uint64_t ms)
{
    return st->report.sampled_ms && ms - st->report.sampled_ms <= CMD_REPORT_FRESH_MS;
}

static int cmd_report_blocked(const cmd_task_state_t *st, uint64_t ms)
{
    return cmd_report_fresh(st, ms)
           && (st->report.state == GNR_BLOCKED || st->report.state == GNR_UNREACHABLE);
}

static int cmd_hard_loss(void *host, int army, const ground_task_order_t *o, uint64_t ms)
{
    if (o->deadline_ms && ms >= o->deadline_ms)
        return GTR_EXPIRED;
    if (o->type == OB_CAPTURE || o->type == OB_HOLD) {
        const lane_t *l = &lanes[o->lane];
        int owner = zone_owner(host, l->zx, l->zz);
        if (o->type == OB_CAPTURE && owner == army)
            return GTR_CAPTURE_COMPLETE;
        if (o->type == OB_HOLD && owner >= 0 && owner != army)
            return GTR_POINT_CHANGED;
    } else if (o->type == OB_SUPPORT) {
        int h = o->objective;
        if (last_unit_scan >= o->issued_at_ms
            && (human_slot != h || !slot_alive[h] || slot_army[h] != army || slot_life[h] != o->objective_life))
            return GTR_OBJECTIVE_LOST;
    } else if (o->type == OB_PRESS) {
        for (int k = 0; k < nlanes; ++k) {
            if (lanes[k].army != army || lanes[k].kind != 0)
                continue;
            int owner = zone_owner(host, lanes[k].zx, lanes[k].zz);
            if (owner >= 0 && owner != army)
                return GTR_POINT_CHANGED;
        }
    } else if (o->post_option >= 0 && ms < cmd_posts[army][o->objective - CMD_POST_ID_BASE].pos[o->post_option].blocked_until) {
        return GTR_POST_INVALID;
    }
    return GTR_NONE;
}

int cmd_validate_pending(void *host, const lane_seat_t *s, int seat, uint64_t ms,
                                const ground_task_order_t *order)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    if (order->life_id != s->born_ms || !cmd_task_unit_alive(s, seat)
        || !cmd_same_key(order, &st->pending) || order->issued_at_ms != st->pending.issued_at_ms)
        return GTR_STALE;
    if (order->type == OB_NONE)
        return !st->invalid_reason && !st->missing_since_ms && !(cmd_is_post(s) && s->cmd_release)
               ? GTR_STALE : GTR_NONE;
    return cmd_hard_loss(host, s->army, order, ms);
}

static void cmd_track_objectives(int army, const cmd_obj_t *objectives, int count, uint64_t ms)
{
    for (int i = 0; i < count; ++i) {
        const cmd_obj_t *o = &objectives[i];
        uint64_t *last = &cmd_objective_last[army][o->type][o->id];
        if (!*last || *last + 1 != cmd_epoch[army])
            cmd_objective_since[army][o->type][o->id] = ms;
        *last = cmd_epoch[army];
    }
}

static int cmd_objective_ready(int army, const cmd_obj_t *o, uint64_t ms)
{
    if (o->type == OB_CAPTURE || o->type == OB_HOLD || o->id >= CMD_POST_ID_BASE)
        return 1;
    return ms - cmd_objective_since[army][o->type][o->id] >= CMD_SOFT_ENTRY_MS;
}

static void cmd_check_orders(void *host, lane_seat_t *s, int seat, uint64_t ms, int current_found, int pending_found)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if (st->active.task_id) {
        int loss = cmd_hard_loss(host, s->army, &st->active, ms);
        if (loss)
            cmd_invalidate(host, st, ms, loss);
        if (current_found)
            st->missing_since_ms = 0;
        else if (!st->missing_since_ms)
            st->missing_since_ms = ms;
    }
    if (st->pending.task_id && st->pending.type != OB_NONE) {
        if (pending_found)
            st->pending_missing_ms = 0;
        else if (!st->pending_missing_ms)
            st->pending_missing_ms = ms;
        int loss = cmd_hard_loss(host, s->army, &st->pending, ms);
        if (!loss && st->pending_missing_ms && ms - st->pending_missing_ms >= CMD_SOFT_EXIT_MS)
            loss = GTR_SOFT_EXIT;
        if (loss) {
            cmd_task_event(host, "pending_cancelled", &st->pending, ms, loss);
            st->rejected = st->pending;
            cmd_clear_pending(s, st);
        }
    } else if (st->pending.task_id && current_found && !st->invalid_reason && !(cmd_is_post(s) && s->cmd_release)) {
        cmd_task_event(host, "pending_cancelled", &st->pending, ms, GTR_STALE);
        cmd_clear_pending(s, st);
    }
}

static int cmd_order_blocks_reassignment(const cmd_task_state_t *st, uint64_t ms)
{
    /* A stay order has no commitment: it is held only while the tank is immobilised. */
    return st->active.task_id && !st->invalid_reason && st->active.type != OB_STAY
           && (st->active.type == OB_CAPTURE || ms < st->active.min_commit_until_ms
               || (st->missing_since_ms && ms - st->missing_since_ms < CMD_SOFT_EXIT_MS));
}

static int cmd_pending_reserves(const cmd_task_state_t *st)
{
    return st->pending.task_id && !st->deferred_reason;
}

static int cmd_accepted_reserves(const cmd_task_state_t *st, uint64_t ms)
{
    return st->active.task_id && !st->invalid_reason
           && (st->active.type == OB_CAPTURE || (!cmd_report_blocked(st, ms)
               && (cmd_report_fresh(st, ms) || ms - st->accepted_at_ms < CMD_REPORT_FRESH_MS)));
}

static void cmd_support_region(int army, int slot, float hx, float hz, float *x, float *z)
{
    const strat_meta_t *m = strat.m;
    static const float back[3] = {60.0f, 60.0f, 90.0f}, side[3] = {-50.0f, 50.0f, -75.0f};
    float sg = army == 1 ? -1.0f : 1.0f;
    *x = hx + sg * m->axx * back[slot] + m->latx * side[slot];
    *z = hz + sg * m->axz * back[slot] + m->latz * side[slot];
}

static int cmd_wing_slot(int army, int seat, int human)
{
    int used = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        if (i == seat || cmd_tasks[i].army != army)
            continue;
        const cmd_task_state_t *other = &cmd_tasks[i];
        if (other->active.type == OB_SUPPORT && other->active.objective == human)
            used |= 1 << other->active.support_slot;
        if (other->pending.type == OB_SUPPORT && other->pending.objective == human)
            used |= 1 << other->pending.support_slot;
    }
    return !(used & 1) ? 0 : !(used & 2) ? 1 : 2;
}

static int cmd_same_goal(const ground_task_order_t *a, const ground_task_order_t *b)
{
    return a->type == b->type && a->objective == b->objective && a->objective_life == b->objective_life;
}

static ground_task_order_t cmd_make_order(int army, int seat, const cmd_obj_t *o, uint64_t ms, int reason)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    ground_task_order_t out = {
        .life_id = st->life_id, .epoch_id = cmd_epoch[army], .issued_at_ms = ms, .seat = seat,
        .type = o->type, .objective = o->id, .lane = o->lane, .post_option = -1, .reason = reason,
        .x = o->x, .z = o->z, .radius = CMD_ARRIVE_M, .value_rank = o->value
    };
    out.engagement.urgency = o->type == OB_CAPTURE && sview[army].point_pressure >= 0.65f
                             ? GTO_TIME_CRITICAL : GTO_ROUTINE;
    if (o->type == OB_CAPTURE || o->type == OB_HOLD) {
        out.radius = lanes[o->lane].zr;
    } else if (o->type == OB_SUPPORT) {
        out.objective_life = slot_life[o->id];
        out.support_slot = cmd_same_goal(&st->active, &out) ? st->active.support_slot : cmd_wing_slot(army, seat, o->id);
        cmd_support_region(army, out.support_slot, o->x, o->z, &out.x, &out.z);
    } else if (o->type == OB_OVERWATCH && o->id >= CMD_POST_ID_BASE) {
        out.post_option = cmd_post_option(&cmd_posts[army][o->id - CMD_POST_ID_BASE], ms);
        out.radius = CMD_POST_ARRIVE_M;
    }
    return out;
}

static int cmd_publish(void *host, lane_seat_t *s, int seat, ground_task_order_t out, uint64_t ms)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    const ground_task_order_t *active = &st->active;
    if (st->pending.task_id || !st->life_id
        || (st->rejected.task_id && ms < st->rejected_until_ms && cmd_same_goal(&st->rejected, &out)
            && st->rejected.post_option == out.post_option && dist2d(st->rejected.x, st->rejected.z, out.x, out.z) < 10.0f))
        return 0;
    if (active->task_id && (out.type == OB_NONE || cmd_same_goal(active, &out))) {
        out.task_id = active->task_id;
        out.revision = ++st->last_active_revision;
        out.min_commit_until_ms = active->min_commit_until_ms;
    } else {
        out.task_id = ++cmd_task_serial;
        out.revision = 1;
    }
    if (st->rejected.task_id && out.type != OB_NONE && cmd_same_goal(&st->rejected, &out)) {
        if (!active->task_id || (out.task_id != active->task_id && st->rejected.task_id != active->task_id)) {
            out.task_id = st->rejected.task_id;
            out.revision = st->rejected.revision + 1;
        }
    }
    cmd.rng = cmd.rng * 1664525u + 1013904223u;
    out.apply_at_ms = ms + 1000 + (cmd.rng >> 8) % 2000u;
    if (!out.min_commit_until_ms)
        out.min_commit_until_ms = out.apply_at_ms + CMD_KEEP_MS;
    const int urgent = out.type == OB_CAPTURE && out.engagement.urgency == GTO_TIME_CRITICAL;
    out.engagement = (ground_engagement_policy_t){
        .task_id = out.task_id, .revision = out.revision, .deadline_ms = out.deadline_ms,
        .allowed = GFM_STOP_FIRE | GFM_HULL_ALIGN | GFM_HOLD,
        .short_stop_budget_ms = urgent ? CMD_URGENT_STOP_BUDGET_MS : CMD_STOP_BUDGET_MS,
        .urgency = urgent ? GTO_TIME_CRITICAL : GTO_ROUTINE
    };
    if (!urgent && out.type != OB_STAY)     /* an immobilised tank can only shoot */
        out.engagement.allowed |= GFM_ADVANCE | GFM_RETURN_COVER;
    st->pending = out;
    s->cmd_ntype = out.type;
    s->cmd_nid = out.objective;
    s->cmd_nlane = out.lane;
    s->cmd_nx = out.x;
    s->cmd_nz = out.z;
    s->cmd_apply_ms = out.apply_at_ms;
    cmd_task_event(host, "published", &out, ms, out.reason);
    return 1;
}

static int cmd_refresh_capture_policy(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    cmd_task_state_t *st = &cmd_tasks[seat];
    if (st->active.type != OB_CAPTURE || st->invalid_reason)
        return 0;
    float pressure = sview[s->army].point_pressure;
    int old_urgent = st->active.engagement.urgency == GTO_TIME_CRITICAL;
    int urgent = pressure >= (old_urgent ? 0.55f : 0.65f);
    if (urgent == old_urgent) {
        st->urgency_since_ms = 0;
        return 0;
    }
    if (!st->urgency_since_ms)
        st->urgency_since_ms = ms;
    if (ms - st->urgency_since_ms < CMD_SOFT_ENTRY_MS || st->pending.task_id)
        return 0;
    ground_task_order_t updated = st->active;
    updated.epoch_id = cmd_epoch[s->army];
    updated.issued_at_ms = ms;
    updated.reason = GTR_NONE;
    updated.engagement.urgency = urgent ? GTO_TIME_CRITICAL : GTO_ROUTINE;
    return cmd_publish(host, s, seat, updated, ms);
}

static int cmd_offer_unavailable(int army, int seat, const cmd_obj_t *o, uint64_t ms)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    if (!st->invalid_reason && (!st->rejected.task_id || ms >= st->rejected_until_ms))
        return 0;
    ground_task_order_t next = cmd_make_order(army, seat, o, ms, GTR_NONE);
    if (st->invalid_reason && cmd_same_goal(&st->active, &next)
        && st->active.post_option == next.post_option && dist2d(st->active.x, st->active.z, next.x, next.z) < 10.0f)
        return 1;
    return st->rejected.task_id && ms < st->rejected_until_ms && cmd_same_goal(&st->rejected, &next)
           && st->rejected.post_option == next.post_option && dist2d(st->rejected.x, st->rejected.z, next.x, next.z) < 10.0f;
}

static void cmd_publish_release(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    const cmd_task_state_t *st = &cmd_tasks[seat];
    if (!st->active.task_id || st->pending.task_id || cmd_order_blocks_reassignment(st, ms)
        || (!st->invalid_reason && !st->missing_since_ms && !(cmd_is_post(s) && s->cmd_release)))
        return;
    ground_task_order_t release = st->active;
    release.type = OB_NONE;
    release.epoch_id = cmd_epoch[s->army];
    release.issued_at_ms = ms;
    release.deadline_ms = 0;
    release.reason = st->invalid_reason ? st->invalid_reason : GTR_SOFT_EXIT;
    cmd_publish(host, s, seat, release, ms);
}

static int tp2_resume_barred(int army, int seat, int lane, uint64_t ms);   /* ground_teamplan.inc (s37) */
int tp2_area_at(float x, float z);                                   /* ground_teamplan.inc (s40) */
void tp2_area_count(int army, uint64_t ms, int *area_have);
int tp2_area_open(int army, int seat, int a, const int *area_have);

void cmd_request_post_resume(void *host, lane_seat_t *s, int seat, uint64_t ms, int reason)
{
    const ground_task_order_t *active = cmd_active_order(seat);
    if (reason == GTR_NONE || !active || active->type != OB_OVERWATCH || active->post_option < 0)
        return;
    const int was_invalid = cmd_tasks[seat].invalid_reason != GTR_NONE;
    cmd_post_t *p = &cmd_posts[s->army][active->objective - CMD_POST_ID_BASE];
    if (ms >= p->pos[active->post_option].blocked_until) {
        cmd_invalidate(host, &cmd_tasks[seat], ms, reason);
        p->pos[active->post_option].blocked_until = ms + CMD_POST_BLOCK_MS;
    }
    if (cmd_tasks[seat].pending.task_id)
        return;
    int option = tp2_old_post_off(s->army, p) ? -1 : cmd_post_option(p, ms);    /* s41: an old post is off */
    if (option >= 0) {
        /* s40: another position of the post in an area at its cap (for this tank) is not taken: as no position left */
        int ah[TP_MAX_ZONES];
        tp2_area_count(s->army, ms, ah);
        if (!tp2_area_open(s->army, seat, tp2_area_at(p->pos[option].x, p->pos[option].z), ah)) {
            event(host, "cmd: army %d seat %d post %d position %d not taken: its area is at its cap", s->army, seat,
                  active->objective, option + 1);
            option = -1;
        }
    }
    if (option < 0 && tp2_resume_barred(s->army, seat, p->lane, ms)) {
        /* s37 G4: no position left and its zone at its quota: no capture / hold order; the order is invalidated and
         * released (a kept plan post would otherwise hold the crew), the commander gives it another under the quota */
        if (cmd_tasks[seat].invalid_reason == GTR_NONE)
            cmd_invalidate(host, &cmd_tasks[seat], ms, reason);
        if (!was_invalid)
            event(host, "cmd: army %d seat %d post %d has no usable position; zone of lane %d at its quota: no capture / hold "
                  "order, released for another order", s->army, seat, active->objective, p->lane);
        cmd_publish_release(host, s, seat, ms);
        return;
    }
    const lane_t *lane = &lanes[p->lane];
    cmd_obj_t o = {
        .type = option >= 0 ? OB_OVERWATCH : zone_owner(host, lane->zx, lane->zz) == s->army ? OB_HOLD : OB_CAPTURE,
        .id = option >= 0 ? active->objective : p->lane, .lane = p->lane,
        .x = option >= 0 ? p->pos[option].x : lane->zx, .z = option >= 0 ? p->pos[option].z : lane->zz,
        .value = active->value_rank
    };
    cmd_publish(host, s, seat, cmd_make_order(s->army, seat, &o, ms, reason), ms);
}
