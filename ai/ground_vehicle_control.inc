/* The world observer publishes one fleet plan. The drive wrapper owns the
 * final four actuator bytes and can only consume a matching publication. */
#define CONTROL_STAMP_MS 500
/* How long a move order counts as fresh. The game's publications come 90 to
 * 125 ms apart, but under load several hundred: at 200 ms every such gap
 * dropped the whole team from the plan at once. */
#define MOTION_EXPIRY_MS 500
/* Without a plan for this frame the last written command is held this long
 * before the default (neutral, full brake) takes over. */
#define MOTION_HOLD_MS 400
#define MOTION_FOOTPRINT_MARGIN 0.10f
#define MOTION_SWEEP_DISTANCE 0.45f
#define MOTION_SWEEP_ANGLE 0.08f
#define MOTION_CAPTURE_DWELL_MS 1600
#define MOTION_CAPTURE_STOP_MPS 0.35f
#define MOTION_OBSTACLE_PROBES 3

typedef struct {
    u8 *unit;
    uint64_t life;
    float length, width, forward, right;
} traffic_hull_t;

typedef struct {
    u8 *unit;
    float x, y, z, fx, fz, length, width;
} traffic_body_t;

static traffic_hull_t traffic_hulls[MAX_SEATS];

/* Measured hulls of tanks the adapter does not own (humans, native AI,
 * wrecks). The route layer plans around stationary hulls, so these need the
 * same collision-node footprint as owned bots. Keyed by unit and alive state
 * and refreshed periodically, since a unit object can be reused. */
#define MOTION_OTHER_HULLS 256
#define MOTION_OTHER_HULL_REFRESH_MS 5000
typedef struct {
    traffic_hull_t hull;
    uint64_t measured_ms;
    int alive;
} motion_other_hull_t;
static motion_other_hull_t motion_other_hulls[MOTION_OTHER_HULLS];

typedef struct {
    uint64_t generation, task, ms, native_retry_marker;
    unsigned revision, route_generation;
    int waypoint, source, move, align, native_path, native_direct, native_reaches_goal, capture;
    int order_goal;     /* s37 G11: `goal` is the order's own drive (see motion_intent), not a reaction / fight / recovery */
    float arrival_radius, align_heading;
    gm_point_t origin, goal;
    gm_point_t path[GM_MAX_ROUTE_POINTS];
    int path_count;
} motion_intent_t;

typedef struct {
    uint64_t id, committed_ms, spent_ms, limit_ms;
    uint64_t follow_ms, follow_task;
    float follow_x, follow_z, goal_x, goal_z, progress_m, progress_x, progress_z;
    int stop, align, route_hold, started;
    int face_owned;             /* s29: the last request was the threat facing's (never charged to the budget) */
} control_engagement_t;

typedef struct {
    uint64_t life, last_plan_seq, next_request;
    uint64_t capture_still_since_ms, capture_still_task;
    unsigned capture_still_revision;
    gm_yaw_response_t yaw_response;
    gm_control_history_t history;
    gm_controls_t written;
    u8 *owned_controls;
    int has_written;
    uint64_t written_ms;  /* s20: when the tracker last wrote (control graft hold) */
    /* s22 WASD driver (cfg.graft, ground_control_graft.inc): the tick computes the keys, phase 0 writes them.
     * With the driver on, `written` is what the layer makes of the keys (bookkeeping only). */
    gd_input_t drive_in;
    gd_memory_t drive;
    gd_info_t drive_info;
    gd_keys_t keys;
    uint64_t keys_ms;
    int drive_phys, drive_can_pivot, drive_separate, drive_boost;
    float drive_brake;
    /* s26 probe (wasd_drive.csv): the plan's library columns as the driver got them (wasd_tick) */
    int lib_source, lib_route, lib_sample;
    float lib_cross, lib_clear, stop_vehicle, stop_turn, stop_end;
    uint64_t lib_blocker;
    uint64_t applied_ms; /* last frame that had a matching plan */
    /* Without a scan: whether the engine supplied path points (1 yes, 2 no), for the log. */
    int native_seen;
    uint64_t native_log_ms;
    control_engagement_t engagement;
    motion_intent_t intent;
} vehicle_control_t;

static gm_state_t motion_state;
static gm_plan_t motion_plan;
static gm_vehicle_t motion_vehicles[GM_MAX_VEHICLES];
static int motion_vehicle_count;    /* s17: entries of motion_vehicles of the last publication (overhead hull filter) */
static gm_point_t motion_paths[MAX_SEATS][GM_MAX_ROUTE_POINTS];
static gm_pending_command_t motion_packets[MAX_SEATS][GM_CONTROL_HISTORY_CAPACITY];
static gm_spawn_site_t motion_spawn_sites[MAX_AUTHORED_SPAWN_SITES];
static uint64_t motion_opening_ms;
static unsigned char motion_seen_seat[MAX_SEATS];
static int motion_index[MAX_SEATS];
static int motion_tactical_points[MAX_SEATS][GM_MAX_ROUTE_POINTS];
static uint64_t next_motion_ms;
static vehicle_control_t vehicle_control[MAX_SEATS];

/* Zeroing motion_state also clears motion_state.nav. The planner then
 * re-acquires the process-wide route layer (gm_nav_shared) on the next
 * snapshot and calls gm_nav_reset on it, so the adapter never resets nav
 * itself. A changed snapshot.map_epoch additionally rebuilds the grid. */
static void motion_reset(void)
{
    memset(&motion_state, 0, sizeof motion_state);
    memset(&motion_plan, 0, sizeof motion_plan);
    memset(vehicle_control, 0, sizeof vehicle_control);
    memset(traffic_hulls, 0, sizeof traffic_hulls);
    memset(motion_other_hulls, 0, sizeof motion_other_hulls);
    memset(motion_seen_seat, 0, sizeof motion_seen_seat);
    motion_opening_ms = 0;
    next_motion_ms = 0;
}
typedef struct control_frame {
    struct control_frame *previous;
    u8 *node;
    u8 *unit;
    uint64_t life, plan_seq;
    int seat;
    float saved_arrival_radius;
} control_frame_t;

_Static_assert(sizeof(control_frame_t) == 48, "drive wrapper frame ABI");


static __declspec(thread) control_frame_t *active_control_frame;

static vehicle_control_t *control_for(const lane_seat_t *s, int seat)
{
    vehicle_control_t *c = &vehicle_control[seat];
    if (c->life != s->born_ms)
        *c = (vehicle_control_t){.life = s->born_ms};
    return c;
}

void ground_control_begin(u8 *node, control_frame_t *frame)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || (!cfg.lanes && !cfg.native_control) || !node)
        return;
    *frame = (control_frame_t){.previous = active_control_frame, .node = node};
    active_control_frame = frame;
}

/* Navigation calls this after its sole publication point inside the native
 * call. A planning sample on another thread or invocation cannot be used. */
#define CONTROL_PORTAL_RADIUS GM_PRODUCTION_TIGHT_ARRIVAL_M

static void control_note_plan(const lane_seat_t *s, int seat, u8 *node, int tight_portal)
{
    control_frame_t *frame = active_control_frame;
    if (!frame || frame->node != node)
        return;
    frame->unit = s->unit;
    frame->seat = seat;
    frame->life = s->born_ms;
    frame->plan_seq = s->move_seq;
    /* The native driver tests this exact field before its arrival return.
     * Scope the smaller radius to the paired rolling-portal invocation. */
    if (tight_portal && !frame->saved_arrival_radius) {
        const float radius = gh_node_arrival(node);
        if (isfinite(radius) && radius > CONTROL_PORTAL_RADIUS) {
            frame->saved_arrival_radius = radius;
            gh_node_set_arrival(node, CONTROL_PORTAL_RADIUS);
        }
    }
}

#define FACE_START_DEG 35.0f    /* 2026-11-11: nose within ~30 deg (review_5003 A4: 45, before 25) */
#define FACE_STOP_DEG 15.0f     /* (5003: 20, before 10) */
#define CLOSE_FACE_DEG 30.0f    /* B1: nose within +-30 deg of a close enemy */
#define CLOSE_FACE_STOP_DEG 12.0f
#define FACE_READY_S 1.5f       /* B3 / A4: no pivot from 1.5 s before the round is in */
#define FACE_MAX_MS 4000
#define FACE_MAX_BIG_MS 8000    /* pivots that start more than 90 deg off */
#define FACE_COOL_MS 4000
static int turret_can_bear(u8 *unit, const lane_seat_t *s, float tx, float tz)
{
    u8 *turrets = gh_unit_turrets(unit);
    float lo, hi;
    if (!turrets || !turret_yaw_limits(turrets, 0, &lo, &hi))
        return 1;
    if (!yaw_sign)
        return 0;               /* limited traverse, sign unknown: let the hull do it */
    float rel = (float)yaw_sign * wrap180((atan2f(tz - s->z, tx - s->x) - atan2f(s->fz, s->fx)) * 57.29578f);
    return rel >= lo + 2.0f && rel <= hi - 2.0f;
}

static u8 *aim_coll(u8 *unit, u8 **hdr);
static int aim_node_box(void *host, u8 *coll, u8 *hdr, int n, float *centre, float *half);
static int aim_prefix(const char *s, const char *p);

/* s49: the guarded block of traffic_hull, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; u8 *unit; traffic_hull_t hull; } traffic_hull_ctx_t;
static void traffic_hull_read(void *ctx)
{
    traffic_hull_ctx_t *const context = ctx;
    void *const host = context->host;
    u8 *const unit = context->unit;
    traffic_hull_t hull = context->hull;
    {
        u8 *hdr, *coll = aim_coll(unit, &hdr);
        if (!coll || !gh_coll_has_names(hdr))
            goto leave;
        const uint32_t count = gh_coll_count(hdr);
        if (count > 4096)
            goto leave;
        const u8 *nodes = gh_coll_nodes(hdr);
        const char *names = gh_coll_names(hdr);
        float x0 = 50.0f, z0 = 50.0f, x1 = -50.0f, z1 = -50.0f;
        for (uint32_t i = 0; i < count; ++i) {
            const char *name = gh_coll_node_name(names, nodes, i);
            const size_t len = strlen(name);
            if (len <= 3 || strcmp(name + len - 3, "_dm")
                || !(aim_prefix(name, "body_") || aim_prefix(name, "superstructure_") || aim_prefix(name, "hull_")))
                continue;
            float c[3], h[3];
            if (!aim_node_box(host, coll, hdr, (int)i, c, h))
                continue;
            x0 = fminf(x0, c[0] - h[0]);
            x1 = fmaxf(x1, c[0] + h[0]);
            z0 = fminf(z0, c[2] - h[2]);
            z1 = fmaxf(z1, c[2] + h[2]);
        }
        const float length = 0.5f * (x1 - x0), width = 0.5f * (z1 - z0);
        const float forward = 0.5f * (x1 + x0), right = 0.5f * (z1 + z0);
        if (length >= 1.4f && length <= 5.5f && width >= 0.65f && width <= 2.5f
            && fabsf(forward) <= 2.0f && fabsf(right) <= 0.75f) {
            hull.length = length;
            hull.width = width;
            hull.forward = forward;
            hull.right = right;
        }
    }
leave:
    context->hull = hull;
}

/* Read model-space collision nodes without touching the gunner's part cache.
 * Hull and superstructure nodes supply the footprint, excluding barrels. */
static traffic_hull_t traffic_hull(void *host, u8 *unit, uint64_t life)
{
    traffic_hull_t hull = {unit, life, 3.1f, 1.65f, 0.0f, 0.0f};
    if (!host || !unit)
        return hull;
    traffic_hull_ctx_t context = {host, unit, hull};
    const int faulted = gh_guard(traffic_hull_read, &context);
    hull = context.hull;
    if (faulted) {
        /* Retain the nominal hull if the model's nodes are unavailable. */
    }
    return hull;
}


#include "ground_geometry.inc"


#define CONTROL_ENGAGEMENT_REARM_M 8.0f

int control_engagement_active(const lane_seat_t *s, uint64_t ms)
{
    const vehicle_control_t *c = &vehicle_control[s - lseat];
    const control_engagement_t *e = &c->engagement;
    return c->life == s->born_ms && e->committed_ms && ms >= e->committed_ms && e->spent_ms < e->limit_ms
           && ms - e->committed_ms < e->limit_ms - e->spent_ms;
}

static void control_charge_engagement(control_engagement_t *e, uint64_t ms)
{
    if (!e->committed_ms || ms < e->committed_ms)
        return;
    const uint64_t elapsed = ms - e->committed_ms;
    const uint64_t remaining = e->limit_ms - e->spent_ms;
    e->spent_ms += elapsed < remaining ? elapsed : remaining;
    e->committed_ms = 0;
}

static void control_follow_progress(control_engagement_t *e, const lane_seat_t *s, uint64_t ms)
{
    if (e->follow_ms && ms >= e->follow_ms && ms - e->follow_ms <= CONTROL_STAMP_MS
        && s->nav.task_id == e->follow_task && s->pose_observed_ms > e->follow_ms
        && dist2d(s->x, s->z, e->follow_x, e->follow_z) <= 12.0f) {
        const float gain = dist2d(e->follow_x, e->follow_z, e->goal_x, e->goal_z)
                         - dist2d(s->x, s->z, e->goal_x, e->goal_z);
        e->progress_m = fmaxf(0.0f, e->progress_m + gain);
    }
    e->follow_ms = 0;
    if (e->started && e->progress_m >= CONTROL_ENGAGEMENT_REARM_M
        && dist2d(s->x, s->z, e->progress_x, e->progress_z) >= CONTROL_ENGAGEMENT_REARM_M) {
        e->id = e->spent_ms = e->limit_ms = 0;
        e->progress_m = 0.0f;
        e->started = 0;
    }
}


/* Tactical engagement supplies a motion request. Traffic and geometry are
 * evaluated by the same planner as ordinary movement. */
static int control_face_requested(lane_seat_t *s, int seat, uint64_t ms)
{
    const float dx = s->face_x - s->x, dz = s->face_z - s->z;
    if (!cfg.face || !s->face || s->fight_move == FIGHT_LOCAL || ms < s->pivot_cool_until
        || hypotf(dx, dz) <= 1.0f || s->fx * s->fx + s->fz * s->fz <= 0.25f)
        return 0;
    const float error = movement_turn_error(s, s->face_x, s->face_z);
    const int post_idle = cmd_is_post(s) && s->cmd_parked && s->parked
                          && !s->post_defending && s->gt_slot < 0 && s->mode != M_ENGAGE;
    const int window = post_idle || s->rear || seat_ready_in(seat, ms) > FACE_READY_S
                       || !turret_can_bear(s->unit, s, s->face_x, s->face_z);
    const float start = s->rear ? REAR_DONE_DEG : s->close ? CLOSE_FACE_DEG
                        : post_idle ? FACE_STOP_DEG + 5.0f : FACE_START_DEG;
    const float stop = s->rear ? REAR_DONE_DEG * 0.6f : s->close ? CLOSE_FACE_STOP_DEG : FACE_STOP_DEG;
    const int target = s->mode == M_ENGAGE ? s->eng : -1;
    if (!window || error < stop || (s->pivoting && target != s->pivot_tgt))
        return 0;
    if (s->pivoting) {
        const uint64_t limit = s->pivot_err0 > 90.0f ? FACE_MAX_BIG_MS : FACE_MAX_MS;
        if (ms - s->pivot_start <= limit)
            return 1;
        s->pivot_cool_until = ms + FACE_COOL_MS;
        return 0;
    }
    if (error <= start)
        return 0;
    s->pivot_err0 = error;
    s->pivot_tgt = target;
    return 1;
}

static void control_prepare_engagement(lane_seat_t *s, int seat, uint64_t ms, int route_hold)
{
    vehicle_control_t *c = control_for(s, seat);
    control_engagement_t *e = &c->engagement;
    if (e->face_owned) {
        e->committed_ms = 0;            /* s29: the facing's time is not charged to the short-stop budget */
        e->face_owned = 0;
    }
    control_charge_engagement(e, ms);
    control_follow_progress(e, s, ms);
    e->stop = e->align = 0;
    e->route_hold = route_hold;
    if (cfg.threatface) {
        /* s29: the reaction layer's threat facing is the one hull request: turn (align, with its stop) or hold; no
         * budget, no cap, no cool-down, no reload window (ground_tactics.inc tac_face_hull) */
        const int tf = tac_face_hull(s, seat, ms);
        if (tf) {
            e->stop = 1;
            e->align = tf == 1;
            e->face_owned = 1;
            if (!e->id)
                e->id = ++c->next_request;
            e->limit_ms = e->spent_ms + 3600000u;
            return;
        }
        if (tac_face_busy(s, seat))
            return;                     /* on the way to cover: no stop to fire (as the old cover hop) */
    }
    const int fire = (s->stopping || s->close || s->rear) && !(s->cov_use && s->cov_state == 1);
    /* threatface=1: the face / close / rear rules no longer turn the hull; their stop to fire stays */
    const int face = !cfg.threatface && s->face && (fire || route_hold);
    if (!fire && !face)
        return;
    const ground_engagement_policy_t policy = cmd_motion_policy(s, seat, ms);
    const uint64_t budget = fire ? policy.short_stop_budget_ms : FACE_MAX_MS;
    if (!budget || e->spent_ms >= budget || (e->limit_ms && e->spent_ms >= e->limit_ms)
        || (policy.deadline_ms && ms >= policy.deadline_ms)
        || (policy.urgency == GTO_TIME_CRITICAL && !route_hold && !s->close && !s->rear))
        return;
    e->stop = fire && (policy.allowed & GFM_STOP_FIRE);
    e->align = face && (policy.allowed & GFM_HULL_ALIGN) && control_face_requested(s, seat, ms);
    if (e->stop || e->align) {
        if (!e->id)
            e->id = ++c->next_request;
        if (!e->limit_ms || budget < e->limit_ms)
            e->limit_ms = budget;
    }
}

/* The game's own move order measures speed from vehicle-dependent sources. Reuse them
 * sources as vectors; position differencing covers unavailable physics data. */
static void motion_velocity(const u8 *unit, float fallback_x, float fallback_z, float *vx, float *vz)
{
    float x, z;
    gh_unit_velocity(unit, fallback_x, fallback_z, &x, &z);
    *vx = isfinite(x) ? x : fallback_x;
    *vz = isfinite(z) ? z : fallback_z;
}

static gm_controls_t control_native(const u8 *controls)
{
    return (gm_controls_t){gh_drive_throttle(controls), gh_drive_steer(controls),
                           gh_drive_brake_left(controls), gh_drive_brake_right(controls)};
}

static void control_write(u8 *controls, gm_controls_t action)
{
    _Static_assert(sizeof action == 4, "native ground control packet");
    gh_drive_write(controls, &action, sizeof action);
}

/* Outside the drive branch, return only a brake pair still owned by the
 * adapter. Native writers may already have replaced one or both inputs. */
static void control_release(void *host, const lane_seat_t *s, vehicle_control_t *c, int seat)
{
    if (!c->has_written)
        return;
    u8 *controls = gh_unit_controls(s->unit);
    /* s22: with the WASD driver on, no brake byte was ever ours: nothing to give back. */
    if (!cfg.graft && controls && controls == c->owned_controls) {
        const gm_controls_t native = control_native(controls);
        if (native.brake_left == c->written.brake_left && native.brake_right == c->written.brake_right
            && (native.brake_left || native.brake_right)) {
            gh_drive_release_brakes(controls);
            event(host, "motion: seat %d life %llu released brake pair %u/%u", seat,
                  (unsigned long long)s->born_ms, (unsigned)native.brake_left, (unsigned)native.brake_right);
        }
    }
    c->has_written = 0;
    c->owned_controls = NULL;
    gm_control_yaw_release(&c->yaw_response);
    c->history = (gm_control_history_t){0};
}

/* Publish a route identity independently of this invocation's native
 * actuator result. The stock path can refine the same goal, but cannot
 * introduce a different goal or create a second vehicle controller. */
static const gm_vehicle_plan_t *motion_current_plan(int seat, uint64_t life, uint64_t route, uint64_t ms)
{
    const int index = motion_index[seat];
    if (index < 0 || index >= motion_plan.count || ms < motion_plan.ms || ms > motion_plan.valid_until_ms)
        return NULL;
    const gm_vehicle_plan_t *plan = &motion_plan.vehicles[index];
    return plan->id == (uint64_t)seat && plan->life_id == life && plan->route_id == route ? plan : NULL;
}

/* With a usable scan the nav route layer owns static routing. Without one
 * (no scan, or the planner reports GM_NAV_NO_MAP because the route layer
 * could not be allocated) the planner follows the upstream path directly, and
 * stock can refresh a genuinely blocked route unless a nav route is followed. */
static int motion_native_repath_allowed(const lane_seat_t *s, int seat, uint64_t ms)
{
    const gm_vehicle_plan_t *plan = motion_current_plan(seat, s->born_ms, vehicle_control[seat].intent.generation, ms);
    if (geometry.game && geometry.scan && !(plan && plan->route_status == GM_NAV_NO_MAP))
        return 0;
    return !plan || !plan->repair_active;
}

static const lane_t *motion_capture_lane(const lane_seat_t *s, int seat, uint64_t ms)
{
    const ground_task_order_t *order = cmd_active_order(seat);
    if (!order || order->type != OB_CAPTURE || order->revision != s->nav.revision
        || order->lane < 0 || order->lane >= nlanes || !cmd_control_task_valid(s, seat, ms))
        return NULL;
    return &lanes[order->lane];
}

static int motion_capture_holding(const lane_seat_t *s, int seat, float x, float z, uint64_t ms)
{
    const lane_t *lane = motion_capture_lane(s, seat, ms);
    return lane && dist2d(x, z, lane->zx, lane->zz) <= lane->zr;
}

static void motion_intent(lane_seat_t *s, int seat, vehicle_control_t *c, const u8 *ai, float arrival, uint64_t ms,
                          unsigned native_status, gm_controls_t native)
{
    motion_intent_t *in = &c->intent;
    const control_engagement_t *e = &c->engagement;
    gm_point_t goal = {s->move_target_x, s->move_target_z};
    const float heading = atan2f(s->face_z - s->z, s->face_x - s->x);
    const int native_done = native_status == 1 && s->nav.applied_source == NAV_SOURCE_COUNT;
    const int terminal = s->nav.applied_source != NAV_ROUTE || s->wpi + 1 >= s->wpn;
    const int held_capture = terminal && s->nav.applied_source == NAV_LANE && e->route_hold
        && motion_capture_holding(s, seat, goal.x, goal.z, ms);
    const int capture_hold = held_capture && !e->stop && !e->align;
    const int matching_capture = in->capture && in->task == s->nav.task_id && in->revision == s->nav.revision
        && in->route_generation == s->route_generation;
    if (held_capture && matching_capture)
        goal = in->goal;
    const int capture = terminal && (s->nav.applied_source == NAV_ROUTE || s->nav.applied_source == NAV_LANE)
        && motion_capture_holding(s, seat, goal.x, goal.z, ms);
    const int move = capture_hold || (s->wants_move && !e->stop && !e->align && !e->route_hold && !native_done);
    if (capture_hold && matching_capture && in->move) {
        /* A lane hold records arrival for tactics. Its native target can be
         * the old capture center, while the same central slot may still yield. */
        in->ms = ms;
        return;
    }
    if (!in->generation || in->task != s->nav.task_id || in->revision != s->nav.revision
        || in->route_generation != s->route_generation || in->waypoint != s->wpi
        || in->source != (int)s->nav.applied_source || in->move != move || in->align != e->align || in->capture != capture
        || (e->align && fabsf(remainderf(heading - in->align_heading, 6.2831853f)) > 0.20f)
        || dist2d(goal.x, goal.z, in->goal.x, in->goal.z) > 1.0f) {
        const uint64_t generation = in->generation + 1;
        /* s37 G11: the goal is the order's own when nav_commit took it from the commander's drive (NAV_LANE / NAV_ROUTE:
         * lane_drive, cmd_route, the post park), not from the approach's cover stop (also NAV_LANE) or the head bias;
         * NAV_REACTION / NAV_FIGHT / NAV_RECOVERY goals are not the order's. Decided with the goal it describes. */
        const int src = (int)s->nav.applied_source;
        const int order_goal = (src == NAV_LANE || src == NAV_ROUTE) && !s->bias_on && s->ap_state != AP_TO_COVER
                               && s->ap_state != AP_OBSERVE;
        *in = (motion_intent_t){.generation = generation, .task = s->nav.task_id, .revision = s->nav.revision,
            .route_generation = s->route_generation, .waypoint = s->wpi, .source = src,
            .move = move, .align = e->align, .capture = capture, .align_heading = heading,
            .origin = {s->x, s->z}, .goal = goal, .order_goal = order_goal};
    }
    in->ms = ms;
    in->arrival_radius = isfinite(arrival) ? fmaxf(GM_PRODUCTION_MIN_ARRIVAL_M, fminf(arrival, GM_PRODUCTION_MAX_ARRIVAL_M))
                                         : GM_PRODUCTION_DEFAULT_ARRIVAL_M;
    if (!move || !isfinite(in->goal.x) || !isfinite(in->goal.z))
        return;

    const gm_vehicle_plan_t *prior = motion_current_plan(seat, s->born_ms, in->generation, ms);
    if (in->native_path && in->native_retry_marker == s->nav.next_replan_ms
        && (in->native_reaches_goal || (prior && (prior->repair_active || prior->path_cursor + 2 < in->path_count))))
        return;
    if (in->native_retry_marker != s->nav.next_replan_ms)
        in->native_direct = 0;
    in->native_retry_marker = s->nav.next_replan_ms;

    const float *path = gh_ai_path(ai);
    const unsigned count = gh_ai_path_count(ai);
    const unsigned cursor = gh_ai_path_cursor(ai);
    if (!geometry.scan) {
        /* No-map mode (Japan's first battle: no native path points arrived,
         * cause unknown): say whether the engine supplies any, on each change
         * and at most every 10 s per seat. */
        const int seen = path && count >= 2 && count <= 8192 && cursor < count ? 1 : 2;
        if (seen != c->native_seen && (!c->native_log_ms || ms - c->native_log_ms >= 10000)) {
            event(geometry.host, "motion: seat %d native path %s (%u points, cursor %u, data %s), no scan",
                  seat, seen == 1 ? "present" : "absent", count, cursor, path ? "set" : "null");
            c->native_seen = seen;
            c->native_log_ms = ms;
        }
    }
    in->native_path = 0;
    in->native_reaches_goal = 0;
    in->native_direct |= native_status == 0 && native.throttle > 128;
    in->path_count = 0;
    if (!path || count < 2 || count > 8192 || cursor >= count)
        return;
    const float *end = path + 3u * (count - 1);
    if (!isfinite(end[0]) || !isfinite(end[2])
        || dist2d(end[0], end[2], in->goal.x, in->goal.z) > fmaxf(12.0f, in->arrival_radius))
        return;

    unsigned first = cursor;
    /* Stock stores one sampled cursor, including the start point at index
     * zero. Skip only coincident points on this measured invocation. */
    while (first + 1 < count && dist2d(s->x, s->z, path[3u * first], path[3u * first + 2]) < 1.0f)
        ++first;
    for (unsigned j = first; j < count && in->path_count < GM_MAX_ROUTE_POINTS; ++j) {
        const gm_point_t pt = {path[3u * j], path[3u * j + 2]};
        if (!isfinite(pt.x) || !isfinite(pt.z)) {
            in->path_count = 0;
            return;
        }
        in->path[in->path_count++] = pt;
    }
    in->native_path = in->path_count > 0;
    in->native_reaches_goal = first + (unsigned)in->path_count == count;
    if (first && isfinite(path[3u * (first - 1)]) && isfinite(path[3u * (first - 1) + 2]))
        in->origin = (gm_point_t){path[3u * (first - 1)], path[3u * (first - 1) + 2]};
}

typedef struct {
    int count;
    gm_point_t hits[MOTION_OBSTACLE_PROBES];
} motion_probe_t;

static motion_probe_t motion_probes[GM_MAX_VEHICLES];

static void motion_probe(void *host, const gm_vehicle_t *v, motion_probe_t *probe)
{
    probe->count = 0;
    if ((!v->move && !v->align) || !v->controlled)
        return;
    const float fx = cosf(v->heading), fz = sinf(v->heading);
    const float speed = hypotf(v->vx, v->vz);
    const float length = fminf(52.0f, v->length + fmaxf(12.0f, speed * speed / (2.0f * v->braking)));
    const float dx = v->path_count ? v->path[0].x - v->x : v->goal.x - v->x;
    const float dz = v->path_count ? v->path[0].z - v->z : v->goal.z - v->z;
    const float distance = hypotf(dx, dz);
    const int use_path = speed < 2.0f && distance > 1.0f && v->move;
    const float ax = use_path ? dx / distance : fx, az = use_path ? dz / distance : fz;
    const float from = 0.5f * v->length + v->hull_forward;
    for (int lane = -1; lane <= 1; ++lane) {
        const float side = (float)lane * 0.45f * v->width;
        const float p[3] = {v->x + from * fx - side * fz, v->y + 0.75f, v->z + from * fz + side * fx};
        const float direction[3] = {ax, 0.0f, az};
        float t = length;
        int kind;
        if (trace_with_flags(host, p, direction, &t, &kind, GH_RAY_STATIC_AND_OBJECTS) == 1 && isfinite(t) && t >= 0.0f && t <= length)
            probe->hits[probe->count++] = (gm_point_t){p[0] + t * ax, p[2] + t * az};
    }
}

static traffic_body_t motion_footprint(const gm_vehicle_t *v, float x, float z, float heading)
{
    const float fx = cosf(heading), fz = sinf(heading);
    return (traffic_body_t){
        .x = x + fx * v->hull_forward - fz * v->hull_right, .y = v->y,
        .z = z + fz * v->hull_forward + fx * v->hull_right,
        .fx = fx, .fz = fz, .length = 0.5f * v->length, .width = 0.5f * v->width
    };
}

static enum gm_geometry motion_geometry(void *context, const gm_vehicle_t *v,
                                       const gm_pose_t *from, const gm_pose_t *to)
{
    const motion_probe_t *probes = context;
    const int index = v->id < MAX_SEATS ? motion_index[v->id] : -1;
    const motion_probe_t *probe = index >= 0 ? &probes[index] : NULL;
    enum gm_geometry result = GM_CLEAR;
    const float turn = remainderf(to->heading - from->heading, 6.2831853f);
    const float distance = hypotf(to->x - from->x, to->z - from->z);
    if (!isfinite(turn) || !isfinite(distance))
        return GM_UNKNOWN;
    const int steps = (int)fmaxf(1.0f, ceilf(fmaxf(distance / MOTION_SWEEP_DISTANCE, fabsf(turn) / MOTION_SWEEP_ANGLE)));
    for (int k = 0; k <= steps; ++k) {
        const float f = (float)k / (float)steps;
        const traffic_body_t pose = motion_footprint(v, from->x + f * (to->x - from->x),
            from->z + f * (to->z - from->z), from->heading + f * turn);
        for (int j = 0; probe && j < probe->count; ++j)
            if (geometry_box_hits(&pose, probe->hits[j].x, probe->hits[j].z, MOTION_FOOTPRINT_MARGIN))
                return GM_BLOCKED;
        const enum nav_geometry evidence = geometry_pose_check(pose, 0).state;
        if (evidence == NAV_BLOCKED)
            return GM_BLOCKED;
        if (evidence != NAV_CLEAR)
            result = GM_UNKNOWN;
    }
    return result;
}

static enum gm_geometry motion_static_probe(void *context, const gm_vehicle_t *v,
                                           const gm_pose_t *from, const gm_pose_t *to)
{
    const enum gm_geometry mapped = motion_geometry(context, v, from, to);
    if (mapped != GM_UNKNOWN || dist2d(v->x, v->z, to->x, to->z) > GEO_LIVE_REACH_M)
        return mapped;
    const traffic_body_t pose = motion_footprint(v, to->x, to->z, to->heading);
    const enum nav_geometry native = geometry_stationary_probe(&pose);
    return native == NAV_BLOCKED ? GM_BLOCKED : native == NAV_CLEAR ? GM_CLEAR : GM_UNKNOWN;
}

/* Engine path points cut for the route's waypoints: the intent generation last logged, per seat. */
static uint64_t motion_window_logged[MAX_SEATS];

static void motion_add_route(gm_vehicle_t *v, const lane_seat_t *s, const motion_intent_t *in, int seat)
{
    gm_point_t *path = motion_paths[seat];
    int *strategic = motion_tactical_points[seat];
    for (int i = 0; i < GM_MAX_ROUTE_POINTS; ++i)
        strategic[i] = -1;
    v->route_origin = in->origin;
    v->goal = in->goal;
    v->path = path;
    /* Give the shared planner real downstream portals to rejoin around a
     * blocked intermediate point. Only the stock prefix has native evidence.
     * The stock path to the current waypoint must end at it. */
    const int rest = s->wpn - 1 - s->wpi;  /* route waypoints after the current one */
    const int route = in->source == NAV_ROUTE && nav_route_active(s) && v->move && rest >= 0
        && (!in->path_count || dist2d(in->path[in->path_count - 1].x, in->path[in->path_count - 1].z,
                                      in->goal.x, in->goal.z) <= fmaxf(12.0f, in->arrival_radius));
    /* The route's own waypoints take precedence over the stock path in the
     * window: the planner's goal is the route's last waypoint whenever the
     * route fits. Poland 01:40 seat 15: 61 stock points to waypoint 8 left room
     * for waypoints 9-11 of 28; the planner's goal was waypoint 11, it arrived
     * there and the drive ended (motion_apply_rejoin retired arrival only at
     * the last waypoint). When the stock path does not leave room, only its
     * leading points (the stretch nearest the hull) are kept and the current
     * waypoint itself follows them, so that the window still passes through
     * it. MAX_TWP == GM_MAX_ROUTE_POINTS, so the rest of a route always fits;
     * a goal short of the route's end is still handed over (motion_apply_rejoin). */
    int keep = in->path_count;
    if (route && keep + rest > GM_MAX_ROUTE_POINTS)
        keep = GM_MAX_ROUTE_POINTS - 1 - rest > 0 ? GM_MAX_ROUTE_POINTS - 1 - rest : 0;
    v->path_count = keep;
    if (keep) {
        memcpy(path, in->path, (size_t)keep * sizeof *path);
        v->goal = path[keep - 1];
    }
    if (v->move && keep < (in->path_count ? in->path_count : 1)) {
        path[v->path_count++] = in->goal;   /* no stock path, or its end was cut: the waypoint itself */
        v->goal = in->goal;
    }
    if (v->native_corridor)
        v->native_corridor_points = v->path_count;
    /* s24 path mode (cfg.path): the planner's goal is the target of this move itself, not where the engine's
     * stock path to it happens to end (up to 12 m short); the window stays for the waypoint bookkeeping only
     * (the planner routes from where the tank stands to the goal, with no via points). Routes: below. */
    if (cfg.graft && (cfg.path || routes_lib_state > 0) && v->move && !route) /* s26: library mode as s24 */
        v->goal = in->goal;
    if (keep < in->path_count && motion_window_logged[seat] != in->generation) {
        motion_window_logged[seat] = in->generation;
        event(geometry.host, "motion: seat %d route %d/%u waypoint %d: stock path cut from %d to %d points so that the "
              "%d route waypoints after it fit the planner window", seat, s->nav.route_id, s->route_generation, s->wpi,
              in->path_count, keep, rest);
    }
    if (!route || !v->path_count)
        return;
    strategic[v->path_count - 1] = s->wpi;
    const troute_t *r = &troutes[s->tr];
    for (int wp = s->wpi + 1; wp < s->wpn && v->path_count < GM_MAX_ROUTE_POINTS; ++wp) {
        const gm_point_t next = {r->x[wp], r->z[wp]};
        strategic[v->path_count] = wp;
        path[v->path_count++] = next;
        v->goal = next;
    }
}

/* The planner, not the native driver, decides when an upstream waypoint is
 * passed: path_cursor counts the passed entries of the published upstream
 * window (v->path: the installed route's remaining waypoints, its last one
 * the goal) and is monotone per route_id. It is taken whether or not a nav
 * route is followed: without one the planner steers along the waypoints
 * itself, and a cursor the task layer never heard of handed it, after the
 * next interruption, waypoints it had passed 85 m before (Spaceport seat 28).
 * The last waypoint is the planner's goal, which the cursor never passes: it
 * is retired when the planner reports arrival there, so a route ends where
 * it ends instead of at the task layer's 3.5 m portal, which a hull that
 * came in at speed and turned round never met aligned (Poland seats 5, 10
 * orbited their last flank-lane waypoint for minutes). Idempotent, forwards
 * only: only entries beyond the current waypoint move it. */
static void motion_apply_rejoin(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (!nav_route_active(s))
        return;
    const motion_intent_t *in = &vehicle_control[seat].intent;
    const gm_vehicle_plan_t *plan = motion_current_plan(seat, s->born_ms, in->generation, ms);
    if (!plan || in->route_generation != s->route_generation || in->source != NAV_ROUTE || in->waypoint != s->wpi
        || in->task != s->nav.task_id || in->revision != s->nav.revision)
        return;
    int passed = s->wpi - 1;
    const gm_vehicle_t *v = &motion_vehicles[motion_index[seat]];
    if (!(in->capture && plan->state == GM_ARRIVED)) {
        for (int i = 0; i < plan->path_cursor && i < v->path_count; ++i) {
            const int wp = motion_tactical_points[seat][i];
            if (wp > passed && wp < s->wpn)
                passed = wp;
        }
    }
    /* The planner's goal is the window's last entry (motion_add_route), as a
     * rule the route's last waypoint: its arrival there (or at the standing
     * room beside it) ends the route, also when a waypoint on the way was never
     * passed close enough to count (one in a pond the route went round). When
     * the goal is a waypoint short of the end, arrival there retires every
     * waypoint up to and including it, so that the next tick hands over the
     * rest instead of the hull standing at it (Poland 01:40 seat 15: arrived at
     * waypoint 11 of 28 at 72.8 s, stood 215 s with the cursor at 1 and
     * waypoint 8 never met within the task layer's 3.5 m portal). */
    const troute_t *route = &troutes[s->tr];
    int arrived_at = -1;
    if (plan->state == GM_ARRIVED && !in->capture && in->move && v->path_count > 0) {
        const int goal_wp = motion_tactical_points[seat][v->path_count - 1];
        if (goal_wp >= 0 && goal_wp < s->wpn)
            arrived_at = goal_wp;
        else if (dist2d(v->goal.x, v->goal.z, route->x[s->wpn - 1], route->z[s->wpn - 1]) < 1.0f)
            arrived_at = s->wpn - 1;
    }
    const int by_arrival = arrived_at > passed;
    if (by_arrival)
        passed = arrived_at;
    if (passed < s->wpi)
        return;
    s->wpi = passed + 1;
    s->nav.actual_m = fmaxf(s->nav.actual_m, s->nav.arc[passed]);
    s->nav.completed_m = fmaxf(s->nav.completed_m, s->nav.arc[passed]);
    s->nav.previous_segment = passed;
    s->nav.pose_ms = 0;
    s->route_interrupted = 1;
    s->ax = s->x;
    s->az = s->z;
    s->anchor_ms = ms;
    if (!by_arrival)
        event(host, "motion: seat %d route %d/%u rejoined beyond waypoint %d", seat, s->nav.route_id, s->route_generation,
              passed);
    else if (passed < s->wpn - 1)
        event(host, "motion: seat %d route %d/%u rejoined beyond waypoint %d (arrived at a mid-route goal, %d of %d "
              "waypoints handed over)", seat, s->nav.route_id, s->route_generation, passed, s->wpn - 1 - passed, s->wpn);
    else
        event(host, "motion: seat %d route %d/%u rejoined beyond waypoint %d (arrived at the route's end)", seat,
              s->nav.route_id, s->route_generation, passed);
}

static void motion_capture_region(gm_vehicle_t *v, const lane_seat_t *s, const motion_intent_t *in, int seat, uint64_t ms)
{
    const lane_t *lane = motion_capture_lane(s, seat, ms);
    if (!lane || !in->capture || !v->move || dist2d(v->goal.x, v->goal.z, lane->zx, lane->zz) > lane->zr)
        return;
    for (int i = 0; i < nlanes; ++i) {
        if (lanes[i].kind == 0 && dist2d(lanes[i].zx, lanes[i].zz, lane->zx, lane->zz) < 40.0f) {
            v->goal_region_id = (uint64_t)i + 1;
            v->goal_region_center = (gm_point_t){lane->zx, lane->zz};
            v->goal_region_radius = lane->zr;
            break;
        }
    }
}

int motion_capture_occupied(const lane_seat_t *s, int seat, uint64_t ms)
{
    const vehicle_control_t *c = &vehicle_control[seat];
    const lane_t *lane = motion_capture_lane(s, seat, ms);
    if (!lane || c->life != s->born_ms || !c->capture_still_since_ms || !c->intent.ms
        || c->capture_still_task != s->nav.task_id || c->capture_still_revision != s->nav.revision
        || ms < c->intent.ms || ms - c->intent.ms > MOTION_EXPIRY_MS
        || ms < c->capture_still_since_ms || ms - c->capture_still_since_ms < MOTION_CAPTURE_DWELL_MS)
        return 0;
    const float *position = gh_unit_pos(s->unit);
    float vx, vz;
    motion_velocity(s->unit, s->vx, s->vz, &vx, &vz);
    return hypotf(vx, vz) < MOTION_CAPTURE_STOP_MPS
        && dist2d(position[0], position[2], lane->zx, lane->zz) < lane->zr;
}

static void motion_capture_observe(lane_seat_t *s, int seat, vehicle_control_t *c, const gm_vehicle_t *v,
                                   const gm_vehicle_plan_t *plan, const gm_pose_t *pose, float speed,
                                   gm_controls_t action, uint64_t ms)
{
    const int parked_plan = plan && v->goal_region_id && c->intent.capture
        && plan->target_speed == 0.0f && fabsf(plan->target_yaw_rate) <= 0.001f
        && action.throttle == 128 && action.steer == 128 && action.brake_left && action.brake_right;
    if (!parked_plan || speed >= MOTION_CAPTURE_STOP_MPS
        || !motion_capture_holding(s, seat, pose->x, pose->z, ms)) {
        c->capture_still_since_ms = 0;
    } else if (!c->capture_still_since_ms || c->capture_still_task != s->nav.task_id
               || c->capture_still_revision != s->nav.revision) {
        c->capture_still_since_ms = ms;
        c->capture_still_task = s->nav.task_id;
        c->capture_still_revision = s->nav.revision;
    }
}

/* The first birth of an authored seat opens the window and marks the seat
 * born. Sites are not retired by a birth: the recorded Fulda opening re-uses
 * a site 4.0-8.5 s after its first birth. */
static void motion_observe_birth(int seat, uint64_t ms)
{
    if (seat <= 0 || seat >= MAX_SEATS || !authored_spawn_seat_army[seat] || motion_seen_seat[seat])
        return;
    if (!motion_opening_ms)
        motion_opening_ms = ms;
    motion_seen_seat[seat] = 1;
}

/* Every authored site of an army stays pending while that army still has an
 * unborn authored seat inside the opening window, whether or not a hull
 * currently covers it. */
static int motion_pending_spawns(void *host, int observed_count, uint64_t ms)
{
    if (!motion_opening_ms || ms < motion_opening_ms || ms - motion_opening_ms > GM_PRODUCTION_OPENING_SPAWN_MS)
        return 0;
    const u8 *const *players = gh_players(host);
    int pending[3] = {0};
    for (int seat = 1; seat < MAX_SEATS; ++seat) {
        const int army = authored_spawn_seat_army[seat];
        if (!army || motion_seen_seat[seat])
            continue;
        const u8 *player = players ? gh_player_at(players, seat) : NULL;
        /* The configured roster is the upper bound while a native slot is
         * absent. A registered human or a used life rules out a pending bot. */
        if (player && (gh_player_slot(player) != seat || !gh_player_bot(player)
                       || gh_player_spawned(player)))
            continue;
        pending[army] = 1;
    }

    int count = 0;
    for (int i = 0; i < authored_spawn_site_count; ++i) {
        const authored_spawn_site_t *site = &authored_spawn_sites[i];
        if (site->army < 1 || site->army > 2 || !pending[site->army])
            continue;
        /* Occupied sites stay pending as well, including the occupant's own
         * birth site: the planner's spawn-departure rule drives a controlled
         * vehicle straight off a pending site it overlaps. */
        motion_spawn_sites[count++] = (gm_spawn_site_t){.p = site->p, .heading = NAN,
            .half_length = GM_PRODUCTION_SPAWN_HALF_LENGTH_M, .half_width = GM_PRODUCTION_SPAWN_HALF_WIDTH_M};
    }
    (void)observed_count;
    return count;
}

/* Hull of a tank without an owned seat. A miss measures the collision nodes
 * (nominal hull if unavailable) and evicts the oldest entry. */
static traffic_hull_t motion_other_hull(void *host, u8 *unit, int alive, uint64_t ms)
{
    int slot = 0;
    for (int i = 0; i < MOTION_OTHER_HULLS; ++i) {
        motion_other_hull_t *h = &motion_other_hulls[i];
        if (h->hull.unit == unit && h->alive == alive && ms >= h->measured_ms
            && ms - h->measured_ms < MOTION_OTHER_HULL_REFRESH_MS)
            return h->hull;
        if (h->hull.unit == unit || !h->hull.unit) {
            slot = i;
            break;
        }
        if (h->measured_ms < motion_other_hulls[slot].measured_ms)
            slot = i;
    }
    motion_other_hulls[slot] = (motion_other_hull_t){traffic_hull(host, unit, 0), ms, alive};
    return motion_other_hulls[slot].hull;
}

/* Identity of the static map content behind snapshot.map. The scan file is a
 * read-only mapping, so its cells change only when the scene changes or the
 * scan is mapped/unmapped (geometry.scan_generation). */
static uint64_t motion_map_epoch(void)
{
    return (ground_scene_epoch << 32) ^ (uint64_t)geometry.scan_generation;
}

/* s13 local planner: the lidar grid of a seat (ground_lidar_adapter.inc) and the decision log. */
static int lidar_local_block(void *context, uint64_t vehicle_id, int wc0, int wr0, int n, uint8_t *out);
/* s17: the shared overhead-probe cache (ground_overhead_adapter.inc). */
static int overhead_world_block(void *context, float cell, int wc0, int wr0, int n, uint8_t *out);

static void motion_local_log(void *context, int kind, const char *line)
{
    /* The planner stamps its lines with the snapshot clock: the logs count from t0. */
    char *rest = NULL;
    const unsigned long long stamp = strtoull(line, &rest, 10);
    const unsigned long long rel = stamp >= t0 ? stamp - t0 : stamp;
    if (kind == 1) {
        event((u8 *)context, "local: %llu%s", rel, rest ? rest : "");
        return;
    }
    if (kind == 2) {
        /* s24 path mode: "path: ..." / "avoid: ..." lines as they are (event stamps the time) */
        event((u8 *)context, "%s", rest && *rest == ' ' ? rest + 1 : rest ? rest : "");
        return;
    }
    static FILE *csv;
    static uint64_t flush_ms;
    if (!csv)
        csv = open_csv("ground_local.csv", GM_LOCAL_LOG_HEADER);
    if (!csv)
        return;
    fprintf(csv, "%llu%s\n", rel, rest ? rest : "");
    if (stamp >= flush_ms + 1000) {
        fflush(csv);
        flush_ms = stamp;
    }
}

static void motion_publish(void *host, uint64_t ms)
{
    u8 *game = gh_game(host);
    u8 **units = game ? gh_game_units(game) : NULL;
    const uint32_t count = game ? gh_game_unit_count(game) : 0;
    for (int i = 0; i < MAX_SEATS; ++i)
        motion_index[i] = -1;
    const gr_lib_t *const motion_library = routes_library(host); /* s26: NULL unless graft, routes and both files */
    if (!units || count > 8192) {
        motion_plan.count = 0;
        return;
    }
    int n = 0;
    for (uint32_t i = 0; i < count && n < GM_MAX_VEHICLES; ++i) {
        u8 *unit = gh_unit_at(units, i);
        if (!unit || !is_tank(unit))
            continue;
        const float *p = gh_unit_pos(unit);
        const float fx = gh_unit_fwd_x(unit), fz = gh_unit_fwd_z(unit);
        if (!isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]) || !(hypotf(fx, fz) > 0.25f))
            continue;
        const int seat = bot_seat(unit);
        const int alive = gh_unit_not_active(unit) == 0;
        if (alive)
            motion_observe_birth(seat, ms);
        lane_seat_t *s = seat > 0 ? &lseat[seat] : NULL;
        const int owned = alive && s && s->unit == unit && s->scene_epoch == ground_scene_epoch && !cmd_life_replaced(s, seat);
        vehicle_control_t *c = owned ? control_for(s, seat) : NULL;
        const int controlled = c && c->intent.generation && c->intent.ms
            && ms >= c->intent.ms && ms - c->intent.ms <= MOTION_EXPIRY_MS;
        if (c && !controlled)
            control_release(host, s, c, seat);
        traffic_hull_t hull;
        if (owned) {
            if (traffic_hulls[seat].unit != unit || traffic_hulls[seat].life != s->born_ms)
                traffic_hulls[seat] = traffic_hull(host, unit, s->born_ms);
            hull = traffic_hulls[seat];
        } else {
            hull = motion_other_hull(host, unit, alive, ms);
        }
        gm_vehicle_t *v = &motion_vehicles[n];
        *v = (gm_vehicle_t){.id = controlled ? (uint64_t)seat : (uint64_t)(uintptr_t)unit,
            .life_id = owned ? s->born_ms : 1, .route_id = controlled ? c->intent.generation : 1,
            .controlled = controlled, .x = p[0], .y = p[1], .z = p[2], .heading = atan2f(fz, fx),
            .length = hull.length * 2.0f, .width = hull.width * 2.0f,
            .hull_forward = hull.forward, .hull_right = hull.right,
            .cruise_speed = GM_PRODUCTION_CRUISE_MPS, .acceleration = GM_PRODUCTION_ACCEL_MPS2,
            .braking = GM_PRODUCTION_BRAKE_MPS2, .max_yaw_rate = GM_PRODUCTION_MAX_YAW_RADPS,
            .yaw_acceleration = GM_PRODUCTION_YAW_ACCEL_RADPS2, .arrival_radius = GM_PRODUCTION_DEFAULT_ARRIVAL_M};
        if (alive)
            motion_velocity(unit, owned ? s->vx : 0.0f, owned ? s->vz : 0.0f, &v->vx, &v->vz);
        if (c)
            v->yaw_rate = gm_control_yaw_observe(&c->yaw_response, v->heading, ms);
        if (controlled) {
            const motion_intent_t *in = &c->intent;
            const vcls_t *vc = active_cls(seat, unit);
            v->move = in->move;
            v->align = in->align;
            v->align_heading = in->align_heading;
            v->can_pivot = !vc->ok || !(vc->flags & VC_WHEELED);
            v->max_yaw_rate = gm_control_yaw_limit(&c->yaw_response, v->max_yaw_rate);
            v->arrival_radius = in->arrival_radius;
            v->native_corridor = in->native_path || in->native_direct;
            v->corridor_half_width = 3.5f;
            /* Measured rates re-seed every snapshot. This conservative
             * command propagation interval includes the next drive callback. */
            v->command_delay_s = GM_PRODUCTION_COMMAND_DELAY_S;
            v->command_delay_max_s = 0.001f * GM_PRODUCTION_MAX_COMMAND_DELAY_MS;
            v->pending_commands = motion_packets[seat];
            v->pending_command_count = gm_control_history_snapshot(&c->history, motion_packets[seat], ms);
            v->reverse_speed = GM_PRODUCTION_REVERSE_MPS;
            if (c->has_written) {
                const float speed = v->vx * cosf(v->heading) + v->vz * sinf(v->heading);
                const int braking = c->written.brake_left || c->written.brake_right;
                v->pending_reverse = !braking && c->written.throttle < 128;
                gm_control_pending(v, c->written, speed, &v->pending_acceleration, &v->pending_yaw_target);
            }
            motion_add_route(v, s, in, seat);
            motion_capture_region(v, s, in, seat, ms);
            /* s24 route spread; s26 library mode: the army a post target must belong to */
            v->group = cfg.graft && (cfg.path || motion_library) && (s->army == 1 || s->army == 2) ? s->army : 0;
            /* s32: the order's route variant (teamplan flank) and this life's spawn pose */
            v->route_variant = s->route_variant;
            v->route_pose = s->spawn_pose1 - 1;
            /* s35: the lane slot's library target; s37 G11: only while the goal handed over is the order's own (a
             * reaction's, a fight's or a recovery's goal point is mapped by the driving side itself, as before s35) */
            v->goal_target1 = in->order_goal ? s->goal_target1 : 0;
            motion_index[seat] = n;
        }
        motion_probe(host, v, &motion_probes[n]);
        ++n;
    }
    motion_vehicle_count = n;
    geometry_stationary_begin();
    /* Static: the route layer keeps a copy of the gm_map_t (cell callback and
     * this context) between snapshots; the epoch check in geometry_map_cell
     * still rejects a view from an earlier scene. */
    static geometry_map_view_t map;
    const gm_snapshot_t snapshot = {.ms = ms, .count = n, .vehicles = motion_vehicles,
        .static_sweep = motion_geometry, .static_probe = motion_static_probe, .map = geometry_map_snapshot(&map),
        .map_epoch = motion_map_epoch(),
        .pending_spawn_sites = motion_spawn_sites, .pending_spawn_site_count = motion_pending_spawns(host, n, ms),
        .geometry_context = motion_probes,
        .local_planner = cfg.local, .local_block = lidar_local_block, .local_log = motion_local_log,
        .local_context = host, .spawn_exit = cfg.spawn_exit,
        .world_block = cfg.overhead ? overhead_world_block : NULL, .world_context = host,
        /* s24: only with the WASD driver; s26: library mode (graft, routes on, both files loaded) */
        .path_mode = cfg.graft && (cfg.path || motion_library), .routes_lib = motion_library,
        .routes_net = motion_library ? routes_network() : NULL}; /* s38: NULL without the network: s37 */
    if (!gm_plan_snapshot(&motion_state, &snapshot, &motion_plan))
        motion_plan.count = 0;
}

static const char *motion_state_name(enum gm_state state)
{
    static const char *const names[] = {
        "observed", "parked", "arrived", "driving", "turning", "wait_traffic",
        "wait_geometry", "static_blocked", "unavoidable", "wait_spawn"
    };
    _Static_assert(sizeof names / sizeof names[0] == GM_WAIT_SPAWN + 1, "motion state names");
    return (unsigned)state < sizeof names / sizeof names[0] ? names[state] : "unknown";
}

static void motion_log(lane_seat_t *s, int seat, uint64_t ms, const gm_vehicle_plan_t *plan,
                       const char *consume, const gm_pose_t *pose, unsigned native_status,
                       gm_controls_t native, gm_controls_t final)
{
    static FILE *csv;
    static uint64_t flush_ms;
    const enum gm_state state = plan ? plan->state : GM_OBSERVED;
    if (!cfg.log_dir[0] || (s->move_log_ms && ms - s->move_log_ms < 500
        && s->move_log_reason == (int)state && s->move_log_peer == s->yield_to
        && s->move_log_gen == s->route_generation && s->move_log_wp == s->wpi))
        return;
    if (!csv)
        csv = open_csv("ground_movement.csv",
            "ms,plan_ms,seat,life,seq,route_generation,route,index,plan_recovery,owner,peer,cap,reason,attempts,target_x,target_z,"
            "native_throttle,native_steer,final_throttle,final_steer,x,z,speed,heading,anchor_displacement,"
            "native_brake_left,native_brake_right,final_brake_left,final_brake_right,plan_state,geometry,native_status,publication_ms,route_intent,"
            "plan_consume,target_speed,target_yaw,plan_goal_x,plan_goal_z,measured_speed,measured_yaw_rate,"
            "repair_active,repair_rejoined,terrain_recovery,path_cursor,route_status,route_revision,route_target_x,route_target_z");
    if (!csv)
        return;
    fprintf(csv, "%llu,%llu,%d,%llu,%llu,%u,%d,%d,0,%d,%d,%u,%d,0,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.1f,%.2f,"
                 "%u,%u,%u,%u,%s,%d,%u,%llu,%llu,%s,%.3f,%.4f,%.2f,%.2f,%.3f,%.4f,%d,%d,%d,%d,%d,%u,%.2f,%.2f\n",
            (unsigned long long)(ms - t0), (unsigned long long)(s->last_ms - t0), seat,
            (unsigned long long)(s->born_ms - t0), (unsigned long long)s->move_seq, s->route_generation, s->tr,
            s->tr >= 0 ? s->wpi : -1, state == GM_DRIVING ? 0 : 5, s->yield_to,
            final.brake_left ? 128u : 255u, state, s->move_target_x, s->move_target_z,
            (unsigned)native.throttle, (unsigned)native.steer, (unsigned)final.throttle, (unsigned)final.steer,
            s->x, s->z, hypotf(s->vx, s->vz), atan2f(s->fz, s->fx) * 57.29578f,
            dist2d(s->x, s->z, s->ax, s->az),
            (unsigned)native.brake_left, (unsigned)native.brake_right, (unsigned)final.brake_left, (unsigned)final.brake_right,
            plan ? motion_state_name(state) : "pending_publication", plan ? (int)plan->geometry : -1, native_status,
            (unsigned long long)(plan ? motion_plan.ms - t0 : 0),
            (unsigned long long)(plan ? plan->route_id : 0), consume, plan ? plan->target_speed : 0.0f,
            plan ? plan->target_yaw_rate : 0.0f, plan ? plan->goal.x : 0.0f, plan ? plan->goal.z : 0.0f,
            pose->speed, pose->yaw_rate, plan ? plan->repair_active : 0, plan ? plan->repair_rejoined : 0,
            plan ? plan->terrain_recovery : 0, plan ? plan->path_cursor : -1, plan ? plan->route_status : -1,
            plan ? plan->route_revision : 0u, plan ? plan->route_target.x : 0.0f, plan ? plan->route_target.z : 0.0f);
    s->move_log_ms = ms;
    s->move_log_reason = state;
    s->move_log_peer = s->yield_to;
    s->move_log_gen = s->route_generation;
    s->move_log_wp = s->wpi;
    if (ms - flush_ms >= 1000) {
        fflush(csv);
        flush_ms = ms;
    }
}

/* s22 WASD driver, tick side (ground_control_graft.inc). */
static gm_controls_t wasd_tick(void *host, const lane_seat_t *s, int seat, vehicle_control_t *c, u8 *phys,
                               const gm_vehicle_plan_t *plan, const gm_vehicle_t *v, uint64_t ms);

static int ground_after_at(control_frame_t *frame, void *host, uint64_t ms, unsigned native_status)
{
    const int seat = frame->seat;
    if (!seat) {
        for (int i = 1; i < MAX_SEATS; ++i)
            if (lseat[i].node == frame->node && lseat[i].unit && lseat[i].scene_epoch == ground_scene_epoch)
                return 1;
        return 0;
    }
    lane_seat_t *s = &lseat[seat];
    vehicle_control_t *c = control_for(s, seat);
    if (s->scene_epoch != ground_scene_epoch || frame->unit != s->unit || frame->node != s->node
        || frame->life != s->born_ms || frame->plan_seq != s->move_seq || bot_seat(s->unit) != seat
        || ms < s->last_ms || ms - s->last_ms > CONTROL_STAMP_MS) {
        event(host, "motion: seat %d rejected stale or mismatched drive frame", seat);
        return 1;
    }
    if (frame->plan_seq <= c->last_plan_seq) {
        event(host, "motion: seat %d rejected duplicate drive frame", seat);
        return 1;
    }
    c->last_plan_seq = frame->plan_seq;
    u8 *controls = gh_unit_controls(s->unit);
    const u8 *ai = gh_unit_ai(s->unit);
    if (!controls || !ai || gh_unit_not_active(s->unit) != 0)
        return 1;
    const unsigned status = native_status & 255u;
    if (status > 1) {
        c->intent.ms = 0;
        control_release(host, s, c, seat);
        return 1;
    }

    const gm_controls_t native = control_native(controls);
    const float arrival = frame->saved_arrival_radius ? CONTROL_PORTAL_RADIUS : gh_node_arrival(frame->node);
    if (!c->intent.ms || ms - c->intent.ms > MOTION_EXPIRY_MS)
        c->capture_still_since_ms = 0;
    motion_intent(s, seat, c, ai, arrival, ms, status, native);
    const int index = motion_index[seat];
    const gm_vehicle_plan_t *plan = NULL;
    const gm_vehicle_t *v = NULL;
    const char *consume = "awaiting_snapshot";
    if (index >= 0 && index < motion_plan.count)
        consume = ms < motion_plan.ms || ms > motion_plan.valid_until_ms ? "expired" : "identity_changed";
    plan = motion_current_plan(seat, s->born_ms, c->intent.generation, ms);
    if (plan) {
        v = &motion_vehicles[index];
        consume = "applied";
    }
    const float fx = gh_unit_fwd_x(s->unit), fz = gh_unit_fwd_z(s->unit);
    const float *pos = gh_unit_pos(s->unit);
    float vx, vz;
    motion_velocity(s->unit, s->vx, s->vz, &vx, &vz);
    const float heading = hypotf(fx, fz) > 0.25f ? atan2f(fz, fx) : NAN;
    const gm_pose_t measured = {pos[0], pos[2], heading, vx * cosf(heading) + vz * sinf(heading), c->yaw_response.rate};
    const float age = plan ? (float)(ms - motion_plan.ms) / 1000.0f : 0.0f;
    /* A frame without a matching plan is usually a matter of one publication:
     * the move order moved on to the next waypoint, or a hitch delayed the
     * plan. Braking fully for it, at speed and several times a minute, is
     * what made the tanks lurch; the last command is held instead. */
    if (plan)
        c->applied_ms = ms;
    const int held = !plan && c->has_written && c->applied_ms && ms >= c->applied_ms
        && ms - c->applied_ms <= MOTION_HOLD_MS;
    if (held && !cfg.graft)
        consume = "held";
    /* s22: with the WASD driver on (cfg.graft) the tick only computes and stores the keys; the write point is
     * phase 0 of the input-layer hook (ground_control_graft.inc). Off: the s15 tracker writes, as before. */
    gm_controls_t action;
    if (cfg.graft) {
        action = wasd_tick(host, s, seat, c, controls, plan, v, ms);
    } else {
        action = held ? c->written : gm_control_track(v, plan, &measured, age);
        control_write(controls, action);
    }
    gm_control_yaw_note(&c->yaw_response, action, ms);
    gm_control_history_note(&c->history, action, ms);
    c->written = action;
    c->written_ms = ms;
    c->owned_controls = controls;
    c->has_written = 1;

    motion_capture_observe(s, seat, c, v, plan, &measured, hypotf(vx, vz), action, ms);
    const int occupying = motion_capture_occupied(s, seat, ms);
    const enum gm_state state = plan ? plan->state : GM_OBSERVED;
    const int blocked = !occupying && (state == GM_STATIC_BLOCKED || (state == GM_UNAVOIDABLE && plan->geometry == GM_BLOCKED));
    const int waiting = !occupying && !blocked
        && (!plan || state == GM_WAIT_TRAFFIC || state == GM_WAIT_GEOMETRY || state == GM_UNAVOIDABLE
            || state == GM_WAIT_SPAWN);
    s->motion_state = state;
    if (plan && plan->street_closed)
        s->street_closed_ms = ms;       /* s34: no other street (the command layer may give another order) */
    nav_control_feedback(s, ms, waiting, blocked, 0);
    s->yield_to = plan && plan->blocker_id > 0 && plan->blocker_id < MAX_SEATS ? (int)plan->blocker_id : -1;
    control_engagement_t *e = &c->engagement;
    if (plan && !blocked && !waiting && (e->stop || e->align)) {
        if (!e->started) {
            e->started = 1;
            e->progress_x = s->x;
            e->progress_z = s->z;
        }
        e->committed_ms = ms;
    }
    const int pivoting = e->align && state == GM_TURNING;
    if (pivoting && !s->pivoting)
        s->pivot_start = ms;
    s->pivoting = pivoting;
    if (e->started && state == GM_DRIVING && action.throttle > 128) {
        e->follow_ms = ms;
        e->follow_task = s->nav.task_id;
        e->follow_x = s->x;
        e->follow_z = s->z;
        e->goal_x = s->move_target_x;
        e->goal_z = s->move_target_z;
    }
    motion_log(s, seat, ms, plan, consume, &measured, status, native, action);

    /* A managed waypoint advances through measured nav progress. The stock
     * done result can occur before the native driver even writes the axes. */
    return status == 1 && ((s->wants_move && s->nav.applied_source != NAV_SOURCE_COUNT)
                          || c->intent.capture) ? 2 : 1;
}

int ground_control_after(u8 *node, void *host, control_frame_t *frame, unsigned native_status)
{
    if (active_control_frame != frame || frame->node != node)
        return 0;
    active_control_frame = frame->previous;
    gh_lock(GH_LOCK_MAIN);
    const uint64_t ms = gh_ms();
    geometry_context(host, ms);
    const int owned = ground_after_at(frame, host, ms, native_status);
    if (frame->saved_arrival_radius) {
        const lane_seat_t *s = &lseat[frame->seat];
        if (s->scene_epoch == ground_scene_epoch && s->born_ms == frame->life && s->unit == frame->unit
            && s->node == node && gh_node_arrival(node) == CONTROL_PORTAL_RADIUS)
            gh_node_set_arrival(node, frame->saved_arrival_radius);
    }
    gh_unlock(GH_LOCK_MAIN);
    return owned;
}

/* The behavior-tree phase is also present in branches without the game's own move order.
 * Read every current tank or wreck at one observation time. A newborn is
 * immediately an observed obstacle and joins as controlled after it reports
 * the first native intent. */
static void lidar_observe(void *host, uint64_t ms);
static void overhead_observe(void *host, uint64_t ms);

void ground_observe(void *host)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || (!cfg.lanes && !cfg.native_control) || !host)
        return;
    gh_lock(GH_LOCK_MAIN);
    const uint64_t ms = gh_ms();
    geometry_context(host, ms);
    if (ms >= next_motion_ms) {
        next_motion_ms = ms + GM_PRODUCTION_PUBLISH_MS;
        scan_units(host, ms);
        for (int seat = 1; seat < MAX_SEATS; ++seat) {
            lane_seat_t *s = &lseat[seat];
            if (s->unit && s->scene_epoch == ground_scene_epoch && slot_alive[seat] && slot_unit[seat] == s->unit
                && !cmd_life_replaced(s, seat) && gh_unit_not_active(s->unit) == 0) {
                ground_observe_pose(s, s->unit, ms);
                cmd_unit_observe(host, s, seat, ms);
            }
        }
        if (cfg.lanes) {
            strat_tick(host, 1, ms);
            strat_tick(host, 2, ms);
        }
        motion_publish(host, ms);
    }
    lidar_observe(host, ms);   /* s11: perception only, nothing reads it (ground_lidar_adapter.inc) */
    overhead_observe(host, ms); /* s17: overhead probes into the shared cache (ground_overhead_adapter.inc) */
    gh_unlock(GH_LOCK_MAIN);
}
