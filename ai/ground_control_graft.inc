/* ---- s22 2026-11-17: one driver, one write point (single-vehicle-control handoff, items 1, 2, 5, A) ----
 *
 * The game's input layer runs on every unit update, inside the game's
 * unit update. It reads only the throttle and steer bytes of the vehicle physics record (gh_unit_controls) and
 * writes back throttle, steer and both brakes; the brakes we would write are never read. It reads its own last
 * output, so the input must be rewritten on every update.
 *
 * With cfg.graft (WT_GROUND_GRAFT / graft=, default 1) the launcher's input-layer hook
 * calls wt_ground_controls(unit, host, 0) right before the layer and (unit, host, 1) right after it:
 *  phase 0  THE write point. For a seat we control (same unit, alive, same life, same control block, keys owned):
 *           the keys the behaviour-tree tick computed (wasd_tick, the WASD driver of ground_drive.inc) when they
 *           are at most WASD_KEYS_MS old, else "no keys" (128, 128). Only throttle and steer: never the brakes,
 *           never the two hand-brake bytes. Players, bots we do not control, released seats, other lives: untouched.
 *  phase 1  read only: the layer's output, for the probe.
 * The tick path (ground_after_at) no longer writes game bytes: it stores the keys per seat.
 * graft=0: the launcher installs no hook, this file returns at once, and the tick path writes the s15 tracker's
 * four bytes exactly as before (rollback switch).
 *
 * Probe: wasd_drive.csv, one row per unit update for the first WASD_TRACE_SEATS seats we control (every physics
 * tick), 4 Hz for the others: the six control bytes as found in phase 0 before our write, our keys, the layer's
 * four output bytes, the vehicle physics record's six applied control bytes (what the last physics step used) and
 * the physics tick, gear, asked gear, the gear's ratio, signed speed along the hull, omega y, acceleration along
 * the hull, position, heading, the automatic-gear flag and the three input-layer branch flags, and the driver's state;
 * s24: the stop distance published (stop_m, nan none), the aim point (aim_x / aim_z, nan for a heading aim) and
 * the nav revision followed (route_rev), whatever the switch (path mode or s22 feeding).
 * ground_plan.log every 10 s ("wasd:"): the share of physics ticks on which the applied controls (throttle, steer, both
 * brakes) equals the layer's output for our keys. Physics parameters once per life ("graft: seat .. phys .."
 * as in s20/s21, and "wasd: seat .. can pivot .."). A fault turns the write point off for the process. */
#define WASD_KEYS_MS 250u
#define GRAFT_CSV_MS 250u
#define GRAFT_SUMMARY_MS 10000u
#define WASD_TRACE_SEATS 3
#define WASD_NO_HOOK_WARN_MS 5000u

typedef struct {
    uint64_t life, csv_ms;
    int params_logged;
    int armed;                     /* phase 0 ran this update: phase 1 reads */
    int wrote;                     /* 1 our keys, 2 stale: "no keys" */
    uint32_t tick;                 /* physics tick read in phase 0 (with the applied control bytes) */
    u8 pre[6], applied[6], keys[2];
    int last_valid;                /* the layer's output of the last update, for the per-tick comparison */
    uint32_t last_tick;
    u8 last_layer[4];
} graft_seat_t;

static graft_seat_t graft_seat[MAX_SEATS];
static FILE *graft_csv;
static int graft_dead;
static int graft_trace[WASD_TRACE_SEATS];
static uint64_t graft_summary_ms, graft_csv_flush_ms, graft_phase0_ms, graft_first_keys_ms;
static int graft_no_hook_warned;
static unsigned graft_updates, graft_ticks, graft_ticks_match, graft_stale_writes;

static const float *graft_gear_ratios(const u8 *phys, unsigned *count)
{
    return gh_vphys_gear_ratios(phys, count);
}

/* Ratio of the gear in use (negative: a reverse gear); 0 when unknown. */
static float graft_gear_ratio(const u8 *phys)
{
    unsigned n;
    const float *g = graft_gear_ratios(phys, &n);
    const unsigned gear = gh_vphys_gear(phys);
    return g && gear < n && isfinite(g[gear]) ? g[gear] : 0.0f;
}

static void graft_log_params(void *host, int seat, uint64_t life, const u8 *phys)
{
    unsigned n;
    const float *g = graft_gear_ratios(phys, &n);
    char gears[256];
    int len = 0;
    gears[0] = 0;
    for (unsigned k = 0; g && k < n && len < (int)sizeof gears - 16; ++k)
        len += snprintf(gears + len, sizeof gears - (size_t)len, "%s%.2f", k ? " " : "", (double)g[k]);
    const float mass = gh_vphys_mass(phys), brake = gh_vphys_max_brake(phys);
    event(host, "graft: seat %d life %llu phys mass %.0f kg, max brake %.0f N (%.2f m/s2), wheel r %.3f m, "
          "gears %u [%s], neutral %d, rear gear below %.2f m/s, min shift %.2f s, tracked %d, steer type %d, "
          "timestep %.4f s", seat, (unsigned long long)life, (double)mass, (double)brake,
          mass > 0.0f ? (double)(brake / mass) : 0.0, (double)gh_vphys_wheel_radius(phys), n, gears,
          gh_vphys_neutral_gear(phys), (double)gh_vphys_rear_engage_speed(phys), (double)gh_vphys_min_shift_time(phys),
          (int)gh_vphys_tracked(phys), gh_vphys_steer_type(phys), (double)gh_vphys_timestep(phys));
}

/* ---- The tick side: the driver's inputs (stage A feeding) and keys. ---- */

/* Stage A: the planner is unchanged and publishes gm_vehicle_plan_t; the driver's inputs come from it.
 *  - speed cap = |target_speed|; 0 = stop and hold; negative = reverse leg. A reverse plan the s15 tracker
 *    would not have driven (not a recovery leg, geometry not clear, a blocker, faster than the reverse speed)
 *    is a hold, as it was.
 *  - aim: an align on the spot (GM_TURNING with align and no move): the heading the planner turns to
 *    (gm_vehicle_t.align_heading, the very value its target_yaw_rate is computed from);
 *    a local-planner drive (local_drive): the heading the chosen candidate reaches 1 s ahead (gm_commitment_t
 *    cmd_heading = heading_1s of the chosen trajectory, stored by gm_plan_snapshot next to the plan; only read);
 *    anything else: route_target (the nav vertex steered to, or the goal without a route).
 *  - no stop distance yet (stage B): NAN.
 *  - s22c lift-off speed `cruise` from wanted_speed (see below). */
static gd_input_t wasd_input(const gm_vehicle_plan_t *plan, const gm_vehicle_t *v)
{
    gd_input_t in = {.ms = motion_plan.ms, .aim = GD_AIM_POINT, .x = plan->route_target.x, .z = plan->route_target.z,
                     .v_cap = fabsf(plan->target_speed), .reverse = plan->target_speed < 0.0f, .stop_m = NAN,
                     /* s22b: the s15 tracker's at_once (gm_control_track: plan->blocker_id || plan->firm) */
                     .firm = plan->blocker_id || plan->firm};
    if (!isfinite(in.v_cap))
        in.v_cap = 0.0f;
    if (in.reverse && (!plan->terrain_recovery || plan->geometry != GM_CLEAR || plan->blocker_id || !v
                       || !(v->reverse_speed > 0.0f) || in.v_cap > v->reverse_speed)) {
        in.reverse = 0;
        in.v_cap = 0.0f;
    }
    /* s22c: the speed to lift off at. target_speed is the chosen candidate's speed (a fraction of v_des, held
     * down on curves by the old yaw-rate cap); v_des (wanted_speed) tops out at the nominal cruise_speed unless
     * the planner lowered it on purpose. Not lowered: no lift-off speed, hold W. Reverse legs, or not
     * published (0): |target_speed| (s22b). Firm keeps v_cap = |target_speed| as the hard limit. */
    const float wanted = plan->wanted_speed;
    in.cruise = in.v_cap;
    if (!in.reverse && isfinite(wanted) && wanted > 0.0f)
        in.cruise = v && wanted >= v->cruise_speed - 0.05f ? INFINITY : wanted;
    /* s29: an align planned by gl_align (library mode) comes as a path-mode input below (turn-room rule, gear) */
    if (plan->state == GM_TURNING && v && v->align && !v->move && isfinite(v->align_heading) && !plan->lib_align) {
        in.aim = GD_AIM_HEADING;
        in.heading = v->align_heading;
        in.align = 1;
        in.v_cap = 0.0f;
        in.reverse = 0;
        return in;
    }
    if (plan->path) {
        /* s24 path mode (cfg.path): the plan is the driver's whole input (gm_path_decide): go or hold, the aim
         * point, the stop distance from the published pose, a reverse leg (no s15 reverse guard: the planner
         * reverses only on nav's escape legs and its own 3 m back-up), the speed cap of a tight nav leg. */
        in.path = 1;
        in.go = plan->go;
        in.aim = GD_AIM_POINT;
        in.x = plan->route_target.x;
        in.z = plan->route_target.z;
        in.reverse = plan->go && plan->path_reverse;
        /* s26: a designated route's start turn in reverse gear: the driver's existing reverse-gear pivot row */
        in.pivot_reverse = plan->go && !plan->path_reverse && plan->pivot_reverse;
        in.stop_m = plan->stop_distance >= 0.0f && isfinite(plan->stop_distance) ? plan->stop_distance : NAN;
        in.from_x = plan->blocked_at.x;
        in.from_z = plan->blocked_at.z;
        in.cruise = plan->speed_cap > 0.0f ? plan->speed_cap : INFINITY;
        in.v_cap = plan->go ? fabsf(plan->target_speed) : 0.0f;
        in.firm = 0;
        in.route_rev = plan->route_revision;
        return in;
    }
    const int index = (int)(plan - motion_plan.vehicles);
    if (plan->local_drive && index >= 0 && index < motion_state.count) {
        const gm_commitment_t *c = &motion_state.commitments[index];
        if (c->id == plan->id && c->life_id == plan->life_id && c->cmd_valid && isfinite(c->cmd_heading)) {
            in.aim = GD_AIM_HEADING;
            in.heading = c->cmd_heading;
        }
    }
    return in;
}

/* s49: the guarded block of wasd_read, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { const u8 *unit, *phys; gd_vehicle_t *out; int ret; } wasd_read_ctx_t;
static void wasd_read_run(void *ctx)
{
    wasd_read_ctx_t *const context = ctx;
    const u8 *const unit = context->unit;
    const u8 *const phys = context->phys;
    gd_vehicle_t *const out = context->out;
    {
        const float fx = gh_unit_fwd_x(unit), fz = gh_unit_fwd_z(unit);
        const float *p = gh_unit_pos(unit);
        const double *vel = gh_vphys_velocity(phys);
        const float h = atan2f(fz, fx);
        out->x = p[0];
        out->z = p[2];
        out->heading = h;
        out->speed = (float)(vel[0] * cos(h) + vel[2] * sin(h));
        out->reverse_gear = graft_gear_ratio(phys) < 0.0f;
        context->ret = hypotf(fx, fz) > 0.25f && isfinite(out->speed);
    }
}

/* What the game object says now. 0 on a fault (no keys then). */
static int wasd_read(const u8 *unit, const u8 *phys, gd_vehicle_t *out)
{
    wasd_read_ctx_t context = {unit, phys, out, 0};
    if (gh_guard(wasd_read_run, &context)) {
        return 0;
    }
    return context.ret;
}

static void wasd_params(void *host, const lane_seat_t *s, int seat, vehicle_control_t *c, const u8 *phys)
{
    int tracked = 0, separate = 0, boost = 0;
    float mass = 0.0f, brake = 0.0f;
    if (gh_vphys_drive_params(phys, &tracked, &separate, &boost, &mass, &brake)) {
        tracked = separate = boost = 0;
    }
    c->drive_phys = 1;
    c->drive_separate = separate != 0;
    c->drive_boost = boost != 0;
    c->drive_can_pivot = tracked && (separate || boost);
    c->drive_brake = mass > 0.0f && isfinite(brake / mass) ? brake / mass : 0.0f;
    event(host, "wasd: seat %d life %llu can pivot on A/D %d (tracked %d, separate brakes %d, steer boost %d), "
          "brake %.2f m/s2, a_use %.2f m/s2 (0.7 a; slope term and stop distance unused in stage A)",
          seat, (unsigned long long)s->born_ms, c->drive_can_pivot, tracked, separate, boost, (double)c->drive_brake,
          (double)(GD_BRAKE_SHARE * c->drive_brake));
}

/* The behaviour-tree tick (ground_after_at) with the driver on: compute and store the keys; write nothing.
 * Returns the bytes the layer will make of them (gd_layer), for the planner's bookkeeping (pending commands,
 * capture parking, logs); an idle input (no keys) is booked as the s15 "stop" packet. */
static gm_controls_t wasd_tick(void *host, const lane_seat_t *s, int seat, vehicle_control_t *c, u8 *phys,
                               const gm_vehicle_plan_t *plan, const gm_vehicle_t *v, uint64_t ms)
{
    if (!c->drive_phys)
        wasd_params(host, s, seat, c, phys);
    if (plan) {
        c->drive_in = wasd_input(plan, v);
        /* s26 probe columns (read only) */
        c->lib_source = plan->lib_source;
        c->lib_route = plan->lib_route;
        c->lib_sample = plan->lib_sample;
        c->lib_cross = plan->lib_cross;
        c->lib_clear = plan->lib_clear;
        c->stop_vehicle = plan->stop_vehicle;
        c->stop_turn = plan->stop_turn;
        c->stop_end = plan->stop_end;
        c->lib_blocker = plan->go ? plan->stop_blocker : 0;
    }
    gd_vehicle_t veh = {.can_pivot = c->drive_can_pivot, .brake_mps2 = c->drive_brake};
    gd_keys_t keys = {GD_KEY_NONE, GD_KEY_NONE};
    if (wasd_read(s->unit, phys, &veh))
        keys = gd_drive(&c->drive, &c->drive_in, &veh, ms, &c->drive_info);
    else
        c->drive_info = (gd_info_t){.mode = GD_HOLD, .stale = 1};
    c->keys = keys;
    c->keys_ms = ms;
    if (!graft_first_keys_ms)
        graft_first_keys_ms = ms;
    if (!graft_phase0_ms && !graft_no_hook_warned && ms - graft_first_keys_ms > WASD_NO_HOOK_WARN_MS) {
        graft_no_hook_warned = 1;
        event(host, "wasd: driver on but the input-layer hook has not run in %u s: no keys reach the game "
              "(launcher without the input-layer hook?)", WASD_NO_HOOK_WARN_MS / 1000u);
    }
    const gd_layer_t out = gd_layer(keys.throttle, keys.steer, veh.reverse_gear, c->drive_boost, c->drive_separate);
    return out.idle ? (gm_controls_t){128, 128, 255, 255}
                    : (gm_controls_t){out.throttle, out.steer, out.brake_left, out.brake_right};
}

/* ---- The write point (phase 0) and the probe (phase 1). ---- */

/* The seat that controls this unit's control block now, or 0. */
static int graft_owner(u8 *unit, vehicle_control_t **out_c, graft_seat_t **out_g)
{
    const int seat = bot_seat(unit);
    if (seat <= 0 || seat >= MAX_SEATS)
        return 0;
    lane_seat_t *s = &lseat[seat];
    if (s->unit != unit || gh_unit_not_active(unit) != 0)
        return 0;
    vehicle_control_t *c = &vehicle_control[seat];
    u8 *controls = gh_unit_controls(unit);
    if (!controls || c->life != s->born_ms || !c->has_written || controls != c->owned_controls)
        return 0;
    graft_seat_t *g = &graft_seat[seat];
    if (g->life != s->born_ms)
        *g = (graft_seat_t){.life = s->born_ms};
    *out_c = c;
    *out_g = g;
    return seat;
}

static int graft_traced(int seat)
{
    for (int k = 0; k < WASD_TRACE_SEATS; ++k) {
        if (graft_trace[k] == seat)
            return 1;
        if (!graft_trace[k]) {
            graft_trace[k] = seat;
            return 1;
        }
    }
    return 0;
}

static void graft_before(u8 *unit, void *host, uint64_t ms)
{
    graft_phase0_ms = ms;
    vehicle_control_t *c;
    graft_seat_t *g;
    const int seat = graft_owner(unit, &c, &g);
    if (!seat)
        return;
    u8 *phys = gh_unit_controls(unit);
    if (!g->params_logged) {
        g->params_logged = 1;
        graft_log_params(host, seat, g->life, phys);
    }
    gh_drive_read(g->pre, phys, sizeof g->pre);
    gh_vphys_applied(g->applied, phys, sizeof g->applied);
    g->tick = gh_vphys_tick(phys);
    if (g->last_valid && g->tick != g->last_tick) {
        ++graft_ticks;
        graft_ticks_match += !memcmp(g->applied, g->last_layer, 4);
    }
    const int fresh = c->keys_ms && ms >= c->keys_ms && ms - c->keys_ms <= WASD_KEYS_MS;
    g->keys[0] = fresh ? c->keys.throttle : GD_KEY_NONE;
    g->keys[1] = fresh ? c->keys.steer : GD_KEY_NONE;
    g->wrote = fresh ? 1 : 2;
    graft_stale_writes += !fresh;
    gh_drive_set_throttle(phys, g->keys[0]);
    gh_drive_set_steer(phys, g->keys[1]);
    g->armed = 1;
}

static void graft_after(u8 *unit, void *host, uint64_t ms)
{
    vehicle_control_t *c;
    graft_seat_t *g;
    const int seat = graft_owner(unit, &c, &g);
    if (!seat || !g->armed)
        return;
    g->armed = 0;
    const u8 *phys = gh_unit_controls(unit);
    gh_drive_read(g->last_layer, phys, sizeof g->last_layer);
    g->last_tick = gh_vphys_tick(phys);
    g->last_valid = 1;
    ++graft_updates;
    const int trace = graft_traced(seat);
    if (cfg.log_dir[0] && (trace || !g->csv_ms || ms - g->csv_ms >= GRAFT_CSV_MS)) {
        g->csv_ms = ms;
        if (!graft_csv)
            graft_csv = open_csv("wasd_drive.csv",
                "ms,seat,life,trace,tick,tick1,mode,reverse,stale,e_deg,v_cap,v_allow,firm,cruise,"
                "pre_throttle,pre_steer,pre_brake_left,pre_brake_right,pre_hand_brake_a,pre_hand_brake_b,key_throttle,key_steer,wrote,"
                "layer_throttle,layer_steer,layer_brake_left,layer_brake_right,"
                "applied_throttle,applied_steer,applied_brake_left,applied_brake_right,applied_hand_brake_a,applied_hand_brake_b,"
                "gear,gear_req,gear_ratio,speed,omega_y,accel,x,z,heading,auto_gear,input_flag_a,input_flag_b,input_flag_c,"
                "stop_m,aim_x,aim_z,route_rev,"
                "path_source,lib_route,lib_sample,cross_m,clear_m,stop_vehicle,stop_turn,stop_end,blocker_seat");
        if (graft_csv) {
            const double *vel = gh_vphys_velocity(phys), *acc = gh_vphys_accel(phys);
            const double *omega = gh_vphys_omega(phys);
            const float fx = gh_unit_fwd_x(unit), fz = gh_unit_fwd_z(unit);
            const float h = atan2f(fz, fx);
            const float *p = gh_unit_pos(unit);
            const u8 *L = g->last_layer, *P = g->pre, *A = g->applied;
            char cruise[24] = "inf";   /* s22c: lift-off speed; "inf" = none (Python float() reads it) */
            if (isfinite(c->drive_in.cruise))
                snprintf(cruise, sizeof cruise, "%.2f", (double)c->drive_in.cruise);
            /* s24: the stop distance published (nan: none), the aim point (nan: a heading aim), the nav revision */
            char stop_m[24] = "nan", aim_x[24] = "nan", aim_z[24] = "nan";
            if (isfinite(c->drive_in.stop_m))
                snprintf(stop_m, sizeof stop_m, "%.2f", (double)c->drive_in.stop_m);
            if (c->drive_in.aim == GD_AIM_POINT && isfinite(c->drive_in.x) && isfinite(c->drive_in.z)) {
                snprintf(aim_x, sizeof aim_x, "%.2f", (double)c->drive_in.x);
                snprintf(aim_z, sizeof aim_z, "%.2f", (double)c->drive_in.z);
            }
            /* s26: the curve's source, library route / sample, cross-track error and stored clearance at the
             * projection, the stop-distance components (nan none), the seat whose hull limits it (0 none or not a
             * seat of ours) */
            static const char *const src_name[GM_LIB_SRC_COUNT] = {"none", "designated", "attach", "grid", "nav",
                                                                   "detour", "slot", "finish", "net"};
            char lib_cols[200];
            {
                char cr[16] = "nan", cl[16] = "nan", sv[16] = "nan", st[16] = "nan", se[16] = "nan";
                if (isfinite(c->lib_cross))
                    snprintf(cr, sizeof cr, "%.2f", (double)c->lib_cross);
                if (isfinite(c->lib_clear))
                    snprintf(cl, sizeof cl, "%.1f", (double)c->lib_clear);
                if (isfinite(c->stop_vehicle))
                    snprintf(sv, sizeof sv, "%.2f", (double)c->stop_vehicle);
                if (isfinite(c->stop_turn))
                    snprintf(st, sizeof st, "%.2f", (double)c->stop_turn);
                if (isfinite(c->stop_end))
                    snprintf(se, sizeof se, "%.2f", (double)c->stop_end);
                const int src = c->lib_source >= 0 && c->lib_source < GM_LIB_SRC_COUNT ? c->lib_source : 0;
                snprintf(lib_cols, sizeof lib_cols, "%s,%d,%d,%s,%s,%s,%s,%s,%llu", src_name[src], c->lib_route,
                         c->lib_sample, cr, cl, sv, st, se,
                         (unsigned long long)(c->lib_blocker < MAX_SEATS ? c->lib_blocker : 0));
            }
            fprintf(graft_csv, "%llu,%d,%llu,%d,%u,%u,%d,%d,%d,%.2f,%.2f,%.2f,%d,%s,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,"
                               "%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%.3f,%.4f,%.3f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%s,%s,%s,%u,%s\n",
                    (unsigned long long)(ms - t0), seat, (unsigned long long)g->life, trace, g->tick, g->last_tick,
                    c->drive_info.mode, c->drive_in.reverse, c->drive_info.stale, (double)c->drive_info.e_deg,
                    (double)c->drive_in.v_cap, (double)c->drive_info.v_allow, c->drive_in.firm, cruise, P[0], P[1], P[2], P[3], P[4], P[5],
                    g->keys[0], g->keys[1], g->wrote, L[0], L[1], L[2], L[3], A[0], A[1], A[2], A[3], A[4], A[5],
                    (unsigned)gh_vphys_gear(phys), (unsigned)gh_vphys_gear_request(phys),
                    (double)graft_gear_ratio(phys), vel[0] * cos(h) + vel[2] * sin(h), omega[1],
                    acc[0] * cos(h) + acc[2] * sin(h), (double)p[0], (double)p[2], (double)(h * 57.29578f),
                    (unsigned)gh_vphys_auto_gear(phys), (unsigned)gh_vphys_input_flag_a(phys), (unsigned)gh_vphys_input_flag_b(phys),
                    (unsigned)gh_vphys_input_flag_c(phys), stop_m, aim_x, aim_z, c->drive_in.route_rev, lib_cols);
            if (!graft_csv_flush_ms || ms - graft_csv_flush_ms >= 1000) {
                fflush(graft_csv);
                graft_csv_flush_ms = ms;
            }
        }
    }
    if (!graft_summary_ms)
        graft_summary_ms = ms;
    if (ms - graft_summary_ms >= GRAFT_SUMMARY_MS && graft_updates) {
        event(host, "wasd: %u updates, %u physics ticks in %.0f s; applied controls = layer output for our keys on %u ticks "
              "(%.1f%%); %u updates wrote no keys (keys older than %u ms)", graft_updates, graft_ticks,
              0.001 * (double)(ms - graft_summary_ms), graft_ticks_match,
              graft_ticks ? 100.0 * graft_ticks_match / graft_ticks : 0.0, graft_stale_writes, WASD_KEYS_MS);
        graft_summary_ms = ms;
        graft_updates = graft_ticks = graft_ticks_match = graft_stale_writes = 0;
    }
}

/* s49: the guarded block of wt_ground_controls, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *unit; void *host; int phase; } graft_phase_ctx_t;
static void graft_phase_run(void *ctx)
{
    graft_phase_ctx_t *const context = ctx;
    u8 *const unit = context->unit;
    void *const host = context->host;
    const int phase = context->phase;
    {
        if (phase == 0)
            graft_before(unit, host, gh_ms());
        else
            graft_after(unit, host, gh_ms());
    }
}

/* Called by the launcher's input-layer hook around the game's input layer in the game's unit update:
 * phase 0 right before it, phase 1 right after it (arguments: the unit, the host context, the phase). */
void ground_controls(u8 *unit, void *host, int phase)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || !cfg.graft || graft_dead || !unit || !(cfg.lanes || cfg.native_control))
        return;
    gh_lock(GH_LOCK_MAIN);
    graft_phase_ctx_t context = {unit, host, phase};
    if (gh_guard(graft_phase_run, &context)) {
        graft_dead = 1;
        event(host, "graft: fault at unit %p, WASD write point off for this process", (void *)unit);
    }
    gh_unlock(GH_LOCK_MAIN);
}
