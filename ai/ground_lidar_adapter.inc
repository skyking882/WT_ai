/* s11 (2026-11-15): live local perception, adapter side of ground_lidar.c.
 * Called from wt_ground_observe (the per-unit behaviour-tree tick on the game
 * thread) under the ground lock, after the motion publication. It hands the
 * lidar the live bot tanks (owned lane seats; not the human, not wrecks), the
 * game's collision ray (trace_with_flags: the same function and fault guard as
 * the obstacle scan and the live geometry tiles), the 2 m WTOBS001 scan as the
 * prior for the evaluation logs, and the map water level of the terrain layer.
 * s13: the motion layer's local planner reads the grids through
 * lidar_local_block (cfg.local); control and nav do not.
 * cfg.lidar = 0 (WT_GROUND_LIDAR=0): this returns at once; no rays, no files,
 * and the local planner runs on the obstacle cells alone. */
#include "ground_lidar.h"

#define LIDAR_GAP_US 2000

static struct {
    int inited;
    uint64_t scene, last_us;
    u8 *host;
    const u8 *prior;
    double us_per_count;
} lidar_adapter;

static int lidar_trace(void *context, const float p[3], const float dir[3], float *t, int *kind, int flags)
{
    return trace_with_flags((u8 *)context, p, dir, t, kind, flags);
}

static void lidar_event(void *context, const char *message)
{
    event((u8 *)context, "%s", message);
}

/* s13: the planner's read of a seat's grid (gm_snapshot_t.local_block). Only
 * the bot seats the lidar serves; foreign hulls have no grid. */
static int lidar_local_block(void *context, uint64_t vehicle_id, int wc0, int wr0, int n, uint8_t *out)
{
    (void)context;
    if (!cfg.lidar || !lidar_adapter.inited || !gl_active() || !vehicle_id || vehicle_id >= GL_MAX_SEATS)
        return -1;
    return lidar_block((int)vehicle_id, wc0, wr0, n, out);
}

static void lidar_observe(void *host, uint64_t ms)
{
    if (!cfg.lidar || !host)
        return;
    /* A new scene (battle) or a scan mapped mid-battle: a fresh lidar. */
    if (lidar_adapter.inited && (lidar_adapter.scene != ground_scene_epoch || lidar_adapter.host != host
                                 || lidar_adapter.prior != geometry.scan)) {
        gl_shutdown(ms - t0);
        lidar_adapter.inited = 0;
    }
    if (!lidar_adapter.inited) {
        gl_config_t c = {.trace = lidar_trace, .trace_context = host, .event = lidar_event, .event_context = host,
            .range_m = cfg.lidar_range_m, .us_per_tick = cfg.lidar_us_per_tick,
            .water = terrain.t.h ? terrain.t.water : NAN, .log_dir = cfg.log_dir};
        if (geometry.scan) {
            c.prior = (const uint16_t *)geometry.scan;
            c.prior_nx = geometry.nx;
            c.prior_nz = geometry.nz;
            c.prior_x0 = geometry.x0;
            c.prior_z0 = geometry.z0;
            c.prior_step = geometry.step;
        }
        gl_init(&c);
        long long f;
        gh_qpc_frequency(&f);
        lidar_adapter.us_per_count = 1e6 / (double)f;
        lidar_adapter.inited = 1;
        lidar_adapter.scene = ground_scene_epoch;
        lidar_adapter.host = host;
        lidar_adapter.prior = geometry.scan;
    }
    if (!gl_active())
        return;
    /* wt_ground_observe runs once per unit and tree tick: the units of one
     * frame come back to back, so the lidar runs once per LIDAR_GAP_US. */
    long long now;
    gh_qpc(&now);
    const uint64_t now_us = (uint64_t)((double)now * lidar_adapter.us_per_count);
    if (lidar_adapter.last_us && now_us >= lidar_adapter.last_us && now_us - lidar_adapter.last_us < LIDAR_GAP_US)
        return;
    lidar_adapter.last_us = now_us;
    static gl_tank_t tanks[MAX_SEATS];
    int n = 0;
    for (int seat = 1; seat < MAX_SEATS; ++seat) {
        const lane_seat_t *s = &lseat[seat];
        if (!s->unit || s->scene_epoch != ground_scene_epoch || !slot_alive[seat] || slot_unit[seat] != s->unit
            || cmd_life_replaced(s, seat) || gh_unit_not_active(s->unit) != 0)
            continue;
        const float *pos = gh_unit_pos(s->unit);
        const float fx = gh_unit_fwd_x(s->unit), fz = gh_unit_fwd_z(s->unit);
        if (!isfinite(pos[0]) || !isfinite(pos[1]) || !isfinite(pos[2]) || !(hypotf(fx, fz) > 0.25f))
            continue;
        const int index = motion_index[seat];
        const gm_vehicle_plan_t *plan = index >= 0 && index < motion_plan.count
                                        && motion_plan.vehicles[index].id == (uint64_t)seat
                                        ? &motion_plan.vehicles[index] : NULL;
        const enum gm_state state = s->motion_state;
        const traffic_hull_t hull = traffic_hulls[seat].unit == s->unit && traffic_hulls[seat].life == s->born_ms
                                    ? traffic_hulls[seat] : (traffic_hull_t){s->unit, s->born_ms, 3.1f, 1.65f, 0.0f, 0.0f};
        tanks[n++] = (gl_tank_t){.seat = seat, .life = s->born_ms, .x = pos[0], .y = pos[1], .z = pos[2],
            .heading = atan2f(fz, fx), .speed = hypotf(s->vx, s->vz), .state = (int)state,
            .state_name = motion_state_name(state),
            .snap_state = state == GM_STATIC_BLOCKED || state == GM_WAIT_TRAFFIC,
            .target_x = plan ? plan->route_target.x : s->move_target_x,
            .target_z = plan ? plan->route_target.z : s->move_target_z,
            .goal_x = plan ? plan->goal.x : s->move_target_x, .goal_z = plan ? plan->goal.z : s->move_target_z,
            .hull_half_length = hull.length, .hull_half_width = hull.width, .hull_forward = hull.forward,
            .hull_right = hull.right};
    }
    gl_tick(ms - t0, now_us, tanks, n);
}
