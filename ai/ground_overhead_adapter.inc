/* s17 (2026-11-16): overhead probes, adapter side of ground_overhead.c.
 * Called from wt_ground_observe right after lidar_observe, under the ground
 * lock. It hands the cache the controlled bot tanks (the lidar's set: owned
 * lane seats, alive; with their route aim and upcoming route points), every
 * vehicle hull of the last motion publication (humans, other bots and wrecks
 * included: none of them may enter the cache), the game's collision ray
 * (lidar_trace: trace_with_flags with its fault guard; vertical rays with flags
 * GH_RAY_GROUND_AND_OBJECTS = land | static | placed objects, as the aim ray, and GH_RAY_GROUND for the ground
 * under an object, as the lidar's ground probes) and the 2 m WTOBS001 scan
 * (probe start heights over tall obstacles, snapshots).
 * The planner reads the cache through overhead_world_block (gm_snapshot_t
 * world_block, set only with cfg.overhead).
 * cfg.overhead = 0 (WT_GROUND_OVERHEAD=0): this returns at once; no rays, no
 * files, and the planner gets no world_block (exactly s16). */
#include "ground_overhead.h"

#define OVERHEAD_GAP_US 2000

static struct {
    int inited;
    uint64_t scene, last_us;
    u8 *host;
    const u8 *prior;
    double us_per_count;
} overhead_adapter;

static int overhead_world_block(void *context, float cell, int wc0, int wr0, int n, uint8_t *out)
{
    (void)context;
    if (!cfg.overhead || !overhead_adapter.inited || !go_active())
        return -1;
    return go_block(cell, wc0, wr0, n, out);
}

static void overhead_observe(void *host, uint64_t ms)
{
    if (!cfg.overhead || !host)
        return;
    if (overhead_adapter.inited && (overhead_adapter.scene != ground_scene_epoch || overhead_adapter.host != host
                                    || overhead_adapter.prior != geometry.scan)) {
        go_shutdown(ms - t0);
        overhead_adapter.inited = 0;
    }
    if (!overhead_adapter.inited) {
        go_config_t c = {.trace = lidar_trace, .trace_context = host, .event = lidar_event, .event_context = host,
            .cell_m = cfg.overhead_cell_m, .range_m = cfg.overhead_range_m, .us_per_tick = cfg.overhead_us_per_tick,
            .log_dir = cfg.log_dir};
        if (geometry.scan) {
            c.prior = (const uint16_t *)geometry.scan;
            c.prior_nx = geometry.nx;
            c.prior_nz = geometry.nz;
            c.prior_x0 = geometry.x0;
            c.prior_z0 = geometry.z0;
            c.prior_step = geometry.step;
        }
        go_init(&c);
        long long f;
        gh_qpc_frequency(&f);
        overhead_adapter.us_per_count = 1e6 / (double)f;
        overhead_adapter.inited = 1;
        overhead_adapter.scene = ground_scene_epoch;
        overhead_adapter.host = host;
        overhead_adapter.prior = geometry.scan;
    }
    if (!go_active())
        return;
    long long now;
    gh_qpc(&now);
    const uint64_t now_us = (uint64_t)((double)now * overhead_adapter.us_per_count);
    if (overhead_adapter.last_us && now_us >= overhead_adapter.last_us && now_us - overhead_adapter.last_us < OVERHEAD_GAP_US)
        return;
    overhead_adapter.last_us = now_us;
    static go_tank_t tanks[MAX_SEATS];
    static go_hull_t hulls[GM_MAX_VEHICLES];
    int n = 0;
    for (int seat = 1; seat < MAX_SEATS && seat < GO_MAX_SEATS; ++seat) {
        const lane_seat_t *s = &lseat[seat];
        if (!s->unit || s->scene_epoch != ground_scene_epoch || !slot_alive[seat] || slot_unit[seat] != s->unit
            || cmd_life_replaced(s, seat) || gh_unit_not_active(s->unit) != 0)
            continue;
        const float *pos = gh_unit_pos(s->unit);
        const float fx = gh_unit_fwd_x(s->unit), fz = gh_unit_fwd_z(s->unit);
        if (!isfinite(pos[0]) || !isfinite(pos[1]) || !isfinite(pos[2]) || !(hypotf(fx, fz) > 0.25f))
            continue;
        go_tank_t *t = &tanks[n++];
        *t = (go_tank_t){.seat = seat, .life = s->born_ms, .x = pos[0], .y = pos[1], .z = pos[2],
            .heading = atan2f(fz, fx), .speed = hypotf(s->vx, s->vz)};
        const int index = motion_index[seat];
        const gm_vehicle_plan_t *plan = index >= 0 && index < motion_plan.count
                                        && motion_plan.vehicles[index].id == (uint64_t)seat
                                        ? &motion_plan.vehicles[index] : NULL;
        float lx = pos[0], lz = pos[2];
        if (plan && isfinite(plan->route_target.x) && isfinite(plan->route_target.z)) {
            t->route_x[t->route_count] = lx = plan->route_target.x;
            t->route_z[t->route_count++] = lz = plan->route_target.z;
        }
        if (plan && index < motion_vehicle_count) {
            const gm_vehicle_t *v = &motion_vehicles[index];
            for (int k = plan->path_cursor > 0 ? plan->path_cursor : 0; v->path && k < v->path_count
                 && t->route_count < GO_ROUTE_POINTS; ++k) {
                const gm_point_t q = v->path[k];
                if (!isfinite(q.x) || !isfinite(q.z) || hypotf(q.x - lx, q.z - lz) < 1.0f)
                    continue;
                t->route_x[t->route_count] = lx = q.x;
                t->route_z[t->route_count++] = lz = q.z;
            }
        }
        if (!t->route_count && isfinite(s->move_target_x) && isfinite(s->move_target_z)
            && hypotf(s->move_target_x - pos[0], s->move_target_z - pos[2]) > 1.0f) {
            t->route_x[0] = s->move_target_x;
            t->route_z[0] = s->move_target_z;
            t->route_count = 1;
        }
    }
    int nh = 0;
    for (int i = 0; i < motion_vehicle_count && i < GM_MAX_VEHICLES; ++i) {
        const gm_vehicle_t *v = &motion_vehicles[i];
        if (!isfinite(v->x) || !isfinite(v->z) || !isfinite(v->heading))
            continue;
        hulls[nh++] = (go_hull_t){.x = v->x, .z = v->z, .heading = v->heading, .half_length = 0.5f * v->length,
            .half_width = 0.5f * v->width, .forward = v->hull_forward, .right = v->hull_right,
            .speed = hypotf(v->vx, v->vz)};
    }
    go_tick(ms - t0, now_us, tanks, n, hulls, nh);
}
