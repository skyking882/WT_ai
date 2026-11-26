#ifndef GROUND_MOTION_H
#define GROUND_MOTION_H

#include <stdint.h>

#define GM_MAX_VEHICLES 128
#define GM_STEPS 24
#define GM_STEP_S 0.20f
#define GM_STEP_MS 200u
/* A published command stays usable this long. The game's publications come
 * 90 to 125 ms apart but now and then several hundred: a speed and a heading
 * rate held over such a gap do less harm than the full brake a missing plan means. */
#define GM_PLAN_VALID_MS 500u
#define GM_MAX_ROUTE_POINTS 64
/* Must equal GM_NAV_ROUTE_POINTS / GM_NAV_MAX_GOALS of ground_nav.h (checked in ground_motion.c). */
#define GM_ROUTE_COPY 256
#define GM_SLOT_GOALS 96
#define GM_ABANDONED_SLOTS 8
#define GM_LOCAL_CRUMBS 32

typedef struct { float x, z; } gm_point_t;

/* Recently written controls, oldest to newest. Ages are measured at the
 * snapshot. The first entry covers the oldest supported transport delay. */
typedef struct { float age_s, drive, brake, steer; } gm_pending_command_t;

typedef struct {
    gm_point_t p;
    float heading, half_length, half_width; /* NAN heading reserves all orientations. */
} gm_spawn_site_t;

/* Positions refer to the track rotation centre. Heading 0 points along +x,
 * positive yaw rotates towards +z. SI units throughout. */
typedef struct {
    float x, z, heading, speed, yaw_rate;
} gm_pose_t;

typedef struct {
    uint64_t id, life_id, route_id;
    uint64_t goal_region_id; /* Shared ID for vehicles capturing the same region. */
    gm_point_t goal_region_center;
    float goal_region_radius; /* Zero or no ID disables slot allocation. */
    int controlled, move, native_corridor, can_pivot, align;
    float align_heading; /* Pivot/steer while braking when move is false. */
    float x, y, z, heading, vx, vz, yaw_rate;
    /* Full hull dimensions. The planner converts these to half-extents. */
    float length, width, hull_forward, hull_right;
    float cruise_speed, acceleration, braking, max_yaw_rate, yaw_acceleration;
    /* Time until a newly committed command can affect motion. Pending
     * acceleration is signed along the hull axis and yaw is its prior target. */
    float command_delay_s, pending_acceleration, pending_yaw_target;
    float command_delay_max_s; /* Transport bound plus next command consumption interval. */
    const gm_pending_command_t *pending_commands;
    int pending_command_count;
    float reverse_speed; /* Zero disables reverse legs (nav initial escape). */
    int pending_reverse;
    float arrival_radius, corridor_half_width;
    gm_point_t route_origin, goal;
    const gm_point_t *path; /* Upcoming waypoints, in travel order. Advisory via points for nav. */
    int path_count, native_corridor_points; /* Zero licenses all original points. */
    /* s13 local planner: rad of heading per metre rolled at full steering, the
     * same forwards and backwards per signed metre (the game steers a hull like
     * a car). Zero: GM_TRACKED_TURN_PER_M, or GM_WHEELED_TURN_PER_M without can_pivot. */
    float turn_per_m;
    /* s24 path mode: the side (1 or 2, 0 none). Routes searched for one side in the same period spread out:
     * the cells of a teammate's present route cost a little more for the next search (ground_nav share). */
    int group;
    /* s32 library mode: the order's route variant (0 recommended, 1 / 2 a flank corridor: gr_route_variant) and the
     * spawn pose of this life (-1 unknown), for the designated and the attach picks. */
    int route_variant, route_pose;
    int goal_target1;          /* s35: 1 + the library target the order names (a lane slot); 0: map the goal (gr_target_of) */
} gm_vehicle_t;

#define GM_TRACKED_TURN_PER_M 0.42f /* measured 2026-11-14/15: no turn at rest, ~0.42 rad per metre at a crawl */
#define GM_WHEELED_TURN_PER_M 0.15f /* assumed (s8) */
/* Yaw rate the local planner and its steering count on at most: measured
 * 0.3-0.4 rad/s in the games of 2026-11-14/15 (the nominal 0.65 is learned
 * down only after a while). */
#define GM_LOCAL_YAW_CAP 0.38f

enum gm_geometry { GM_UNKNOWN, GM_CLEAR, GM_BLOCKED };

/* Equal poses query the footprint at a prospective parking position. */
typedef enum gm_geometry (*gm_static_sweep_fn)(void *context, const gm_vehicle_t *vehicle,
                                              const gm_pose_t *from, const gm_pose_t *to);

/* This strategic occupancy hint is independent of the authoritative swept
 * footprint query. Queries use integer coordinates in the map grid. */
typedef enum gm_geometry (*gm_map_cell_fn)(void *context, int column, int row);
/* Travel cost of a free cell for the route layer, as a factor on its length:
 * 1 on level dry ground, more where the ground is steep or the water shallow
 * (terrain layer). Optional: without it every free cell costs 1. */
typedef float (*gm_map_cost_fn)(void *context, int column, int row);
typedef struct {
    float x0, z0, step;
    int columns, rows;
    gm_map_cell_fn cell;
    void *context;
    gm_map_cost_fn cost;
    /* Clutter (optional, v13 spec section 3): one byte per square cell of
     * clutter_step, row-major (row = +z), 0..255 = how built-up the ground
     * around it is (bot_trees/clutter/<mission>.bin, WTCLT001). Ordinary route
     * searches weigh every metre by 1 + clutter_k * value / 255 on top of the
     * terrain cost; moves that free a hull from an obstacle are not charged.
     * NULL or clutter_k <= 0: no clutter cost. Must stay valid between
     * snapshots, like `cell`. */
    const uint8_t *clutter;
    int clutter_columns, clutter_rows;
    float clutter_x0, clutter_z0, clutter_step, clutter_k;
} gm_map_t;

/* s13 local planner inputs. local_block: the vehicle's own perception (the
 * lidar grid, ground_lidar.h lidar_block) for world cells wc0 .. wc0+n-1 x
 * wr0 .. wr0+n-1 of GM_LOCAL_CELL metres, world cell = floor(coordinate /
 * GM_LOCAL_CELL), into out[row * n + col] as GM_LOCAL_* ; returns < 0 when the
 * vehicle has no perception. local_log: one decision row (kind 0, CSV) or one
 * event line (kind 1, wedge / recovery); both optional. */
#define GM_LOCAL_CELL 0.25f
#define GM_LOCAL_UNKNOWN 0
#define GM_LOCAL_FREE 1
#define GM_LOCAL_OCCUPIED 2
typedef int (*gm_local_block_fn)(void *context, uint64_t vehicle_id, int wc0, int wr0, int n, uint8_t *out);
typedef void (*gm_local_log_fn)(void *context, int kind, const char *line);
#define GM_LOCAL_LOG_HEADER "ms,id,x,z,heading,speed,state,v_des,type,dir,cmd_speed,kappa,cost,progress,clear," \
    "head_err,dyn,prox,end_depth,start_depth,feasible,total,static_refused,hull_refused,lidar_cells," \
    "override_cells,over_path,pose_checks,wedged,stand_cost,exit_mode,exit_dir,exit_free"
/* s17: the map-wide overhead-probe cache (ground_overhead.h go_block): world
 * cells of cell metres (world cell = floor(coordinate / cell)) wc0 .. wc0+n-1 x
 * wr0 .. wr0+n-1 into out[row * n + col] as GM_WORLD_* (the GO_BLOCK_* values);
 * returns < 0 when there is no cache. */
#define GM_WORLD_UNKNOWN 0
#define GM_WORLD_FREE 1        /* probed free at least twice in a row */
#define GM_WORLD_OCCUPIED 2
#define GM_WORLD_FREE_ONCE 3
#define GM_WORLD_OVERHEAD 4    /* something above the hull height: says nothing about the ground */
typedef int (*gm_world_block_fn)(void *context, float cell, int wc0, int wr0, int n, uint8_t *out);

typedef struct {
    uint64_t ms;
    int count;
    const gm_vehicle_t *vehicles;
    gm_static_sweep_fn static_sweep;
    gm_static_sweep_fn static_probe; /* Optional separate stationary map queries. */
    const gm_map_t *map; /* Static occupancy for the route layer (ground_nav). NULL: degraded mode. */
    uint64_t map_epoch;  /* Change whenever the cells behind an unchanged map geometry change. */
    void *geometry_context;
    /* Spawn points where a birth may still happen. Nothing is reserved: a
     * vehicle among them just leaves straight ahead, slowly, before it turns. */
    const gm_spawn_site_t *pending_spawn_sites;
    int pending_spawn_site_count;
    /* s13 (WT_GROUND_LOCAL): 1 replaces the static candidate check (gm_choose)
     * by the local trajectory sampler over the fused occupancy (perception
     * where it has seen, the obstacle cells elsewhere). 0: exactly s12. */
    int local_planner;
    gm_local_block_fn local_block;
    gm_local_log_fn local_log;
    void *local_context;
    /* s16 (WT_GROUND_SPAWN_EXIT, config spawn_exit): 1 = "exit mode" for a
     * tank in the spawn exit queue (local planner only): it steers for the
     * open space around it first and the route after, the queue holds a tank
     * only for a group member in its own way out, and it does not reverse
     * there unless pressed, wedged or unable to go forward. 0: exactly s15. */
    int spawn_exit;
    /* s17 (WT_GROUND_OVERHEAD, config overhead): the overhead-probe cache, local
     * planner only. lp_build fuses it where the vehicle's lidar has not seen:
     * an obstacle there occupies even where the obstacle cells are clear, free
     * twice clears a solid obstacle cell (the lidar's rule), anything else
     * leaves the obstacle cells in charge. The spawn exit scan (ex_scan) also
     * reads it beyond the 32 m raster. NULL: exactly s16. */
    gm_world_block_fn world_block;
    void *world_context;
    /* s24 (WT_GROUND_PATH, config path, only with the WASD driver): command -> route -> avoidance -> driver.
     * A route from where the tank stands to its target (the vehicle's goal, no via points); the aim is the next
     * route vertex (earlier the one after it, where the hull's straight sweep there is free); a stop distance
     * along the present heading; an avoidance layer that is idle unless something keeps the stop distance
     * under 2 m. The local sampler (gm_local_choose / gm_choose), the spawn queue and exit mode, the old
     * back-off / wedge / jam rules take no part. 0: exactly s22. local_log kind 2: one ground_plan.log line
     * ("path: ..." / "avoid: ..."). */
    int path_mode;
    /* s26 library mode (path_mode on and this set; ground_routes.h, ground_motion_library.inc): routes picked from
     * the pre-generated library on events only (designated / attach / grid / nav), followed by one rule, vehicles
     * only in the avoidance layer. NULL: s24's path mode as it was. */
    const struct gr_lib *routes_lib;
    /* s38: the road network over the library's curves (ground_network.h), library mode only; NULL: no NET source, the
     * decision paths exactly as s37. */
    const struct gn_net *routes_net;
} gm_snapshot_t;

enum gm_state {
    GM_OBSERVED,
    GM_PARKED,
    GM_ARRIVED,
    GM_DRIVING,
    GM_TURNING,
    GM_WAIT_TRAFFIC,
    GM_WAIT_GEOMETRY,
    GM_STATIC_BLOCKED,
    GM_UNAVOIDABLE,
    GM_WAIT_SPAWN
};

typedef struct {
    uint64_t id, life_id, route_id, blocker_id;
    enum gm_state state;
    enum gm_geometry geometry;
    /* The command for the tracker: a speed along the hull axis and a heading rate. */
    float target_speed, target_yaw_rate;
    /* s22c: the speed the planner wanted before choosing a candidate (a->v_des); below the vehicle's
     * cruise_speed when it was lowered on purpose: spawn area, exit mode, a corner ahead, a narrow-passage
     * hold, yielding, arrival. 0 = not published (gm_publish zeroes the whole plan first). */
    float wanted_speed;
    /* The speed is all that the room before an obstacle allows: a hull that is
     * faster brakes down to it at once instead of coasting. */
    int firm;
    int blocked_segment; /* Index of upcoming waypoint. -1 if unavailable. */
    gm_point_t blocked_at, goal;
    int release_step; /* 0 while moving, -1 when held. */
    /* Leading upstream waypoints (vehicle path) already behind the vehicle,
     * monotone while the upstream window is unchanged. */
    int path_cursor;
    int terrain_recovery; /* The published command drives a nav GM_NAV_REVERSE leg. */
    /* repair_active: a nav route is being followed. repair_rejoined: pulses on
     * the publication where path_cursor advanced while on a nav route. */
    int repair_active, repair_rejoined;
    uint64_t commit_until_ms;
    int route_status;        /* enum gm_nav_status of this vehicle's nav request. */
    unsigned route_revision; /* Nav revision being followed, 0 if none. */
    gm_point_t route_target; /* Nav vertex currently steered to (goal without a route). */
    gm_pose_t poses[GM_STEPS + 1]; /* Open-loop preview of the command, informational. */
    /* s13 local planner: the tracker steers by this curvature (rad per signed
     * metre, positive towards +z) instead of target_yaw_rate, forwards and in
     * reverse; zero local_drive (s12 plans): unchanged tracker. */
    int local_drive;
    float curvature;
    /* s24 path mode (snapshot path_mode): the driver's whole input. go: it may drive (0 = stop and hold);
     * route_target: the aim point; path_reverse: a reverse leg (nav's escape leg or the avoidance's 3 m
     * back-up); stop_distance: the one hard limit, m from the published pose (blocked_at) along the present
     * heading (backwards on a reverse leg), < 0 none; stop_blocker: the hull that ended it, 0 static or none;
     * speed_cap: m/s, INFINITY none (only on a tight nav leg). */
    int path, go, path_reverse;
    float stop_distance, speed_cap;
    uint64_t stop_blocker;
    /* s26 library mode: pivot_reverse: the start turn of a designated route is done in reverse gear (S + A / D, the
     * driver's existing reverse-gear pivot row); probe only: the curve's source (GM_LIB_SRC_*), library route and
     * sample index (-1 on a connector), cross-track error (m, signed, + left of the curve) and the stored clearance
     * (m) at the projection, the three stop-distance components (vehicle, turn, end; INFINITY none). */
    int pivot_reverse;
    int lib_source, lib_route, lib_sample;
    float lib_cross, lib_clear, stop_vehicle, stop_turn, stop_end;
    /* s29 library mode: an align (turn to a heading while holding) planned by gl_align with s27's turn-room rule; the
     * driver's input is then the path-mode one (go, aim point along the heading, stop distance 0, the gear), not the
     * s22 align row. */
    int lib_align;
    /* s34 library mode: 1 = the street is closed by a standing hull and the route layer found no other street to the
     * target (the tank stands short of the blocker and tries again in 20 s); the command layer may give it another
     * order (a support slot of its zone). */
    int street_closed;
} gm_vehicle_plan_t;

/* s38: GM_LIB_SRC_NET, a curve over the road network (ground_network.h), appended so that the earlier numbers stay */
enum { GM_LIB_SRC_NONE, GM_LIB_SRC_DESIGNATED, GM_LIB_SRC_ATTACH, GM_LIB_SRC_GRID, GM_LIB_SRC_NAV, GM_LIB_SRC_DETOUR,
       GM_LIB_SRC_SLOT, GM_LIB_SRC_FINISH, GM_LIB_SRC_NET, GM_LIB_SRC_COUNT };
/* s26 library mode counters (tests, logs): picks per source, grid searches and their wall time, failures, NAV asks. */
typedef struct {
    unsigned picks[GM_LIB_SRC_COUNT];
    unsigned searches, failures, nav_requests;
    double search_us_sum, search_us_max;
} gm_lib_stats_t;
void gm_lib_stats(gm_lib_stats_t *out);
void gm_lib_stats_reset(void);
/* s36: the library tank (id, life) reaches library target `target` from (x, z) without passing through a street it has
 * closed (the driving side's own "no other street" search). 1 open (also: no closed street now), 0 not; *closed_until
 * when its last closed box expires (0 none). */
int gm_lib_street_open(const struct gr_lib *lib, uint64_t id, uint64_t life, int target, float x, float z, uint64_t ms,
                       uint64_t *closed_until);

typedef struct {
    uint64_t ms, valid_until_ms;
    int count;
    gm_vehicle_plan_t vehicles[GM_MAX_VEHICLES];
} gm_plan_t;

/* Persistent per-vehicle state, keyed by (id, life_id). Zero-initialise the
 * whole gm_state_t on map or match change. */
typedef struct {
    uint64_t id, life_id, route_id;
    /* Upstream window (identified by its first waypoint) and waypoints passed. */
    gm_point_t first_waypoint;
    int window_valid, cursor;
    /* This vehicle's own lateral track: its offset from the upstream line
     * through lane_ref along lane_dir. Tracks are handed out centrally so a
     * column is staggered left and right instead of nose to tail. */
    float lane_offset;
    int lane_offset_valid;
    gm_point_t lane_ref, lane_dir;
    /* Heading it was born with: the way out of the spawn area. */
    float birth_heading;
    int birth_valid;
    /* Route layer request identity. */
    uint64_t route_key, key_counter, key_region;
    gm_point_t key_goal;
    /* Where the route really ends when the goal itself lies in an obstacle:
     * the nearest standing room. Reaching it is arrival. */
    gm_point_t route_goal;
    int route_goal_moved;
    int route_status, replan;
    uint64_t replan_ms;
    gm_point_t via_sent[GM_MAX_ROUTE_POINTS];
    int via_sent_count;
    /* Followed copy of the last published nav route. */
    uint64_t followed_key;
    unsigned nav_revision;
    int nav_count, nav_vertex;
    gm_point_t nav_origin;
    gm_point_t nav_points[GM_ROUTE_COPY];
    uint8_t nav_flags[GM_ROUTE_COPY];
    /* Progress and blockage bookkeeping (0 = not in that condition). */
    gm_point_t progress_anchor;
    uint64_t progress_ms, traffic_since_ms, static_since_ms, unknown_since_ms;
    uint64_t body_id, body_until_ms; /* neighbour routed around as a stationary body */
    uint64_t replanned_body;         /* standing hull the present route was last re-planned for */
    /* Backing off from something that will not move: since when it is held,
     * until when it reverses, where the last back-off began and when it ended. */
    uint64_t hard_since_ms, escape_until_ms, escape_done_ms;
    gm_point_t escape_from;
    /* Last snapshot that listed this vehicle under this identity. The state of
     * one that drops out for a moment (its move order went stale during a
     * hitch) is kept, route and track included. */
    uint64_t seen_ms;
    /* Ordered forward with nothing happening: since when and from where. */
    uint64_t stall_since_ms;
    gm_point_t stall_anchor;
    float stall_heading;
    int stall_reported;
    /* Immobile (damage, or wedged where backing off does not help): left alone until then. */
    uint64_t disabled_until_ms;
    uint64_t no_reverse_until_ms;    /* a route's reverse leg is passed over until then: a hull stands behind */
    /* Having backed off for a hull it stood nose to nose with, it lets that hull pass before it tries again. */
    uint64_t yield_until_ms, yield_id;
    int unreachable_tries;           /* consecutive failed route searches: the next comes later each time */
    float cmd_heading;               /* heading chosen last snapshot */
    int cmd_valid, last_state, arrived;
    int arrived_short;               /* arrived beside an occupied goal, not on it */
    /* Capture slot: 0 unbound/far, 1 binding (multi-goal search), 2 bound. */
    uint64_t goal_region_id;
    int slot_phase, slot_valid, parked, slot_count, abandoned_count;
    gm_point_t slot;
    gm_point_t slots[GM_SLOT_GOALS];
    float slot_cost[GM_SLOT_GOALS];
    gm_point_t abandoned[GM_ABANDONED_SLOTS];
    /* s13 local planner (only with gm_snapshot_t.local_planner). */
    gm_point_t lp_crumbs[GM_LOCAL_CRUMBS];  /* breadcrumb trail, ring buffer, 1 m apart (last ~30 m) */
    int lp_crumb_count, lp_crumb_head;
    int lp_valid, lp_type, lp_dir;          /* previous choice: candidate type, direction */
    float lp_kappa, lp_speed;
    int lp_seg_active;                      /* a reverse-then-forward manoeuvre under way */
    gm_point_t lp_seg_from;
    float lp_seg_d1, lp_seg_k1, lp_seg_k2, lp_seg_speed2;
    int lp_seg_pursuit;
    gm_point_t lp_veto[2];                  /* wedged forwards [0] / backwards [1]: what the sensors miss is here */
    uint64_t lp_veto_until_ms[2], lp_log_ms;
    int lp_log_state, lp_log_type, lp_log_dir, lp_wedge_open, lp_push_dir;
    /* s13 spawn exit queue (local planner only): 0 never joined, 1 in the
     * queue, 2 left it (40 m from its spawn point). */
    int sq_state, sq_late, sq_released, sq_held, sq_timeout_logged;
    gm_point_t sq_spawn, sq_dir, sq_last;
    float sq_path;
    uint64_t sq_join_ms;
    /* s16 exit mode (snapshot spawn_exit): the open-space way out chosen
     * (unit vector), how far the hull gets along it, and its score; whether
     * exit mode held last snapshot; since when no forward motion makes
     * progress (reversing allowed after a while); a route search still owed
     * for having left the spawn off the route; the hull last logged as
     * holding it in its corridor. */
    int ex_valid, ex_active, ex_replan_pending, ex_replan_done;
    int ex_route;                           /* steering for the route aim (1) or the open way out ex_dir (0) */
    uint64_t ex_route_fail_ms, ex_open_ms;  /* since when the route aim gets nowhere; since when open space */
    gm_point_t ex_dir;
    float ex_free, ex_score;
    uint64_t ex_nofwd_ms, ex_held_by;
    /* s24 path mode. path_reason: why the route now asked for was asked for (GM_PATH_R_*), for its log line;
     * path_logged: the nav revision last logged. Avoidance: pa_state (GM_PA_*), since when the stop
     * distance has stayed under 2 m (or the turn has had no room), the hull or 0 that held it, the nav
     * revision the reroute was asked from, where and when the back-up began and until when it may run, when
     * the next try after "no way round" is due, the last nose-to-nose wait logged. */
    int path_reason;
    unsigned path_logged;
    int pa_state, pa_reason;
    uint64_t pa_low_since_ms, pa_blocker, pa_until_ms, pa_retry_ms, pa_backed_ms, pa_waits_for;
    unsigned pa_rev0;
    gm_point_t pa_from, pa_backed_at;
} gm_commitment_t;

struct gm_nav;

typedef struct {
    int count;
    gm_commitment_t commitments[GM_MAX_VEHICLES];
    struct gm_nav *nav; /* Acquired (and reset) on the first snapshot after zero-initialisation. */
    uint64_t nav_retry_ms;
    unsigned lib_session; /* s26: the library mode's per-tank table belongs to this session (0: new, cleared) */
} gm_state_t;

/* Publish the result as one plan. Array entries correspond to snapshot order. */
int gm_plan_snapshot(gm_state_t *state, const gm_snapshot_t *snapshot, gm_plan_t *plan);

#endif
