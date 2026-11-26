#ifndef GROUND_NAV_H
#define GROUND_NAV_H
/* Static navigation layer for the central ground planner.
 *
 * The planner asks this module for a body-valid route up front. Every motion
 * on a published route (in-place pivot at a vertex followed by a straight
 * translation to the next vertex) has been validated by the same swept
 * footprint authority the local planner uses (gm_static_sweep_fn), with the
 * hull inflated by request->margin. Deterministic: no wall clock is used. */
#include "ground_motion.h"

#define GM_NAV_ROUTE_POINTS 256
#define GM_NAV_MAX_GOALS 96
#define GM_NAV_MAX_BODIES 128
#define GM_NAV_CLEAR_CAP 12.0f

enum gm_nav_status { GM_NAV_NONE, GM_NAV_NO_MAP, GM_NAV_BUILDING, GM_NAV_PENDING, GM_NAV_READY, GM_NAV_UNREACHABLE };

/* Flags describe the leg that ARRIVES at a vertex, and the vertex itself. */
/* GM_NAV_TIGHT with the comfort tier enabled: the leg arriving here, or the pivot at
 * this vertex, was validated only at the minimum `margin` (tight parts are isolated
 * into their own short legs; every non-TIGHT leg holds comfort_margin and every
 * non-TIGHT pivot comfort_margin + pivot_slack). Also set on body-escape legs.
 * Without the tier: the leg or pivot needed the authority (not proven free space). */
#define GM_NAV_TIGHT   1u /* track exactly, pivot at the vertex */
#define GM_NAV_REVERSE 2u /* leg is driven backwards with unchanged heading (initial escape only) */
#define GM_NAV_UNKNOWN 4u /* authority answered GM_UNKNOWN somewhere on this leg */

typedef struct gm_nav gm_nav_t;

/* Stationary body (parked, dead or long-stopped vehicle), oriented rectangle.
 * pose.x/z is the rotation centre; the hull centre is offset by hull_forward
 * along the heading and hull_right to the right (towards -z-rotated side), like
 * the vehicle convention. Bodies block motions overlapping them inflated by the
 * request's body_clearance (default 0.6 m). */
typedef struct {
    uint64_t id;
    gm_pose_t pose;
    float half_length, half_width, hull_forward, hull_right;
} gm_nav_body_t;

typedef struct {
    const gm_vehicle_t *vehicle;  /* id, life_id, pose (x,z,heading), dims, hull offsets, can_pivot */
    uint64_t route_key;           /* caller's task identity; a change means: drop the route, search from scratch */
    const gm_point_t *via;        /* upstream waypoints still ahead, travel order; advisory */
    int via_count;
    const gm_point_t *goals;      /* candidate end points */
    const float *goal_cost;       /* extra cost in metres per goal, may be NULL */
    int goal_count;               /* 1..GM_NAV_MAX_GOALS */
    float goal_radius;            /* a goal is reached within this distance of it */
    float margin;                 /* static clearance margin added around the hull for every validated motion */
    float body_clearance;         /* hull-to-hull clearance kept from stationary bodies, on top of margin
                                     (replaces the former fixed 0.6 m); <= 0 means the default 0.6 m. Also
                                     defines the inflated zone used by the body-aware start escape. */
    float comfort_margin;         /* comfort tier: translations are preferred with the hull inflated by this,
                                     in-place rotations with comfort_margin + pivot_slack. Motions that are
                                     clear only at `margin` are allowed but costed higher (translation x3,
                                     rotation x6) and flagged GM_NAV_TIGHT. <= margin (or 0) disables the tier:
                                     then TIGHT means "the authority was needed" (pre-v4 behaviour). */
    float pivot_slack;            /* extra inflation for in-place rotations in the comfort tier (>= 0) */
    int replan;                 /* 1: evidence invalidated the current route; search again from the current pose */
    /* s24 spread: group 1 or 2 (a side) and share_cost > 0: every published route of the group is stamped on
     * the cells within one cell of it (once per route), and while this vehicle's route is searched a cell that
     * k other routes of its group cover costs (1 + share_cost * min(k, 3)) times its own cost. Its own old
     * route is taken off when its search starts. 0: no stamping, no extra cost (exactly s22). */
    int group;
    float share_cost;
} gm_nav_request_t;

typedef struct {
    enum gm_nav_status status;
    unsigned revision;            /* +1 for every newly published route of this vehicle */
    uint64_t route_key;
    int count;                    /* vertices after the start pose */
    gm_point_t points[GM_NAV_ROUTE_POINTS];
    uint8_t flags[GM_NAV_ROUTE_POINTS];
    float remaining[GM_NAV_ROUTE_POINTS];   /* path length from vertex i to the route end */
    short via_vertex[GM_MAX_ROUTE_POINTS];  /* request via k -> vertex index where it counts as passed, -1 = skipped */
    int via_count;
    int goal_index;               /* goal the route ends at; -1 while partial */
    int partial;                  /* ends before a goal (vertex cap); nav extends it as the vehicle nears the end */
    float length;
    gm_point_t origin;            /* position the route was searched from */
} gm_nav_route_t;

/* Process-wide instance, lazily allocated, NULL on failure. There is ONE
 * instance per process: run one planner per process, and that planner must
 * call gm_nav_reset when its own state is zeroed (map or match change). */
gm_nav_t *gm_nav_shared(void);
/* Forget routes, bodies, vetoes, caches; keep the grid if the map is unchanged. */
void gm_nav_reset(gm_nav_t *nav);
/* Start a planning snapshot. map == NULL -> GM_NAV_NO_MAP and every other call
 * is a no-op until a map is given. A change of (x0,z0,step,columns,rows,
 * map_epoch) rebuilds the clearance grid (time-sliced in gm_nav_step, status
 * GM_NAV_BUILDING meanwhile) and drops all routes, caches and vetoes.
 * ms is informational only (never used for decisions).
 * nav keeps a COPY of *map (not the pointer): its `cell` callback and
 * `context` must stay valid between snapshots, because the time-sliced grid
 * build calls them from later gm_nav_step calls. */
enum gm_nav_status gm_nav_begin(gm_nav_t *nav, const gm_map_t *map, uint64_t map_epoch, uint64_t ms);
/* Replace the stationary body set (at most GM_NAV_MAX_BODIES are kept). A body
 * with the requesting vehicle's id is ignored for that vehicle. */
void gm_nav_set_bodies(gm_nav_t *nav, const gm_nav_body_t *bodies, int count);
/* The local authoritative sweep failed near `at` for this hull: drop cached
 * answers near it and veto motions of that hull class through the disc. */
void gm_nav_report_blocked(gm_nav_t *nav, const gm_vehicle_t *vehicle, gm_point_t at, float radius);
/* The authority refused exactly this motion for this hull class (same dims,
 * any margin/tier). Positions coincide (< 1 mm): in-place rotation, vetoing
 * every in-place rotation within 1 m of `from` whose swept heading interval
 * overlaps the refused one in the same turning direction (turning the other
 * way, translating first or reversing out stay possible). Otherwise a
 * translation, vetoing translations in the same direction of travel (within
 * 45 deg) whose swept segment overlaps the refused one within 1 m laterally.
 * A zero motion is ignored. Vetoes are permanent for the map (monotone), apply
 * to both tiers, the start connection and string pulling, and drop affected
 * cached answers; at most 1024 are kept (the oldest is evicted beyond that);
 * gm_nav_reset and a map change clear them. Send replan=1 afterwards. */
void gm_nav_report_blocked_motion(gm_nav_t *nav, const gm_vehicle_t *vehicle, const gm_pose_t *from, const gm_pose_t *to);
/* Persistent per-vehicle request; call once per snapshot with the CURRENT pose.
 * Pointers are copied from, not retained. */
void gm_nav_request(gm_nav_t *nav, const gm_nav_request_t *request);
/* Drop the request and route of a vehicle. */
void gm_nav_forget(gm_nav_t *nav, uint64_t vehicle_id);
/* Do at most work_units of work (1 = state expansion, 8 = authority sweep,
 * 1 per 64 cells of grid build). Returns the units used. */
int gm_nav_step(gm_nav_t *nav, gm_static_sweep_fn sweep, void *context, int work_units);
/* Route of a vehicle, NULL if the vehicle never requested. Valid until the
 * next gm_nav_* call that mutates the instance. */
const gm_nav_route_t *gm_nav_route(const gm_nav_t *nav, uint64_t vehicle_id);
/* Distance from p to the nearest BLOCKED-or-UNKNOWN cell square, capped at
 * GM_NAV_CLEAR_CAP, 0 outside the map or while the grid is not built. */
float gm_nav_clearance(const gm_nav_t *nav, gm_point_t p);
/* Obstacle cells (BLOCKED or UNKNOWN, as whole squares) under the hull at this
 * pose, grown by margin on every side; cells outside the map count. 0 while
 * the grid is not built. The local layer uses the count to let a hull that
 * already stands in such a cell drive out but never further in. */
int gm_nav_overlap(const gm_nav_t *nav, const gm_vehicle_t *vehicle, const gm_pose_t *pose, float margin);
/* s13 local planner: the obstacle-cell grid itself. gm_nav_grid gives its
 * placement (0 while it is not built); gm_nav_cell_solid(col, row) is 1 for a
 * blocked-or-unknown cell and for any cell outside the map, else 0. */
int gm_nav_grid(const gm_nav_t *nav, float *x0, float *z0, float *step, int *cols, int *rows);
int gm_nav_cell_solid(const gm_nav_t *nav, int col, int row);
/* The same along a turn on the spot or a straight translation: GM_BLOCKED if
 * the hull meets an obstacle cell anywhere on it, else GM_CLEAR; GM_UNKNOWN
 * while there is no grid. gm_nav_step asks this before, and with a NULL sweep
 * instead of, the authority: every published motion keeps its margin from
 * the obstacle cells. */
enum gm_geometry gm_nav_solid(const gm_nav_t *nav, const gm_vehicle_t *vehicle, const gm_pose_t *from,
                              const gm_pose_t *to, float margin);
/* Reachability by connected components of the free cells (4-neighbour, built
 * with the grid). From `from` (a blocked start counts as the nearest free
 * cell within a few metres): 1 = goal in the same component, *out = goal;
 * 2 = it is not, *out = the nearest point of from's component with at least
 * `need` clearance within GM_NAV_SNAP_RANGE of the goal; 0 = there is none.
 * While the components are not built, or `from` has no free cell near it,
 * every goal counts as reachable (1). With out NULL nothing is searched: a
 * goal in another component answers 0. */
#define GM_NAV_SNAP_RANGE 300.0f
int gm_nav_reach(const gm_nav_t *nav, gm_point_t from, gm_point_t goal, float need, gm_point_t *out);
/* Work counters since gm_nav_shared or the last gm_nav_reset (diagnostics):
 * route jobs started, lattice searches started, their expansions, and the
 * most expansions one search took. */
typedef struct { uint64_t jobs, searches, expansions; int max_expansions; } gm_nav_stats_t;
void gm_nav_stats(const gm_nav_t *nav, gm_nav_stats_t *out);
#endif
