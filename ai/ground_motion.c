#include "ground_motion.h"
#include "ground_nav.h"
#include "ground_routes.h"
#include "ground_network.h"
#include "ground_host.h"   /* s49: the performance counter (gh_qpc) */

#include <intrin.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#ifdef GM_TRACE_CHOICE
#include <stdio.h>
#include <stdlib.h>
#endif

/* Central ground motion. Every snapshot each controlled vehicle picks a
 * velocity from the world as measured now. Nothing is reserved in space-time
 * and nothing relies on a vehicle executing a trajectory exactly.
 *
 *  1. Route. ground_nav gives a body-valid polyline around static geometry.
 *     Upstream waypoints that are behind the vehicle are never revisited, and
 *     every vehicle keeps its own lateral offset from the upstream line, so a
 *     group stays spread out instead of funnelling through one point.
 *  2. Preferred velocity: along that route, slowing for its end, for tight
 *     legs and for corners.
 *  3. Avoidance: velocity obstacles (Fiorini and Shiller; van den Berg et
 *     al.) of the real hull boxes. Every candidate velocity, taken from the
 *     headings and speeds a tracked hull can reach within its reaction time
 *     (a dynamic window), is tested for the time until the two boxes would
 *     overlap: too soon to brake for is refused, a few seconds away costs a
 *     little, so it steers early.
 *  4. Static safety. The obstacle scan decides: a cell the scan calls solid
 *     (an edge ray blocked at hull height, or something up to 3.5 m tall in
 *     it) is solid as a whole square. For every candidate heading the hull is
 *     swept over those cells, along the arc of the turn first, then ahead as
 *     far as it takes to stop from the speed it really has; it may leave a
 *     cell it stands in, never enter one. At speed it needs GM_FAST_MARGIN
 *     of room beside it as well; with less it crawls. The game's own sweep
 *     is asked for the chosen motion only: what it objects to beyond the
 *     cells is something its rays hit that the scan does not call solid, so
 *     the hull goes on slowly instead of stopping.
 *  5. A turn from a standstill creeps forward in the game: it is an arc,
 *     checked against the cells and against every hull standing near.
 *  6. The published command is a speed and a heading rate for the tracker,
 *     and whether that speed is a limit to brake down to at once.
 *
 * Whatever stops a vehicle has a way out: a new route, and when it is ordered
 * forward and does not move, can neither go on nor turn, or has pulled up so
 * close to a standing hull that it has no room for the turn it needs, a short
 * straight back-off. Short of a capture region it tries for another slot
 * before it settles outside. */

#ifndef GM_NAV_TIME_CAP_US
#define GM_NAV_TIME_CAP_US 4000 /* route search per snapshot, wall clock; 0: only the unit budget (tests) */
#endif
/* s49: the clock's two counter reads come from the host (ground_host.h: gh_qpc, gh_qpc_frequency, both on a
 * 64-bit integer). */
/* Microseconds on a clock that only measures how long the route search (and,
 * s13, the local planner, for its statistics only) ran. */
static double gm_clock_us(void)
{
    static long long frequency;
    long long now = 0;
    if (!frequency)
        gh_qpc_frequency(&frequency);
    gh_qpc(&now);
    return frequency > 0 ? (double)now * 1e6 / (double)frequency : 0.0;
}

_Static_assert(GM_ROUTE_COPY == GM_NAV_ROUTE_POINTS, "route copy must hold a full nav route");
_Static_assert(GM_SLOT_GOALS == GM_NAV_MAX_GOALS, "slot candidates are passed as nav goals");
_Static_assert(GM_MAX_VEHICLES <= GM_NAV_MAX_BODIES, "every vehicle can be a nav body");

#define GM_PI 3.14159265358979323846f
#define GM_STATIONARY_MPS 0.3f
/* Avoidance. */
#define GM_TAU 4.0f               /* s a chosen velocity must stay collision-free */
#define GM_NEIGHBOR_RANGE 70.0f
#define GM_DISC_MARGIN 0.3f       /* margin on the hull circumradius, for whole-hull distances */
#define GM_SIDE_GAP 1.0f          /* m kept between two hulls, measured along the line between them */
#define GM_FOLLOW_GAP 1.5f        /* nose-to-tail gap behind another hull, at standstill */
#define GM_HEADWAY_S 0.4f         /* plus this many seconds of own speed */
#define GM_SPAWN_HEADWAY_S 0.15f  /* the same while leaving the spawn area */
#define GM_IMPATIENCE 0.5f        /* per second without progress the avoidance weight is divided by 1 + this */
#define GM_LANE_MARGIN 2.0f       /* a moving hull ahead within this of the own swept width is "in my lane" */
#define GM_PASS_MARGIN 1.0f       /* the same for a standing hull: narrower, so it can be passed */
#define GM_REACT_S 1.0f           /* a velocity is chosen among those reachable in this time */
#define GM_TURN_GAP 2.0f          /* m beside the hull below which it does not turn towards a neighbour */
#define GM_TURN_GAP_MIN 1.0f      /* and below which it does not turn at all (the tail swings out) */
#define GM_DYN_WEIGHT 200.0f      /* avoidance outweighs a detour or slow-down */
#define GM_SOFT_WEIGHT 6.0f       /* a collision course still seconds away: worth a small early correction */
#define GM_SKIN 0.2f              /* m around the bare hulls that counts as touching */
#define GM_GIVE_WAY_S 1.0f        /* of two crossing vehicles the one farther from its goal reacts this much earlier */
#define GM_HOLD_SETBACK 25.0f     /* m before a single-lane stretch where a waiting vehicle stops: room for whoever comes out to steer past */
#define GM_SMOOTH_WEIGHT 0.15f    /* on (speed x change of commanded heading)^2 */
#define GM_AIM_WEIGHT 0.3f
#define GM_YAW_GAIN 1.5f          /* 1/s, heading error to yaw rate */
#define GM_PIVOT_ANGLE 1.75f      /* rad; beyond this the hull turns before it drives */
#define GM_SWING_GROW 0.3f        /* m kept between a turning hull's swept corners and a hull nearby */
#define GM_PIVOT_CREEP 3.0f       /* m a hull creeps forward in the game per radian it turns "on the spot":
                                   * 1.5 to 2 m in an ordinary turn, 8 m at most in a turn right round */
#define GM_CREEP_TURN 0.3f        /* rad of turn from which the steering is hard enough for that creep */
#define GM_CRAWL_SPEED 0.5f       /* m/s; creeping up on a hull in its way slower than this is waiting for it */
#define GM_MOVE_MIN 0.3f          /* m/s; a speed under the tracker's rest band (GM_CONTROL_REST_MPS) is no drive */
#define GM_TURN_LOOK 0.6f         /* rad of a turn that are checked ahead; the rest when it gets there */
#define GM_SWING_REACH 14.0f      /* m beyond touching within which a hull matters to a turn */
/* Static safety. */
#define GM_SWEEP_MARGIN 0.05f     /* hull growth of the static tests: the obstacle cells are whole squares already */
#define GM_FAST_MARGIN 0.3f       /* m of room beside the hull that it needs at speed: it holds its line to a few decimetres */
#define GM_NARROW_SPEED 0.8f      /* m/s with less room than that: at this speed touching something is no crash */
#define GM_PIVOT_MARGIN 0.4f      /* further growth for a turn on the spot, which creeps in the game */
#define GM_STATIC_GAP 0.6f        /* m left in front of the hull when it stops for something static */
#define GM_LOOK_MAX 60.0f         /* m the static sweep reaches ahead at most */
#define GM_PUSH_SPEED 4.0f        /* m/s towards something the game's sweep objects to and the scan does not call solid */
#define GM_BRAKE_LAG_S 0.5f       /* s before the brakes bite, counted into every static stopping distance */
#define GM_SPEED_MAX 18.0f        /* m/s; a speed reading beyond this is a glitch */
/* Backing off. */
#define GM_HARD_WAIT_MS 3000u     /* able neither to go on nor to turn for this long */
#define GM_STALL_MS 3000u         /* ordered forward for this long without moving: wedged */
#define GM_JAM_MS 6000u           /* waiting this long, without progress, on a hull that itself stands */
#define GM_YIELD_MS 8000u         /* after backing off for a hull, that hull is let through for this long */
#define GM_YIELD_RANGE 16.0f      /* m; farther than that it has passed */
#define GM_STALL_MOVE 0.5f        /* m that count as moving */
#define GM_STALL_TURN 0.15f       /* rad that count as turning */
/* 20261115 (Eastern Europe, seat 8): pushing at full throttle against something the scan does not show (a
 * stump, rock or debris under 0.5 m), it crept 0.05-0.25 m/s for 50 s; each 0.5 m of creep or a little
 * steering restarted the wedge clock, so it never counted as wedged, never reported the spot and never
 * backed off for long. The wedge clock now restarts only once the hull has really got away. */
#define GM_WEDGE_MOVE 1.5f        /* m from where the push began */
#define GM_WEDGE_TURN 0.6f        /* rad turned since then */
#define GM_WEDGE_MS 1000u         /* 20261115: ordered forward, under 0.5 m/s and not away for 1 s: wedged (was 3 s) */
#define GM_DISABLED_MS 20000u     /* a hull that neither drives nor backs off is left alone this long */
#define GM_RETAIN_MS 5000u        /* state kept for a vehicle missing from the snapshot */
#define GM_ESCAPE_BACK 3.0f       /* m reversed */
#define GM_ESCAPE_STEP 1.5f       /* m of room behind it needs to go on reversing */
#define GM_ESCAPE_MS 4500u        /* at most this long */
#define GM_ESCAPE_SPEED 2.0f
#define GM_ESCAPE_REPEAT_MS 30000u /* not again at the same place before this */
#define GM_ESCAPE_SAME_PLACE 6.0f
#define GM_UNKNOWN_HOLD_MS 3500u  /* wait this long for geometry evidence, then creep */
#define GM_UNKNOWN_SPEED 4.0f
/* Route following. */
#define GM_JOIN_RADIUS 20.0f      /* an upstream waypoint this close counts as passed */
#define GM_JOIN_CORRIDOR 40.0f
#define GM_LANE_SPREAD 18.0f      /* largest lateral track offset from the upstream line */
#define GM_TRACK_GAP 1.5f         /* m between the hull sides of neighbouring tracks */
#define GM_TRACK_SAME_LINE 60.0f  /* reference points this close belong to the same upstream line */
#define GM_VERTEX_CAPTURE 4.0f
#define GM_TIGHT_CAPTURE 1.0f
#define GM_TIGHT_SPEED 2.5f
#define GM_ROUTE_BAND 1.5f        /* m from a nav leg within which the leg's own validation holds */
#define GM_LOST_ROUTE 25.0f       /* m from the followed leg: ask for a new route */
#define GM_DECEL_FRACTION 0.6f    /* of the braking limit, for planned slow-downs */
#define GM_NARROW_LOOK 80.0f
#define GM_SPAWN_FIELD 12.0f      /* m from a pending spawn point: still in the spawn area */
#define GM_SPAWN_SPEED 6.0f
#define GM_TURN_FRACTION 0.6f     /* of the reported yaw rate, counted on when sizing an arc */
/* Route layer. */
#define GM_NAV_MARGIN 0.45f       /* no route through a gap the hull cannot be steered through: it tracks to a few decimetres */
#define GM_NAV_COMFORT 1.2f
#define GM_NAV_PIVOT_SLACK 1.5f
#define GM_NAV_BODY_CLEARANCE 1.5f
#define GM_BUILD_BUDGET 40000
#define GM_STEADY_BUDGET 12000
#define GM_NAV_SLICE 1500         /* units between looks at the clock */
#define GM_UNREACHABLE_RETRY_MS 10000u
#define GM_UNREACHABLE_MOVING_MS 3000u /* sooner while it is moving: the start pose has changed */
#define GM_UNREACHABLE_MAX_MS 40000u   /* each failed search doubles the wait, up to this */
#define GM_GOAL_SNAP 40.0f             /* m around a goal that lies in an obstacle to look for standing room */
#define GM_REPLAN_COOLDOWN_MS 5000u
#define GM_STUCK_MS 20000u        /* held this long by a standing hull that is itself waiting: go around it */
#define GM_BODY_MS 20000u
#define GM_REVERSE_AFTER_MS 1500u
/* Capture slots. */
#define GM_BIND_RANGE 60.0f
#define GM_SLOT_INSET 1.5f
#define GM_SLOT_GOAL_RADIUS 1.0f
#define GM_SLOT_GAP 0.6f
#define GM_PARK_RADIUS 2.5f
#define GM_PARK_HYSTERESIS 1.5f
#define GM_PARK_WAIT_MS 2000u
#define GM_RIM_PARK 12.0f         /* m outside a capture region where a vehicle held by parked hulls stays */
#define GM_PARK_NEAR 5.0f         /* m from a slot or goal that will do when the hull would have to turn for the rest */
#define GM_SETTLE_MS 15000u       /* without progress this long near the goal: it has arrived as far as it will */
#define GM_RESLOT_MS 6000u        /* held up outside its region this long: another slot, before it parks out there */
#define GM_SETTLE_RANGE 20.0f
#define GM_STOP_RADIUS 1.5f       /* a vehicle stops this close to its goal */
/* Having arrived it stays arrived until pushed or slid this far from the goal.
 * With 2 m a hull that came in at speed overshot, counted as gone again,
 * turned round, came back facing the other way and overshot again: the
 * orbits at the end of the short flank-lane routes (Poland seats 5, 10). */
#define GM_ARRIVE_SLIP 8.0f
/* s24 path mode (snapshot path_mode; see gm_path_follow / gm_path_decide). Controller settings and
 * geometry, not a vehicle model. */
#define GM_PATH_STOP_MAX 60.0f       /* m the stop distance is marched along the present heading (s23 LP_STOP_MAX) */
#define GM_PATH_STOP_FINE 20.0f      /* m marched in LP_CHECK_M steps, 1 m steps beyond (s23 LP_LINE_MAX) */
#define GM_PATH_STOP_POSES 120       /* its own pose budget (s23 LP_STOP_POSES) */
#define GM_PATH_SWEEP_MAX 80.0f      /* m: the aim moves on early only to a vertex this close */
#define GM_PATH_TIGHT_CAPTURE 1.5f   /* m: a tight or reverse vertex counts as reached (more than the driver's 1 m
                                      * stop margin, or a stop at the vertex would end just outside it) */
#define GM_PATH_ADVANCE_MAX 8        /* vertices passed early per snapshot at most */
#define GM_PATH_TURN_STOP 0.5236f    /* rad (30 deg): the route turning more than this at the aim is a stop there */
#define GM_PATH_E_DRIVE 0.5236f      /* rad (30 deg): beyond this the driver turns on the spot (ground_drive.inc) */
#define GM_PATH_E_SETTLE 0.1396f     /* rad (8 deg): it turns on the spot until under this (ground_drive.inc) */
#define GM_PATH_NO_ROOM 1.0f         /* m: at or under this stop distance the driver turns on the spot (ground_drive.inc) */
#define GM_PATH_PIVOT_R 2.4f         /* m: the hull centre's path per radian of an A / D pivot at rest (game: 2.0-2.8) */
#define GM_PATH_PIVOT_STEP 0.1f      /* rad between checks of the pivot arc */
#define GM_PATH_BLOCKED_M 2.0f       /* avoidance: the stop distance under this ... */
#define GM_PATH_BLOCKED_MS 2000u     /* ... for this long while it is ordered to go: one decision */
#define GM_PATH_OFF_ROUTE 10.0f      /* m from the followed route (and no free straight line to the aim): new route */
#define GM_PATH_BACK_M 3.0f          /* the back-up: straight back this far */
#define GM_PATH_BACK_MS 6000u        /* or for this long at most */
#define GM_PATH_BACK_MIN 1.0f        /* m of room behind that a back-up needs */
#define GM_PATH_BACK_AGAIN_MS 30000u /* not again within GM_PATH_BACK_SAME m of the last one before this */
#define GM_PATH_BACK_SAME 6.0f
#define GM_PATH_RETRY_MS 4000u       /* no way round: wait, and ask again after this */
#ifndef GM_PATH_SHARE
#define GM_PATH_SHARE 0.0f           /* spread off (0.2 = x(1 + 0.2 k) on cells k teammates' routes cover, ground_nav
                                      * share): on Sweden's real map it gave no clearer spread of exits, and the first
                                      * routes of 30 tanks took 9.5 s instead of 4.1 s (tests\run_path_real.py) */
#endif
enum { GM_PA_IDLE, GM_PA_REROUTE, GM_PA_WAIT, GM_PA_BACK };
enum { GM_PATH_R_NONE, GM_PATH_R_FIRST, GM_PATH_R_TARGET, GM_PATH_R_OFF, GM_PATH_R_HULL, GM_PATH_R_STATIC,
       GM_PATH_R_RETRY, GM_PATH_R_SLOT };

/* One ground_plan.log line (snapshot local_log, kind 2), stamped with the snapshot clock. */
static void gm_path_log(const gm_snapshot_t *s, const char *fmt, ...)
{
    if (!s->local_log)
        return;
    char line[320];
    int n = snprintf(line, sizeof line, "%llu ", (unsigned long long)s->ms);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + n, sizeof line - (size_t)n, fmt, ap);
    va_end(ap);
    s->local_log(s->local_context, 2, line);
}

/* One neighbour as seen from a vehicle: both hulls as boxes on their four
 * edge directions, once bare and once with the distance to be kept. */
typedef struct {
    gm_point_t axis[4], vel;
    float hull_offset[4], hull_reach[4]; /* neighbour's centre along each axis; summed half-extents */
    float keep_offset[4], keep_reach[4]; /* the same with the keep-out distance and the headway */
    int touching_axis, inside_axis;      /* axis of least overlap now, -1 if apart */
    int rear;                            /* it follows this vehicle: only a turn has to mind it */
    float give_way_s;                    /* this vehicle yields: it reacts this much earlier */
    uint64_t id;
    int yields; /* the other hull is itself moving and avoiding: impatience may ease past it */
    int standing; /* it stands: what a turn sweeps is then checked against its hull exactly */
} gm_pair_t;

typedef struct {
    const gm_vehicle_t *v;
    gm_commitment_t c;
    gm_point_t p, vel, fwd, goal, target, pref;
    float speed, radius, half_length, half_width, v_des, remaining, cross;
    int region, mobile, aligning, stationary, body, requested;
    int following, tight, reverse_leg, cursor, rejoined, straight, among, pref_blocked;
    int no_turn_room;      /* turning to the preferred heading here would swing the hull into an obstacle cell */
    int turn_boxed;        /* it stands and needs a real turn that a hull close by leaves no room for */
    int firm;              /* the speed ordered is all the room ahead allows: to be braked down to, not coasted to */
    int approach;          /* the speed is what it takes to stop on the goal: braked down to as well */
    int disabled;          /* immobile: it is not commanded and counts as a standing body */
    float hold_arc;        /* single-lane hold: stop within this route distance, < 0 none */
    uint64_t hold_blocker;
    /* Decision. */
    float cmd_speed, cmd_yaw, cmd_heading;
    enum gm_state state;
    enum gm_geometry geometry;
    uint64_t blocker;
    int reversing;
    /* s13 local planner. */
    gm_point_t aim;        /* the point on the route gm_follow steers at (before the spawn rule) */
    int lp_moving;         /* the local planner commanded a motion */
    float cmd_curvature;   /* rad per signed metre */
    int sq_hold;           /* held in the spawn exit queue by sq_blocker */
    uint64_t sq_blocker;
    int ex_mode;           /* s16 exit mode this snapshot (gm_spawn_queue) */
    /* s24 path mode (gm_path_follow, gm_path_decide). path_free: free length of the hull's straight sweep along
     * its present heading (backwards on a reverse leg), path_blocker the hull that ended it (0 static or none),
     * path_blocker_moving whether that hull moves; path_d: the stop distance published; path_back: free length
     * straight back; path_e: heading error to the aim (rad, from the tail on a reverse leg); path_turn: it has
     * to turn on the spot (|e| >= 8 deg and |e| > 30 deg or no room ahead), path_turn_room: the forward-gear
     * pivot's arc to the aim heading is free; path_go / path_rev: the driver may go / on a reverse leg;
     * path_cap: speed cap (tight nav leg), INFINITY none. */
    float path_free, path_d, path_back, path_e, path_cap;
    uint64_t path_blocker;
    int path_blocker_moving, path_turn, path_turn_room, path_go, path_rev;
    /* s26 library mode (ground_motion_library.inc): lib = library mode this snapshot; the curve's source, library
     * route / sample at the projection, cross-track error and stored clearance there, the stop-distance components
     * (vehicle, turn, end), a start turn in progress and its gear, the curve's revision. */
    int lib, lib_source, lib_route, lib_sample, lib_pivot_rev, lib_turning, lib_pivot_blocked;
    uint64_t lib_pivot_blocker; /* a turn on the spot that would sweep into this hull */
    int lib_will_pivot;         /* s27: the driver will turn on the spot (gl_follow) ... */
    float lib_pivot_e;          /* ... towards this heading error (rad) */
    unsigned lib_rev;
    float lib_cross, lib_clear, stop_vehicle, stop_turn, stop_end;
    int lib_align;              /* s29: this snapshot's align was planned by gl_align (turn-room rule) */
    int lib_street;             /* s34: street closed and no other street found (published as street_closed) */
} gm_agent_t;

static gm_agent_t gm_agents[GM_MAX_VEHICLES];

static float gm_clamp(float v, float lo, float hi)
{
    return fminf(hi, fmaxf(lo, v));
}

static float gm_angle(float a)
{
    return remainderf(a, 2.0f * GM_PI);
}

static gm_point_t gm_sub(gm_point_t a, gm_point_t b)
{
    return (gm_point_t){a.x - b.x, a.z - b.z};
}

static gm_point_t gm_add(gm_point_t a, gm_point_t b)
{
    return (gm_point_t){a.x + b.x, a.z + b.z};
}

static gm_point_t gm_scale(gm_point_t a, float k)
{
    return (gm_point_t){a.x * k, a.z * k};
}

static float gm_dot(gm_point_t a, gm_point_t b)
{
    return a.x * b.x + a.z * b.z;
}

static float gm_det(gm_point_t a, gm_point_t b)
{
    return a.x * b.z - a.z * b.x;
}

static float gm_len(gm_point_t a)
{
    return hypotf(a.x, a.z);
}

static float gm_dist(gm_point_t a, gm_point_t b)
{
    return hypotf(a.x - b.x, a.z - b.z);
}

static gm_point_t gm_unit(gm_point_t a)
{
    const float len = gm_len(a);
    return len > 1e-5f ? gm_scale(a, 1.0f / len) : (gm_point_t){1.0f, 0.0f};
}

static gm_point_t gm_axis(float heading)
{
    return (gm_point_t){cosf(heading), sinf(heading)};
}

static float gm_segment_distance(gm_point_t p, gm_point_t a, gm_point_t b)
{
    const gm_point_t ab = gm_sub(b, a);
    const float len2 = gm_dot(ab, ab);
    const float t = len2 > 1e-6f ? gm_clamp(gm_dot(gm_sub(p, a), ab) / len2, 0.0f, 1.0f) : 0.0f;
    return gm_dist(p, gm_add(a, gm_scale(ab, t)));
}

/* Whole-hull radius, for coarse distances (hulls ahead of a turn on the spot,
 * single-lane stretches, slot spacing). Avoidance and static safety work on
 * the hull's own rectangle. */
static float gm_disc_radius(const gm_vehicle_t *v)
{
    return hypotf(0.5f * v->length, 0.5f * v->width) + GM_DISC_MARGIN;
}

static int gm_valid_vehicle(const gm_vehicle_t *v)
{
    if (!v->id || !isfinite(v->x) || !isfinite(v->y) || !isfinite(v->z) || !isfinite(v->heading)
        || !isfinite(v->vx) || !isfinite(v->vz) || !isfinite(v->yaw_rate)
        || !isfinite(v->length) || v->length <= 0.0f || !isfinite(v->width) || v->width <= 0.0f
        || !isfinite(v->hull_forward) || !isfinite(v->hull_right))
        return 0;
    if (!v->controlled)
        return 1;
    if (!isfinite(v->reverse_speed) || v->reverse_speed < 0.0f
        || (v->goal_region_id && (!isfinite(v->goal_region_center.x) || !isfinite(v->goal_region_center.z)
                              || !isfinite(v->goal_region_radius) || v->goal_region_radius < 0.0f))
        || (v->align && !isfinite(v->align_heading))
        || !isfinite(v->cruise_speed) || v->cruise_speed <= 0.0f
        || !isfinite(v->acceleration) || v->acceleration <= 0.0f
        || !isfinite(v->braking) || v->braking <= 0.0f
        || !isfinite(v->max_yaw_rate) || v->max_yaw_rate <= 0.0f
        || !isfinite(v->arrival_radius) || v->arrival_radius <= 0.0f
        || !isfinite(v->goal.x) || !isfinite(v->goal.z)
        || v->path_count < 0 || v->path_count > GM_MAX_ROUTE_POINTS || (v->path_count && !v->path))
        return 0;
    for (int i = 0; i < v->path_count; ++i)
        if (!isfinite(v->path[i].x) || !isfinite(v->path[i].z))
            return 0;
    return 1;
}

/* Keyed by (id, life). A new route_id keeps the route, the lane offset and
 * the capture claim; it only clears task-scoped flags. */
static void gm_lookup(const gm_state_t *state, const gm_vehicle_t *v, gm_commitment_t *out)
{
    for (int i = 0; i < state->count; ++i) {
        const gm_commitment_t *c = &state->commitments[i];
        if (c->id == v->id && c->life_id == v->life_id) {
            *out = *c;
            if (c->route_id != v->route_id) {
                out->route_id = v->route_id;
                out->arrived = out->arrived_short = 0;
            }
            return;
        }
    }
    memset(out, 0, sizeof *out);
    out->id = v->id;
    out->life_id = v->life_id;
    out->route_id = v->route_id;
    out->via_sent_count = -1;
    out->birth_heading = v->heading;
    out->birth_valid = 1;
}

static int gm_region_goal(const gm_vehicle_t *v)
{
    return v->controlled && v->move && v->goal_region_id && v->goal_region_radius > 0.0f
        && gm_dist(v->goal_region_center, v->goal) <= v->goal_region_radius;
}

static uint64_t gm_route_key(const gm_vehicle_t *v, uint64_t counter)
{
    const uint64_t key = v->id * 0x9E3779B97F4A7C15ull ^ (v->life_id + 0x632BE59BD9B4E019ull) * 0xBF58476D1CE4E5B9ull
        ^ counter * 0x94D049BB133111EBull;
    return key ? key : 1u;
}

static void gm_new_key(gm_commitment_t *c, const gm_vehicle_t *v)
{
    ++c->key_counter;
    c->route_key = gm_route_key(v, c->key_counter);
    c->replan = 0;
    c->replanned_body = 0;
    c->via_sent_count = -1; /* refreshed with the request */
}

static int gm_agent_index(const gm_agent_t *agents, int count, uint64_t id)
{
    for (int i = 0; i < count; ++i)
        if (agents[i].v->id == id)
            return i;
    return -1;
}

/* ---- Upstream path: what is still ahead, and this vehicle's own track. ---- */

/* Leading upstream waypoints that are behind the vehicle. A waypoint is
 * passed when the vehicle is near it or beyond it in the travel direction;
 * it is never steered back to. Monotone while the window is unchanged. */
static int gm_upstream_cursor(const gm_vehicle_t *v, gm_commitment_t *c, gm_point_t p)
{
    const int n = v->path_count;
    if (!n) {
        c->window_valid = 0;
        c->cursor = 0;
        return 0;
    }
    int k = c->window_valid && gm_dist(c->first_waypoint, v->path[0]) < 0.05f ? c->cursor : 0;
    k = k > n ? n : k;
    while (k < n) {
        const gm_point_t w = v->path[k];
        if (k == n - 1 && gm_dist(w, v->goal) < 0.05f)
            break; /* the last point is the goal itself */
        const gm_point_t rel = gm_sub(p, w);
        int passed = gm_len(rel) < GM_JOIN_RADIUS;
        if (!passed && k == 0) {
            const gm_point_t next = n > 1 ? v->path[1] : v->goal;
            passed = gm_dot(rel, gm_sub(next, w)) > 0.0f && gm_segment_distance(p, w, next) < GM_JOIN_CORRIDOR;
        } else if (!passed) {
            passed = gm_dot(rel, gm_sub(w, v->path[k - 1])) > 0.0f;
        }
        if (!passed)
            break;
        ++k;
    }
    c->first_waypoint = v->path[0];
    c->window_valid = 1;
    c->cursor = k;
    return k;
}

static gm_point_t gm_path_direction(const gm_vehicle_t *v, int k)
{
    const int n = v->path_count;
    if (n < 2)
        return gm_unit(gm_sub(v->goal, v->path[0]));
    return gm_unit(gm_sub(v->path[k < n - 1 ? k + 1 : k], v->path[k > 0 ? k - 1 : k]));
}

/* This vehicle's lateral track beside the upstream line, chosen once and kept
 * for life. It starts from where the vehicle is when it first sees a path, so
 * vehicles born side by side drive side by side instead of converging on the
 * line; and of the tracks next to that one it takes the least used by the
 * others on the same line, so those born one behind the other end up
 * staggered left and right rather than nose to tail. Where the ground does
 * not allow it, the route layer skips the shifted waypoint. */
static void gm_lane_offset(const gm_state_t *state, const gm_vehicle_t *v, gm_commitment_t *c, gm_point_t p, int k)
{
    if (c->lane_offset_valid || v->path_count < 2)
        return;
    const int j = k < v->path_count ? k : v->path_count - 1;
    const gm_point_t dir = gm_path_direction(v, j), ref = v->path[j];
    const float own = gm_clamp(gm_det(dir, gm_sub(p, ref)), -GM_LANE_SPREAD, GM_LANE_SPREAD);
    const float track = v->width + GM_TRACK_GAP;
    float best = own, best_shift = 0.0f;
    int best_used = -1;
    for (int m = 0; m < 3; ++m) {
        const float shift = m == 0 ? 0.0f : m == 1 ? track : -track;
        const float offset = own + shift;
        if (fabsf(offset) > GM_LANE_SPREAD + track)
            continue;
        int used = 0;
        for (int q = 0; q < state->count; ++q) {
            const gm_commitment_t *o = &state->commitments[q];
            if (o->id == c->id || !o->lane_offset_valid
                || gm_dist(o->lane_ref, ref) > GM_TRACK_SAME_LINE || gm_dot(o->lane_dir, dir) < 0.8f)
                continue;
            /* Their track, measured from this vehicle's reference line. */
            const float theirs = o->lane_offset + gm_det(dir, gm_sub(o->lane_ref, ref));
            used += fabsf(theirs - offset) < 0.5f * track;
        }
        if (best_used < 0 || used < best_used || (used == best_used && fabsf(shift) < fabsf(best_shift))) {
            best = offset;
            best_shift = shift;
            best_used = used;
        }
    }
    c->lane_offset = best;
    c->lane_ref = ref;
    c->lane_dir = dir;
    c->lane_offset_valid = 1;
}

static gm_point_t gm_shifted_waypoint(const gm_nav_t *nav, const gm_vehicle_t *v, const gm_commitment_t *c, int k)
{
    gm_point_t point = v->path[k];
    if (c->lane_offset_valid && v->path_count >= 2) {
        const gm_point_t dir = gm_path_direction(v, k);
        float offset = c->lane_offset;
        if (nav) {
            /* Only as far to the side as the ground there is open: in a
             * street the track is the street. */
            const float room = fmaxf(0.0f, gm_nav_clearance(nav, point) - (0.5f * v->width + 1.0f));
            offset = gm_clamp(offset, -room, room);
        }
        point.x -= dir.z * offset;
        point.z += dir.x * offset;
    }
    return point;
}

/* Via points for nav: upstream waypoints still ahead, on this vehicle's
 * track. The goal itself and points inside a capture region are not via points. */
static void gm_capture_via(const gm_nav_t *nav, gm_agent_t *a)
{
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    c->via_sent_count = 0;
    for (int k = a->cursor; k < v->path_count && c->via_sent_count < GM_MAX_ROUTE_POINTS; ++k) {
        if (k == v->path_count - 1 && gm_dist(v->path[k], v->goal) < 0.05f)
            break;
        if (a->region && gm_dist(v->path[k], v->goal_region_center) < v->goal_region_radius)
            break;
        c->via_sent[c->via_sent_count++] = gm_shifted_waypoint(nav, v, c, k);
    }
}

/* Where to send a vehicle whose goal lies in or against an obstacle: the
 * nearest spot with standing room for its hull. A goal like that has no route
 * and would cost a full search every time it is asked for. */
static gm_point_t gm_standing_room(const gm_nav_t *nav, const gm_agent_t *a, gm_point_t goal)
{
    const float need = a->half_width + 0.8f;
    if (!nav)
        return goal;
    /* A goal in a region nobody can drive into from here (a pond, a yard
     * ringed by walls, a sheet of hits ringed by solid cells) is moved to the
     * nearest point that can be reached: otherwise the search for it runs to
     * its cap and the vehicle stands at the edge. The point taken has room
     * for the hull whichever way it stands: the route has to end there. */
    gm_point_t reach;
    if (gm_nav_reach(nav, a->p, goal, hypotf(a->half_length, a->half_width) + 0.5f, &reach) == 2)
        return reach;
    if (gm_nav_clearance(nav, goal) >= need)
        return goal;
    for (float ring = 2.0f; ring <= GM_GOAL_SNAP; ring += 2.0f) {
        gm_point_t best = goal;
        float best_d = INFINITY;
        const int n = (int)fmaxf(8.0f, ceilf(2.0f * GM_PI * ring / 2.0f));
        for (int k = 0; k < n; ++k) {
            const float angle = 2.0f * GM_PI * (float)k / (float)n;
            const gm_point_t q = {goal.x + ring * cosf(angle), goal.z + ring * sinf(angle)};
            /* Of the free spots on this ring the one nearest the vehicle. */
            if (gm_nav_clearance(nav, q) >= need && gm_dist(q, a->p) < best_d) {
                best_d = gm_dist(q, a->p);
                best = q;
            }
        }
        if (isfinite(best_d))
            return best;
    }
    return goal;
}

/* ---- Capture slots (late binding through nav's multi-goal search). ---- */

static float gm_slot_spacing(const gm_snapshot_t *s, const gm_agent_t *agents, int index)
{
    /* Far enough apart that a hull can drive up to its slot past, or
     * behind, one already parked on the next. */
    const gm_vehicle_t *v = agents[index].v;
    float disc = gm_disc_radius(v), length = v->length;
    for (int j = 0; j < s->count; ++j) {
        const gm_vehicle_t *peer = agents[j].v;
        if (gm_region_goal(peer) && peer->goal_region_id == v->goal_region_id) {
            disc = fmaxf(disc, gm_disc_radius(peer));
            length = fmaxf(length, peer->length);
        }
    }
    return fmaxf(2.0f * disc, length + GM_FOLLOW_GAP) + GM_SLOT_GAP;
}

/* One lattice for the whole region, minus other claims, abandoned slots and
 * points near stationary hulls; the deepest slots along the approach are
 * cheapest, so early arrivals park at the far side. */
static void gm_slot_candidates(const gm_nav_t *nav, const gm_snapshot_t *s, const gm_agent_t *agents, int index,
                               gm_commitment_t *c)
{
    const gm_vehicle_t *v = agents[index].v;
    const gm_point_t center = v->goal_region_center;
    const float spacing = gm_slot_spacing(s, agents, index);
    const float room = fmaxf(0.0f, v->goal_region_radius - GM_SLOT_INSET);
    gm_point_t approach = gm_sub(center, agents[index].p);
    approach = gm_len(approach) > 0.5f ? gm_unit(approach) : agents[index].fwd;
    const int limit = (int)gm_clamp(ceilf(room / spacing), 0.0f, 40.0f);
    c->slot_count = 0;
    for (int zi = -limit; zi <= limit; ++zi) {
        for (int xi = -limit; xi <= limit; ++xi) {
            const gm_point_t offset = {(float)xi * spacing, (float)zi * spacing};
            if (gm_len(offset) > room)
                continue;
            const gm_point_t slot = gm_add(center, offset);
            /* Not in or against an obstacle, nor where it cannot be reached
             * from here: such a slot has no route. */
            const float need = agents[index].half_width + 0.8f;
            int free = !nav || (gm_nav_clearance(nav, slot) >= need
                                && gm_nav_reach(nav, agents[index].p, slot, need, NULL) == 1);
            for (int j = 0; free && j < s->count; ++j) {
                if (j == index)
                    continue;
                if (agents[j].c.slot_valid && gm_dist(agents[j].c.slot, slot) < spacing - 0.01f)
                    free = 0;
                else if (agents[j].stationary && gm_dist(agents[j].p, slot) < spacing - 0.01f)
                    free = 0;
            }
            for (int k = 0; free && k < c->abandoned_count; ++k)
                if (gm_dist(c->abandoned[k], slot) < 0.5f)
                    free = 0;
            if (!free)
                continue;
            const float cost = 2.0f * (room - gm_dot(offset, approach));
            int at = c->slot_count < GM_SLOT_GOALS ? c->slot_count++ : GM_SLOT_GOALS;
            if (at == GM_SLOT_GOALS) {
                if (cost >= c->slot_cost[GM_SLOT_GOALS - 1])
                    continue;
                at = GM_SLOT_GOALS - 1;
            }
            while (at > 0 && c->slot_cost[at - 1] > cost) {
                c->slots[at] = c->slots[at - 1];
                c->slot_cost[at] = c->slot_cost[at - 1];
                --at;
            }
            c->slots[at] = slot;
            c->slot_cost[at] = cost;
        }
    }
}

static int gm_slot_taken(const gm_snapshot_t *s, const gm_agent_t *agents, int index, gm_point_t slot)
{
    const float spacing = gm_slot_spacing(s, agents, index);
    for (int j = 0; j < s->count; ++j)
        if (j != index && agents[j].c.slot_valid && gm_dist(agents[j].c.slot, slot) < spacing - 0.01f)
            return 1;
    return 0;
}

static void gm_abandon_slot(gm_commitment_t *c, gm_point_t slot)
{
    if (c->abandoned_count < GM_ABANDONED_SLOTS)
        c->abandoned[c->abandoned_count++] = slot;
    else
        c->abandoned[GM_ABANDONED_SLOTS - 1] = slot;
}

/* ---- Route layer. ---- */

static int gl_wants_nav(const gm_agent_t *a); /* s26 library mode: ground_motion_library.inc */

static void gm_adopt(gm_commitment_t *c, const gm_nav_route_t *route)
{
    c->followed_key = route->route_key;
    c->nav_revision = route->revision;
    c->nav_count = route->count < GM_ROUTE_COPY ? route->count : GM_ROUTE_COPY;
    c->nav_vertex = 0;
    c->nav_origin = route->origin;
    c->unreachable_tries = 0;
    memcpy(c->nav_points, route->points, (size_t)c->nav_count * sizeof c->nav_points[0]);
    memcpy(c->nav_flags, route->flags, (size_t)c->nav_count * sizeof c->nav_flags[0]);
}

static void gm_route_layer(gm_state_t *state, const gm_snapshot_t *s, gm_agent_t *agents, int routed)
{
    gm_nav_t *nav = state->nav;
    static gm_nav_body_t bodies[GM_MAX_VEHICLES];
    int body_count = 0;
    /* Route obstacles: hulls that will not move by themselves, and the
     * stationary neighbour a stuck vehicle has decided to go around. */
    for (int i = 0; i < s->count; ++i) {
        gm_agent_t *a = &agents[i];
        a->body = a->stationary && (!a->v->controlled || !a->v->move || a->c.parked || a->c.arrived || a->disabled
                                    || a->c.route_status == GM_NAV_UNREACHABLE);
    }
    for (int i = 0; i < s->count; ++i) {
        const gm_commitment_t *c = &agents[i].c;
        if (c->body_id && s->ms < c->body_until_ms) {
            const int j = gm_agent_index(agents, s->count, c->body_id);
            if (j >= 0 && agents[j].stationary)
                agents[j].body = 1;
        }
    }
    for (int i = 0; i < s->count && body_count < GM_NAV_MAX_BODIES; ++i) {
        const gm_vehicle_t *v = agents[i].v;
        if (agents[i].body)
            bodies[body_count++] = (gm_nav_body_t){.id = v->id,
                .pose = {v->x, v->z, v->heading, 0.0f, 0.0f},
                .half_length = 0.5f * v->length, .half_width = 0.5f * v->width,
                .hull_forward = v->hull_forward, .hull_right = v->hull_right};
    }
    for (int i = 0; i < s->count; ++i) {
        gm_agent_t *a = &agents[i];
        a->requested = routed && a->v->controlled && a->v->move && !a->c.parked;
        if (s->path_mode && s->routes_lib && !gl_wants_nav(a))
            a->requested = 0; /* s26 library mode: the run-time search only for a tank on the NAV fallback */
    }
    if (!routed) {
        for (int i = 0; i < s->count; ++i) {
            agents[i].c.route_status = GM_NAV_NO_MAP;
            agents[i].c.nav_count = 0;
            agents[i].c.followed_key = 0;
        }
        return;
    }
    /* A route outlives a pause: a vehicle that stops to shoot, or drops out of
     * the snapshot for a moment, finds it again. It goes with the life, or
     * once the vehicle has been gone for good. Dropping it on every pause made
     * whole teams search again together, on the game's own thread. */
    for (int j = 0; j < state->count; ++j) {
        const gm_commitment_t *old = &state->commitments[j];
        const int k = gm_agent_index(agents, s->count, old->id);
        if (old->route_key && (k >= 0 ? agents[k].c.life_id != old->life_id : s->ms > old->seen_ms + GM_RETAIN_MS))
            gm_nav_forget(nav, old->id);
    }
    gm_nav_set_bodies(nav, bodies, body_count);

    int missing = 0;
    for (int i = 0; i < s->count; ++i) {
        gm_agent_t *a = &agents[i];
        const gm_vehicle_t *v = a->v;
        gm_commitment_t *c = &a->c;
        if (!a->requested)
            continue; /* no move order now, or parked: route and request stay for when it moves on */
        if (a->region) {
            if (c->key_region != v->goal_region_id || !c->route_key) {
                if (s->path_mode)
                    c->path_reason = c->route_key ? GM_PATH_R_TARGET : GM_PATH_R_FIRST;
                c->key_region = v->goal_region_id;
                c->slot_phase = c->slot_valid ? 2 : 0;
                c->unreachable_tries = 0;
                gm_new_key(c, v);
            }
            if (c->slot_phase == 0 && gm_dist(a->p, v->goal_region_center) <= v->goal_region_radius + GM_BIND_RANGE) {
                gm_slot_candidates(nav, s, agents, i, c);
                if (c->slot_count) {
                    c->slot_phase = 1;
                    gm_new_key(c, v);
                    if (s->path_mode)
                        c->path_reason = GM_PATH_R_SLOT;
                }
            }
        } else if (c->key_region || !c->route_key || gm_dist(v->goal, c->key_goal) > 2.0f) {
            if (s->path_mode)
                c->path_reason = c->route_key ? GM_PATH_R_TARGET : GM_PATH_R_FIRST;
            c->key_region = 0;
            c->key_goal = v->goal;
            c->route_goal = gm_standing_room(nav, a, v->goal);
            c->route_goal_moved = gm_dist(c->route_goal, v->goal) > 0.5f;
            c->slot_phase = 0;
            c->unreachable_tries = 0;
            gm_new_key(c, v);
        }
        if (s->path_mode)
            c->via_sent_count = 0; /* s24: from where it stands to the target, no via points */
        else if (c->via_sent_count < 0 || c->replan)
            gm_capture_via(nav, a);
        /* A route past a standing hull must leave what the avoidance layer
         * will insist on beside it, or the vehicle stops in front of it. */
        gm_nav_request_t request = {.vehicle = v, .route_key = c->route_key,
            .via = c->via_sent, .via_count = c->via_sent_count, .margin = GM_NAV_MARGIN,
            .body_clearance = fmaxf(GM_NAV_BODY_CLEARANCE, GM_SIDE_GAP + 0.3f),
            .comfort_margin = GM_NAV_COMFORT,
            .pivot_slack = GM_NAV_PIVOT_SLACK, .replan = c->replan,
            .group = s->path_mode ? v->group : 0, .share_cost = s->path_mode ? GM_PATH_SHARE : 0.0f};
        gm_point_t goal = v->goal;
        if (a->region && c->slot_phase == 1) {
            request.goals = c->slots;
            request.goal_cost = c->slot_cost;
            request.goal_count = c->slot_count;
            request.goal_radius = GM_SLOT_GOAL_RADIUS;
        } else if (a->region && c->slot_phase == 2) {
            request.goals = &c->slot;
            request.goal_count = 1;
            request.goal_radius = GM_SLOT_GOAL_RADIUS;
        } else if (a->region) {
            goal = v->goal_region_center;
            request.goals = &goal;
            request.goal_count = 1;
            request.goal_radius = 0.5f * v->goal_region_radius;
        } else {
            goal = c->route_goal;
            request.goals = &goal;
            request.goal_count = 1;
            request.goal_radius = fminf(1.0f, 0.3f * v->arrival_radius);
        }
        if (c->replan)
            c->replan_ms = s->ms;
        c->replan = 0;
        gm_nav_request(nav, &request);
        if (!c->nav_count && c->route_status != GM_NAV_UNREACHABLE)
            missing = 1;
    }
    /* The search runs on the obstacle cells alone, without the sweep callback:
     * what the game's sweep knows beyond them arrives as reported vetoes. It
     * goes in slices, and no snapshot spends long here: this is the game's own
     * thread, and a long search there costs every tank its plan. */
    {
        int left = missing ? GM_BUILD_BUDGET : GM_STEADY_BUDGET;
#if GM_NAV_TIME_CAP_US > 0
        const double deadline = gm_clock_us() + (double)GM_NAV_TIME_CAP_US;
#endif
        while (left > 0) {
            const int slice = left < GM_NAV_SLICE ? left : GM_NAV_SLICE;
            const int used = gm_nav_step(nav, NULL, NULL, slice);
            left -= slice;
            if (2 * used < slice)
                break; /* nothing left to search */
#if GM_NAV_TIME_CAP_US > 0
            if (gm_clock_us() >= deadline)
                break;
#endif
        }
    }

    for (int i = 0; i < s->count; ++i) {
        gm_agent_t *a = &agents[i];
        const gm_vehicle_t *v = a->v;
        gm_commitment_t *c = &a->c;
        if (!a->requested)
            continue;
        const gm_nav_route_t *route = gm_nav_route(nav, v->id);
        const int matching = route && route->route_key == c->route_key;
        c->route_status = matching ? (int)route->status : route ? GM_NAV_PENDING : GM_NAV_NONE;
        if (matching && route->status == GM_NAV_READY && route->count > 0
            && (c->followed_key != c->route_key || route->revision != c->nav_revision)) {
            gm_adopt(c, route);
            if (s->path_mode && c->path_logged != route->revision) {
                static const char *const why[] = {"extension", "first", "target moved", "off route", "blocked by seat",
                                                  "blocked static", "retry", "capture slot"};
                const gm_point_t end = route->points[route->count - 1];
                char by[32] = "";
                if (c->path_reason == GM_PATH_R_HULL)
                    snprintf(by, sizeof by, " %llu", (unsigned long long)c->pa_blocker);
                gm_path_log(s, "path: seat %llu route to (%.1f, %.1f) from (%.1f, %.1f): %d points, %.0f m, reason %s%s",
                            (unsigned long long)v->id, (double)end.x, (double)end.z, (double)route->origin.x,
                            (double)route->origin.z, route->count, (double)route->length,
                            why[c->path_reason >= 0 && c->path_reason <= GM_PATH_R_SLOT ? c->path_reason : 0], by);
                c->path_logged = route->revision;
                c->path_reason = GM_PATH_R_NONE;
            }
            if (a->region && c->slot_phase == 1 && route->goal_index >= 0 && route->goal_index < c->slot_count) {
                const gm_point_t slot = c->slots[route->goal_index];
                if (gm_slot_taken(s, agents, i, slot)) {
                    /* Claims are exclusive: someone bound it first. */
                    gm_abandon_slot(c, slot);
                    gm_slot_candidates(nav, s, agents, i, c);
                    gm_new_key(c, v);
                } else {
                    c->slot = slot;
                    c->slot_valid = 1;
                    c->slot_phase = 2;
                }
            }
        } else if (matching && route->status == GM_NAV_UNREACHABLE) {
            /* s24: a route of the same request that cannot be replaced (a replan: off route, blocked, retry)
             * stays the reference; the avoidance layer waits and asks again. */
            const int keep = s->path_mode && c->nav_count && c->followed_key == c->route_key;
            if (s->path_mode && c->path_reason != GM_PATH_R_NONE) {
                gm_path_log(s, "path: seat %llu no route to (%.1f, %.1f) from (%.1f, %.1f), reason %d%s",
                            (unsigned long long)v->id, (double)a->goal.x, (double)a->goal.z, (double)a->p.x,
                            (double)a->p.z, c->path_reason, keep ? ": keeps its old route" : "");
                c->path_reason = GM_PATH_R_NONE;
            }
            if (!keep) {
                c->nav_count = 0;
                c->followed_key = 0;
            }
            /* A search that finds nothing costs the most, and the next one
             * rarely finds more: each failure doubles the wait. */
            const int tries = c->unreachable_tries < 4 ? c->unreachable_tries : 4;
            const uint64_t wait = (uint64_t)(a->stationary ? GM_UNREACHABLE_RETRY_MS : GM_UNREACHABLE_MOVING_MS) << tries;
            if (a->region && c->slot_phase == 2) {
                gm_abandon_slot(c, c->slot);
                c->slot_valid = 0;
                c->slot_phase = 1;
                gm_slot_candidates(nav, s, agents, i, c);
                gm_new_key(c, v);
                c->replan_ms = s->ms;
            } else if (!keep && s->ms >= c->replan_ms + (wait < GM_UNREACHABLE_MAX_MS ? wait : GM_UNREACHABLE_MAX_MS)) {
                /* (s24: with the old route kept, the avoidance layer asks again: gm_path_decide.) */
                if (a->region && c->slot_phase == 1) {
                    gm_slot_candidates(nav, s, agents, i, c);
                    gm_new_key(c, v);
                } else {
                    c->replan = 1;
                }
                if (s->path_mode)
                    c->path_reason = GM_PATH_R_RETRY;
                c->unreachable_tries = tries + 1;
                c->replan_ms = s->ms;
            }
        }
    }
}

/* ---- Route following: where to aim and how fast. ---- */

static void gm_follow(const gm_snapshot_t *s, const gm_nav_t *nav, gm_agent_t *a)
{
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    const float decel = GM_DECEL_FRACTION * v->braking;
    const int reverse_allowed = v->reverse_speed > 0.0f && c->static_since_ms
        && s->ms - c->static_since_ms >= GM_REVERSE_AFTER_MS && s->ms >= c->no_reverse_until_ms;
    gm_point_t aim = a->goal;
    float corner_speed = v->cruise_speed, corner_dist = 0.0f;
    a->following = a->tight = a->reverse_leg = 0;
    a->target = a->goal;
    a->cross = 0.0f;

    while (c->nav_vertex < c->nav_count) {
        const int q = c->nav_vertex, last = q == c->nav_count - 1;
        const gm_point_t pt = c->nav_points[q], prev = q ? c->nav_points[q - 1] : c->nav_origin;
        const uint8_t flags = c->nav_flags[q];
        if ((flags & GM_NAV_REVERSE) && !reverse_allowed) {
            ++c->nav_vertex; /* reversing is the last resort of a statically boxed-in hull */
            continue;
        }
        const gm_point_t leg = gm_sub(pt, prev);
        const float capture = last || (flags & (GM_NAV_TIGHT | GM_NAV_REVERSE)) ? GM_TIGHT_CAPTURE : GM_VERTEX_CAPTURE;
        const int reached = gm_dist(a->p, pt) < capture;
        const int beyond = !last && !(flags & GM_NAV_REVERSE) && gm_dot(leg, leg) > 0.01f
            && gm_dot(gm_sub(a->p, pt), leg) > 0.0f;
        if (!reached && !beyond)
            break;
        ++c->nav_vertex;
    }

    if (c->nav_vertex < c->nav_count) {
        const int q = c->nav_vertex;
        const gm_point_t pt = c->nav_points[q], prev = q ? c->nav_points[q - 1] : c->nav_origin;
        const gm_point_t leg = gm_sub(pt, prev);
        const float len = gm_len(leg);
        a->following = 1;
        a->target = aim = pt;
        a->tight = (c->nav_flags[q] & GM_NAV_TIGHT) != 0;
        a->reverse_leg = (c->nav_flags[q] & GM_NAV_REVERSE) != 0;
        a->cross = len > 0.05f ? fabsf(gm_det(gm_scale(leg, 1.0f / len), gm_sub(a->p, prev))) : gm_dist(a->p, pt);
        a->remaining = gm_dist(a->p, pt) + gm_dist(c->nav_points[c->nav_count - 1], a->goal);
        for (int k = q + 1; k < c->nav_count; ++k)
            a->remaining += gm_dist(c->nav_points[k - 1], c->nav_points[k]);
        /* Aim a little ahead along the polyline so free corners are rounded;
         * a tight vertex is reached exactly. */
        float look = gm_clamp(1.2f * fabsf(a->speed) + 4.0f, 6.0f, 18.0f) - gm_dist(a->p, pt);
        for (int k = q; look > 0.0f && k + 1 < c->nav_count && !(c->nav_flags[k] & GM_NAV_TIGHT)
             && !(c->nav_flags[k + 1] & (GM_NAV_TIGHT | GM_NAV_REVERSE)); ++k) {
            const gm_point_t from = c->nav_points[k], to = c->nav_points[k + 1];
            const float seg = gm_dist(from, to);
            if (seg >= look) {
                aim = gm_add(from, gm_scale(gm_sub(to, from), look / fmaxf(seg, 0.01f)));
                look = 0.0f;
            } else {
                aim = to;
                look -= seg;
            }
        }
        if (q + 1 < c->nav_count && len > 0.05f) {
            const gm_point_t out = gm_sub(c->nav_points[q + 1], pt);
            const float turn = fabsf(gm_angle(atan2f(out.z, out.x) - atan2f(leg.z, leg.x)));
            corner_speed = a->tight && turn > 0.3f ? 0.5f
                : v->cruise_speed * fmaxf(0.25f, 1.0f - 0.75f * fminf(1.0f, turn / (0.5f * GM_PI)));
            corner_dist = gm_dist(a->p, pt);
        }
    } else {
        /* No route (yet): the first upstream waypoint ahead, on this
         * vehicle's own track, or the goal. */
        if (c->route_status != GM_NAV_READY && a->cursor < v->path_count
            && !(a->cursor == v->path_count - 1 && gm_dist(v->path[a->cursor], v->goal) < 0.05f))
            a->target = aim = gm_shifted_waypoint(nav, v, c, a->cursor);
        a->remaining = gm_dist(a->p, a->target) + gm_dist(a->target, a->goal);
    }

    float speed = v->cruise_speed;
    if (a->tight)
        speed = fminf(speed, GM_TIGHT_SPEED);
    /* Stopping on a point goal: counting the half second the brakes take to
     * bite (as every static stopping distance does), and braked down to at
     * once, not coasted to: hulls run faster than ordered and came in at
     * 7-10 m/s with 3 m to go, overshot and turned back. A capture slot needs
     * none of it: a hull parks where it comes to rest near its slot. */
    {
        const float room = fmaxf(0.0f, a->remaining - 0.5f);
        if (a->region) {
            speed = fminf(speed, sqrtf(2.0f * decel * room) + 0.3f);
        } else {
            const float lag = decel * GM_BRAKE_LAG_S;
            const float stop = sqrtf(lag * lag + 2.0f * decel * room) - lag + 0.3f;
            a->approach = stop < speed;
            speed = fminf(speed, stop);
        }
    }
    speed = fminf(speed, sqrtf(corner_speed * corner_speed + 2.0f * decel * corner_dist));
    gm_point_t to_aim = gm_sub(aim, a->p);
    /* Among spawn points where tanks are still being born: straight out the
     * way it was born facing, at a speed it can stop from, and turn after.
     * Crossing the other points is how newborn tanks get rammed. */
    int among = 0;
    for (int k = 0; k < s->pending_spawn_site_count && !among; ++k)
        among = gm_dist(a->p, s->pending_spawn_sites[k].p) < GM_SPAWN_FIELD;
    a->among = among;
    a->aim = aim;
    /* Unless that way is walled: then it steers like anywhere else. */
    a->straight = among && c->birth_valid && !a->tight
        && !(c->static_since_ms && s->ms - c->static_since_ms >= 1000u);
    if (s->local_planner && nav && a->straight) {
        /* s13: the straight-out rule drove Sweden's tanks (lab run 20261115_235404,
         * seats 1, 3, 13) at the birth heading into walls 12-40 m away while
         * their routes turned off at once: the spawn field of 12 m around any
         * pending site covers 30-50 m of the way out there. The local planner
         * keeps the slow exit and prices crossing a pending site instead. */
        a->straight = 0;
        speed = fminf(speed, GM_SPAWN_SPEED);
    }
    if (a->straight) {
        speed = fminf(speed, GM_SPAWN_SPEED);
        to_aim = gm_axis(c->birth_heading);
    } else if (gm_len(to_aim) > 0.3f && !(s->local_planner && nav)) {
        /* A hull turns at a limited rate: only as fast as the arc to the aim
         * point allows, or it circles a close point without ever reaching it.
         * (s13: the local planner simulates the turn itself; the progress it
         * makes towards the aim point decides.) */
        const float off = fabsf(gm_angle(atan2f(to_aim.z, to_aim.x) - v->heading));
        const float arc = GM_TURN_FRACTION * v->max_yaw_rate * fmaxf(gm_len(to_aim), 1.0f)
            / (2.0f * fmaxf(sinf(fminf(off, 0.5f * GM_PI)), 0.1f));
        speed = fminf(speed, fmaxf(arc, 0.5f));
    }
    a->v_des = speed;
    a->pref = gm_scale(gm_len(to_aim) > 0.3f ? gm_unit(to_aim) : a->fwd, speed);
}

/* ---- Single-lane stretch ahead: do not enter against oncoming traffic. ---- */

static void gm_narrow_hold(const gm_nav_t *nav, gm_agent_t *agents, int count, int i)
{
    gm_agent_t *a = &agents[i];
    const gm_commitment_t *c = &a->c;
    a->hold_arc = -1.0f;
    a->hold_blocker = 0;
    if (!nav || !a->mobile || !a->following)
        return;
    const float narrow = 1.5f * a->radius;
    if (gm_nav_clearance(nav, a->p) < narrow)
        return; /* already inside: keep going, the others wait */
    float arc = 0.0f, entry = -1.0f;
    gm_point_t from = a->p, in_pt = a->p, out_pt = a->p;
    int found = 0, closed = 0;
    for (int k = c->nav_vertex; k < c->nav_count && arc < GM_NARROW_LOOK && !closed; ++k) {
        const gm_point_t to = c->nav_points[k];
        const float len = gm_dist(from, to);
        const gm_point_t dir = gm_unit(gm_sub(to, from));
        for (float d = 3.0f; d <= len && arc + d < GM_NARROW_LOOK; d += 3.0f) {
            const gm_point_t q = gm_add(from, gm_scale(dir, d));
            const int tight = gm_nav_clearance(nav, q) < narrow;
            if (tight && !found) {
                found = 1;
                entry = arc + d;
                in_pt = q;
            }
            if (found && tight)
                out_pt = q;
            if (found && !tight) {
                closed = 1;
                break;
            }
        }
        arc += len;
        from = to;
    }
    if (!found || entry < 0.5f)
        return;
    const float length = gm_dist(in_pt, out_pt);
    const gm_point_t axis = length > 0.5f ? gm_unit(gm_sub(out_pt, in_pt)) : gm_unit(a->pref);
    int opposing_inside = 0, same_inside = 0, opposing_first = 0;
    uint64_t blocker = 0;
    for (int j = 0; j < count; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i || b->body || (b->stationary && !b->mobile))
            continue;
        const gm_point_t rel = gm_sub(b->p, in_pt);
        const float t = gm_dot(rel, axis), lateral = fabsf(gm_det(axis, rel));
        if (lateral > fmaxf(5.0f, narrow) || t < -5.0f || t > length + 30.0f)
            continue;
        const float along = gm_dot(gm_len(b->vel) > 0.5f ? gm_unit(b->vel) : b->fwd, axis);
        const int inside = t >= -2.0f && t <= length + 2.0f;
        if (along > 0.3f) {
            same_inside |= inside;
        } else if (along < -0.6f) { /* really coming the other way, not crossing */
            const float theirs = t - length; /* their distance to their entry */
            if (inside) {
                opposing_inside = 1;
                blocker = b->v->id;
            } else if (theirs < entry - 0.5f || (fabsf(theirs - entry) <= 0.5f && b->v->id < a->v->id)) {
                opposing_first = 1;
                if (!blocker)
                    blocker = b->v->id;
            }
        }
    }
    if (opposing_inside || (!same_inside && opposing_first)) {
        /* Stop on the own route, well before the mouth: whoever comes out
         * needs the room in front of it to get past. */
        a->hold_arc = fmaxf(0.0f, entry - GM_HOLD_SETBACK);
        a->hold_blocker = blocker;
    }
}

/* ---- Avoidance: velocity obstacles of the real hull boxes. ---- */

/* Two boxes overlap when they overlap along all four edge directions.
 * Returns the direction of least overlap, or -1 when they are apart. */
static int gm_box_axis(const float *offset, const float *reach)
{
    int best = 0;
    float least = INFINITY;
    for (int k = 0; k < 4; ++k) {
        const float overlap = reach[k] - fabsf(offset[k]);
        if (overlap <= 0.0f)
            return -1;
        if (overlap < least) {
            least = overlap;
            best = k;
        }
    }
    return best;
}

/* Time until the boxes overlap when the neighbour's centre moves at -s along
 * each axis (s: this vehicle's velocity relative to it). INFINITY if not
 * within the horizon; 0 if they overlap now. */
static float gm_box_ttc(const float *offset, const float *reach, const float *s, float horizon)
{
    float enter = 0.0f, leave = horizon;
    for (int k = 0; k < 4; ++k) {
        if (fabsf(s[k]) < 1e-4f) {
            if (fabsf(offset[k]) >= reach[k])
                return INFINITY;
            continue;
        }
        const float t0 = (offset[k] - reach[k]) / s[k], t1 = (offset[k] + reach[k]) / s[k];
        enter = fmaxf(enter, fminf(t0, t1));
        leave = fminf(leave, fmaxf(t0, t1));
        if (enter >= leave)
            return INFINITY;
    }
    return enter;
}

/* The neighbours of vehicle i that it has to keep clear of. A tank is long
 * and narrow: side by side it needs GM_SIDE_GAP, nose to tail behind another
 * GM_FOLLOW_GAP plus a headway that grows with its speed. */
static int gm_pairs(const gm_agent_t *agents, int count, int i, gm_pair_t *pairs)
{
    const gm_agent_t *a = &agents[i];
    int n = 0;
    for (int j = 0; j < count; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i)
            continue;
        const gm_point_t rel = gm_sub(b->p, a->p);
        const float dist = gm_len(rel);
        if (dist > GM_NEIGHBOR_RANGE)
            continue;
        const gm_point_t toward = dist > 1e-3f ? gm_scale(rel, 1.0f / dist) : a->fwd;
        const int a_faces = gm_dot(a->fwd, toward) > 0.3f, b_faces = gm_dot(b->fwd, toward) < -0.3f;
        /* Following: the other hull is ahead in this one's own lane and not
         * coming towards it (and the same seen from the other hull). A moving
         * hull one staggered track over is still followed at a distance; a
         * standing one beside the lane may be passed. */
        const float skew = fabsf(gm_det(a->fwd, b->fwd)), aligned = fabsf(gm_dot(a->fwd, b->fwd));
        const float a_side = a->half_length * skew + a->half_width * aligned;
        const float b_side = b->half_length * skew + b->half_width * aligned;
        const int a_follows = !b_faces && gm_dot(rel, a->fwd) > 0.0f
            && fabsf(gm_det(a->fwd, rel)) < a->half_width + b_side + (b->stationary ? GM_PASS_MARGIN : GM_LANE_MARGIN);
        const int b_follows = !a_faces && gm_dot(rel, b->fwd) < 0.0f
            && fabsf(gm_det(b->fwd, rel)) < b->half_width + a_side + (a->stationary ? GM_PASS_MARGIN : GM_LANE_MARGIN);
        gm_pair_t *q = &pairs[n];
        q->axis[0] = a->fwd;
        q->axis[1] = (gm_point_t){a->fwd.z, -a->fwd.x};
        q->axis[2] = b->fwd;
        q->axis[3] = (gm_point_t){b->fwd.z, -b->fwd.x};
        /* The keep-out box reaches further in front of a follower. */
        const float front = a_follows ? GM_FOLLOW_GAP - GM_SIDE_GAP
            + (a->straight ? GM_SPAWN_HEADWAY_S : GM_HEADWAY_S) * fmaxf(0.0f, a->speed) : 0.0f;
        const gm_point_t keep_rel = gm_sub(rel, gm_scale(a->fwd, 0.5f * front));
        for (int k = 0; k < 4; ++k) {
            const gm_point_t axis = q->axis[k];
            const float along_a = fabsf(gm_dot(a->fwd, axis));
            const float reach = a->half_length * along_a + a->half_width * fabsf(gm_det(a->fwd, axis))
                + b->half_length * fabsf(gm_dot(b->fwd, axis)) + b->half_width * fabsf(gm_det(b->fwd, axis));
            q->hull_offset[k] = gm_dot(rel, axis);
            q->hull_reach[k] = reach + GM_SKIN;
            q->keep_offset[k] = gm_dot(keep_rel, axis);
            q->keep_reach[k] = reach + GM_SIDE_GAP + 0.5f * front * along_a;
        }
        q->touching_axis = gm_box_axis(q->hull_offset, q->hull_reach);
        q->inside_axis = gm_box_axis(q->keep_offset, q->keep_reach);
        /* Of two in a column the one behind keeps the distance: the one in
         * front may slow or carry on as it likes, and looks back only before
         * it turns. */
        q->rear = q->touching_axis < 0 && b->mobile && b_follows && !a_follows;
        q->vel = b->vel;
        q->id = b->v->id;
        q->yields = b->mobile;
        q->standing = b->stationary;
        q->give_way_s = b->mobile && !a_follows
            && (a->remaining > b->remaining || (a->remaining == b->remaining && a->v->id > b->v->id)) ? GM_GIVE_WAY_S : 0.0f;
        ++n;
    }
    return n;
}

/* How bad velocity `cand` is against one neighbour that keeps its own
 * velocity. Level 0: clear. 1: on a collision course still some seconds
 * away. 2: it could not brake for it any more, or it is closing in inside
 * the distance to be kept. 3: the same for the bare hulls. The amount grades
 * candidates of one level, so that the slower one is the better one. */
static float gm_pair_risk(const gm_pair_t *q, gm_point_t cand, float decel, int *level)
{
    const gm_point_t w = gm_sub(cand, q->vel);
    const float speed = gm_len(w);
    float s[4];
    for (int k = 0; k < 4; ++k)
        s[k] = gm_dot(w, q->axis[k]);
    *level = 0;
    if (q->touching_axis >= 0) {
        /* In contact: anything that does not press in further is free. */
        const int m = q->touching_axis;
        const float closing = q->hull_offset[m] >= 0.0f ? s[m] : -s[m];
        if (closing <= 0.02f)
            return 0.0f;
        *level = 3;
        return 2.0f + closing;
    }
    /* What it takes to cancel this closing speed. */
    const float hard = GM_BRAKE_LAG_S + speed / (2.0f * decel) + q->give_way_s;
    if (q->inside_axis >= 0) {
        /* Inside the distance to be kept (dense spawn, a leader that slowed,
         * hulls born side by side): do not close it further. */
        const int m = q->inside_axis;
        const float closing = q->keep_offset[m] >= 0.0f ? s[m] : -s[m];
        const float t = gm_box_ttc(q->hull_offset, q->hull_reach, s, hard);
        if (t < hard) {
            *level = 3;
            return 2.0f + (hard - t) / hard + 0.3f * speed;
        }
        if (closing <= 0.02f)
            return 0.0f;
        *level = 2;
        return 1.0f + closing;
    }
    const float horizon = fmaxf(GM_TAU, hard + 0.5f);
    const float t = gm_box_ttc(q->keep_offset, q->keep_reach, s, horizon);
    if (t >= horizon)
        return 0.0f;
    if (t < hard) {
        *level = 2;
        return 1.0f + (hard - t) / hard;
    }
    *level = 1;
    return (horizon - t) / (horizon - hard);
}

/* ---- Static safety on the obstacle cells. ----
 * A hull never enters an obstacle cell. One that stands in a cell already
 * (born beside a building, pushed, or the cell is larger than what is in it)
 * may move as long as it overlaps no more cells than before: `allowed` carries
 * that count along a motion and only ever falls. */

/* A turn: an arc from p, at `speed` and turning at `rate`, until the hull
 * faces `to`. A long hull in a street cannot turn on the spot, but it follows
 * the bend; and even a turn "on the spot" creeps forward in the game. *end is
 * where the arc leaves it. */
static int gm_arc_free(const gm_nav_t *nav, const gm_vehicle_t *v, gm_point_t p, float from, float to, float speed,
                       float rate, float margin, int *allowed, gm_point_t *end)
{
    const float turn = gm_angle(to - from);
    const float length = speed * fabsf(turn) / fmaxf(rate, 0.05f);
    const int steps = (int)ceilf(fmaxf(fabsf(turn) / 0.12f, length / 1.5f));
    gm_pose_t pose = {p.x, p.z, from, 0.0f, 0.0f};
    for (int k = 1; k <= steps; ++k) {
        const float mid = from + turn * ((float)k - 0.5f) / (float)steps;
        pose.x += cosf(mid) * length / (float)steps;
        pose.z += sinf(mid) * length / (float)steps;
        pose.heading = from + turn * (float)k / (float)steps;
        const int overlap = gm_nav_overlap(nav, v, &pose, margin);
        if (overlap > *allowed)
            return 0;
        *allowed = overlap;
    }
    *end = (gm_point_t){pose.x, pose.z};
    return 1;
}

/* How far the hull, facing `heading`, can drive from p up to `reach`:
 * forwards (way = 1) or straight back (way = -1). */
static float gm_free_run(const gm_nav_t *nav, const gm_vehicle_t *v, gm_point_t p, float heading, float way, float reach,
                         int allowed, float margin)
{
    const gm_point_t dir = gm_scale(gm_axis(heading), way);
    gm_pose_t pose = {p.x, p.z, heading, 0.0f, 0.0f};
    float d = 0.0f;
    while (d < reach) {
        const float next = fminf(reach, d + 1.0f);
        pose.x = p.x + dir.x * next;
        pose.z = p.z + dir.z * next;
        const int overlap = gm_nav_overlap(nav, v, &pose, margin);
        if (overlap > allowed)
            return d;
        allowed = overlap;
        d = next;
    }
    return reach;
}

/* Speed from which the hull still stops within `room`, counting the time the
 * brakes take to bite. */
static float gm_stop_speed(float decel, float room)
{
    const float lag = decel * GM_BRAKE_LAG_S;
    return room > 0.0f ? sqrtf(lag * lag + 2.0f * decel * room) - lag : 0.0f;
}

/* Room to reverse straight for another metre and a half: no hull behind and
 * nothing static. Asked again every snapshot while it backs off. */
static int gm_escape_clear(const gm_snapshot_t *s, const gm_nav_t *nav, const gm_agent_t *agents, int count, int i)
{
    const gm_agent_t *a = &agents[i];
    if (a->v->reverse_speed <= 0.0f)
        return 0;
    for (int j = 0; j < count; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i)
            continue;
        const gm_point_t rel = gm_sub(b->p, a->p);
        const float behind = -gm_dot(rel, a->fwd);
        const float skew = fabsf(gm_det(a->fwd, b->fwd)), aligned = fabsf(gm_dot(a->fwd, b->fwd));
        if (behind > 0.0f && behind < a->half_length + b->half_length * aligned + b->half_width * skew + GM_ESCAPE_STEP
            && fabsf(gm_det(a->fwd, rel)) < a->half_width + b->half_length * skew + b->half_width * aligned + 0.4f)
            return 0;
    }
    /* The obstacle cells answer even where the hull already touches something
     * in front; without them the authority must call it clear. */
    if (nav) {
        const gm_pose_t here = {a->p.x, a->p.z, a->v->heading, 0.0f, 0.0f};
        return gm_free_run(nav, a->v, a->p, a->v->heading, -1.0f, GM_ESCAPE_STEP,
                           gm_nav_overlap(nav, a->v, &here, GM_SWEEP_MARGIN), GM_SWEEP_MARGIN) >= GM_ESCAPE_STEP;
    }
    if (!s->static_sweep)
        return 0;
    const gm_point_t back = gm_sub(a->p, gm_scale(a->fwd, GM_ESCAPE_STEP));
    const gm_pose_t from = {a->p.x, a->p.z, a->v->heading, 0.0f, 0.0f}, to = {back.x, back.z, a->v->heading, 0.0f, 0.0f};
    return s->static_sweep(s->geometry_context, a->v, &from, &to) == GM_CLEAR;
}

#define GM_FAN 15
#define GM_HEADINGS (GM_FAN + 3)
#define GM_SPEEDS 6

/* ---- Turning beside other hulls. ---- */

typedef struct {
    gm_point_t p, fwd;
    float half_length, half_width;
    uint64_t id;
    /* Closer than the margin a turn keeps: then the bare hulls decide, so that
     * it may turn away from that hull but not into it. Overlapping already:
     * only then is the hull left out, or it could not move at all. */
    int touching, overlapping;
} gm_hull_t;

/* Do two hull rectangles overlap? */
static int gm_boxes_meet(gm_point_t pa, gm_point_t fa, float la, float wa, gm_point_t pb, gm_point_t fb, float lb, float wb)
{
    const gm_point_t rel = gm_sub(pb, pa);
    const gm_point_t axes[4] = {fa, {fa.z, -fa.x}, fb, {fb.z, -fb.x}};
    for (int k = 0; k < 4; ++k) {
        const float ra = la * fabsf(gm_dot(fa, axes[k])) + wa * fabsf(gm_det(fa, axes[k]));
        const float rb = lb * fabsf(gm_dot(fb, axes[k])) + wb * fabsf(gm_det(fb, axes[k]));
        if (fabsf(gm_dot(rel, axes[k])) >= ra + rb)
            return 0;
    }
    return 1;
}

/* Can the hull turn where it stands from one heading to another without its
 * corners sweeping into a hull nearby? `front` lengthens it ahead: a turn on
 * the spot creeps forward in the game. */
static int gm_swing_clear(const gm_agent_t *a, float from, float to, float front, const gm_hull_t *hulls, int hull_count,
                          uint64_t *blocker)
{
    const float turn = gm_angle(to - from);
    const int steps = (int)ceilf(fabsf(turn) / 0.12f);
    for (int k = 1; k <= steps; ++k) {
        const gm_point_t fwd = gm_axis(from + turn * (float)k / (float)steps);
        const gm_point_t centre = gm_add(a->p, gm_scale(fwd, 0.5f * front));
        for (int n = 0; n < hull_count; ++n) {
            const gm_hull_t *h = &hulls[n];
            const float grow = h->touching ? 0.0f : GM_SWING_GROW;
            if (!h->overlapping
                && gm_boxes_meet(centre, fwd, a->half_length + 0.5f * front + grow, a->half_width + grow,
                                 h->p, h->fwd, h->half_length, h->half_width)) {
                *blocker = h->id;
                return 0;
            }
        }
    }
    return 1;
}

/* The same for a hull that turns from a standstill. In the game that is no
 * turn on the spot: the hull creeps forward all the while (1.5 to 2 m for an
 * ordinary turn, 8 m for a turn right round), so the path it sweeps is an arc. */
static int gm_arc_clear(const gm_agent_t *a, float from, float to, float speed, float rate, const gm_hull_t *hulls,
                        int hull_count, uint64_t *blocker)
{
    const float turn = gm_angle(to - from);
    const float length = speed * fabsf(turn) / fmaxf(rate, 0.05f);
    const int steps = (int)ceilf(fmaxf(fabsf(turn) / 0.12f, length / 1.0f));
    gm_point_t p = a->p;
    for (int k = 1; k <= steps; ++k) {
        const gm_point_t mid = gm_axis(from + turn * ((float)k - 0.5f) / (float)steps);
        p = gm_add(p, gm_scale(mid, length / (float)steps));
        const gm_point_t fwd = gm_axis(from + turn * (float)k / (float)steps);
        for (int n = 0; n < hull_count; ++n) {
            const gm_hull_t *h = &hulls[n];
            const float grow = h->touching ? 0.0f : GM_SWING_GROW;
            if (!h->overlapping
                && gm_boxes_meet(p, fwd, a->half_length + grow, a->half_width + grow, h->p, h->fwd,
                                 h->half_length, h->half_width)) {
                *blocker = h->id;
                return 0;
            }
        }
    }
    return 1;
}

/* How a hull turns: a tracked one on the spot, a wheeled one only while it rolls. */
static float gm_turn_cap(const gm_vehicle_t *v, float turn)
{
    if (turn > GM_PIVOT_ANGLE)
        return v->can_pivot ? 0.0f : 0.2f;
    return turn < 0.35f ? 1.0f : fmaxf(0.2f, 1.0f - 0.8f * (turn - 0.35f) / 1.05f);
}

/* Pick the velocity for vehicle i: closest to the preferred one among what
 * the hull can take, weighed against the collision courses with the hulls
 * around, and never faster than it can stop before the obstacle cells ahead. */
static void gm_choose(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i, const gm_nav_t *nav)
{
    static const float fan[GM_FAN] = {0.0f, 0.10f, -0.10f, 0.21f, -0.21f, 0.44f, -0.44f, 0.70f, -0.70f,
                                      1.05f, -1.05f, 1.48f, -1.48f, 2.1f, -2.1f};
    static const float fraction[GM_SPEEDS] = {1.0f, 0.75f, 0.5f, 0.3f, 0.15f, 0.0f};
    static gm_pair_t pairs[GM_MAX_VEHICLES];
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    const float decel = GM_DECEL_FRACTION * v->braking;
    const int pair_count = gm_pairs(agents, count, i, pairs);
    const float heading = v->heading;
    const float pref_heading = gm_len(a->pref) > 0.05f ? atan2f(a->pref.z, a->pref.x) : heading;
    const gm_pose_t here = {a->p.x, a->p.z, heading, 0.0f, 0.0f};

    /* Only what the hull can do within the reaction time is a candidate: a
     * rolling hull turns and changes speed at a limited rate; a standing
     * tracked one may turn anywhere. A velocity it cannot reach avoids nothing. */
    const int rolling = fabsf(a->speed) > 0.5f;
    const int arcs = rolling || !v->can_pivot;
    const float reach_turn = GM_TURN_FRACTION * v->max_yaw_rate * GM_REACT_S + 0.06f;
    const float slowest = fmaxf(0.0f, a->speed - v->braking * GM_REACT_S);
    const float fastest = fmaxf(0.0f, a->speed) + v->acceleration * GM_REACT_S;
    const float off_pref = fabsf(gm_angle(pref_heading - heading));
    const float pref_cap = gm_turn_cap(v, off_pref);

    /* Static limit per heading. The hull is swept over the obstacle cells as
     * it would move: the turn to that heading where it stands, then straight
     * on for as far as it takes to stop from the speed it really has, which
     * in the game can be half as much again as the one it was told. */
    const int standing_in = nav ? gm_nav_overlap(nav, v, &here, GM_SWEEP_MARGIN) : 0;
    const int standing_roomy = nav ? gm_nav_overlap(nav, v, &here, GM_FAST_MARGIN) : 0;
    const float real_speed = fmaxf(fmaxf(0.0f, a->speed), a->v_des);
    const float look = fminf(GM_LOOK_MAX, GM_BRAKE_LAG_S * real_speed + real_speed * real_speed / (2.0f * decel)
                                              + GM_STATIC_GAP + 2.0f);
    /* A hull turns along an arc: a rolling one at the speed it has, one that
     * stands at the creep its turn "on the spot" makes in the game. That
     * creep goes with the angle turned, so the arc of a standing hull is
     * GM_PIVOT_CREEP metres long per radian whatever its turn rate; to the
     * hulls around it shows as a speed of that times the rate it really turns at. */
    const float arc_rate = GM_TURN_FRACTION * v->max_yaw_rate;
    const float creep = GM_PIVOT_CREEP * arc_rate;
    const float creep_speed = gm_clamp(GM_PIVOT_CREEP * v->max_yaw_rate, 0.5f, 2.0f);
    const float arc_speed = rolling ? fmaxf(creep, a->speed) : creep;
    float angles[GM_HEADINGS], limit[GM_HEADINGS], run[GM_HEADINGS];
    gm_point_t arc_end[GM_HEADINGS];
    int vetoed[GM_HEADINGS] = {0}, turn_blocked[GM_HEADINGS] = {0}, no_pivot[GM_HEADINGS] = {0};
    for (int k = 0; k < GM_HEADINGS; ++k) {
        angles[k] = k < GM_FAN ? pref_heading + fan[k]
            : k == GM_FAN ? heading + reach_turn : k == GM_FAN + 1 ? heading - reach_turn : heading;
        limit[k] = v->cruise_speed + GM_SPEED_MAX;
        run[k] = look;
        arc_end[k] = a->p;
        if (!nav)
            continue; /* no obstacle cells: the authority below is all there is */
        const float turn = fabsf(gm_angle(angles[k] - heading));
        if (arcs && k < GM_HEADINGS - 1 && turn > reach_turn + 0.01f) {
            limit[k] = 0.0f; /* not a candidate anyway */
            continue;
        }
        /* The turn is checked as far as it gets in the next second or two:
         * it is chosen again every snapshot, and asking for the whole of a
         * turn right round to be free would keep a hull among others from
         * ever starting one. */
        const float signed_turn = gm_angle(angles[k] - heading);
        const float swept = heading + gm_clamp(signed_turn, -GM_TURN_LOOK, GM_TURN_LOOK);
        /* Beyond the first part of a wide turn there is only its own creep to go on. */
        const int wide_turn = turn > GM_TURN_LOOK + 0.01f;
        /* First with the room a hull at speed needs beside it, since it holds
         * its line only to a few decimetres: where that is free all the way,
         * so is the bare hull, and nothing more is asked. */
        float fast = 0.0f;
        int roomy = standing_roomy;
        if (gm_arc_free(nav, v, a->p, heading, swept, arc_speed, arc_rate, GM_FAST_MARGIN, &roomy, &arc_end[k])) {
            const float arc = gm_dist(a->p, arc_end[k]), reach = fmaxf(1.0f, look - arc);
            const float clear = wide_turn ? 3.0f : gm_free_run(nav, v, arc_end[k], angles[k], 1.0f, reach, roomy, GM_FAST_MARGIN);
            fast = gm_stop_speed(decel, arc + clear - GM_STATIC_GAP);
            if (!standing_roomy && (wide_turn || clear >= reach)) {
                run[k] = clear;
                limit[k] = fast;
                continue;
            }
        }
        /* Then bare: that is what the hull can do at all, at a crawl. */
        int allowed = standing_in;
        if (!gm_arc_free(nav, v, a->p, heading, swept, arc_speed, arc_rate, GM_SWEEP_MARGIN, &allowed, &arc_end[k])) {
            turn_blocked[k] = 1;
            limit[k] = 0.0f;
            continue;
        }
        const float arc = gm_dist(a->p, arc_end[k]);
        run[k] = wide_turn ? 3.0f
            : gm_free_run(nav, v, arc_end[k], angles[k], 1.0f, fmaxf(1.0f, look - arc), allowed, GM_SWEEP_MARGIN);
        limit[k] = gm_stop_speed(decel, arc + run[k] - GM_STATIC_GAP);
        if (limit[k] > GM_NARROW_SPEED)
            limit[k] = fmaxf(GM_NARROW_SPEED, fminf(limit[k], fast));
        if (standing_in)
            limit[k] = fminf(limit[k], GM_NARROW_SPEED); /* out of the cell slowly: what is in it may be right there */
    }
    if (nav && standing_in) {
        /* Deep in obstacle cells with no way that does not touch more of
         * them: towards open ground then, slowly. */
        int open = 0;
        for (int k = 0; k < GM_HEADINGS; ++k)
            open |= limit[k] > 0.3f;
        const float clear_here = gm_nav_clearance(nav, a->p);
        for (int k = 0; k < GM_HEADINGS && !open; ++k) {
            const float turn = fabsf(gm_angle(angles[k] - heading));
            if (arcs && k < GM_HEADINGS - 1 && turn > reach_turn + 0.01f)
                continue;
            if (gm_nav_clearance(nav, gm_add(a->p, gm_scale(gm_axis(angles[k]), 3.0f))) > clear_here + 0.3f) {
                limit[k] = 1.0f;
                turn_blocked[k] = 0;
                run[k] = 3.0f;
            }
        }
    }

    /* A turn swings the corners of a long hull well outside its track: no
     * heading whose turn would sweep them into a hull standing close by. The
     * hulls' own rectangles decide, so two tanks side by side can still turn
     * away from each other, and one may turn behind another's tail. */
    static gm_hull_t hulls[GM_MAX_VEHICLES];
    int hull_count = 0;
    for (int j = 0; j < count; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i || gm_dist(a->p, b->p) > a->radius + b->radius + GM_SWING_REACH)
            continue;
        hulls[hull_count] = (gm_hull_t){b->p, b->fwd, b->half_length, b->half_width, b->v->id, 0, 0};
        hulls[hull_count].touching = gm_boxes_meet(a->p, a->fwd, a->half_length + GM_SWING_GROW, a->half_width + GM_SWING_GROW,
                                                    b->p, b->fwd, b->half_length, b->half_width);
        hulls[hull_count].overlapping = gm_boxes_meet(a->p, a->fwd, a->half_length, a->half_width,
                                                       b->p, b->fwd, b->half_length, b->half_width);
        ++hull_count;
    }
    int swing_blocked[GM_HEADINGS] = {0};
    uint64_t swing_blocker = 0;
    for (int k = 0; k < GM_HEADINGS; ++k) {
        const float turn = fabsf(gm_angle(angles[k] - heading));
        if (turn <= 0.05f || (arcs && k < GM_HEADINGS - 1 && turn > reach_turn + 0.01f))
            continue;
        /* Rolling, the corners' swing where it is now; where it will be by
         * then is the avoidance's reckoning. From a standstill, the whole
         * creeping arc. */
        uint64_t who = 0;
        const float swept = heading + gm_clamp(gm_angle(angles[k] - heading), -GM_TURN_LOOK, GM_TURN_LOOK);
        if (rolling ? !gm_swing_clear(a, heading, angles[k], 0.0f, hulls, hull_count, &who)
                    : !gm_arc_clear(a, heading, swept, arc_speed, arc_rate, hulls, hull_count, &who)) {
            swing_blocked[k] = 1;
            if (k == 0)
                swing_blocker = who;
        }
    }
    a->no_turn_room = turn_blocked[0];

    /* Keeping moving matters more than never touching: the longer it has
     * made no progress, the less the keep-out of other moving vehicles
     * counts, so it eases through instead of standing in a jam. Parked hulls
     * and static geometry never relax: pushing into them leads nowhere. */
    const float stalled = c->progress_ms && a->v_des > 1.0f && s->ms > c->progress_ms + 1000u
        ? fminf(20.0f, 0.001f * (float)(s->ms - c->progress_ms - 1000u)) : 0.0f;
    const float dyn_weight = GM_DYN_WEIGHT / (1.0f + GM_IMPATIENCE * stalled);

    int best_k = GM_HEADINGS - 1, static_stop = 0, pivot_refused = 0;
    float best_speed = 0.0f, pref_risk = 0.0f;
    uint64_t pref_blocker = 0, pivot_hull = 0;
    enum gm_geometry geometry = GM_CLEAR;
    for (int attempt = 0; attempt < 6; ++attempt) {
        float best_cost = INFINITY;
        best_k = GM_HEADINGS - 1;
        best_speed = 0.0f;
        for (int k = 0; k < GM_HEADINGS; ++k) {
            if (swing_blocked[k])
                continue; /* the turn to it would sweep a corner into a hull nearby */
            if (a->straight && k < GM_HEADINGS - 1 && fabsf(gm_angle(angles[k] - c->birth_heading)) > 0.12f)
                continue; /* among spawn points: straight out, only the speed varies */
            const float turn = fabsf(gm_angle(angles[k] - heading));
            if (arcs && k < GM_HEADINGS - 1 && turn > reach_turn + 0.01f)
                continue;
            if (turn_blocked[k])
                continue; /* the turn to it would swing the hull into an obstacle cell */
            const float top = vetoed[k] ? 0.0f : fminf(a->v_des * fminf(gm_turn_cap(v, turn), pref_cap), limit[k]);
            const gm_point_t dir = gm_axis(angles[k]);
            const float aim_error = fabsf(gm_angle(angles[k] - pref_heading));
            const float smooth = c->cmd_valid ? gm_angle(angles[k] - c->cmd_heading) : 0.0f;
            for (int f = 0; f < GM_SPEEDS; ++f) {
                const float speed = top * fraction[f];
                /* A speed the tracker cannot hold is no choice. Under its rest
                 * band it gives the least throttle there is, which with the
                 * steering hard over turns the hull a degree a second: Poland
                 * 2026-11-15, two tanks stood 20 s "driving" at 0.1 m/s with a
                 * few centimetres of room ahead and a right angle to turn, and
                 * nothing counted them as held. What is left for such a heading
                 * is to stand, or to turn on the spot, and then the ladder for a
                 * held hull applies (hard hold, back-off). */
                if (fraction[f] > 0.0f && speed < GM_MOVE_MIN)
                    continue;
                if (speed <= 0.0f && turn > 0.05f && (!v->can_pivot || no_pivot[k]))
                    continue; /* would be a turn on the spot, without room or by a hull that cannot */
                /* Judged on the speed it will really have by then. A hull
                 * told to turn hard goes forward while it does, however
                 * little speed it was told: that creep is what the hulls
                 * around have to be clear of. */
                const float real = fmaxf(slowest, fminf(speed, fastest));
                const gm_point_t intended = gm_scale(dir, real);
                const int crept = turn > GM_CREEP_TURN && real < creep_speed;
                const gm_point_t cand = crept
                    ? gm_scale(gm_axis(heading + 0.5f * gm_clamp(gm_angle(angles[k] - heading), -GM_TURN_LOOK, GM_TURN_LOOK)),
                               creep_speed)
                    : intended;
                /* What it is after is the velocity it asks for; the creep is
                 * only what the others have to be safe from. */
                const gm_point_t miss = gm_sub(intended, a->pref);
                /* All terms in (m/s)^2, so that the balance is the same at a
                 * crawl as at cruise: at a crawl a fixed price on changing
                 * the heading would keep it creeping straight past its goal.
                 * A change of heading is priced only for a hull that moves:
                 * for one that stands, the price made a turn on the spot of
                 * more than a right angle dearer than standing with its back
                 * to the route for good. */
                float cost = gm_dot(miss, miss) + GM_AIM_WEIGHT * a->v_des * aim_error;
                if (speed > 0.05f)
                    cost += GM_SMOOTH_WEIGHT * a->v_des * a->v_des * smooth * smooth;
                for (int l = 0; l < pair_count; ++l) {
                    if (pairs[l].rear && turn <= 0.12f)
                        continue;
                    int level;
                    /* The creep is a metre or two along the arc, and against
                     * a hull that stands that arc has been checked exactly
                     * above. Taken as a velocity kept for seconds it ran into
                     * every hull parked within a few metres, and a tank free
                     * to turn stood beside it for good. It is for those that
                     * move, which may come into that arc. */
                    const float amount = gm_pair_risk(&pairs[l], crept && pairs[l].standing ? intended : cand, decel, &level);
                    /* Impatience eases the distance kept, never a hull-to-hull course. */
                    cost += amount * (level == 3 ? GM_DYN_WEIGHT
                        : level == 2 ? (pairs[l].yields ? dyn_weight : GM_DYN_WEIGHT) : GM_SOFT_WEIGHT);
                }
                if (pair_count && gm_angle(angles[k] - pref_heading) > 0.0f)
                    cost += 0.2f * aim_error; /* pass on the right: breaks head-on symmetry */
                if (cost < best_cost) {
                    best_cost = cost;
                    best_k = k;
                    best_speed = speed;
                }
            }
        }
        const float best_turn = gm_angle(angles[best_k] - heading);
        if (best_speed <= 0.05f && fabsf(best_turn) > 0.05f) {
            /* A turn on the spot creeps in the game: it needs more room than
             * the turn alone, among the obstacle cells and among the hulls. */
            int refused = 0;
            const float swept = heading + gm_clamp(best_turn, -GM_TURN_LOOK, GM_TURN_LOOK);
            if (nav) {
                int allowed = gm_nav_overlap(nav, v, &here, GM_SWEEP_MARGIN + GM_PIVOT_MARGIN);
                gm_point_t end;
                refused = !gm_arc_free(nav, v, a->p, heading, swept, creep, arc_rate,
                                       GM_SWEEP_MARGIN + GM_PIVOT_MARGIN, &allowed, &end);
                pivot_refused |= refused;
            }
            uint64_t who = 0;
            if (!refused && !gm_arc_clear(a, heading, swept, creep, arc_rate, hulls, hull_count, &who)) {
                refused = 1;
                pivot_hull = who;
            }
            if (refused) {
                /* A wider turn the same way passes through this one. */
                for (int k = 0; k < GM_HEADINGS; ++k) {
                    const float other = gm_angle(angles[k] - heading);
                    if (other * best_turn > 0.0f && fabsf(other) >= fabsf(best_turn) - 0.01f)
                        no_pivot[k] = 1;
                }
                continue;
            }
        }
        if (!s->static_sweep)
            break;
        /* The game's own sweep on the chosen motion, for what the scan does
         * not have: first the turn it starts with, then the stretch it covers
         * before it could stop. Past where the obstacle cells end the free
         * run that sweep has nothing to add, so it is not asked there. */
        gm_vehicle_t hull = *v;
        hull.length += 2.0f * GM_SWEEP_MARGIN;
        hull.width += 2.0f * GM_SWEEP_MARGIN;
        const float swing = gm_clamp(best_turn, -0.6f, 0.6f);
        const gm_point_t turned_at = arc_end[best_k];
        enum gm_geometry answer = GM_CLEAR;
        gm_pose_t from = here, to = here;
        if (gm_dist(a->p, turned_at) > 0.3f) {
            /* The arc, as the chord with the turn spread along it. */
            to = (gm_pose_t){turned_at.x, turned_at.z, heading + best_turn, 0.0f, 0.0f};
            answer = s->static_sweep(s->geometry_context, &hull, &from, &to);
        } else if (fabsf(swing) > 0.1f) {
            to.heading = heading + swing;
            answer = s->static_sweep(s->geometry_context, &hull, &from, &to);
        }
        if (answer != GM_BLOCKED && best_speed > 0.05f) {
            const float fast = fmaxf(best_speed, fmaxf(0.0f, a->speed));
            const float span = gm_clamp(fast * 1.2f, 2.0f, 12.0f);
            const gm_point_t end = gm_add(turned_at, gm_scale(gm_axis(angles[best_k]), span));
            from = (gm_pose_t){turned_at.x, turned_at.z, angles[best_k], 0.0f, 0.0f};
            to = (gm_pose_t){end.x, end.z, angles[best_k], 0.0f, 0.0f};
            const enum gm_geometry ahead = s->static_sweep(s->geometry_context, &hull, &from, &to);
            if (!nav && ahead == GM_BLOCKED && fabsf(swing) <= 0.1f
                && s->static_sweep(s->geometry_context, &hull, &here, &here) == GM_BLOCKED) {
                /* Standing against something already, every sweep from here
                 * is refused and says nothing about this motion: straight
                 * along the hull it may creep, that is the way out. */
                best_speed = fminf(best_speed, GM_TIGHT_SPEED);
                geometry = GM_CLEAR;
                break;
            }
            answer = ahead == GM_BLOCKED ? GM_BLOCKED : ahead == GM_UNKNOWN ? GM_UNKNOWN : answer;
        }
        if (nav) {
            /* The obstacle cells have already cleared this motion. What the
             * game's sweep objects to beyond them is one of its rays ahead
             * hitting something the scan does not call solid: a tree, a
             * fence, a hull, now and then something that really stops a tank.
             * It cannot tell which, so the hull goes on slowly: it pushes
             * through what gives way, and what does not is found without a
             * crash and backed away from. "Unknown" from that sweep means a
             * scanned cell under the hull, which the cells have judged. */
            if (answer == GM_BLOCKED)
                best_speed = fminf(best_speed, GM_PUSH_SPEED);
            break;
        }
        if (answer != GM_BLOCKED) {
            geometry = answer;
            break;
        }
        vetoed[best_k] = 1;
        static_stop = 1;
        best_speed = 0.0f;
    }
    if (vetoed[best_k]) {
        best_speed = 0.0f; /* every try was refused: stand still on the present heading */
        best_k = GM_HEADINGS - 1;
    }

    /* Why is it slower than it wants: traffic or geometry? */
    for (int l = 0; l < pair_count; ++l) {
        if (pairs[l].rear)
            continue;
        int level;
        const float amount = gm_pair_risk(&pairs[l], a->pref, decel, &level);
        const float risk = level >= 2 ? (float)(level - 1) + amount : 0.0f;
        if (risk > pref_risk) {
            pref_risk = risk;
            pref_blocker = pairs[l].id;
        }
    }
    /* Without obstacle cells the game's sweep is all there is, and it has no
     * answer here: wait a little for one, then creep. */
    int unknown_hold = 0;
    if (geometry == GM_UNKNOWN && best_speed > 0.05f) {
        if (!c->unknown_since_ms)
            c->unknown_since_ms = s->ms;
        if (s->ms - c->unknown_since_ms < GM_UNKNOWN_HOLD_MS) {
            unknown_hold = 1;
            best_speed = 0.0f;
        } else {
            best_speed = fminf(best_speed, GM_UNKNOWN_SPEED);
        }
    } else if (geometry == GM_CLEAR) {
        c->unknown_since_ms = 0;
    }

#ifdef GM_TRACE_CHOICE
    /* Offline diagnosis: GM_TRACE_CHOICE=<vehicle id> in the environment prints this vehicle's choice. */
    {
        static long traced = -2;
        if (traced == -2) {
            const char *id = getenv("GM_TRACE_CHOICE");
            traced = id ? atol(id) : -1;
        }
        if ((long)v->id == traced) {
            fprintf(stderr, "choice t=%.2f p=(%.1f,%.1f) hdg %.2f v %.2f v_des %.2f pref %.2f in %d hold %.1f straight %d follow %d\n",
                    (double)s->ms / 1000.0, a->p.x, a->p.z, heading, a->speed, a->v_des, pref_heading, standing_in,
                    a->hold_arc, a->straight, a->following);
            for (int k = 0; k < GM_HEADINGS; ++k)
                fprintf(stderr, "   k%-2d ang %+.2f limit %5.2f run %5.1f turn_blocked %d swing %d veto %d nopivot %d\n", k,
                        gm_angle(angles[k] - heading), limit[k], run[k], turn_blocked[k], swing_blocked[k], vetoed[k], no_pivot[k]);
            fprintf(stderr, "   -> k%d speed %.2f geometry %d pref_risk %.2f static_stop %d pivot_refused %d pivot_hull %llu\n", best_k,
                    best_speed, (int)geometry, pref_risk, static_stop, pivot_refused, (unsigned long long)pivot_hull);
        }
    }
#endif
    const float error = gm_angle(angles[best_k] - heading);
    const int moving = best_speed > 0.05f;
    const int wants = a->v_des > 0.3f;
    const float pref_limit = limit[0];
    /* A rolling hull whose way lies beyond what it can turn to in the
     * reaction time has no static limit worked out for that heading: it is
     * slowing down for the turn, which is no obstacle. Only when none of the
     * headings it can reach is free does geometry hold it. */
    const int pref_judged = !(arcs && off_pref > reach_turn + 0.01f);
    int reach_free = 0;
    for (int k = 0; k < GM_HEADINGS; ++k)
        reach_free |= !turn_blocked[k] && limit[k] >= 0.3f;
    const int pref_static = pref_judged ? pref_limit < 0.3f : !reach_free;
    a->turn_boxed = !moving && off_pref > GM_CREEP_TURN && (pivot_hull || swing_blocked[0]);
    a->firm = moving && (best_speed >= limit[best_k] - 0.05f || (a->approach && best_speed >= a->v_des - 0.05f));
    a->cmd_heading = angles[best_k];
    a->cmd_speed = moving ? best_speed : 0.0f;
    /* Standing still it turns when a turn on the spot came out best. That
     * turn has been checked: its arc against every hull standing near, its
     * creep against those that move. Whether the way it then faces is free
     * is another matter and no reason not to turn: the straight line from
     * here may graze a hull that the turn itself swings clear of, and a tank
     * that waited for that line to clear stood beside a parked one for good. */
    const int turn_on_spot = !moving && v->can_pivot && !unknown_hold && wants && fabsf(error) > 0.2f;
    a->cmd_yaw = moving || turn_on_spot ? gm_clamp(GM_YAW_GAIN * error, -v->max_yaw_rate, v->max_yaw_rate) : 0.0f;
    a->geometry = geometry;
    a->blocker = 0;
    a->pref_blocked = pref_risk >= 1.0f;
    if (moving) {
        a->state = GM_DRIVING;
        a->blocker = a->pref_blocked ? pref_blocker : 0;
    } else if (!wants) {
        a->state = GM_WAIT_TRAFFIC; /* held before a single-lane stretch or at the route end */
        a->blocker = a->hold_blocker;
    } else if (unknown_hold) {
        a->state = GM_WAIT_GEOMETRY;
    } else if (a->cmd_yaw != 0.0f && fabsf(error) > 0.2f) {
        a->state = GM_TURNING;
    } else if (pref_risk >= 1.0f) {
        a->state = GM_WAIT_TRAFFIC;
        a->blocker = pref_blocker;
    } else if (pivot_hull || swing_blocked[0]) {
        a->state = GM_WAIT_TRAFFIC; /* a hull close by leaves no room for the turn it needs */
        a->blocker = pivot_hull ? pivot_hull : swing_blocker;
    } else if (static_stop || pivot_refused || pref_static || (a->no_turn_room && off_pref > 0.2f)) {
        a->state = GM_STATIC_BLOCKED;
        a->geometry = GM_BLOCKED;
    } else if (!pref_judged && rolling) {
        a->state = GM_TURNING; /* braking for a turn it cannot make while it rolls */
    } else {
        a->state = GM_WAIT_TRAFFIC;
        a->blocker = pref_blocker;
    }
}

/* ---- s13: local trajectory sampler (snapshot->local_planner). ----
 *
 * Replaces the static candidate check of gm_choose (headings swept over the
 * 2 m obstacle cells, "may leave a solid cell, never enter one") with what an
 * autonomous car does: local perception plus a sampling planner, the route as
 * the reference path. The vehicle-vehicle arbitration stays: the velocity
 * obstacles of gm_pair_risk price every candidate, and everything after the
 * choice (holds, yields, jams, parking, re-plans, back-off) is unchanged.
 *
 *  1. Fused occupancy, one raster per vehicle and snapshot, LP_N x LP_N cells
 *     of 0.25 m on the lidar's world-aligned cells, centred on the vehicle: a
 *     cell the lidar holds occupied is occupied; one it saw free at least twice
 *     and holds free now is free even where the 2 m prior is solid; one it has
 *     not seen is the prior cell (blocked or unscanned). Other hulls are not in
 *     it (the rays do not hit vehicles): they are rectangles at constant
 *     velocity along the horizon.
 *  2. Candidates, simulated with the game's measured kinematics: no turn at
 *     rest, at most turn_per_m (0.42) rad per signed metre and the yaw-rate
 *     limit, the same curvature per signed metre backwards (the tracker
 *     steers by curvature), the command delay, acceleration and braking
 *     limits, a 0.3 s steering response. Forward at up to six speeds (creep
 *     included) x seven curvatures from full left to full right plus pure
 *     pursuit of the route point ahead; reverse at five curvatures; retracing
 *     the breadcrumb trail; and reverse-then-forward manoeuvres (back up to
 *     swing the nose: 2.5 or 5 m back at five curvatures, then forward on full
 *     lock either way or pursuing the route) when going forward gets nowhere.
 *     Horizon 2.5 s or 10 m, manoeuvres 6.5 s.
 *  3. Collision check: the hull rectangle, grown by 0.1 m beside and 0.15 m
 *     at the ends at a creep up to 0.3 / 0.5 m from 4 m/s, at every 0.35 m or
 *     0.08 rad along the trajectory, then the stopping distance straight on
 *     (brake lag + 0.6 x braking, as every static stopping distance here).
 *     A hull that already overlaps occupied cells may move as long as no cell
 *     gets deeper into it than at the start (it may pivot on a wall, never
 *     press into it): there is no "no candidate" while a way out exists.
 *  4. Cost (in (m/s)^2 like gm_choose, so the velocity-obstacle weights keep
 *     their meaning; S = 2 max(v_des, 2), what 1 m/s more progress is worth):
 *     progress towards the route point ahead as (v_des - gained / horizon)^2,
 *     heading at the end against the way on, clearance of the bare hull
 *     (below 1 m, steeply below 0.4 m), reversing, hull proximity, crossing or
 *     ending on a pending spawn site (backing onto one is refused), smoothness
 *     against the previous choice (hysteresis), velocity obstacles. Standing
 *     is a candidate too; held for static reasons it grows dearer with the
 *     time without progress. A reverse-then-forward manoeuvre once begun is
 *     driven to the end of its reverse part while it stays feasible. Among
 *     pending spawn sites no manoeuvres; on its own site the aim is first
 *     LP_EXIT_M along the birth heading. Tanks of one spawn leave in turn
 *     (gm_spawn_queue).
 *  5. Wedged (the 1 s clock of gm_plan_snapshot): what holds the hull is not
 *     in the grid, so a veto point goes there for LP_VETO_MS and the planner
 *     itself finds the reverse or manoeuvre; the old back-off only runs when
 *     the planner stands. */

#define LP_N 128                   /* raster cells per side: 32 m */
#define LP_W (LP_N / 64)
#define LP_DT 0.1f
#define LP_HORIZON_S 2.5f
#define LP_HORIZON_M 10.0f
#define LP_TWO_S 6.5f              /* reverse-then-forward manoeuvres */
#define LP_CHECK_M 0.5f            /* a cell entering the hull stays in it for metres: found at the next check */
#define LP_CHECK_RAD 0.1f          /* corners of a 7.8 m hull move 0.43 m */
#define LP_CREEP 1.0f              /* m/s: rolls enough to turn (tracker pivot band 1.2 m/s) */
#define LP_TWO_SPEED 2.0f          /* forward part of a manoeuvre */
#define LP_STEER_TAU 0.3f
#define LP_SIDE_FAST 0.3f
#define LP_END_FAST 0.5f
#define LP_SIDE_CREEP 0.15f        /* more than half a cell: cells count by their centres */
#define LP_END_CREEP 0.2f
#define LP_DEPTH_SLACK 0.01f       /* m deeper than at the start that still counts as not pressing in */
#define LP_CLEAR_REACH 1.0f
#define LP_MOVING_HARD_S 1.2f      /* a moving hull's predicted rectangle refuses up to this time, then costs */
#define LP_VEHICLE_POSES 3000      /* pose checks per vehicle and snapshot */
#define LP_SNAPSHOT_POSES 60000    /* and per snapshot; beyond, the reduced candidate set */
#define LP_VETO_MS 8000u
#define LP_VETO_R 0.6f
#define LP_LOG_MS 500u
#define LP_MAX_HULLS GM_MAX_VEHICLES
#define LP_MAX_SITES 64
#define LP_EXIT_M 8.0f             /* off its own spawn site along the birth heading for this far */
/* s16 exit mode (snapshot spawn_exit; design rule: at the start getting
 * out of the spawn through whatever open space there is matters more than the
 * route). A tank in the spawn exit queue (sq_state 1, within SQ_LEAVE_M of its
 * spawn point) for at most EX_MODE_MS: its hull is marched straight out along
 * EX_DIRS directions over the local raster, the standing hulls and the
 * pending spawn sites (ex_scan). It steers for its route aim while that gets
 * it somewhere; held in the queue on its route's way, or getting nowhere
 * towards the route aim for EX_ROUTE_FAIL_MS, it aims along the best open
 * way instead, until the route's way is open again (after EX_OPEN_HOLD_MS).
 * No reversing there unless pressed, wedged or stuck (ex_reverse_gate). */
#define EX_MODE_MS 60000u
#define EX_DIRS 24
#define EX_STEP_M 1.0f
#define EX_REACH_M 15.0f           /* m the hull is marched out at most */
#define EX_AIM_M 12.0f             /* aim point at most this far out along the way out */
#define EX_MIN_FREE 4.0f           /* m of room a way out must give to count (a 7.8 m hull turns in about that) */
#define EX_BLOCKED_M 5.0f          /* the kept way out is given up when it gives less room than this */
#define EX_SHUT 100.0f             /* score off a way out under EX_MIN_FREE: taken only when all are shut */
#define EX_BETTER 0.25f            /* another way out must score this fraction better to replace it */
#define EX_AWAY_W 4.0f             /* score (m of room) for heading straight away from the spawn cluster */
#define EX_BIRTH_W 2.0f            /* ... along the birth heading */
#define EX_ROUTE_W 1.5f            /* ... towards the route aim (not out by the back door) */
#define EX_BONUS_ROOM 10.0f        /* the bonuses count in full from this much room, in proportion below */
#define EX_TURN_W 2.0f             /* m per rad of turn from the present heading: turning takes rolling (0.42 rad/m) */
#define EX_CLUSTER_M 30.0f         /* pending sites this close to its spawn point belong to its spawn cluster */
#define EX_CORRIDOR_SIDE 0.5f      /* m beside the hull by which the exit corridor is wider */
#define EX_REV_AFTER_MS 2000u      /* no forward candidate getting anywhere this long: reversing allowed */
#define EX_FWD_PROGRESS 1.0f       /* m over the horizon that count as getting somewhere */
#define EX_STALL_MS 3000u          /* the hull not 1.5 m further (progress clock) this long: reversing allowed */
#ifndef EX_REHOLD
#define EX_REHOLD 0
#endif
#define EX_ROUTE_FAIL_MS 1500u     /* the local planner getting nowhere towards the route aim this long: open space */
#define EX_OPEN_HOLD_MS 3000u      /* steering for open space at least this long before the route aim is tried again */
#define EX_ROUTE_OPEN 8.0f       /* m of room along the route aim (or up to it) that leave the route aim in charge */
#define EX_TURN_FIRST 0.5f        /* rad between heading and way out from which forward means turning that way */
#define EX_REV_COST 0.5f           /* x S on every reverse in exit mode */
#define EX_OFF_ROUTE_M 4.0f        /* m off the followed nav leg on leaving exit mode: new route from here */
#define EX_SPEED 5.0f              /* m/s at most in exit mode: the room is judged over 15 m, the lidar sees 12 m
                                    * from the hull centre, and 5 m/s stops in about 7.5 m (0.5 s lag, 0.6 x 3.2 m/s^2) */

enum { LP_SINGLE, LP_TWO, LP_RETRACE, LP_CONTINUE, LP_STAND };
static const char *const lp_type_name[] = {"single", "two_seg", "retrace", "continue", "stand"};

typedef struct {
    int ox, oz;                         /* world cell of raster cell (0, 0) */
    uint64_t occ[LP_N][LP_W];           /* fused occupancy */
    uint64_t over[LP_N][LP_W];          /* prior solid, lidar free */
    uint16_t sat[LP_N + 1][LP_N + 1];   /* summed-area table of occ */
    uint8_t lidar[LP_N * LP_N];
    int lidar_cells, override_cells;
    uint8_t world[LP_N * LP_N];         /* s17: the overhead-probe cache (snapshot world_block), GM_WORLD_* */
    int world_cells;
} gm_lp_grid_t;

static gm_lp_grid_t gm_lp_grid;
static int gm_lp_snapshot_poses;

/* Timing of the local planner, reported every LP_STATS_MS through local_log. */
#define LP_STATS_MS 60000u
#define LP_HIST 200                 /* 100 us bins of the time per snapshot */
static struct {
    uint64_t since_ms;
    unsigned snapshots, plans;
    double sum_snapshot_us, max_snapshot_us, max_plan_us;
    uint64_t poses;
    unsigned hist[LP_HIST + 1];
} gm_lp_stats;

static void lp_stats_note(const gm_snapshot_t *s, double snapshot_us, int plans, int poses)
{
    if (!plans)
        return;
    if (!gm_lp_stats.since_ms || s->ms < gm_lp_stats.since_ms) {
        memset(&gm_lp_stats, 0, sizeof gm_lp_stats);
        gm_lp_stats.since_ms = s->ms;
    }
    ++gm_lp_stats.snapshots;
    gm_lp_stats.plans += (unsigned)plans;
    gm_lp_stats.sum_snapshot_us += snapshot_us;
    gm_lp_stats.poses += (uint64_t)poses;
    if (snapshot_us > gm_lp_stats.max_snapshot_us)
        gm_lp_stats.max_snapshot_us = snapshot_us;
    const int bin = (int)(snapshot_us / 100.0);
    ++gm_lp_stats.hist[bin < 0 ? 0 : bin > LP_HIST ? LP_HIST : bin];
    if (s->ms - gm_lp_stats.since_ms < LP_STATS_MS)
        return;
    unsigned acc = 0, want = (gm_lp_stats.snapshots * 95u + 99u) / 100u;
    int p95 = 0;
    while (p95 < LP_HIST && (acc += gm_lp_stats.hist[p95]) < want)
        ++p95;
    if (s->local_log) {
        char line[320];
        snprintf(line, sizeof line,
                 "%llu local planner: %u snapshots, %u vehicle plans; per snapshot mean %.0f us, p95 < %d us, max %.0f us;"
                 " per vehicle mean %.0f us, max %.0f us; %.0f pose checks per vehicle",
                 (unsigned long long)s->ms, gm_lp_stats.snapshots, gm_lp_stats.plans,
                 gm_lp_stats.sum_snapshot_us / (double)gm_lp_stats.snapshots, (p95 + 1) * 100, gm_lp_stats.max_snapshot_us,
                 gm_lp_stats.sum_snapshot_us / (double)gm_lp_stats.plans, gm_lp_stats.max_plan_us,
                 (double)gm_lp_stats.poses / (double)gm_lp_stats.plans);
        s->local_log(s->local_context, 1, line);
    }
    memset(&gm_lp_stats, 0, sizeof gm_lp_stats);
    gm_lp_stats.since_ms = s->ms;
}

typedef struct {
    int type, dir, kindex;              /* kindex: curvature sample (firmness), -1 none */
    float speed, kappa, d1, speed2, kappa2;
    int pursuit2;
} lp_cand_t;

typedef struct {
    int feasible, why;                  /* why: 1 static, 2 hull, 3 veto, 4 budget, 5 stopping distance */
    uint64_t blocker;
    float cost, progress, clear, head_err, dyn, prox, end_depth, start_depth, spawn, s_rev;
    float heading_1s, kappa_now;
    gm_point_t vel_early, end;
    int over;
} lp_eval_t;

typedef struct {
    gm_point_t p, f, vel;
    float hl, hw, pen0, diag;
    uint64_t id;
    int moving;
} lp_hull_t;

typedef struct {
    const gm_snapshot_t *s;
    const gm_nav_t *nav;
    const gm_agent_t *a;
    const gm_lp_grid_t *g;
    float bl, bw, per_m, yaw_max, accel, decel, stop_decel, delay, v_des, S, k0;
    gm_point_t aim;
    lp_hull_t hulls[LP_MAX_HULLS];
    int hull_count;
    int veto[2];
    gm_point_t veto_p[2];
    int site_count;
    gm_point_t sites[LP_MAX_SITES];
    float site_r, site_d0[LP_MAX_SITES];
    int budget, poses, want_over;
} lp_ctx_t;

static void lp_build(const gm_snapshot_t *s, const gm_nav_t *nav, const gm_agent_t *a, gm_lp_grid_t *g)
{
    g->ox = (int)floorf(a->p.x / GM_LOCAL_CELL) - LP_N / 2;
    g->oz = (int)floorf(a->p.z / GM_LOCAL_CELL) - LP_N / 2;
    memset(g->occ, 0, sizeof g->occ);
    memset(g->over, 0, sizeof g->over);
    const int seen = s->local_block ? s->local_block(s->local_context, a->v->id, g->ox, g->oz, LP_N, g->lidar) : -1;
    if (seen < 0)
        memset(g->lidar, GM_LOCAL_UNKNOWN, sizeof g->lidar);
    g->lidar_cells = seen > 0 ? seen : 0;
    g->override_cells = 0;
    /* s17: the shared overhead cache where the lidar has not seen (the lidar
     * keeps precedence where it has: its cells decay to unknown within 5 s,
     * so what it says is recent). Occupied there occupies; free twice clears
     * a solid obstacle cell as lidar free does; the rest is unknown. */
    const int wseen = s->world_block ? s->world_block(s->world_context, GM_LOCAL_CELL, g->ox, g->oz, LP_N, g->world) : -1;
    g->world_cells = wseen > 0 ? wseen : 0;
    const uint8_t *world = wseen > 0 ? g->world : NULL;
    float x0 = 0.0f, z0 = 0.0f, step = 0.0f;
    int cols = 0, rows = 0;
    const int prior = nav && gm_nav_grid(nav, &x0, &z0, &step, &cols, &rows) && step > 0.0f;
    static int pcol[LP_N];
    if (prior)
        for (int c = 0; c < LP_N; ++c)
            pcol[c] = (int)floorf((((float)(g->ox + c) + 0.5f) * GM_LOCAL_CELL - x0) / step);
    for (int r = 0; r < LP_N; ++r) {
        const int prow = prior ? (int)floorf((((float)(g->oz + r) + 0.5f) * GM_LOCAL_CELL - z0) / step) : 0;
        const uint8_t *l = g->lidar + (size_t)r * LP_N;
        const uint8_t *w = world ? world + (size_t)r * LP_N : NULL;
        int last = INT_MIN, solid = 0;
        for (int c = 0; c < LP_N; ++c) {
            if (prior && pcol[c] != last) {
                last = pcol[c];
                solid = gm_nav_cell_solid(nav, last, prow);
            }
            const uint64_t bit = 1ull << (c & 63);
            uint8_t seen_here = l[c];
            if (w && seen_here == GM_LOCAL_UNKNOWN)
                seen_here = w[c] == GM_WORLD_OCCUPIED ? GM_LOCAL_OCCUPIED
                            : w[c] == GM_WORLD_FREE ? GM_LOCAL_FREE : GM_LOCAL_UNKNOWN;
            if (seen_here == GM_LOCAL_OCCUPIED || (seen_here != GM_LOCAL_FREE && solid)) {
                g->occ[r][c >> 6] |= bit;
            } else if (seen_here == GM_LOCAL_FREE && solid) {
                g->over[r][c >> 6] |= bit;
                ++g->override_cells;
            }
        }
    }
    for (int c = 0; c <= LP_N; ++c)
        g->sat[0][c] = 0;
    for (int r = 0; r < LP_N; ++r) {
        unsigned run = 0;
        g->sat[r + 1][0] = 0;
        for (int c = 0; c < LP_N; ++c) {
            run += (unsigned)((g->occ[r][c >> 6] >> (c & 63)) & 1u);
            g->sat[r + 1][c + 1] = (uint16_t)(g->sat[r][c + 1] + run);
        }
    }
}

/* Narrow [lo, hi] to the dx with |dx * k + b| <= h. */
static void lp_clip(float *lo, float *hi, float k, float b, float h)
{
    if (fabsf(k) < 1e-6f) {
        if (fabsf(b) > h) {
            *lo = 1.0f;
            *hi = 0.0f;
        }
        return;
    }
    float t1 = (-h - b) / k, t2 = (h - b) / k;
    if (t1 > t2) {
        const float t = t1;
        t1 = t2;
        t2 = t;
    }
    *lo = fmaxf(*lo, t1);
    *hi = fminf(*hi, t2);
}

typedef struct {
    int hits, over;
    float depth, clear;
} lp_hit_t;

/* The hull rectangle at centre c, axis f, half extents (bl, bw): the occupied
 * cells (by their centres) inside it grown by (ml, mw), the deepest of them,
 * and the clearance of the bare hull up to `reach`. -1: it leaves the raster. */
static int lp_rect(const gm_lp_grid_t *g, gm_point_t c, gm_point_t f, float bl, float bw, float ml, float mw, float reach,
                   int want_over, lp_hit_t *out)
{
    const float hl = bl + ml, hw = bw + mw;
    const float ol = fmaxf(hl, bl + reach), ow = fmaxf(hw, bw + reach);
    const float ax = fabsf(f.x), az = fabsf(f.z);
    const float ex = ol * ax + ow * az, ez = ol * az + ow * ax;
    const float inv = 1.0f / GM_LOCAL_CELL;
    const float cx = c.x * inv - (float)g->ox - 0.5f, cz = c.z * inv - (float)g->oz - 0.5f;
    const int c0 = (int)ceilf(cx - ex * inv), c1 = (int)floorf(cx + ex * inv);
    const int r0 = (int)ceilf(cz - ez * inv), r1 = (int)floorf(cz + ez * inv);
    out->hits = out->over = 0;
    out->depth = 0.0f;
    out->clear = reach;
    if (c0 < 0 || r0 < 0 || c1 >= LP_N || r1 >= LP_N)
        return -1;
    if (c1 < c0 || r1 < r0)
        return 0;
    const int any = (int)g->sat[r1 + 1][c1 + 1] - (int)g->sat[r0][c1 + 1] - (int)g->sat[r1 + 1][c0] + (int)g->sat[r0][c0];
    if (!any && !want_over)
        return 0;
    for (int r = r0; r <= r1; ++r) {
        const float dz = ((float)(g->oz + r) + 0.5f) * GM_LOCAL_CELL - c.z;
        float lo = -ex, hi = ex;
        lp_clip(&lo, &hi, f.x, dz * f.z, ol);
        lp_clip(&lo, &hi, -f.z, dz * f.x, ow);
        if (lo > hi)
            continue;
        int ca = (int)ceilf(cx + lo * inv), cb = (int)floorf(cx + hi * inv);
        ca = ca < c0 ? c0 : ca;
        cb = cb > c1 ? c1 : cb;
        if (ca > cb)
            continue;
        for (int w = ca >> 6; w <= cb >> 6; ++w) {
            uint64_t mask = ~0ull << (w == ca >> 6 ? ca & 63 : 0);
            const int top = w == cb >> 6 ? cb & 63 : 63;
            if (top < 63)
                mask &= (1ull << (top + 1)) - 1ull;
            uint64_t bits = g->occ[r][w] & mask;
            uint64_t over = want_over ? g->over[r][w] & mask : 0ull;
            while (bits | over) {
                unsigned long b;
                const int is_occ = bits != 0;
                _BitScanForward64(&b, is_occ ? bits : over);
                if (is_occ)
                    bits &= bits - 1ull;
                else
                    over &= over - 1ull;
                const int col = (w << 6) + (int)b;
                const float dx = ((float)(g->ox + col) + 0.5f) * GM_LOCAL_CELL - c.x;
                const float u = fabsf(dx * f.x + dz * f.z), v = fabsf(dz * f.x - dx * f.z);
                if (!is_occ) {
                    out->over |= u <= bl && v <= bw;
                    continue;
                }
                if (u <= hl && v <= hw) {
                    ++out->hits;
                    const float depth = fminf(hl - u, hw - v) + 1e-3f;
                    if (depth > out->depth)
                        out->depth = depth;
                }
                const float ou = fmaxf(u - bl, 0.0f), ov = fmaxf(v - bw, 0.0f);
                const float d = ou > 0.0f || ov > 0.0f ? hypotf(ou, ov) : 0.0f;
                if (d < out->clear)
                    out->clear = d;
            }
        }
    }
    return 0;
}

static void lp_margins(float speed, float *ml, float *mw)
{
    const float f = gm_clamp((speed - 1.5f) / 2.5f, 0.0f, 1.0f);
    *ml = LP_END_CREEP + f * (LP_END_FAST - LP_END_CREEP);
    *mw = LP_SIDE_CREEP + f * (LP_SIDE_FAST - LP_SIDE_CREEP);
}

/* Hull centre of a rotation-centre pose. */
static gm_point_t lp_centre(const gm_vehicle_t *v, float x, float z, gm_point_t f)
{
    return (gm_point_t){x + f.x * v->hull_forward - f.z * v->hull_right, z + f.z * v->hull_forward + f.x * v->hull_right};
}

/* How far two rectangles reach into each other along their axis of least
 * overlap; <= 0 apart. */
static float lp_box_pen(gm_point_t pa, gm_point_t fa, float la, float wa, gm_point_t pb, gm_point_t fb, float lb, float wb)
{
    const gm_point_t rel = gm_sub(pb, pa);
    const gm_point_t axes[4] = {fa, {fa.z, -fa.x}, fb, {fb.z, -fb.x}};
    float least = INFINITY;
    for (int k = 0; k < 4; ++k) {
        const float ra = la * fabsf(gm_dot(fa, axes[k])) + wa * fabsf(gm_det(fa, axes[k]));
        const float rb = lb * fabsf(gm_dot(fb, axes[k])) + wb * fabsf(gm_det(fb, axes[k]));
        const float o = ra + rb - fabsf(gm_dot(rel, axes[k]));
        if (o < least)
            least = o;
    }
    return least;
}

/* Distance from a point to the bare hull rectangle (0 inside). */
static float lp_point_rect(gm_point_t q, gm_point_t c, gm_point_t f, float hl, float hw)
{
    const gm_point_t d = gm_sub(q, c);
    const float u = fmaxf(fabsf(gm_dot(d, f)) - hl, 0.0f), v = fmaxf(fabsf(gm_det(f, d)) - hw, 0.0f);
    return hypotf(u, v);
}

/* Pure pursuit: the curvature per signed metre of the circle from the pose,
 * tangent to the hull axis, through q (forwards or backwards alike). */
static float lp_pursuit(const gm_pose_t *pose, gm_point_t q, float limit)
{
    const gm_point_t f = gm_axis(pose->heading), rel = gm_sub(q, (gm_point_t){pose->x, pose->z});
    const float d2 = gm_dot(rel, rel);
    if (d2 < 0.25f)
        return 0.0f;
    return gm_clamp(2.0f * gm_det(f, rel) / d2, -limit, limit);
}

/* One static / hull / veto check of the hull at `pose` (time t). 0: refused. */
static int lp_check(lp_ctx_t *x, const gm_pose_t *pose, float t, float ml, float mw, float depth0, const float *veto0,
                    int *spawn_hit, int backward, lp_eval_t *e)
{
    const gm_agent_t *a = x->a;
    const gm_vehicle_t *v = a->v;
    const gm_point_t f = gm_axis(pose->heading), c = lp_centre(v, pose->x, pose->z, f);
    if (--x->budget < 0) {
        e->why = 4;
        return 0;
    }
    ++x->poses;
    lp_hit_t hit;
    if (lp_rect(x->g, c, f, x->bl, x->bw, ml, mw, LP_CLEAR_REACH, x->want_over, &hit) < 0) {
        /* Beyond the raster only the obstacle cells know. */
        if (x->nav && gm_nav_overlap(x->nav, v, pose, mw) > 0) {
            e->why = 1;
            return 0;
        }
    } else {
        e->over |= hit.over;
        if (hit.hits && hit.depth > depth0 + LP_DEPTH_SLACK) {
            e->why = 1;
            return 0;
        }
        e->end_depth = hit.hits ? hit.depth : 0.0f;
        e->clear = fminf(e->clear, hit.clear);
    }
    const float own_diag = hypotf(x->bl + GM_SKIN, x->bw + GM_SKIN);
    for (int j = 0; j < x->hull_count; ++j) {
        const lp_hull_t *h = &x->hulls[j];
        const gm_point_t q = gm_add(h->p, gm_scale(h->vel, fminf(t, LP_HORIZON_S)));
        if (gm_dist(c, q) > own_diag + h->diag + GM_SIDE_GAP)
            continue; /* out of reach of either box */
        const float pen = lp_box_pen(c, f, x->bl + GM_SKIN, x->bw + GM_SKIN, q, h->f, h->hl, h->hw);
        if (pen > fmaxf(h->pen0, 0.0f) + 0.02f) {
            if (!h->moving || t <= LP_MOVING_HARD_S) {
                e->why = 2;
                e->blocker = h->id;
                return 0;
            }
            e->prox = fmaxf(e->prox, 2.0f);
        } else if (pen > -GM_SIDE_GAP) {
            /* Within the side gap of it (pen > -1 m): the closer the dearer. */
            const float near = 1.0f + pen / GM_SIDE_GAP;
            e->prox = fmaxf(e->prox, near * near);
        }
    }
    for (int k = 0; k < 2; ++k) {
        if (!x->veto[k])
            continue;
        const float d = lp_point_rect(x->veto_p[k], c, f, x->bl, x->bw);
        if (d < LP_VETO_R && d < veto0[k] - 0.02f) {
            e->why = 3;
            return 0;
        }
    }
    for (int k = 0; k < x->site_count && k < LP_MAX_SITES; ++k)
        if (spawn_hit[k] != 1 && lp_point_rect(x->sites[k], c, f, x->bl, x->bw) < x->site_r) {
            if (spawn_hit[k] == 2 && lp_point_rect(x->sites[k], c, f, x->bl, x->bw) >= x->site_d0[k] - 0.3f)
                continue; /* getting off the one it stands on (the end of the motion is priced) */
            if (backward) {
                /* Backing onto a pending spawn site: a tank may be born
                 * there behind it (Fulda, s13 model). Never. */
                e->why = 7;
                return 0;
            }
            spawn_hit[k] = 1;
            e->spawn += 1.5f; /* crossing a pending site: a newborn may appear on it */
        }
    return 1;
}

static void lp_eval(lp_ctx_t *x, const lp_cand_t *cand, lp_eval_t *e)
{
    const gm_agent_t *a = x->a;
    const gm_vehicle_t *v = a->v;
    memset(e, 0, sizeof *e);
    e->feasible = 1;
    e->clear = LP_CLEAR_REACH;
    const int two = cand->type == LP_TWO || cand->type == LP_CONTINUE;
    float ml, mw;
    lp_margins(fmaxf(cand->speed, two ? cand->speed2 : 0.0f), &ml, &mw);
    gm_pose_t pose = {a->p.x, a->p.z, v->heading, 0.0f, 0.0f};
    /* Start: how deep it already stands in occupied cells, and in the hulls. */
    const gm_point_t f0 = gm_axis(pose.heading), c0 = lp_centre(v, pose.x, pose.z, f0);
    lp_hit_t hit;
    float depth0 = 0.0f;
    if (lp_rect(x->g, c0, f0, x->bl, x->bw, ml, mw, LP_CLEAR_REACH, 0, &hit) == 0 && hit.hits)
        depth0 = hit.depth;
    e->start_depth = depth0;
    float veto0[2];
    for (int q = 0; q < 2; ++q)
        veto0[q] = x->veto[q] ? lp_point_rect(x->veto_p[q], c0, f0, x->bl, x->bw) : INFINITY;
    /* Pending spawn sites: 0 clear at the start, 1 crossed, 2 under the hull
     * at the start (it may back off that one; every one costs). */
    int spawn_hit[LP_MAX_SITES] = {0};
    for (int k = 0; k < x->site_count; ++k) {
        x->site_d0[k] = lp_point_rect(x->sites[k], c0, f0, x->bl, x->bw);
        spawn_hit[k] = x->site_d0[k] < x->site_r ? 2 : 0;
    }
    float vel = a->speed, k = x->k0;
    float vt = (float)cand->dir * cand->speed, kt = cand->kappa;
    const float horizon = two ? LP_TWO_S : LP_HORIZON_S;
    float t = 0.0f, s1 = 0.0f, travelled = 0.0f, since = 0.0f, turned = 0.0f;
    int phase = 0, early = 0;
    e->heading_1s = pose.heading;
    e->kappa_now = kt;
    while (t < horizon - 1e-4f) {
        t += LP_DT;
        if (two && !phase && s1 >= cand->d1) {
            phase = 1;
            vt = cand->speed2;
            kt = cand->pursuit2 ? lp_pursuit(&pose, x->aim, x->per_m) : cand->kappa2;
        }
        if (t > x->delay) {
            if (vt * vel < 0.0f || vt == 0.0f)
                vel = vel > 0.0f ? fmaxf(0.0f, vel - x->decel * LP_DT) : fminf(0.0f, vel + x->decel * LP_DT);
            else if (fabsf(vel) < fabsf(vt))
                vel += (vt > 0.0f ? 1.0f : -1.0f) * fminf(x->accel * LP_DT, fabsf(vt) - fabsf(vel));
            else
                vel -= (vt > 0.0f ? 1.0f : -1.0f) * fminf(x->decel * LP_DT, fabsf(vel) - fabsf(vt));
            k += (kt - k) * fminf(1.0f, LP_DT / LP_STEER_TAU);
        }
        const float lim = fminf(x->per_m, x->yaw_max / fmaxf(fabsf(vel), 0.05f));
        const float ke = gm_clamp(k, -lim, lim);
        const float ds = vel * LP_DT;
        const float mid = pose.heading + 0.5f * ke * ds;
        pose.x += cosf(mid) * ds;
        pose.z += sinf(mid) * ds;
        pose.heading += ke * ds;
        if (!phase)
            s1 += fabsf(ds);
        travelled += fabsf(ds);
        since += fabsf(ds);
        turned += fabsf(ke * ds);
        if (ds < 0.0f)
            e->s_rev -= ds;
        if (!early && t >= 0.6f - 1e-4f) {
            early = 1;
            e->vel_early = gm_scale(gm_axis(pose.heading), vel);
        }
        if (t <= 1.0f + 1e-4f)
            e->heading_1s = pose.heading;
        const int last = t >= horizon - 1e-4f || (!two && travelled >= LP_HORIZON_M);
        if (since >= LP_CHECK_M || turned >= LP_CHECK_RAD || last) {
            since = turned = 0.0f;
            if (!lp_check(x, &pose, t, ml, mw, depth0, veto0, spawn_hit, ds < 0.0f, e)) {
                e->feasible = 0;
                return;
            }
        }
        if (last)
            break;
    }
    if (!early)
        e->vel_early = gm_scale(gm_axis(pose.heading), vel);
    e->end = (gm_point_t){pose.x, pose.z};
    {
        /* Ending up on a pending spawn site (its own included) keeps a tank
         * from being born there: dearer than crossing one. */
        const gm_point_t fe = gm_axis(pose.heading), ce = lp_centre(v, pose.x, pose.z, fe);
        for (int q = 0; q < x->site_count; ++q)
            if (lp_point_rect(x->sites[q], ce, fe, x->bl, x->bw) < x->site_r)
                e->spawn += 3.0f;
    }
    /* Room to stop: straight on from where the horizon ends. */
    const float vend = fabsf(vel);
    if (vend > 0.5f) {
        const float stop = vend * GM_BRAKE_LAG_S + vend * vend / (2.0f * x->stop_decel);
        const gm_point_t dir = gm_scale(gm_axis(pose.heading), vel > 0.0f ? 1.0f : -1.0f);
        const float tail_depth = fmaxf(depth0, e->end_depth);
        const gm_point_t f = gm_axis(pose.heading);
        for (float d = 1.0f; d < stop + 0.99f; d += 1.0f) {
            const float run = fminf(d, stop);
            gm_pose_t tp = pose;
            tp.x += dir.x * run;
            tp.z += dir.z * run;
            if (--x->budget < 0) {
                e->feasible = 0;
                e->why = 4;
                return;
            }
            ++x->poses;
            const gm_point_t c = lp_centre(v, tp.x, tp.z, f);
            const int inside = lp_rect(x->g, c, f, x->bl, x->bw, ml, mw, 0.0f, 0, &hit) == 0;
            if ((inside && hit.hits && hit.depth > tail_depth + LP_DEPTH_SLACK)
                || (!inside && x->nav && gm_nav_overlap(x->nav, v, &tp, mw) > 0)) {
                e->feasible = 0;
                e->why = 5;
                return;
            }
            /* Nor into a hull that stands. */
            for (int j = 0; j < x->hull_count; ++j) {
                const lp_hull_t *h = &x->hulls[j];
                if (h->moving)
                    continue;
                const float pen = lp_box_pen(c, f, x->bl + GM_SKIN, x->bw + GM_SKIN, h->p, h->f, h->hl, h->hw);
                if (pen > fmaxf(h->pen0, 0.0f) + 0.02f) {
                    e->feasible = 0;
                    e->why = 2;
                    e->blocker = h->id;
                    return;
                }
            }
        }
    }
    /* Cost. */
    const float vd = x->v_des, S = x->S; /* S: what 1 m/s more progress is worth, 2 x max(v_des, 2) */
    const float d0 = gm_dist(a->p, x->aim), d1 = gm_dist(e->end, x->aim);
    e->progress = d0 - d1;
    const float rate = e->progress / horizon;
    const float miss = fmaxf(0.0f, vd - rate);
    float cost = miss * miss;
    if (d1 > 1.5f) {
        const gm_point_t to = gm_sub(x->aim, e->end);
        e->head_err = fabsf(gm_angle(atan2f(to.z, to.x) - pose.heading));
        cost += 1.5f * vd * vd * (e->head_err / GM_PI) * (e->head_err / GM_PI);
    }
    const float c1 = fmaxf(0.0f, 1.0f - e->clear), c4 = fmaxf(0.0f, 0.4f - e->clear);
    cost += S * (0.5f * c1 * c1 + 2.5f * c4);
    if (depth0 > 0.0f)
        cost += S * 2.0f * (e->end_depth - depth0); /* pressed on something: getting off it pays */
    if (e->s_rev > 0.0f)
        cost += S * (0.15f + 0.05f * e->s_rev);
    cost += S * (0.5f * e->prox + 0.5f * e->spawn);
    e->cost = cost;
}

/* Velocity-obstacle price of a velocity against the moving hulls around
 * (gm_choose's weights); the standing ones are in the swept check. */
static float lp_dyn(const gm_pair_t *pairs, int pair_count, gm_point_t vel, float decel, float dyn_weight, float turn)
{
    float cost = 0.0f;
    for (int l = 0; l < pair_count; ++l) {
        if (pairs[l].standing || (pairs[l].rear && turn <= 0.12f))
            continue;
        int level;
        const float amount = gm_pair_risk(&pairs[l], vel, decel, &level);
        cost += amount * (level == 3 ? GM_DYN_WEIGHT : level == 2 ? (pairs[l].yields ? dyn_weight : GM_DYN_WEIGHT)
                                                                  : GM_SOFT_WEIGHT);
    }
    return cost;
}

static void lp_crumb(gm_commitment_t *c, gm_point_t p)
{
    if (c->lp_crumb_count) {
        const int last = (c->lp_crumb_head + GM_LOCAL_CRUMBS - 1) % GM_LOCAL_CRUMBS;
        if (gm_dist(c->lp_crumbs[last], p) < 1.0f)
            return;
    }
    c->lp_crumbs[c->lp_crumb_head] = p;
    c->lp_crumb_head = (c->lp_crumb_head + 1) % GM_LOCAL_CRUMBS;
    if (c->lp_crumb_count < GM_LOCAL_CRUMBS)
        ++c->lp_crumb_count;
}

/* The crumb about 3.5 m back along the trail and behind the hull. */
static int lp_retrace_point(const gm_agent_t *a, gm_point_t *out)
{
    const gm_commitment_t *c = &a->c;
    for (int n = 1; n <= c->lp_crumb_count; ++n) {
        const gm_point_t q = c->lp_crumbs[(c->lp_crumb_head + GM_LOCAL_CRUMBS - n) % GM_LOCAL_CRUMBS];
        if (gm_dist(q, a->p) >= 3.5f && gm_dot(gm_sub(q, a->p), a->fwd) < 0.0f) {
            *out = q;
            return 1;
        }
    }
    return 0;
}

/* s16 exit mode: may it reverse? Pressed on something or wedged (veto), no
 * forward candidate has got anywhere for EX_REV_AFTER_MS, or the hull itself
 * has not got 1.5 m anywhere for EX_STALL_MS (in the game tanks at the spawn
 * backed off whenever the first route leg was blocked). Once it has backed
 * 1.5 m the progress clock starts again, so a reverse is a step, not a way
 * out; a manoeuvre under way is finished all the same (its 'continue'), or
 * cut off after a step each was begun again a few seconds later without the
 * hull getting anywhere. */
static int ex_reverse_gate(gm_commitment_t *c, uint64_t ms, float best_forward_rate, int pressed)
{
    if (best_forward_rate * LP_HORIZON_S > EX_FWD_PROGRESS)
        c->ex_nofwd_ms = 0;
    else if (!c->ex_nofwd_ms)
        c->ex_nofwd_ms = ms;
    return pressed || (c->ex_nofwd_ms && ms >= c->ex_nofwd_ms + EX_REV_AFTER_MS)
        || (c->progress_ms && ms >= c->progress_ms + EX_STALL_MS);
}

static void gm_local_choose(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i, const gm_nav_t *nav)
{
    static gm_pair_t pairs[GM_MAX_VEHICLES];
    static lp_ctx_t x;
    static lp_cand_t cands[128];
    static lp_eval_t evals[128];
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    const uint64_t ms = s->ms;
    const float decel = GM_DECEL_FRACTION * v->braking;
    const int pair_count = gm_pairs(agents, count, i, pairs);

    lp_build(s, nav, a, &gm_lp_grid);
    memset(&x, 0, sizeof x);
    x.s = s;
    x.nav = nav;
    x.a = a;
    x.g = &gm_lp_grid;
    x.bl = a->half_length;
    x.bw = a->half_width;
    x.per_m = v->turn_per_m > 0.0f ? v->turn_per_m : v->can_pivot ? GM_TRACKED_TURN_PER_M : GM_WHEELED_TURN_PER_M;
    /* Kept below what the steering can give, so that it has some left; and
     * the acceleration in full, which makes the stopping distances safe. */
    x.yaw_max = 0.9f * fminf(v->max_yaw_rate, GM_LOCAL_YAW_CAP);
    x.accel = v->acceleration;
    x.decel = decel;
    x.stop_decel = decel;
    x.delay = v->command_delay_s + 0.1f;
    x.v_des = a->v_des;
    x.S = 2.0f * fmaxf(a->v_des, 2.0f);
    x.aim = a->aim;
    if (gm_dist(x.aim, a->p) < 0.5f)
        x.aim = gm_add(a->p, gm_scale(gm_len(a->pref) > 0.05f ? gm_unit(a->pref) : a->fwd, 6.0f));
    /* The curvature it rolls on now. */
    x.k0 = fabsf(a->speed) > 0.5f ? gm_clamp(v->yaw_rate / a->speed, -x.per_m, x.per_m)
                                  : c->lp_valid ? c->lp_kappa : 0.0f;
    const int pool = LP_SNAPSHOT_POSES - gm_lp_snapshot_poses;
    x.budget = pool < LP_VEHICLE_POSES ? pool : LP_VEHICLE_POSES;
    const int reduced = x.budget < LP_VEHICLE_POSES / 2;
    /* Hulls that can come into reach of the horizon and of the room it takes
     * to stop after it. */
    const float fastest = fmaxf(fabsf(a->speed), a->v_des) + 1.0f; /* no candidate is faster than v_des */
    const float stop_reach = fastest * GM_BRAKE_LAG_S + fastest * fastest / (2.0f * x.stop_decel);
    for (int j = 0; j < count && x.hull_count < LP_MAX_HULLS; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i || gm_dist(a->p, b->p) > a->radius + b->radius + LP_HORIZON_M + stop_reach
                                               + gm_len(b->vel) * LP_HORIZON_S + 2.0f)
            continue;
        lp_hull_t *h = &x.hulls[x.hull_count++];
        h->f = b->fwd;
        h->p = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
        h->vel = b->stationary ? (gm_point_t){0.0f, 0.0f} : b->vel;
        h->hl = b->half_length;
        h->hw = b->half_width;
        h->id = b->v->id;
        h->moving = !b->stationary;
        h->diag = hypotf(h->hl, h->hw);
        const gm_point_t f0 = gm_axis(v->heading);
        h->pen0 = lp_box_pen(lp_centre(v, a->p.x, a->p.z, f0), f0, x.bl + GM_SKIN, x.bw + GM_SKIN, h->p, h->f, h->hl, h->hw);
    }
    for (int q = 0; q < 2; ++q) {
        x.veto[q] = c->lp_veto_until_ms[q] > ms;
        x.veto_p[q] = c->lp_veto[q];
    }
    /* A pending spawn site counts as crossed when the hull comes within the
     * reach of a newborn hull of any heading from the site's point. */
    x.site_r = 3.5f;
    for (int k = 0; k < s->pending_spawn_site_count && x.site_count < LP_MAX_SITES; ++k)
        if (gm_dist(s->pending_spawn_sites[k].p, a->p) < LP_HORIZON_M + a->radius + x.site_r) {
            if (s->pending_spawn_sites[k].half_length > 0.0f)
                x.site_r = hypotf(s->pending_spawn_sites[k].half_length, s->pending_spawn_sites[k].half_width);
            x.sites[x.site_count++] = s->pending_spawn_sites[k].p;
        }
    /* s16 exit mode: the aim lies along the open way out gm_spawn_queue chose
     * (ex_scan), as far as the hull gets along it; the birth heading is only
     * a bonus there. Without any way out giving room: the route aim. */
    const int exit_mode = a->ex_mode;
    const int exit_aim = exit_mode && c->ex_valid && !c->ex_route && c->ex_free >= EX_MIN_FREE;
    if (exit_aim)
        x.aim = gm_add(a->p, gm_scale(c->ex_dir, fminf(c->ex_free, EX_AIM_M)));
    if (!exit_mode) {
        /* Standing on a pending spawn site, near where it was born: first off
         * it, the way it was born facing (the way out of the spawn the map
         * gives), then the route. Only that far: s12 held the birth heading
         * through the whole 12 m field around every pending site and drove
         * Sweden's tanks into walls (lab run 20261115_235404, seats 1, 3, 13).
         * The motion itself is checked like any other. */
        const gm_point_t f0 = gm_axis(v->heading), c0 = lp_centre(v, a->p.x, a->p.z, f0);
        int on_site = 0;
        for (int k = 0; k < x.site_count && !on_site; ++k)
            on_site = lp_point_rect(x.sites[k], c0, f0, x.bl, x.bw) < x.site_r;
        const gm_point_t born = c->sq_join_ms && c->sq_state ? c->sq_spawn : a->p;
        if (on_site && c->birth_valid && gm_dist(a->p, born) < LP_EXIT_M)
            x.aim = gm_add(born, gm_scale(gm_axis(c->birth_heading), LP_EXIT_M + 6.0f));
    }

    /* Candidates. */
    int n = 0;
    const float vd = a->v_des;
    static const float kfrac[7] = {-1.0f, -0.5f, -0.2f, 0.0f, 0.2f, 0.5f, 1.0f};
    float speeds[6];
    int ns = 0;
    if (vd > 0.3f) {
        const float list[6] = {vd, 0.75f * vd, 0.5f * vd, 0.3f * vd, LP_TWO_SPEED, LP_CREEP};
        for (int q = 0; q < 6; ++q) {
            const float sp = fminf(list[q], vd);
            int dup = sp < GM_MOVE_MIN;
            for (int m = 0; m < ns && !dup; ++m)
                dup = fabsf(speeds[m] - sp) < 0.25f;
            if (!dup && (!reduced || q == 0 || q >= 4))
                speeds[ns++] = sp;
        }
        if (!ns)
            speeds[ns++] = fmaxf(vd, LP_CREEP);
    }
    const gm_pose_t here = {a->p.x, a->p.z, v->heading, 0.0f, 0.0f};
    for (int q = 0; q < ns; ++q) {
        const float lim = fminf(x.per_m, x.yaw_max / fmaxf(speeds[q], 0.05f));
        for (int m = 0; m < 7; ++m) {
            if (reduced && (m == 1 || m == 2 || m == 4 || m == 5))
                continue;
            cands[n++] = (lp_cand_t){.type = LP_SINGLE, .dir = 1, .kindex = m, .speed = speeds[q], .kappa = kfrac[m] * lim};
        }
        cands[n++] = (lp_cand_t){.type = LP_SINGLE, .dir = 1, .kindex = 7, .speed = speeds[q],
                                 .kappa = lp_pursuit(&here, x.aim, lim)};
    }
    const int forward_end = n;
    /* s16 exit mode, a way out more than EX_TURN_FIRST off the heading: a
     * hull turns only as it rolls, so rolling on with the steering less than
     * half over only eats the room it needs for the turn (side by side before
     * a fence, the tanks crept up to it and then could turn neither way).
     * Forward then means turning that way; else standing, and after a while
     * backing up (ex_reverse_gate). */
    const float ex_need = exit_aim ? gm_angle(atan2f(c->ex_dir.z, c->ex_dir.x) - v->heading) : 0.0f;
    const int turn_first = exit_aim && fabsf(ex_need) > EX_TURN_FIRST;
    const float turn_sign = ex_need > 0.0f ? 1.0f : -1.0f;
    /* Among pending spawn sites (gm_follow's spawn field) no long manoeuvres:
     * backing up and swinging round there is how newborns get hit (Fulda, s13
     * model). Unless it presses on something or is wedged. Short reverses stay,
     * and none may back onto a pending site (lp_check). */
    int start_pressed = 0;
    {
        const gm_point_t f0 = gm_axis(v->heading);
        lp_hit_t h0;
        start_pressed = lp_rect(&gm_lp_grid, lp_centre(v, a->p.x, a->p.z, f0), f0, x.bl, x.bw, LP_END_CREEP,
                                LP_SIDE_CREEP, 0.0f, 0, &h0) == 0 && h0.hits > 0;
    }
    const int may_manoeuvre = !a->among || start_pressed || x.veto[0] || x.veto[1];
    if (!may_manoeuvre)
        c->lp_seg_active = 0;
    const float rs = fminf(v->reverse_speed, GM_ESCAPE_SPEED);
    if (vd > 0.3f && rs > 0.3f) {
        static const float rfrac[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
        for (int m = 0; m < 5; ++m)
            cands[n++] = (lp_cand_t){.type = LP_SINGLE, .dir = -1, .kindex = -1, .speed = rs, .kappa = rfrac[m] * x.per_m};
        gm_point_t crumb;
        if (lp_retrace_point(a, &crumb))
            cands[n++] = (lp_cand_t){.type = LP_RETRACE, .dir = -1, .kindex = -1, .speed = rs,
                                     .kappa = lp_pursuit(&here, crumb, x.per_m)};
        if (c->lp_seg_active) {
            /* The path driven since the last snapshot counts off the first
             * segment (on a full-lock arc the chord from its start would
             * never reach 5 m: Fulda seat 10 circled backwards for minutes). */
            const float left = c->lp_seg_d1 - gm_dist(a->p, c->lp_seg_from);
            c->lp_seg_d1 = left;
            c->lp_seg_from = a->p;
            if (left > 0.3f)
                cands[n++] = (lp_cand_t){.type = LP_CONTINUE, .dir = -1, .kindex = -1, .speed = rs, .kappa = c->lp_seg_k1,
                                         .d1 = left, .speed2 = c->lp_seg_speed2, .kappa2 = c->lp_seg_k2,
                                         .pursuit2 = c->lp_seg_pursuit};
            else
                c->lp_seg_active = 0;
        }
    }
    int feasible = 0, static_refused = 0, hull_refused = 0, best = -1;
    uint64_t hull_blocker = 0;
    const float stalled = c->progress_ms && a->v_des > 1.0f && ms > c->progress_ms + 1000u
        ? fminf(20.0f, 0.001f * (float)(ms - c->progress_ms - 1000u)) : 0.0f;
    const float dyn_weight = GM_DYN_WEIGHT / (1.0f + GM_IMPATIENCE * stalled);
    const float S = x.S;
    float best_forward_rate = -INFINITY;
    /* What holds it: hulls in its way (velocity obstacles on the way it
     * wants, forward candidates refused for hulls) is traffic, to be waited
     * for; the rest is static and a manoeuvre may get it out. */
    float pref_risk = 0.0f;
    uint64_t pref_blocker = 0;
    /* (s16 exit mode: the way it wants is the way out.) */
    const gm_point_t pref = exit_aim ? gm_scale(c->ex_dir, gm_len(a->pref)) : a->pref;
    for (int l = 0; l < pair_count; ++l) {
        if (pairs[l].rear)
            continue;
        int level;
        const float amount = gm_pair_risk(&pairs[l], pref, decel, &level);
        const float risk = level >= 2 ? (float)(level - 1) + amount : 0.0f;
        if (risk > pref_risk) {
            pref_risk = risk;
            pref_blocker = pairs[l].id;
        }
    }
    float head_now = 0.0f;
    if (gm_dist(a->p, x.aim) > 1.5f) {
        const gm_point_t to = gm_sub(x.aim, a->p);
        head_now = fabsf(gm_angle(atan2f(to.z, to.x) - v->heading));
    }
    int fwd_static = 0, fwd_hull = 0, traffic_held = 0, pressed = 0;
    /* s16 exit mode: reversing (and the manoeuvres that start backwards) only
     * once forward has been judged (ex_reverse_gate); always outside it. */
    int ex_rev = 1, ex_known = !exit_mode;
    const int ex_pressed = start_pressed || x.veto[0] || x.veto[1];
    for (int pass = 0; pass < 2; ++pass) {
        if (pass == 1) {
            if (!ex_known) {
                ex_rev = ex_reverse_gate(c, ms, best_forward_rate, ex_pressed);
                ex_known = 1;
            }
            /* Going forward gets nowhere for static reasons, the hull presses
             * on something, is wedged, or its way lies behind it: back up to
             * swing the nose, then forward. Not in traffic. */
            pressed = n > 0 && evals[0].start_depth > 0.0f;
            traffic_held = pref_risk >= 1.0f || fwd_hull > fwd_static;
            const int static_held = !traffic_held && best_forward_rate < 0.35f * vd;
            if (reduced || !may_manoeuvre || !(vd > 0.3f && rs > 0.3f) || !ex_rev
                || !(static_held || x.veto[0] || x.veto[1] || pressed || head_now > 1.2f))
                break;
            static const float k1f[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
            static const float d1s[2] = {2.5f, 5.0f};
            for (int q = 0; q < 2; ++q)
                for (int m = 0; m < 5; ++m)
                    for (int r = 0; r < 3; ++r)
                        cands[n++] = (lp_cand_t){.type = LP_TWO, .dir = -1, .kindex = -1, .speed = rs,
                                                 .kappa = k1f[m] * x.per_m, .d1 = d1s[q], .speed2 = LP_TWO_SPEED,
                                                 .kappa2 = r == 0 ? -x.per_m : r == 1 ? x.per_m : 0.0f,
                                                 .pursuit2 = r == 2};
        }
        for (int q = pass ? forward_end : 0; q < n; ++q) {
            if (pass == 0 && cands[q].type == LP_TWO)
                continue;
            if (pass == 1 && cands[q].type != LP_TWO)
                continue;
            lp_eval_t *e = &evals[q];
            if (!ex_known && q >= forward_end) {
                /* All forward candidates judged (they come first). */
                ex_rev = ex_reverse_gate(c, ms, best_forward_rate, ex_pressed);
                ex_known = 1;
            }
            if (!ex_rev && cands[q].dir < 0 && cands[q].type != LP_CONTINUE) {
                memset(e, 0, sizeof *e);
                e->why = 8; /* s16 exit mode: no reversing yet */
                continue;
            }
            if (turn_first && cands[q].type == LP_SINGLE && cands[q].dir > 0
                && cands[q].kappa * turn_sign < 0.5f * fminf(x.per_m, x.yaw_max / fmaxf(cands[q].speed, 0.05f))) {
                memset(e, 0, sizeof *e);
                e->why = 9; /* s16 exit mode: the way out needs the turn first */
                continue;
            }
            if (best >= 0) {
                /* No better than the best so far even with all the progress
                 * its speed allows and every bonus: not simulated. */
                const int two = cands[q].type == LP_TWO || cands[q].type == LP_CONTINUE;
                const float miss = fmaxf(0.0f, vd - fmaxf(cands[q].speed, two ? cands[q].speed2 : 0.0f));
                const float bonus = S * (0.25f + 2.0f * evals[0].start_depth);
                if (miss * miss - bonus > evals[best].cost) {
                    memset(e, 0, sizeof *e);
                    e->why = 6;
                    continue;
                }
            }
            lp_eval(&x, &cands[q], e);
            if (!e->feasible) {
                static_refused += e->why == 1 || e->why == 5 || e->why == 3;
                hull_refused += e->why == 2;
                if (cands[q].type == LP_SINGLE && cands[q].dir > 0) {
                    fwd_static += e->why == 1 || e->why == 5 || e->why == 3;
                    fwd_hull += e->why == 2;
                }
                if (e->why == 2 && !hull_blocker)
                    hull_blocker = e->blocker;
                continue;
            }
            ++feasible;
            const float turn = fabsf(gm_angle(e->heading_1s - v->heading));
            e->dyn = lp_dyn(pairs, pair_count, e->vel_early, decel, dyn_weight, turn);
            e->cost += e->dyn;
            if (exit_mode && cands[q].dir < 0)
                e->cost += S * EX_REV_COST; /* s16: out of the spawn forwards if at all possible */
            /* Hysteresis against the previous choice. */
            if (c->lp_valid) {
                const float dk = (e->kappa_now - c->lp_kappa) / x.per_m;
                e->cost += S * 0.1f * dk * dk;
                if (cands[q].dir != c->lp_dir && c->lp_speed > 0.3f && cands[q].type != LP_CONTINUE)
                    e->cost += S * 0.5f; /* no dithering between forwards and backwards */
            }
            if (cands[q].type == LP_RETRACE)
                e->cost -= S * 0.05f;
            if (cands[q].type == LP_SINGLE && cands[q].dir > 0)
                best_forward_rate = fmaxf(best_forward_rate, e->progress / LP_HORIZON_S);
            if (cands[q].type == LP_CONTINUE) {
                /* A manoeuvre under way is driven to its end while it stays
                 * feasible: re-weighing it every tick against going forward
                 * again had a hull at a corner (Spaceport seat 26) back up a
                 * metre and creep back into the same corner for minutes. */
                e->cost = -INFINITY;
            }
            if (best < 0 || e->cost < evals[best].cost)
                best = q;
        }
    }
    gm_lp_snapshot_poses += x.poses;
    traffic_held = pref_risk >= 1.0f || fwd_hull > fwd_static;

    /* Standing: the progress it does not make. It is judged like a motion
     * (its clearance, the hulls coming at it), and held for static reasons it
     * grows dearer the longer the hull makes no progress: what such a hull
     * could do at a price (back out, go close by a wall) it does after a few
     * seconds rather than stand for good. In traffic it waits (the velocity
     * obstacles' own impatience eases it through). */
    float stand_cost = vd * vd + 1.5f * vd * vd * (head_now / GM_PI) * (head_now / GM_PI) + 0.05f * S;
    if (vd > 0.3f) {
        const lp_cand_t stay = {.type = LP_STAND, .dir = 1, .kindex = -1};
        lp_eval_t se;
        x.budget += 40;
        lp_eval(&x, &stay, &se);
        if (se.feasible)
            stand_cost = se.cost + 0.05f * S;
        const float idle_s = c->progress_ms && ms > c->progress_ms ? 0.001f * (float)(ms - c->progress_ms) : 0.0f;
        if (!traffic_held || x.veto[0] || x.veto[1] || pressed)
            stand_cost += S * fminf(3.0f, 0.5f * fmaxf(0.0f, idle_s - 1.0f));
    }
    stand_cost += lp_dyn(pairs, pair_count, (gm_point_t){0.0f, 0.0f}, decel, dyn_weight, 0.0f);
    const int moving = best >= 0 && evals[best].cost < stand_cost;
#ifdef GM_TRACE_CHOICE
    {
        static long traced = -2;
        static double from_s = 0.0, to_s = 1e9;
        if (traced == -2) {
            const char *id = getenv("GM_TRACE_CHOICE"), *win = getenv("GM_TRACE_WINDOW");
            traced = id ? atol(id) : -1;
            if (win)
                sscanf(win, "%lf,%lf", &from_s, &to_s);
        }
        const double now_s = (double)ms / 1000.0 - 100.0;
        if ((long)v->id == traced && now_s >= from_s && now_s <= to_s) {
            fprintf(stderr, "local t=%.2f p=(%.1f,%.1f) hdg %.2f v %.2f v_des %.2f aim (%.1f,%.1f) stand %.2f best %d\n", now_s,
                    (double)a->p.x, (double)a->p.z, (double)v->heading, (double)a->speed, (double)vd, (double)x.aim.x,
                    (double)x.aim.z, (double)stand_cost, best);
            for (int q = 0; q < n; ++q)
                fprintf(stderr, "  %2d %-8s d%+d sp %.2f k %+.3f d1 %.1f k2 %+.3f | %s why %d cost %.2f pr %.2f cl %.2f he %.2f dyn %.2f px %.2f sp %.1f rev %.1f\n",
                        q, lp_type_name[cands[q].type], cands[q].dir, (double)cands[q].speed, (double)cands[q].kappa,
                        (double)cands[q].d1, (double)cands[q].kappa2, evals[q].feasible ? "ok " : "NO ", evals[q].why,
                        (double)evals[q].cost, (double)evals[q].progress, (double)evals[q].clear, (double)evals[q].head_err,
                        (double)evals[q].dyn, (double)evals[q].prox, (double)evals[q].spawn, (double)evals[q].s_rev);
        }
    }
#endif
    const lp_cand_t *ch = moving ? &cands[best] : NULL;
    const lp_eval_t *ev = moving ? &evals[best] : NULL;
    const int wants = vd > 0.3f;

    a->lp_moving = moving;
    a->no_turn_room = 0;
    a->geometry = GM_CLEAR;
    a->blocker = 0;
    a->pref_blocked = pref_risk >= 1.0f;
    a->turn_boxed = 0;
    a->firm = 0;
    if (moving) {
        const int reverse = ch->dir < 0;
        a->cmd_speed = (float)ch->dir * ch->speed;
        a->cmd_curvature = ev->kappa_now;
        a->cmd_heading = ev->heading_1s;
        a->cmd_yaw = ev->kappa_now * a->cmd_speed;
        a->reversing = reverse;
        a->state = GM_DRIVING;
        if (!reverse) {
            a->blocker = a->pref_blocked ? pref_blocker : 0;
            /* Firm: a faster candidate on this curvature was refused for room. */
            int faster_refused = 0;
            for (int q = 0; q < forward_end && !faster_refused; ++q)
                faster_refused = cands[q].kindex == ch->kindex && cands[q].speed > ch->speed + 0.05f
                    && !evals[q].feasible && (evals[q].why == 1 || evals[q].why == 5);
            a->firm = faster_refused || (a->approach && ch->speed >= a->v_des - 0.05f);
        }
    } else {
        a->cmd_speed = a->cmd_yaw = a->cmd_curvature = 0.0f;
        a->cmd_heading = v->heading;
        a->reversing = 0;
        if (!wants) {
            a->state = GM_WAIT_TRAFFIC; /* held before a single-lane stretch or at the route end */
            a->blocker = a->hold_blocker;
        } else if (feasible && pref_risk >= 1.0f) {
            a->state = GM_WAIT_TRAFFIC;
            a->blocker = pref_blocker;
        } else if (hull_refused && (feasible || hull_refused >= static_refused / 4)) {
            a->state = GM_WAIT_TRAFFIC;
            a->blocker = hull_blocker ? hull_blocker : pref_blocker;
            a->turn_boxed = 1;
        } else if (feasible) {
            a->state = GM_WAIT_TRAFFIC; /* moving costs more than standing: traffic */
            a->blocker = pref_blocker;
        } else {
            a->state = GM_STATIC_BLOCKED;
            a->geometry = GM_BLOCKED;
        }
    }
    if (exit_mode && c->ex_route) {
        /* s16: towards the route aim it gets nowhere (stands, backs, or rolls
         * under EX_FWD_PROGRESS over the horizon) for EX_ROUTE_FAIL_MS: the
         * open way out (ex_scan), until the route's way is open again. */
        const int going = moving && ch->dir > 0 && ev->progress > EX_FWD_PROGRESS;
        if (going) {
            c->ex_route_fail_ms = 0;
        } else if (!c->ex_route_fail_ms) {
            c->ex_route_fail_ms = ms;
        } else if (ms >= c->ex_route_fail_ms + EX_ROUTE_FAIL_MS && c->ex_valid && c->ex_free >= EX_MIN_FREE) {
            c->ex_route = 0;
            c->ex_open_ms = ms;
            c->ex_route_fail_ms = 0;
            if (s->local_log) {
                char line[200];
                snprintf(line, sizeof line, "%llu queue: id %llu gets nowhere towards its route aim: out by the open way %.2f rad"
                         " (room %.1f m)", (unsigned long long)ms, (unsigned long long)v->id,
                         (double)atan2f(c->ex_dir.z, c->ex_dir.x), (double)c->ex_free);
                s->local_log(s->local_context, 1, line);
            }
        }
    }
    /* Commitment: the previous choice, and a manoeuvre under way. */
    c->lp_valid = 1;
    c->lp_type = moving ? ch->type : LP_STAND;
    c->lp_dir = moving ? ch->dir : 0;
    c->lp_kappa = moving ? ev->kappa_now : 0.0f;
    c->lp_speed = moving ? ch->speed : 0.0f;
    if (moving && ch->type == LP_TWO) {
        c->lp_seg_active = 1;
        c->lp_seg_from = a->p;
        c->lp_seg_d1 = ch->d1;
        c->lp_seg_k1 = ch->kappa;
        c->lp_seg_k2 = ch->kappa2;
        c->lp_seg_speed2 = ch->speed2;
        c->lp_seg_pursuit = ch->pursuit2;
    } else if (!moving || ch->type != LP_CONTINUE) {
        c->lp_seg_active = 0;
    }

    /* Log at 2 Hz per vehicle, and on a change of state, type or direction. */
    if (s->local_log && (!c->lp_log_ms || ms >= c->lp_log_ms + LP_LOG_MS || c->lp_log_state != (int)a->state
                         || c->lp_log_type != c->lp_type || c->lp_log_dir != c->lp_dir)) {
        int over = 0;
        if (moving) {
            /* Does the chosen way pass prior-solid cells the lidar sees free? */
            lp_eval_t again;
            x.want_over = 1;
            x.budget = 200;
            lp_eval(&x, ch, &again);
            over = again.over;
        }
        char line[512];
        snprintf(line, sizeof line,
                 "%llu,%llu,%.2f,%.2f,%.3f,%.2f,%d,%.2f,%s,%d,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f"
                 ",%d,%.3f,%.1f",
                 (unsigned long long)ms, (unsigned long long)v->id, (double)a->p.x, (double)a->p.z,
                 (double)v->heading, (double)a->speed, (int)a->state, (double)vd,
                 lp_type_name[moving ? ch->type : LP_STAND], moving ? ch->dir : 0, (double)a->cmd_speed,
                 (double)a->cmd_curvature, moving ? (double)ev->cost : 0.0, moving ? (double)ev->progress : 0.0,
                 moving ? (double)ev->clear : 0.0, moving ? (double)ev->head_err : (double)head_now,
                 moving ? (double)ev->dyn : 0.0, moving ? (double)ev->prox : 0.0, moving ? (double)ev->end_depth : 0.0,
                 n ? (double)evals[0].start_depth : 0.0, feasible, n, static_refused, hull_refused,
                 gm_lp_grid.lidar_cells, gm_lp_grid.override_cells, over, x.poses, x.veto[0] + 2 * x.veto[1],
                 (double)stand_cost,
                 /* s16: exit mode 0 off, 1 on (no way out with room: route aim), 2 on, aiming out,
                  * 3 on, the route's own way open (route aim) */
                 exit_aim ? 2 : exit_mode && c->ex_route ? 3 : exit_mode, exit_mode && c->ex_valid ? (double)atan2f(c->ex_dir.z, c->ex_dir.x) : 0.0,
                 exit_mode && c->ex_valid ? (double)c->ex_free : 0.0);
        s->local_log(s->local_context, 0, line);
        c->lp_log_ms = ms;
        c->lp_log_state = (int)a->state;
        c->lp_log_type = c->lp_type;
        c->lp_log_dir = c->lp_dir;
    }
}

/* ---- s13: spawn exit queue (local planner only). ----
 * Design decision (queue out in order): tanks that spawned near each
 * other leave the spawn one after another instead of yielding to each other.
 *  - Group: queued tanks whose spawn points lie within SQ_GROUP_M of each
 *    other (no army field in the snapshot: the two sides spawn far apart).
 *  - A tank joins when first seen (new life, standing) with its spawn point
 *    and its way out (towards the route point it aims at). Order: tanks that
 *    joined more than SQ_LATE_MS after the oldest member near them (later
 *    births, respawns) at the tail; then the one furthest along the group's
 *    common way out (mean of the members' ways out), ties by id. The order
 *    follows from fixed spawn points and ways out: it changes only on a join.
 *  - Every tank behind another of its group that is within SQ_GAP_M holds
 *    still with the brakes on (GM_WAIT_SPAWN, no candidate search). Once let
 *    go it stays let go. Valves: a released tank making no progress for
 *    SQ_WEDGE_MS holds nobody (logged); nobody is held without a tank ahead
 *    within SQ_GAP_M. (Being hit is not in the snapshot: no valve for it.)
 *  - It leaves the queue SQ_LEAVE_M from its spawn point. */
#define SQ_GROUP_M 40.0f
#define SQ_GAP_M 15.0f
#define SQ_LEAVE_M 40.0f
#define SQ_LATE_MS 30000u
#define SQ_WEDGE_MS 4000u
#define SQ_LANE_GAP 1.5f           /* m between hull sides that makes a separate lane */

static void sq_log(const gm_snapshot_t *s, const char *fmt, ...)
{
    if (!s->local_log)
        return;
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    s->local_log(s->local_context, 1, line);
}

/* Is j ahead of i in the queue of their group (direction d)? */
static int sq_ahead(const gm_agent_t *j, const gm_agent_t *i, gm_point_t d)
{
    if (j->c.sq_late != i->c.sq_late)
        return j->c.sq_late < i->c.sq_late;
    const float kj = gm_dot(j->c.sq_spawn, d), ki = gm_dot(i->c.sq_spawn, d);
    if (fabsf(kj - ki) > 0.01f)
        return kj > ki;
    return j->v->id < i->v->id;
}

/* ---- s16: exit mode, the open way out of the spawn (snapshot spawn_exit). ----
 * Per tank in the queue for at most EX_MODE_MS: the hull rectangle, turned to
 * each of EX_DIRS directions, is marched out from where it stands in
 * EX_STEP_M steps until it would press deeper into an occupied cell of the
 * local raster (beyond it the obstacle cells) or into a standing hull (moving
 * ones do not count): the room along that direction. Score: the room, plus
 * EX_AWAY_W for leading straight away from the spawn cluster (its spawn
 * point, the pending sites and the spawn points of its group), EX_BIRTH_W
 * along the birth heading, EX_ROUTE_W towards the route aim, less EX_TURN_W
 * per radian of turn. The direction kept last time stays unless its room
 * fell under EX_BLOCKED_M or another scores EX_BETTER better. gm_local_choose
 * aims along it; the queue holds a tank only for a group member lying in its
 * corridor (its hull swept EX_REACH_M along the way out). */
typedef struct {
    gm_point_t p, f;
    float hl, hw;
} ex_hull_t;

/* Pending spawn sites around the tank being scanned (ex_scan): a newborn may
 * appear on one, so the room ends where the hull comes nearer to one than
 * site_r (and nearer than it starts: the one it stands on it may leave). */
static gm_point_t ex_sites[LP_MAX_SITES];
static float ex_site_r, ex_site_d0[LP_MAX_SITES];
static int ex_site_count;

/* s17: the overhead cache (snapshot world_block) around the tank being
 * scanned, EX_WORLD_CELL cells over +-36 m: beyond the 32 m raster its
 * obstacles end the room as the obstacle cells do. EX_REACH_M stays 15 m:
 * marching 30 m costs little more here (array reads), but it changes the
 * scale of the scores (room against the bonuses) and needs its own trial. */
#define EX_WORLD_CELL 0.5f
#define EX_WORLD_N 144
static uint8_t ex_world[EX_WORLD_N * EX_WORLD_N];
static int ex_world_ok, ex_world_wc0, ex_world_wr0;

/* An occupied cache cell (by its centre) inside the hull rectangle at centre
 * c, axis f, half extents (hl, hw)? */
static int ex_world_hits(gm_point_t c, gm_point_t f, float hl, float hw)
{
    const float ex = hl * fabsf(f.x) + hw * fabsf(f.z), ez = hl * fabsf(f.z) + hw * fabsf(f.x);
    const float inv = 1.0f / EX_WORLD_CELL;
    const int c0 = (int)ceilf((c.x - ex) * inv - 0.5f) - ex_world_wc0, c1 = (int)floorf((c.x + ex) * inv - 0.5f) - ex_world_wc0;
    const int r0 = (int)ceilf((c.z - ez) * inv - 0.5f) - ex_world_wr0, r1 = (int)floorf((c.z + ez) * inv - 0.5f) - ex_world_wr0;
    for (int r = r0 > 0 ? r0 : 0; r <= r1 && r < EX_WORLD_N; ++r) {
        const float dz = ((float)(ex_world_wr0 + r) + 0.5f) * EX_WORLD_CELL - c.z;
        for (int col = c0 > 0 ? c0 : 0; col <= c1 && col < EX_WORLD_N; ++col) {
            if (ex_world[r * EX_WORLD_N + col] != GM_WORLD_OCCUPIED)
                continue;
            const float dx = ((float)(ex_world_wc0 + col) + 0.5f) * EX_WORLD_CELL - c.x;
            if (fabsf(dx * f.x + dz * f.z) <= hl && fabsf(-dx * f.z + dz * f.x) <= hw)
                return 1;
        }
    }
    return 0;
}

static float ex_march(const gm_nav_t *nav, const gm_agent_t *a, gm_point_t c0, gm_point_t f, const ex_hull_t *hulls,
                      int hull_count)
{
    for (int k = 0; k < ex_site_count; ++k)
        ex_site_d0[k] = lp_point_rect(ex_sites[k], c0, f, a->half_length, a->half_width);
    const gm_vehicle_t *v = a->v;
    const float bl = a->half_length, bw = a->half_width;
    lp_hit_t hit;
    float depth0 = 0.0f;
    if (lp_rect(&gm_lp_grid, c0, f, bl, bw, LP_END_CREEP, LP_SIDE_CREEP, 0.0f, 0, &hit) == 0 && hit.hits)
        depth0 = hit.depth;
    static float pen0[LP_MAX_HULLS];
    for (int j = 0; j < hull_count; ++j)
        pen0[j] = fmaxf(lp_box_pen(c0, f, bl + GM_SKIN, bw + GM_SKIN, hulls[j].p, hulls[j].f, hulls[j].hl, hulls[j].hw), 0.0f);
    const float diag = hypotf(bl + GM_SKIN, bw + GM_SKIN);
    float room = 0.0f;
    for (float d = EX_STEP_M; d <= EX_REACH_M + 1e-3f; d += EX_STEP_M) {
        const gm_point_t c = gm_add(c0, gm_scale(f, d));
        if (lp_rect(&gm_lp_grid, c, f, bl, bw, LP_END_CREEP, LP_SIDE_CREEP, 0.0f, 0, &hit) < 0) {
            /* Beyond the raster only the obstacle cells know (s17: and the
             * overhead cache). */
            if (ex_world_ok && ex_world_hits(c, f, bl + LP_END_CREEP, bw + LP_SIDE_CREEP))
                break;
            const gm_pose_t pose = {c.x - f.x * v->hull_forward + f.z * v->hull_right,
                                    c.z - f.z * v->hull_forward - f.x * v->hull_right, atan2f(f.z, f.x), 0.0f, 0.0f};
            if (nav && gm_nav_overlap(nav, v, &pose, LP_SIDE_CREEP) > 0)
                break;
        } else if (hit.hits && hit.depth > depth0 + LP_DEPTH_SLACK) {
            break;
        }
        int hull = 0;
        for (int j = 0; j < hull_count && !hull; ++j)
            hull = gm_dist(c, hulls[j].p) < diag + hypotf(hulls[j].hl, hulls[j].hw)
                && lp_box_pen(c, f, bl + GM_SKIN, bw + GM_SKIN, hulls[j].p, hulls[j].f, hulls[j].hl, hulls[j].hw) > pen0[j] + 0.02f;
        for (int k = 0; k < ex_site_count && !hull; ++k) {
            const float ds = lp_point_rect(ex_sites[k], c, f, bl, bw);
            hull = ds < ex_site_r && ds < ex_site_d0[k] - 0.3f;
        }
        if (hull)
            break;
        room = d;
    }
    return room;
}

static void ex_scan(const gm_snapshot_t *s, const gm_nav_t *nav, gm_agent_t *agents, int count, int i)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    /* The open way out is needed only where it may be taken: not let go yet
     * (the queue), getting nowhere towards the route aim, or steering for it
     * already. A tank let go that rolls on along its route is not scanned
     * (the scan is a raster build and some 360 hull checks). */
    if (c->ex_valid && c->ex_route && c->sq_released && !c->ex_route_fail_ms)
        return;
    lp_build(s, nav, a, &gm_lp_grid);
    ex_world_ok = 0;
    if (s->world_block) {
        ex_world_wc0 = (int)floorf(a->p.x / EX_WORLD_CELL) - EX_WORLD_N / 2;
        ex_world_wr0 = (int)floorf(a->p.z / EX_WORLD_CELL) - EX_WORLD_N / 2;
        ex_world_ok = s->world_block(s->world_context, EX_WORLD_CELL, ex_world_wc0, ex_world_wr0, EX_WORLD_N, ex_world) > 0;
    }
    /* The group's common way out, as the queue orders by it. */
    gm_point_t d = c->sq_dir;
    for (int j = 0; j < count; ++j)
        if (j != i && agents[j].c.sq_state == 1 && agents[j].mobile && gm_dist(agents[j].c.sq_spawn, c->sq_spawn) <= SQ_GROUP_M)
            d = gm_add(d, agents[j].c.sq_dir);
    d = gm_len(d) > 1e-3f ? gm_unit(d) : a->fwd;
    /* Standing hulls bound the room; a group member ahead of it in the queue
     * that still means to go does not: it stands only until its turn has
     * come, and the queue holds this one while it lies in the corridor
     * (else the tanks at the back of a cluster, boxed in by those ahead,
     * would all leave by the back door). */
    static ex_hull_t hulls[LP_MAX_HULLS];
    int hull_count = 0;
    for (int j = 0; j < count && hull_count < LP_MAX_HULLS; ++j) {
        const gm_agent_t *b = &agents[j];
        const gm_commitment_t *o = &b->c;
        if (j == i || !b->stationary || gm_dist(a->p, b->p) > a->radius + b->radius + EX_REACH_M + 1.0f)
            continue;
        if (o->sq_state == 1 && b->mobile && gm_dist(o->sq_spawn, c->sq_spawn) <= SQ_GROUP_M && sq_ahead(b, a, d)
            && !(o->sq_released && o->progress_ms && s->ms >= o->progress_ms + SQ_WEDGE_MS))
            continue;
        hulls[hull_count++] = (ex_hull_t){lp_centre(b->v, b->p.x, b->p.z, b->fwd), b->fwd, b->half_length, b->half_width};
    }
    ex_site_count = 0;
    ex_site_r = 3.5f;
    for (int k = 0; k < s->pending_spawn_site_count && ex_site_count < LP_MAX_SITES; ++k)
        if (gm_dist(s->pending_spawn_sites[k].p, a->p) < EX_REACH_M + a->radius + 6.0f) {
            if (s->pending_spawn_sites[k].half_length > 0.0f)
                ex_site_r = hypotf(s->pending_spawn_sites[k].half_length, s->pending_spawn_sites[k].half_width);
            ex_sites[ex_site_count++] = s->pending_spawn_sites[k].p;
        }
    /* The spawn cluster around it. */
    gm_point_t sum = c->sq_spawn;
    float members = 1.0f;
    for (int k = 0; k < s->pending_spawn_site_count; ++k)
        if (gm_dist(s->pending_spawn_sites[k].p, c->sq_spawn) <= EX_CLUSTER_M) {
            sum = gm_add(sum, s->pending_spawn_sites[k].p);
            members += 1.0f;
        }
    for (int j = 0; j < count; ++j)
        if (j != i && agents[j].c.sq_state == 1 && gm_dist(agents[j].c.sq_spawn, c->sq_spawn) <= SQ_GROUP_M) {
            sum = gm_add(sum, agents[j].c.sq_spawn);
            members += 1.0f;
        }
    const gm_point_t birth = c->birth_valid ? gm_axis(c->birth_heading) : a->fwd;
    const gm_point_t away_v = gm_sub(a->p, gm_scale(sum, 1.0f / members));
    const gm_point_t away = gm_len(away_v) > 1.0f ? gm_unit(away_v) : birth;
    const gm_point_t to_aim = gm_sub(a->aim, a->p);
    const gm_point_t route = gm_len(to_aim) > 0.5f ? gm_unit(to_aim) : a->fwd;
    const gm_point_t c0 = lp_centre(v, a->p.x, a->p.z, a->fwd);
    if (!c->ex_valid)
        c->ex_route = 1; /* the route aim first (gm_local_choose and the queue switch to open space) */
    else if (!c->ex_route && s->ms >= c->ex_open_ms + EX_OPEN_HOLD_MS) {
        /* Back to the route aim once the route's own way is open again. */
        const float route_room = ex_march(nav, a, c0, route, hulls, hull_count);
        if (route_room >= fminf(EX_ROUTE_OPEN, fmaxf(EX_MIN_FREE, gm_len(to_aim)))) {
            c->ex_route = 1;
            c->ex_route_fail_ms = 0;
            sq_log(s, "%llu queue: id %llu back to the route aim (dir %.2f rad, room %.1f m)", (unsigned long long)s->ms,
                   (unsigned long long)v->id, (double)atan2f(route.z, route.x), (double)route_room);
        }
    }
    float best_score = -INFINITY, best_room = 0.0f, keep_score = -INFINITY, keep_room = 0.0f;
    gm_point_t best_dir = a->fwd;
    const int had = c->ex_valid, samples = EX_DIRS + (had ? 1 : 0);
    for (int k = 0; k < samples; ++k) {
        const gm_point_t f = k < EX_DIRS ? gm_axis(2.0f * GM_PI * (float)k / (float)EX_DIRS) : c->ex_dir;
        const float room = ex_march(nav, a, c0, f, hulls, hull_count);
        const float turn = fabsf(gm_angle(atan2f(f.z, f.x) - v->heading));
        /* The bonuses count in proportion to the room (up to EX_BONUS_ROOM):
         * a way into a wall 3 m off is no way out, however well it points. */
        const float bonus = EX_AWAY_W * gm_dot(f, away) + EX_BIRTH_W * gm_dot(f, birth) + EX_ROUTE_W * gm_dot(f, route);
        const float score = room + fminf(room, EX_BONUS_ROOM) / EX_BONUS_ROOM * bonus - EX_TURN_W * turn
            - (room < EX_MIN_FREE ? EX_SHUT : 0.0f);
        if (k == EX_DIRS) {
            keep_score = score;
            keep_room = room;
        } else if (score > best_score) {
            best_score = score;
            best_room = room;
            best_dir = f;
        }
    }
    if (had && keep_room >= EX_BLOCKED_M && best_score <= keep_score + EX_BETTER * fabsf(keep_score)) {
        c->ex_free = keep_room;
        c->ex_score = keep_score;
        return;
    }
    const float change = had ? fabsf(gm_angle(atan2f(best_dir.z, best_dir.x) - atan2f(c->ex_dir.z, c->ex_dir.x))) : 0.0f;
    if (!had || change > 0.2f)
        sq_log(s, "%llu queue: id %llu exit dir %.2f rad %s, room %.1f m, score %.1f%s", (unsigned long long)s->ms,
               (unsigned long long)v->id, (double)atan2f(best_dir.z, best_dir.x), had ? "changed" : "chosen",
               (double)best_room, (double)best_score, had && keep_room < EX_BLOCKED_M ? " (old one blocked)" : "");
    c->ex_dir = best_dir;
    c->ex_free = best_room;
    c->ex_score = best_score;
    c->ex_valid = 1;
}

/* The way a tank in exit mode means to go now: towards its route aim, or the
 * open way out. */
static gm_point_t ex_way(const gm_agent_t *a)
{
    const gm_point_t to_aim = gm_sub(a->aim, a->p);
    return a->c.ex_route ? (gm_len(to_aim) > 0.5f ? gm_unit(to_aim) : a->fwd) : a->c.ex_dir;
}

/* Does b's hull lie in a's corridor along f (a's hull, turned that way, swept
 * from where it stands EX_REACH_M out, EX_CORRIDOR_SIDE wider), with b's
 * centre ahead along it no farther than SQ_GAP_M (as the s13 queue)? */
static int ex_in_corridor(const gm_agent_t *a, const gm_agent_t *b, gm_point_t f)
{
    const gm_point_t ca = lp_centre(a->v, a->p.x, a->p.z, a->fwd), cb = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
    const float along = gm_dot(gm_sub(cb, ca), f);
    if (along <= 0.0f || along > SQ_GAP_M)
        return 0;
    const float half = 0.5f * (EX_REACH_M + a->half_length);
    const gm_point_t mid = gm_add(ca, gm_scale(f, half));
    return lp_box_pen(mid, f, half, a->half_width + EX_CORRIDOR_SIDE, cb, b->fwd, b->half_length, b->half_width) > 0.0f;
}

/* The nearest group member holding agent i in its corridor along f (d: the
 * group's common way out, the tie-break order); 0 none. A member let go that
 * has made no progress for SQ_WEDGE_MS holds nobody (as in the s13 queue). */
static uint64_t ex_holder(const gm_snapshot_t *s, const gm_agent_t *agents, int count, int i, gm_point_t d, gm_point_t f)
{
    const gm_agent_t *a = &agents[i];
    const gm_commitment_t *c = &a->c;
    float nearest = INFINITY;
    uint64_t holder = 0;
    for (int j = 0; j < count; ++j) {
        const gm_agent_t *b = &agents[j];
        const gm_commitment_t *o = &b->c;
        if (j == i || o->sq_state != 1 || !b->mobile || gm_dist(o->sq_spawn, c->sq_spawn) > SQ_GROUP_M
            || !ex_in_corridor(a, b, f)
            || (b->ex_mode && o->ex_valid && ex_in_corridor(b, a, ex_way(b)) && !sq_ahead(b, a, d))
            || (o->sq_released && o->progress_ms && s->ms >= o->progress_ms + SQ_WEDGE_MS))
            continue;
        const float gap = gm_dist(b->p, a->p);
        if (gap < nearest) {
            nearest = gap;
            holder = b->v->id;
        }
    }
    return holder;
}

static void gm_spawn_queue(const gm_snapshot_t *s, gm_agent_t *agents, int count, const gm_nav_t *nav)
{
    const uint64_t ms = s->ms;
    /* Joins, the path driven, leaving. */
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        gm_commitment_t *c = &a->c;
        a->sq_hold = 0;
        a->sq_blocker = 0;
        if (!a->v->controlled || !a->mobile)
            continue;
        if (!c->sq_join_ms) {
            c->sq_join_ms = ms;
            if (fabsf(a->speed) > 1.0f) {
                c->sq_state = 2; /* first seen on the move: not at a spawn */
                continue;
            }
            c->sq_state = 1;
            c->sq_spawn = c->sq_last = a->p;
            c->sq_path = 0.0f;
            const gm_point_t out = gm_sub(a->aim, a->p);
            c->sq_dir = gm_len(out) > 0.5f ? gm_unit(out) : a->fwd;
            c->sq_late = 0;
            for (int j = 0; j < count; ++j) {
                const gm_commitment_t *o = &agents[j].c;
                if (j != i && o->sq_state == 1 && gm_dist(o->sq_spawn, c->sq_spawn) <= SQ_GROUP_M
                    && ms > o->sq_join_ms + SQ_LATE_MS && !o->sq_late)
                    c->sq_late = 1;
            }
            sq_log(s, "%llu queue: id %llu joins at (%.1f,%.1f), way out %.2f rad%s", (unsigned long long)ms,
                   (unsigned long long)a->v->id, (double)a->p.x, (double)a->p.z, (double)atan2f(c->sq_dir.z, c->sq_dir.x),
                   c->sq_late ? ", at the tail" : "");
        }
        if (c->sq_state != 1)
            continue;
        c->sq_path += gm_dist(a->p, c->sq_last);
        c->sq_last = a->p;
        if (gm_dist(a->p, c->sq_spawn) > SQ_LEAVE_M) {
            c->sq_state = 2;
            sq_log(s, "%llu queue: id %llu leaves, %.1f s after joining", (unsigned long long)ms,
                   (unsigned long long)a->v->id, 0.001 * (double)(ms - c->sq_join_ms));
        }
    }
    /* s16 exit mode: who is in it, and its way out. Leaving it (40 m out, or
     * EX_MODE_MS after joining) off the route it was given: one new route
     * search from where it is (gm_plan_snapshot, after the replan cooldown). */
    if (s->spawn_exit) {
        for (int i = 0; i < count; ++i) {
            gm_agent_t *a = &agents[i];
            gm_commitment_t *c = &a->c;
            if (!a->v->controlled || !a->mobile)
                continue;
            a->ex_mode = c->sq_state == 1 && ms < c->sq_join_ms + EX_MODE_MS;
            if (a->ex_mode) {
                ex_scan(s, nav, agents, count, i);
                if (!c->ex_route && c->ex_free >= EX_MIN_FREE && a->v_des > EX_SPEED) {
                    /* Steering for open space off the route: at a speed it
                     * can stop from within what its lidar has seen. */
                    a->v_des = EX_SPEED;
                    a->pref = gm_scale(gm_len(a->pref) > 0.05f ? gm_unit(a->pref) : a->fwd, EX_SPEED);
                }
            } else if (c->ex_active) {
                const int off = a->following && a->cross > EX_OFF_ROUTE_M;
                c->ex_replan_pending = off && !c->ex_replan_done;
                sq_log(s, "%llu queue: id %llu exit mode ends after %.1f s, %.1f m from its spawn point, %.1f m off its route%s",
                       (unsigned long long)ms, (unsigned long long)a->v->id, 0.001 * (double)(ms - c->sq_join_ms),
                       (double)gm_dist(a->p, c->sq_spawn), a->following ? (double)a->cross : -1.0,
                       c->ex_replan_pending ? ": new route" : "");
            }
            c->ex_active = a->ex_mode;
        }
    }
    /* Who holds whom. */
    static gm_point_t group_dir[GM_MAX_VEHICLES];
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        gm_commitment_t *c = &a->c;
        /* s16 exit mode, EX_REHOLD (off: tried 2026-11-16, it held the
         * Sweden and spawn_cluster models up more than it sorted out): a
         * tank let go that has come to a stand again is judged again. */
        const int corridor = a->ex_mode && c->ex_valid;
        if (!a->v->controlled || !a->mobile || c->sq_state != 1
            || (c->sq_released && !(EX_REHOLD && corridor && fabsf(a->speed) < 0.5f)))
            continue;
        gm_point_t d = c->sq_dir;
        for (int j = 0; j < count; ++j)
            if (j != i && agents[j].c.sq_state == 1 && agents[j].mobile
                && gm_dist(agents[j].c.sq_spawn, c->sq_spawn) <= SQ_GROUP_M)
                d = gm_add(d, agents[j].c.sq_dir);
        d = gm_unit(d);
        group_dir[i] = d;
        /* s16 exit mode: held only by a member lying in its own corridor
         * (along the way it means to go), whatever the order; of two in each
         * other's corridors the one ahead in the order goes first. Held on
         * its route's way with the open way out free: it goes that way. */
        if (corridor) {
            a->sq_blocker = ex_holder(s, agents, count, i, d, ex_way(a));
            if (a->sq_blocker && c->ex_route && c->ex_free >= EX_MIN_FREE && !ex_holder(s, agents, count, i, d, c->ex_dir)) {
                sq_log(s, "%llu queue: id %llu held by %llu on its route's way: out by the open way %.2f rad (room %.1f m)",
                       (unsigned long long)ms, (unsigned long long)a->v->id, (unsigned long long)a->sq_blocker,
                       (double)atan2f(c->ex_dir.z, c->ex_dir.x), (double)c->ex_free);
                c->ex_route = 0;
                c->ex_open_ms = ms;
                a->sq_blocker = 0;
            }
        }
        float nearest = INFINITY;
        for (int j = 0; j < count && !corridor; ++j) {
            gm_agent_t *b = &agents[j];
            gm_commitment_t *o = &b->c;
            if (j == i || o->sq_state != 1 || !b->mobile || gm_dist(o->sq_spawn, c->sq_spawn) > SQ_GROUP_M)
                continue;
            const float gap = gm_dist(b->p, a->p);
            if (!sq_ahead(b, a, d))
                continue;
            if (gap > SQ_GAP_M)
                continue;
            /* One queue per lane: a tank beside it on the way out (spawn_29's
             * rows, 5.2 m apart) holds nobody. */
            if (fabsf(gm_det(d, gm_sub(b->p, a->p))) > a->half_width + b->half_width + SQ_LANE_GAP)
                continue;
            if (o->sq_released && o->progress_ms && ms >= o->progress_ms + SQ_WEDGE_MS) {
                if (!o->sq_timeout_logged) {
                    sq_log(s, "%llu queue: id %llu ahead makes no progress for %u ms: the ones behind may go",
                           (unsigned long long)ms, (unsigned long long)b->v->id, SQ_WEDGE_MS);
                    o->sq_timeout_logged = 1;
                }
                continue;
            }
            if (gap < nearest) {
                nearest = gap;
                a->sq_blocker = b->v->id;
            }
        }
        if (a->sq_blocker) {
            a->sq_hold = 1;
            c->sq_held = 1;
            if (corridor && c->ex_held_by != a->sq_blocker) {
                c->ex_held_by = a->sq_blocker;
                sq_log(s, "%llu queue: id %llu held by %llu in its corridor (exit dir %.2f rad)", (unsigned long long)ms,
                       (unsigned long long)a->v->id, (unsigned long long)a->sq_blocker,
                       (double)atan2f(ex_way(a).z, ex_way(a).x));
            }
        } else {
            const int news = !c->sq_released || c->ex_held_by; /* (s16: held again, now let go again) */
            c->sq_released = 1;
            c->ex_held_by = 0;
            if (news)
                sq_log(s, "%llu queue: id %llu goes%s, %.1f s after joining", (unsigned long long)ms,
                       (unsigned long long)a->v->id, c->sq_held ? " (released)" : " (head)", 0.001 * (double)(ms - c->sq_join_ms));
        }
    }
    if (s->spawn_exit) {
        /* s16: corridors can hold round in a circle (a by b, b by c, c by a):
         * the one of them first in the queue order goes. */
        for (int i = 0; i < count; ++i) {
            if (!agents[i].sq_hold || !agents[i].ex_mode)
                continue;
            int k = gm_agent_index(agents, count, agents[i].sq_blocker), steps = 0;
            while (k >= 0 && k != i && agents[k].sq_hold && ++steps <= count)
                k = gm_agent_index(agents, count, agents[k].sq_blocker);
            if (k != i)
                continue;
            int head = i;
            for (k = gm_agent_index(agents, count, agents[i].sq_blocker); k >= 0 && k != i;
                 k = gm_agent_index(agents, count, agents[k].sq_blocker))
                if (sq_ahead(&agents[k], &agents[head], group_dir[head]))
                    head = k;
            gm_agent_t *h = &agents[head];
            h->sq_hold = 0;
            h->sq_blocker = 0;
            h->c.sq_released = 1;
            sq_log(s, "%llu queue: id %llu goes (corridors held in a circle), %.1f s after joining", (unsigned long long)ms,
                   (unsigned long long)h->v->id, 0.001 * (double)(ms - h->c.sq_join_ms));
        }
    }
    for (int i = 0; i < count; ++i)
        if (agents[i].c.sq_released && agents[i].c.progress_ms && ms < agents[i].c.progress_ms + SQ_WEDGE_MS)
            agents[i].c.sq_timeout_logged = 0;
}

/* ---- s24 path mode: command -> route -> avoidance -> driver (snapshot path_mode). ----
 * Design rule: one authority per decision. The command layer decides where (the vehicle's goal); the route
 * layer (ground_nav) finds the route from where the tank stands to it, with no via points, searched on events
 * only (first, target moved, off route, blocked, retry); the avoidance layer (gm_path_decide) is idle unless
 * something keeps the stop distance under GM_PATH_BLOCKED_M for GM_PATH_BLOCKED_MS, then takes ONE decision
 * and keeps it until it is carried out; the driver (ground_drive.inc) presses the keys. Published per tank:
 * go / hold, the aim point, the stop distance, a reverse leg, a speed cap only on a tight nav leg.
 *  - Aim: the next route vertex; the one after it as soon as the hull's straight sweep from where it stands to
 *    it is free (fused raster, and the route layer's cells beyond it): corners are cut only where there is room.
 *  - Stop distance: the least of the free length of the hull's straight sweep along its present heading over the
 *    fused raster and the other hulls (standing ones always, a moving one's predicted rectangle up to
 *    LP_MOVING_HARD_S; s23 lp_stop_distance), and the distance to the aim where the route turns there by more
 *    than 30 deg or ends there; the single-lane hold point (gm_narrow_hold) where it must wait.
 *  - Nothing here scores candidates or re-decides a direction per snapshot. */

/* The hulls a vehicle's sweeps meet, for the s23 stop distance and the checks below (as gm_local_choose). */
static void gm_path_ctx(const gm_snapshot_t *s, const gm_nav_t *nav, const gm_agent_t *agents, int count, int i,
                        lp_ctx_t *x)
{
    const gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    memset(x, 0, sizeof *x);
    x->s = s;
    x->nav = nav;
    x->a = a;
    x->g = &gm_lp_grid;
    x->bl = a->half_length;
    x->bw = a->half_width;
    x->budget = LP_VEHICLE_POSES;
    const gm_point_t f0 = gm_axis(v->heading), c0 = lp_centre(v, a->p.x, a->p.z, f0);
    for (int j = 0; j < count && x->hull_count < LP_MAX_HULLS; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i || gm_dist(a->p, b->p) > a->radius + b->radius + GM_PATH_STOP_MAX + gm_len(b->vel) * LP_MOVING_HARD_S)
            continue;
        lp_hull_t *h = &x->hulls[x->hull_count++];
        h->f = b->fwd;
        h->p = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
        h->vel = b->stationary ? (gm_point_t){0.0f, 0.0f} : b->vel;
        h->hl = b->half_length;
        h->hw = b->half_width;
        h->id = b->v->id;
        h->moving = !b->stationary;
        h->diag = hypotf(h->hl, h->hw);
        h->pen0 = lp_box_pen(c0, f0, x->bl + GM_SKIN, x->bw + GM_SKIN, h->p, h->f, h->hl, h->hw);
    }
}

/* s23 lp_stop_distance (taken read-only from the s23 lab copy): the hull marched from where it stands along
 * its present heading (dir -1: backwards) up to GM_PATH_STOP_MAX, LP_CHECK_M steps to GM_PATH_STOP_FINE and 1 m
 * beyond, with the margins of its present speed: static cells by the depth rule (it may leave an occupied cell,
 * never press deeper in; beyond the raster the obstacle cells) and hulls (a moving one's predicted rectangle up
 * to LP_MOVING_HARD_S). Returns the free length; *blocker the hull that ended it (0: static or none). */
static float gm_path_stop_distance(lp_ctx_t *x, int dir, uint64_t *blocker, int *blocker_moving)
{
    const gm_agent_t *a = x->a;
    const gm_vehicle_t *v = a->v;
    const gm_point_t f = gm_axis(v->heading), d = gm_scale(f, (float)dir);
    float ml, mw;
    lp_margins(fabsf(a->speed), &ml, &mw);
    lp_hit_t hit;
    const gm_point_t c0 = lp_centre(v, a->p.x, a->p.z, f);
    const float depth0 = lp_rect(x->g, c0, f, x->bl, x->bw, ml, mw, 0.0f, 0, &hit) == 0 && hit.hits ? hit.depth : 0.0f;
    const float speed = fmaxf(fabsf(a->speed), 1.0f);
    float free = 0.0f, s = 0.0f;
    *blocker = 0;
    *blocker_moving = 0;
    for (int budget = GM_PATH_STOP_POSES; budget > 0; --budget) {
        s += s < GM_PATH_STOP_FINE - 1e-3f ? LP_CHECK_M : 1.0f;
        if (s > GM_PATH_STOP_MAX + 1e-3f)
            break;
        ++x->poses;
        const gm_pose_t pose = {a->p.x + d.x * s, a->p.z + d.z * s, v->heading, 0.0f, 0.0f};
        const gm_point_t c = lp_centre(v, pose.x, pose.z, f);
        if (lp_rect(x->g, c, f, x->bl, x->bw, ml, mw, 0.0f, 0, &hit) < 0) {
            if (x->nav && gm_nav_overlap(x->nav, v, &pose, mw) > 0)
                break;
        } else if (hit.hits && hit.depth > depth0 + LP_DEPTH_SLACK) {
            break;
        }
        const float t = s / speed;
        int held = 0;
        for (int j = 0; j < x->hull_count && !held; ++j) {
            const lp_hull_t *h = &x->hulls[j];
            if (h->moving && t > LP_MOVING_HARD_S)
                continue;
            const gm_point_t q = gm_add(h->p, gm_scale(h->vel, fminf(t, LP_HORIZON_S)));
            const float pen = lp_box_pen(c, f, x->bl + GM_SKIN, x->bw + GM_SKIN, q, h->f, h->hl, h->hw);
            if (pen > fmaxf(h->pen0, 0.0f) + 0.02f) {
                *blocker = h->id;
                *blocker_moving = h->moving;
                held = 1;
            }
        }
        if (held)
            break;
        free = s;
    }
    return free;
}

/* Static check of one hull pose for the sweeps below: on the raster no occupied cell deeper into the hull
 * than at the start (depth0), off it no more obstacle cells under it than allowed (which only falls). */
static int gm_path_pose_free(lp_ctx_t *x, const gm_pose_t *pose, float ml, float mw, float depth0, int *allowed)
{
    const gm_vehicle_t *v = x->a->v;
    const gm_point_t f = gm_axis(pose->heading), c = lp_centre(v, pose->x, pose->z, f);
    lp_hit_t hit;
    ++x->poses;
    if (lp_rect(x->g, c, f, x->bl, x->bw, ml, mw, 0.0f, 0, &hit) == 0)
        return !(hit.hits && hit.depth > depth0 + LP_DEPTH_SLACK);
    if (!x->nav)
        return 1;
    const int overlap = gm_nav_overlap(x->nav, v, pose, mw);
    if (overlap > *allowed)
        return 0;
    *allowed = overlap;
    return 1;
}

/* Is the hull's straight sweep from p to q free, its axis `heading` (margins of a hull at speed)? Static cells,
 * and the standing hulls: a route that goes round one must not be cut through it. */
static int gm_path_sweep_one(lp_ctx_t *x, gm_point_t p, gm_point_t q, float heading)
{
    const float len = gm_dist(p, q);
    const gm_point_t f = gm_axis(heading), line = gm_unit(gm_sub(q, p));
    float ml, mw;
    lp_margins(4.0f, &ml, &mw);
    const gm_pose_t start = {p.x, p.z, heading, 0.0f, 0.0f};
    const gm_point_t c0 = lp_centre(x->a->v, p.x, p.z, f);
    lp_hit_t hit;
    const float depth0 = lp_rect(x->g, c0, f, x->bl, x->bw, ml, mw, 0.0f, 0, &hit) == 0 && hit.hits ? hit.depth : 0.0f;
    int allowed = x->nav ? gm_nav_overlap(x->nav, x->a->v, &start, mw) : 0;
    const int steps = (int)ceilf(len / LP_CHECK_M);
    for (int k = 1; k <= steps; ++k) {
        const float s = len * (float)k / (float)steps;
        const gm_pose_t pose = {p.x + line.x * s, p.z + line.z * s, heading, 0.0f, 0.0f};
        if (!gm_path_pose_free(x, &pose, ml, mw, depth0, &allowed))
            return 0;
        const gm_point_t c = lp_centre(x->a->v, pose.x, pose.z, f);
        for (int j = 0; j < x->hull_count; ++j) {
            const lp_hull_t *h = &x->hulls[j];
            if (!h->moving && lp_box_pen(c, f, x->bl + GM_SKIN, x->bw + GM_SKIN, h->p, h->f, h->hl, h->hw)
                                  > fmaxf(h->pen0, 0.0f) + 0.02f)
                return 0;
        }
    }
    return 1;
}

static int gm_path_turn_room(lp_ctx_t *x, float turn);

/* The straight sweep from p to q, with the hull on the line's heading and, when that differs from its present
 * heading by up to 30 deg (the driver then steers onto the line while it rolls, it does not turn first), once
 * more on its present heading: no turning model, the two ends of the blend. More than 30 deg off, the driver
 * turns on the spot first: then that pivot needs room too (gm_path_turn_room; p is where the hull stands). */
static int gm_path_sweep_free(lp_ctx_t *x, gm_point_t p, gm_point_t q)
{
    if (gm_dist(p, q) < 0.05f)
        return 1;
    const float line = atan2f(q.z - p.z, q.x - p.x), now = x->a->v->heading;
    if (!gm_path_sweep_one(x, p, q, line))
        return 0;
    const float turn = gm_angle(line - now), off = fabsf(turn);
    if (off > GM_PATH_E_DRIVE)
        return gm_path_turn_room(x, turn);
    return off < 0.02f || gm_path_sweep_one(x, p, q, now);
}

/* Room to turn on the spot by `turn` (rad, + towards +z) in a forward gear: A or D alone at rest turns the hull
 * about a point GM_PATH_PIVOT_R to the steer side of its rotation centre (game: the centre travels 2.0-2.8 m per
 * radian). The hull along that arc, every GM_PATH_PIVOT_STEP, against the raster / obstacle cells (creep
 * margins) and the hulls (no more overlap than at the start). */
static int gm_path_turn_room(lp_ctx_t *x, float turn)
{
    const gm_agent_t *a = x->a;
    const gm_vehicle_t *v = a->v;
    const float side = turn > 0.0f ? 1.0f : -1.0f;
    const gm_point_t f0 = gm_axis(v->heading);
    const gm_point_t centre = {a->p.x - side * f0.z * GM_PATH_PIVOT_R, a->p.z + side * f0.x * GM_PATH_PIVOT_R};
    const gm_point_t rel = gm_sub(a->p, centre);
    lp_hit_t hit;
    const gm_point_t c0 = lp_centre(v, a->p.x, a->p.z, f0);
    const float depth0 = lp_rect(x->g, c0, f0, x->bl, x->bw, LP_END_CREEP, LP_SIDE_CREEP, 0.0f, 0, &hit) == 0 && hit.hits
        ? hit.depth : 0.0f;
    const gm_pose_t here = {a->p.x, a->p.z, v->heading, 0.0f, 0.0f};
    int allowed = x->nav ? gm_nav_overlap(x->nav, v, &here, LP_SIDE_CREEP) : 0;
    const int steps = (int)ceilf(fabsf(turn) / GM_PATH_PIVOT_STEP);
    for (int k = 1; k <= steps; ++k) {
        const float ang = turn * (float)k / (float)steps, cs = cosf(ang), sn = sinf(ang);
        const gm_pose_t pose = {centre.x + rel.x * cs - rel.z * sn, centre.z + rel.x * sn + rel.z * cs,
                                v->heading + ang, 0.0f, 0.0f};
        if (!gm_path_pose_free(x, &pose, LP_END_CREEP, LP_SIDE_CREEP, depth0, &allowed))
            return 0;
        const gm_point_t f = gm_axis(pose.heading), c = lp_centre(v, pose.x, pose.z, f);
        for (int j = 0; j < x->hull_count; ++j) {
            const lp_hull_t *h = &x->hulls[j];
            if (lp_box_pen(c, f, x->bl + GM_SKIN, x->bw + GM_SKIN, h->p, h->f, h->hl, h->hw) > fmaxf(h->pen0, 0.0f) + 0.02f)
                return 0;
        }
    }
    return 1;
}

/* Route following for one mobile tank: the aim, the stop distance, the turn it needs and whether it has room. */
static void gm_path_follow(const gm_snapshot_t *s, const gm_nav_t *nav, gm_agent_t *agents, int count, int i)
{
    static lp_ctx_t x;
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    lp_build(s, nav, a, &gm_lp_grid);
    gm_path_ctx(s, nav, agents, count, i, &x);
    const int back = c->pa_state == GM_PA_BACK;
    a->following = a->tight = a->reverse_leg = 0;
    a->target = a->aim = a->goal;
    a->cross = 0.0f;
    /* The aim: the next route vertex; the one after it as soon as the hull's straight sweep there is free. */
    int early = 0;
    while (c->nav_vertex < c->nav_count - 1) {
        const int q = c->nav_vertex;
        const gm_point_t pt = c->nav_points[q], next = c->nav_points[q + 1];
        const uint8_t flags = c->nav_flags[q], nflags = c->nav_flags[q + 1];
        /* Reached: within 4 m of a vertex where the route bends gently; within 1.5 m of a tight or reverse one and of
         * one where it turns by more than 30 deg (the stop distance ends there: the hull stops on it, then turns). */
        const gm_point_t in_leg = gm_sub(pt, q ? c->nav_points[q - 1] : c->nav_origin), out_leg = gm_sub(next, pt);
        const int sharp = gm_len(in_leg) > 0.05f && gm_len(out_leg) > 0.05f
            && fabsf(gm_angle(atan2f(out_leg.z, out_leg.x) - atan2f(in_leg.z, in_leg.x))) > GM_PATH_TURN_STOP;
        const float capture = flags & (GM_NAV_TIGHT | GM_NAV_REVERSE) || sharp ? GM_PATH_TIGHT_CAPTURE : GM_VERTEX_CAPTURE;
        if (gm_dist(a->p, pt) < capture) {
            ++c->nav_vertex;
            continue;
        }
        if (!back && early < GM_PATH_ADVANCE_MAX && x.poses < LP_VEHICLE_POSES / 2 && !((flags | nflags) & GM_NAV_REVERSE)
            && gm_dist(a->p, next) <= GM_PATH_SWEEP_MAX && gm_path_sweep_free(&x, a->p, next)) {
            ++c->nav_vertex;
            ++early;
            continue;
        }
        break;
    }
    if (c->nav_vertex < c->nav_count) {
        const int q = c->nav_vertex, last = q == c->nav_count - 1;
        const gm_point_t pt = c->nav_points[q];
        a->following = 1;
        /* The route's end is the target (or the standing room beside it): stop on the target itself. */
        a->target = a->aim = last && !a->region && gm_dist(pt, a->goal) <= 2.0f ? a->goal : pt;
        a->tight = (c->nav_flags[q] & GM_NAV_TIGHT) != 0;
        a->reverse_leg = (c->nav_flags[q] & GM_NAV_REVERSE) != 0;
        /* Off the route: farther than the margin from every leg it has passed lately, and no free straight line
         * to the aim (a cut corner is on the route). */
        float cross = INFINITY;
        for (int k = q - GM_PATH_ADVANCE_MAX - 1 > 0 ? q - GM_PATH_ADVANCE_MAX - 1 : 0; k <= q; ++k)
            cross = fminf(cross, gm_segment_distance(a->p, k ? c->nav_points[k - 1] : c->nav_origin, c->nav_points[k]));
        if (cross > GM_PATH_OFF_ROUTE && gm_path_sweep_free(&x, a->p, a->aim))
            cross = 0.0f;
        a->cross = cross;
        a->remaining = gm_dist(a->p, pt) + gm_dist(c->nav_points[c->nav_count - 1], a->goal);
        for (int k = q + 1; k < c->nav_count; ++k)
            a->remaining += gm_dist(c->nav_points[k - 1], c->nav_points[k]);
    } else {
        a->remaining = gm_dist(a->p, a->goal);
    }
    const gm_point_t to = gm_sub(a->aim, a->p);
    const int reverse = a->reverse_leg || back;
    a->path_e = gm_len(to) >= 1.0f ? gm_angle(atan2f(to.z, to.x) - v->heading - (a->reverse_leg ? GM_PI : 0.0f)) : 0.0f;
    a->path_free = gm_path_stop_distance(&x, reverse ? -1 : 1, &a->path_blocker, &a->path_blocker_moving);
    float d = a->path_free;
    if (!back) {
        int stop_at_aim = !a->following || c->nav_vertex >= c->nav_count - 1;
        if (!stop_at_aim) {
            /* the route turns there by more than 30 deg: seen from the hull, or along the route's own legs (the
             * vertex is then reached only within GM_PATH_TIGHT_CAPTURE, above) */
            const int q = c->nav_vertex;
            const gm_point_t out = gm_sub(c->nav_points[q + 1], a->aim);
            const gm_point_t in_leg = gm_sub(a->aim, q ? c->nav_points[q - 1] : c->nav_origin);
            const float out_dir = atan2f(out.z, out.x);
            stop_at_aim = gm_len(out) > 0.05f
                && ((gm_len(to) > 0.05f && fabsf(gm_angle(out_dir - atan2f(to.z, to.x))) > GM_PATH_TURN_STOP)
                    || (gm_len(in_leg) > 0.05f && fabsf(gm_angle(out_dir - atan2f(in_leg.z, in_leg.x))) > GM_PATH_TURN_STOP));
        }
        if (stop_at_aim)
            d = fminf(d, gm_len(to));
    }
    a->path_d = d;
    a->path_turn = !back && fabsf(a->path_e) >= GM_PATH_E_SETTLE && (fabsf(a->path_e) > GM_PATH_E_DRIVE || d <= GM_PATH_NO_ROOM);
    a->path_turn_room = !a->path_turn || a->reverse_leg || gm_path_turn_room(&x, a->path_e);
    a->path_back = back ? a->path_free : 0.0f;
    if (!back && (a->path_free < GM_PATH_BLOCKED_M || !a->path_turn_room)) {
        uint64_t behind;
        int behind_moving;
        a->path_back = gm_path_stop_distance(&x, -1, &behind, &behind_moving);
    }
    a->v_des = a->tight ? GM_TIGHT_SPEED : v->cruise_speed;
    a->path_cap = a->tight ? GM_TIGHT_SPEED : INFINITY;
    a->pref = gm_scale(gm_len(to) > 0.3f ? gm_unit(to) : a->fwd, a->v_des);
    gm_lp_snapshot_poses += x.poses;
}

/* Ask the route layer for a way round what holds the tank, from where it stands: a standing hull becomes a
 * body for its searches (gm_route_layer), a static spot the raster sees and the map does not is reported. */
static void gm_path_reroute(gm_nav_t *nav, const gm_snapshot_t *s, gm_agent_t *agents, int count, int i, uint64_t blocker)
{
    gm_agent_t *a = &agents[i];
    gm_commitment_t *c = &a->c;
    const int j = blocker ? gm_agent_index(agents, count, blocker) : -1;
    const double held_s = c->pa_low_since_ms ? 0.001 * (double)(s->ms - c->pa_low_since_ms) : 0.0; /* 0: after a back-up */
    if (j >= 0) {
        c->body_id = blocker;
        c->body_until_ms = s->ms + GM_BODY_MS;
        c->pa_reason = GM_PATH_R_HULL;
        gm_path_log(s, "avoid: seat %llu round seat %llu (stop distance %.1f m for %.1f s)", (unsigned long long)a->v->id,
                    (unsigned long long)blocker, (double)a->path_free, held_s);
    } else {
        const float way = a->reverse_leg ? -1.0f : 1.0f;
        const gm_point_t spot = gm_add(a->p, gm_scale(a->fwd, way * (a->half_length + a->path_free + 0.5f)));
        if (nav && a->path_free < GM_PATH_BLOCKED_M)
            gm_nav_report_blocked(nav, a->v, spot, 1.5f);
        c->pa_reason = GM_PATH_R_STATIC;
        gm_path_log(s, "avoid: seat %llu round a static obstacle at (%.1f, %.1f) (stop distance %.1f m%s, %.1f s)",
                    (unsigned long long)a->v->id, (double)spot.x, (double)spot.z, (double)a->path_free,
                    a->path_turn_room ? "" : ", no room to turn", held_s);
    }
    c->replan = 1;
    c->path_reason = c->pa_reason;
    c->pa_blocker = blocker;
    c->pa_state = GM_PA_REROUTE;
    c->pa_rev0 = c->nav_revision;
    c->pa_low_since_ms = 0;
}

/* The avoidance layer and the publication for one mobile tank (after gm_path_follow and gm_narrow_hold). */
static void gm_path_decide(gm_nav_t *nav, const gm_snapshot_t *s, gm_agent_t *agents, int count, int i, int routed)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    const uint64_t ms = s->ms;
    a->path_go = a->path_rev = 0;
    a->lp_moving = 0;
    a->cmd_speed = a->cmd_yaw = a->cmd_curvature = 0.0f;
    a->cmd_heading = v->heading;
    a->blocker = 0;
    a->geometry = GM_CLEAR;
    a->firm = 0;
    a->reversing = 0;
    a->state = GM_DRIVING;
    if (gm_dist(a->aim, a->p) > 0.3f)
        a->cmd_heading = atan2f(a->aim.z - a->p.z, a->aim.x - a->p.x);

    /* No route yet (the route layer is on): stand until it comes. */
    if (routed && !c->nav_count) {
        a->state = GM_WAIT_GEOMETRY;
        a->geometry = GM_UNKNOWN;
        c->pa_state = GM_PA_IDLE;
        c->pa_low_since_ms = 0;
        return;
    }
    /* Backing up (decision c): straight back until 3 m are done, the time is up or there is no room. */
    if (c->pa_state == GM_PA_BACK) {
        const float moved = gm_dist(a->p, c->pa_from);
        if (moved >= GM_PATH_BACK_M || ms >= c->pa_until_ms || a->path_back < 0.5f) {
            gm_path_log(s, "avoid: seat %llu backed up %.1f m: asks for a route from here", (unsigned long long)v->id,
                        (double)moved);
            c->pa_backed_ms = ms;
            c->pa_backed_at = c->pa_from;
            const uint64_t blocker = c->pa_blocker;
            gm_path_reroute(nav, s, agents, count, i, blocker && gm_agent_index(agents, count, blocker) >= 0 ? blocker : 0);
            a->state = GM_WAIT_GEOMETRY; /* this snapshot's sweeps looked backwards: hold for one publication */
            return;
        } else {
            a->path_go = a->path_rev = 1;
            a->reversing = 1;
            a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
            a->path_d = a->path_back;
            a->path_cap = INFINITY;
            a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
            a->cmd_heading = v->heading;
            return;
        }
    }
    /* No way round (decision b): wait, and ask again every GM_PATH_RETRY_MS. */
    if (c->pa_state == GM_PA_WAIT) {
        if (ms < c->pa_retry_ms) {
            a->blocker = c->pa_blocker;
            a->state = c->pa_blocker ? GM_WAIT_TRAFFIC : GM_STATIC_BLOCKED;
            a->geometry = c->pa_blocker ? GM_CLEAR : GM_BLOCKED;
            return;
        }
        c->replan = 1;
        c->path_reason = GM_PATH_R_RETRY;
        c->pa_state = GM_PA_REROUTE;
        c->pa_rev0 = c->nav_revision;
    }
    /* A way round asked for: carried out once a new route is followed; none found: wait (decision b). */
    if (c->pa_state == GM_PA_REROUTE) {
        if (c->nav_revision != c->pa_rev0) {
            c->pa_state = GM_PA_IDLE;
            c->pa_low_since_ms = 0;
        } else if (c->route_status == GM_NAV_UNREACHABLE && !c->replan) {
            /* (!replan: the request has gone out; the status is its answer, not the last search's) */
            c->pa_state = GM_PA_WAIT;
            c->pa_retry_ms = ms + GM_PATH_RETRY_MS;
            gm_path_log(s, "avoid: seat %llu waits: no way round %s%llu, asks again in %.0f s", (unsigned long long)v->id,
                        c->pa_blocker ? "seat " : "the static obstacle", (unsigned long long)c->pa_blocker,
                        0.001 * (double)GM_PATH_RETRY_MS);
            a->blocker = c->pa_blocker;
            a->state = c->pa_blocker ? GM_WAIT_TRAFFIC : GM_STATIC_BLOCKED;
            a->geometry = c->pa_blocker ? GM_CLEAR : GM_BLOCKED;
            return;
        }
    }
    /* Two tanks nose to nose, each held by the other: the lower seat goes, the other waits. */
    const int j = a->path_blocker ? gm_agent_index(agents, count, a->path_blocker) : -1;
    if (j >= 0 && agents[j].v->controlled && agents[j].mobile && agents[j].path_blocker == v->id
        && a->path_free < GM_PATH_BLOCKED_M && agents[j].path_free < GM_PATH_BLOCKED_M
        && gm_dot(a->fwd, agents[j].fwd) < -0.5f && v->id > agents[j].v->id) {
        if (c->pa_waits_for != agents[j].v->id)
            gm_path_log(s, "avoid: seat %llu waits for seat %llu (nose to nose: the lower seat goes)",
                        (unsigned long long)v->id, (unsigned long long)agents[j].v->id);
        c->pa_waits_for = agents[j].v->id;
        c->pa_low_since_ms = 0;
        a->blocker = agents[j].v->id;
        a->state = GM_WAIT_TRAFFIC;
        return;
    }
    c->pa_waits_for = 0;

    /* Held: no room ahead (not behind a hull that moves: following it is the stop distance's job), or a turn on
     * the spot it needs and has no room for. */
    const int pressed = a->path_turn ? !a->path_turn_room
                                     : a->path_free < GM_PATH_BLOCKED_M && !(a->path_blocker && a->path_blocker_moving);
    if (!pressed) {
        c->pa_low_since_ms = 0;
    } else if (c->pa_state == GM_PA_IDLE) {
        if (!c->pa_low_since_ms)
            c->pa_low_since_ms = ms;
        if (ms - c->pa_low_since_ms >= GM_PATH_BLOCKED_MS) {
            /* Arrival short of an occupied goal, and parking in a capture region, as the s12 rules have it. */
            const int k = a->path_blocker ? gm_agent_index(agents, count, a->path_blocker) : -1;
            const int goal_taken = k >= 0 && agents[k].stationary && !agents[k].mobile
                && fabsf(gm_dot(gm_sub(a->goal, agents[k].p), agents[k].fwd)) < agents[k].half_length + a->half_width + GM_SIDE_GAP
                && fabsf(gm_det(agents[k].fwd, gm_sub(a->goal, agents[k].p))) < agents[k].half_width + a->half_width + GM_SIDE_GAP
                && gm_dist(a->p, a->goal) < v->arrival_radius + 2.5f * a->radius;
            if (!a->region && goal_taken) {
                c->arrived = c->arrived_short = 1;
                a->state = GM_ARRIVED;
                gm_path_log(s, "avoid: seat %llu stops short of its target: seat %llu stands on it", (unsigned long long)v->id,
                            (unsigned long long)a->path_blocker);
                return;
            }
            if (a->region && gm_dist(a->p, v->goal_region_center) < v->goal_region_radius - 3.0f) {
                c->parked = 1;
                c->slot = a->p;
                c->slot_valid = 1;
                c->slot_phase = 2;
                a->state = GM_PARKED;
                gm_path_log(s, "avoid: seat %llu parks where it stands in its capture region", (unsigned long long)v->id);
                return;
            }
            const int backed_here = c->pa_backed_ms && ms < c->pa_backed_ms + GM_PATH_BACK_AGAIN_MS
                && gm_dist(a->p, c->pa_backed_at) < GM_PATH_BACK_SAME;
            if (a->path_turn && !a->path_turn_room && a->path_back >= GM_PATH_BACK_MIN && !backed_here) {
                /* Decision c: pressed against something with no room to turn: straight back 3 m, once. */
                c->pa_state = GM_PA_BACK;
                c->pa_from = a->p;
                c->pa_until_ms = ms + GM_PATH_BACK_MS;
                c->pa_blocker = a->path_blocker;
                c->pa_low_since_ms = 0;
                gm_path_log(s, "avoid: seat %llu backs up %.0f m: no room to turn %.0f deg (%.1f m free behind)",
                            (unsigned long long)v->id, (double)GM_PATH_BACK_M, (double)(a->path_e * 57.29578f),
                            (double)a->path_back);
                a->path_go = a->path_rev = 1;
                a->reversing = 1;
                a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
                a->path_d = a->path_back;
                a->path_cap = INFINITY;
                a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
                a->cmd_heading = v->heading;
                return;
            }
            /* Decision a: a way round from here. */
            if (routed)
                gm_path_reroute(nav, s, agents, count, i, a->path_blocker_moving ? 0 : a->path_blocker);
        }
    }
    if (a->path_turn && !a->path_turn_room) {
        /* The pivot it needs has no room: it does not press A / D into what is there while the decision above
         * waits for its 2 s (or for the way round it asked for). */
        a->blocker = a->path_blocker;
        a->state = a->path_blocker ? GM_WAIT_TRAFFIC : GM_STATIC_BLOCKED;
        a->geometry = a->path_blocker ? GM_CLEAR : GM_BLOCKED;
        return;
    }
    /* Off the route: a new one from here (an event, with the route layer's cooldown). */
    if (routed && a->following && a->cross > GM_PATH_OFF_ROUTE && c->pa_state == GM_PA_IDLE && !c->replan
        && ms >= c->replan_ms + GM_REPLAN_COOLDOWN_MS) {
        c->replan = 1;
        c->path_reason = GM_PATH_R_OFF;
        gm_path_log(s, "path: seat %llu is %.1f m off its route: asks for a new one from here", (unsigned long long)v->id,
                    (double)a->cross);
    }

    /* Go: aim, stop distance (and the single-lane hold point: it waits there). */
    float d = a->path_d;
    if (a->hold_arc >= 0.0f && a->hold_arc < d) {
        d = a->hold_arc;
        a->blocker = a->hold_blocker;
        if (d <= GM_PATH_NO_ROOM)
            a->state = GM_WAIT_TRAFFIC;
    }
    a->path_d = d;
    a->path_go = 1;
    a->path_rev = a->reverse_leg;
    a->reversing = a->reverse_leg;
    a->cmd_speed = (a->reverse_leg ? -1.0f : 1.0f) * fminf(a->path_cap, v->cruise_speed);
    if (a->path_free <= GM_PATH_NO_ROOM && !a->path_turn) {
        /* Standing before something: the feedback the task layer reads (waiting / blocked). */
        a->blocker = a->path_blocker;
        a->state = a->path_blocker ? GM_WAIT_TRAFFIC : GM_STATIC_BLOCKED;
        a->geometry = a->path_blocker ? GM_CLEAR : GM_BLOCKED;
    } else if (a->path_turn) {
        a->state = GM_TURNING;
    }
}

#include "ground_motion_library.inc"

/* ---- One vehicle's publication. ---- */

static void gm_publish(const gm_snapshot_t *s, const gm_agent_t *a, gm_vehicle_plan_t *out)
{
    const gm_vehicle_t *v = a->v;
    const gm_commitment_t *c = &a->c;
    memset(out, 0, sizeof *out);
    out->id = v->id;
    out->life_id = v->life_id;
    out->route_id = v->route_id;
    out->state = a->state;
    out->geometry = a->geometry;
    out->blocker_id = a->blocker;
    out->target_speed = a->cmd_speed;
    out->wanted_speed = a->v_des; /* s22c: read-only addition for the WASD driver */
    out->target_yaw_rate = a->cmd_yaw;
    out->firm = a->firm && !a->reversing;
    out->blocked_segment = -1;
    out->blocked_at = a->p;
    out->goal = a->goal;
    out->release_step = a->cmd_speed != 0.0f ? 0 : -1;
    out->path_cursor = a->cursor;
    out->terrain_recovery = a->reversing;
    out->repair_active = a->following;
    out->repair_rejoined = a->following && a->rejoined;
    out->commit_until_ms = s->ms + GM_STEP_MS;
    out->route_status = c->route_status;
    out->route_revision = a->following ? c->nav_revision : 0u;
    out->route_target = a->target;
    if (s->local_planner && a->lp_moving) {
        out->local_drive = 1;
        out->curvature = a->cmd_curvature;
    }
    if (s->path_mode && v->controlled) {
        /* s24: the driver's whole input (gm_path_decide); a tank that does not move gets go 0 */
        out->path = 1;
        out->go = a->path_go;
        out->path_reverse = a->path_rev;
        out->route_target = a->path_go ? a->aim : a->target;
        out->stop_distance = a->path_go && isfinite(a->path_d) ? fmaxf(0.0f, a->path_d) : -1.0f;
        out->stop_blocker = a->path_go ? a->path_blocker : 0;
        out->speed_cap = a->path_go ? a->path_cap : INFINITY;
        out->terrain_recovery = a->path_rev;
        out->firm = 0;
    }
    if (s->path_mode && s->routes_lib && v->controlled) {
        /* s26 library mode: the start turn's gear, the curve's revision, and the probe's columns */
        out->pivot_reverse = a->path_go && a->lib_pivot_rev;
        out->lib_align = a->lib_align; /* s29 */
        out->street_closed = a->lib_street; /* s34 */
        out->route_revision = a->following ? a->lib_rev : 0u;
        out->lib_source = a->lib_source;
        out->lib_route = a->lib_route;
        out->lib_sample = a->lib_sample;
        out->lib_cross = a->lib_cross;
        out->lib_clear = a->lib_clear;
        out->stop_vehicle = a->stop_vehicle;
        out->stop_turn = a->stop_turn;
        out->stop_end = a->stop_end;
    } else {
        out->lib_route = out->lib_sample = -1;
        out->lib_cross = out->lib_clear = NAN;
        out->stop_vehicle = out->stop_turn = out->stop_end = INFINITY;
    }
    /* Open-loop preview of the command. */
    gm_pose_t pose = {v->x, v->z, v->heading, a->speed, v->yaw_rate};
    out->poses[0] = pose;
    for (int k = 1; k <= GM_STEPS; ++k) {
        const float want = a->cmd_speed;
        const float step = (want > pose.speed ? v->acceleration : v->braking) * GM_STEP_S;
        pose.speed += gm_clamp(want - pose.speed, -step, step);
        if (v->controlled && v->move && !a->reversing) {
            const float yaw = gm_clamp(GM_YAW_GAIN * gm_angle(a->cmd_heading - pose.heading), -v->max_yaw_rate, v->max_yaw_rate);
            pose.yaw_rate = a->cmd_yaw != 0.0f || a->cmd_speed != 0.0f ? yaw : 0.0f;
        } else {
            pose.yaw_rate = a->cmd_yaw;
        }
        pose.heading = gm_angle(pose.heading + pose.yaw_rate * GM_STEP_S);
        pose.x += cosf(pose.heading) * pose.speed * GM_STEP_S;
        pose.z += sinf(pose.heading) * pose.speed * GM_STEP_S;
        out->poses[k] = pose;
    }
}

int gm_plan_snapshot(gm_state_t *state, const gm_snapshot_t *snapshot, gm_plan_t *plan)
{
    if (!state || !snapshot || !plan || snapshot->count < 0 || snapshot->count > GM_MAX_VEHICLES
        || (snapshot->count && !snapshot->vehicles) || state->count < 0 || state->count > GM_MAX_VEHICLES)
        return 0;
    for (int i = 0; i < snapshot->count; ++i) {
        if (!gm_valid_vehicle(&snapshot->vehicles[i]))
            return 0;
        for (int j = 0; j < i; ++j)
            if (snapshot->vehicles[j].id == snapshot->vehicles[i].id)
                return 0;
    }
    /* Shared route layer (~50 MB). On allocation failure retry later and run
     * without routes meanwhile. */
    if (!state->nav && (!state->nav_retry_ms || snapshot->ms >= state->nav_retry_ms)) {
        state->nav = gm_nav_shared();
        if (state->nav)
            gm_nav_reset(state->nav);
        else
            state->nav_retry_ms = snapshot->ms + 5000u;
    }
    int routed = 0;
    if (state->nav) {
        const int usable = snapshot->map && snapshot->static_sweep;
        routed = gm_nav_begin(state->nav, usable ? snapshot->map : NULL, snapshot->map_epoch, snapshot->ms)
                 != GM_NAV_NO_MAP && usable;
    }
    const gm_nav_t *nav = routed ? state->nav : NULL;
    const uint64_t ms = snapshot->ms;
    gm_agent_t *agents = gm_agents;
    const int count = snapshot->count;
    gm_lp_snapshot_poses = 0;
    double lp_snapshot_us = 0.0;
    int lp_plans = 0;

    plan->count = count;
    plan->ms = ms;
    plan->valid_until_ms = ms + GM_PLAN_VALID_MS;

    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        const gm_vehicle_t *v = &snapshot->vehicles[i];
        memset(a, 0, sizeof *a);
        a->v = v;
        gm_lookup(state, v, &a->c);
        gm_commitment_t *c = &a->c;
        c->seen_ms = ms;
        a->p = (gm_point_t){v->x, v->z};
        a->vel = (gm_point_t){v->vx, v->vz};
        a->fwd = gm_axis(v->heading);
        a->speed = gm_clamp(gm_dot(a->vel, a->fwd), -GM_SPEED_MAX, GM_SPEED_MAX);
        a->half_length = 0.5f * v->length;
        a->half_width = 0.5f * v->width;
        a->radius = gm_disc_radius(v);
        a->stationary = gm_len(a->vel) < GM_STATIONARY_MPS && fabsf(v->yaw_rate) < 0.1f;
        a->goal = v->controlled ? v->goal : a->p;
        a->target = a->goal;
        a->hold_arc = -1.0f;
        a->state = GM_OBSERVED;
        a->geometry = GM_CLEAR;
        a->cmd_heading = v->heading;
        a->region = gm_region_goal(v);
        const uint64_t region = a->region ? v->goal_region_id : 0;
        if (c->goal_region_id != region) {
            c->goal_region_id = region;
            c->slot_phase = c->slot_valid = c->parked = c->slot_count = c->abandoned_count = 0;
        }
        if (!v->controlled)
            continue;
        /* Parking and arrival follow the measured pose. */
        if (a->region) {
            const int in_region = gm_dist(a->p, v->goal_region_center) < v->goal_region_radius;
            if (c->parked && (!in_region
                || (c->slot_valid && gm_dist(a->p, c->slot) > GM_PARK_RADIUS + GM_PARK_HYSTERESIS)))
                c->parked = 0;
            /* On the slot it parks, also while a turn still creeps it along. */
            /* (s26 library mode: parked within GL_PARK_M of its slot) */
            const float park_r = snapshot->path_mode && snapshot->routes_lib ? GL_PARK_M : GM_PARK_RADIUS;
            if (!c->parked && c->slot_valid && gm_dist(a->p, c->slot) <= park_r && fabsf(a->speed) < 2.0f)
                c->parked = 1;
            if (c->slot_valid)
                a->goal = c->slot;
            c->arrived = 0;
        } else if (v->move) {
            /* Stop on the goal, not at the rim of its arrival radius: the
             * next vehicle's goal may be one hull spacing away. */
            /* A goal in an obstacle is reached at the standing room next to it. */
            if (c->route_goal_moved && gm_dist(c->key_goal, v->goal) <= 2.0f)
                a->goal = c->route_goal;
            const float d = gm_dist(a->p, a->goal), stop = fminf(v->arrival_radius, GM_STOP_RADIUS);
            if (d <= stop)
                c->arrived = 1;
            else if (d > stop + GM_ARRIVE_SLIP && !c->arrived_short)
                c->arrived = 0;
        }
        const int prior = c->window_valid && v->path_count
            && gm_dist(c->first_waypoint, v->path[0]) < 0.05f ? c->cursor : 0;
        a->cursor = gm_upstream_cursor(v, c, a->p);
        a->rejoined = a->cursor > prior;
        gm_lane_offset(state, v, c, a->p, a->cursor);
        a->aligning = !v->move && v->align
            && (fabsf(gm_angle(v->align_heading - v->heading)) > 0.035f || fabsf(v->yaw_rate) > 0.075f);
        a->disabled = v->move && c->disabled_until_ms > ms;
        a->mobile = v->move && !c->parked && !c->arrived && !a->disabled;
    }

    const int lib_mode = snapshot->path_mode && snapshot->routes_lib;
    if (lib_mode)
        gl_events(state, snapshot, agents, count); /* s26: goal identity and the picks that need no search */

    gm_route_layer(state, snapshot, agents, routed);

    if (lib_mode) {
        gl_adopt_nav(snapshot, agents, count);
        gl_serve(snapshot, agents, count, routed); /* s26: at most one grid search per snapshot */
    }
    const int path_mode = snapshot->path_mode != 0;
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        if (a->mobile && lib_mode) {
            const double begin_us = gm_clock_us();
            gl_follow(snapshot, agents, count, i);
            const double spent = gm_clock_us() - begin_us;
            lp_snapshot_us += spent;
            ++lp_plans;
            if (spent > gm_lp_stats.max_plan_us)
                gm_lp_stats.max_plan_us = spent;
        } else if (a->mobile && path_mode) {
            /* s24: aim, stop distance, turn room (timed like the local planner, for its statistics line) */
            const double begin_us = gm_clock_us();
            gm_path_follow(snapshot, nav, agents, count, i);
            const double spent = gm_clock_us() - begin_us;
            lp_snapshot_us += spent;
            ++lp_plans;
            if (spent > gm_lp_stats.max_plan_us)
                gm_lp_stats.max_plan_us = spent;
        } else if (a->mobile) {
            gm_follow(snapshot, nav, a);
        } else {
            a->remaining = 0.0f;
        }
    }
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        const gm_vehicle_t *v = a->v;
        if (!a->mobile || lib_mode) /* s26 library mode: no single-lane hold, no yield rule */
            continue;
        gm_narrow_hold(nav, agents, count, i);
        if (a->hold_arc >= 0.0f) {
            const float cap = a->hold_arc < 0.5f ? 0.0f : sqrtf(2.0f * GM_DECEL_FRACTION * v->braking * a->hold_arc);
            if (cap < a->v_des) {
                a->pref = gm_scale(gm_unit(gm_len(a->pref) > 0.05f ? a->pref : a->fwd), cap);
                a->v_des = cap;
            }
        }
        if (ms < a->c.yield_until_ms) {
            /* It backed off for a hull a moment ago: that one passes first. */
            const int other = gm_agent_index(agents, count, a->c.yield_id);
            if (other >= 0 && gm_dist(a->p, agents[other].p) < GM_YIELD_RANGE) {
                a->v_des = 0.0f;
                a->pref = (gm_point_t){0.0f, 0.0f};
                a->hold_arc = 0.0f;
                a->hold_blocker = a->c.yield_id;
            } else {
                a->c.yield_until_ms = 0;
            }
        }
        /* A route the layer could not find (the hull stands against
         * something, or the goal is taken) is no reason to freeze: gm_follow
         * already aims at the next upstream waypoint or the goal, and the
         * static limits and the avoidance below still hold. */
    }
    if (snapshot->local_planner && nav && !path_mode) /* s24: no spawn queue, no exit mode in path mode */
        gm_spawn_queue(snapshot, agents, count, nav);

    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        const gm_vehicle_t *v = a->v;
        gm_commitment_t *c = &a->c;
        if (!v->controlled)
            continue;
        if (!a->mobile) {
            if (a->aligning && lib_mode) {
                gl_align(snapshot, agents, count, i); /* s29: s27's turn-room rule for an align too */
            } else if (a->aligning) {
                a->state = GM_TURNING;
                a->cmd_heading = v->align_heading;
                a->cmd_yaw = gm_clamp(GM_YAW_GAIN * gm_angle(v->align_heading - v->heading), -v->max_yaw_rate, v->max_yaw_rate);
            } else if (a->disabled) {
                a->state = GM_STATIC_BLOCKED; /* immobile: left alone for a while, the others go round it */
                a->geometry = GM_BLOCKED;
            } else {
                a->state = v->move && c->arrived ? GM_ARRIVED : GM_PARKED;
            }
            c->traffic_since_ms = c->static_since_ms = c->hard_since_ms = c->stall_since_ms = 0;
            c->cmd_valid = 0;
            c->lp_valid = c->lp_seg_active = 0;
            if (path_mode) {
                /* s24: an avoidance decision does not outlive the move order */
                c->pa_state = GM_PA_IDLE;
                c->pa_low_since_ms = 0;
                c->pa_waits_for = 0;
            }
            continue;
        }
        if (lib_mode) {
            /* s26: the library mode's avoidance layer (vehicles only) and the driver's input */
            gl_decide(snapshot, agents, count, i);
            continue;
        }
        if (path_mode) {
            /* s24: the avoidance layer and the driver's input; none of the s12..s22 rules below */
            gm_path_decide(routed ? state->nav : NULL, snapshot, agents, count, i, routed);
            continue;
        }
        if (a->sq_hold) {
            /* s13 spawn exit queue: brakes on until the one ahead is away. */
            a->state = GM_WAIT_SPAWN;
            a->blocker = a->sq_blocker;
            a->cmd_speed = a->cmd_yaw = a->cmd_curvature = 0.0f;
            a->cmd_heading = v->heading;
            a->geometry = GM_CLEAR;
            c->traffic_since_ms = c->static_since_ms = c->hard_since_ms = c->stall_since_ms = 0;
            if (!(a->ex_mode && c->sq_released)) {
                /* (s16 exit mode: one let go before and held again keeps its
                 * progress clock: stuck where it stands, it must not hold the
                 * ones behind it for ever, SQ_WEDGE_MS.) */
                c->progress_ms = ms;
                c->progress_anchor = a->p;
            }
            c->cmd_valid = 0;
            c->lp_valid = c->lp_seg_active = 0;
            c->ex_nofwd_ms = 0; /* s16: held is not stuck */
            continue;
        }
        if (a->reverse_leg) {
            /* Statically boxed in and nav's way out starts backwards: only
             * with nothing close behind. */
            int clear_behind = 1;
            for (int j = 0; j < count; ++j) {
                const gm_point_t rel = gm_sub(agents[j].p, a->p);
                if (j != i && gm_dot(rel, a->fwd) < 0.0f && gm_len(rel) < a->radius + agents[j].radius + 6.0f)
                    clear_behind = 0;
            }
            a->state = clear_behind ? GM_DRIVING : GM_WAIT_TRAFFIC;
            a->reversing = clear_behind;
            a->cmd_speed = clear_behind ? -v->reverse_speed : 0.0f;
            a->cmd_yaw = 0.0f;
            a->geometry = GM_CLEAR;
            /* With a hull behind there is no backing out: that leg is passed
             * over for a while and the way on is sought forwards. */
            if (!clear_behind)
                c->no_reverse_until_ms = ms + 5000u;
            continue;
        }
        const int local = snapshot->local_planner && nav;
        if (local) {
            const double begin_us = gm_clock_us();
            lp_crumb(c, a->p);
            gm_local_choose(snapshot, agents, count, i, nav);
            const double spent = gm_clock_us() - begin_us;
            lp_snapshot_us += spent;
            ++lp_plans;
            if (spent > gm_lp_stats.max_plan_us)
                gm_lp_stats.max_plan_us = spent;
        } else {
            gm_choose(snapshot, agents, count, i, nav);
        }
        c->cmd_heading = a->cmd_heading;
        c->cmd_valid = 1;
        const int unreachable = routed && c->route_status == GM_NAV_UNREACHABLE;

        /* Bookkeeping: progress, how long traffic or geometry has held it. */
        if (!c->progress_ms || gm_dist(a->p, c->progress_anchor) > 1.5f) {
            c->progress_anchor = a->p;
            c->progress_ms = ms;
        }
        /* Creeping up on a hull in its way, a few decimetres a second and
         * stopping again, is waiting for it: the clocks below must not start
         * over each time it rolls a hand's breadth. */
        const int crawling = a->state == GM_DRIVING && a->cmd_speed < GM_CRAWL_SPEED && a->pref_blocked;
        const int traffic = (a->state == GM_WAIT_TRAFFIC || crawling) && a->hold_arc < 0.0f;
        const int blocked = a->state == GM_STATIC_BLOCKED;
        c->traffic_since_ms = traffic ? (c->traffic_since_ms ? c->traffic_since_ms : ms) : 0;
        c->static_since_ms = blocked ? (c->static_since_ms ? c->static_since_ms : ms) : 0;
        /* Held by something that will not move by itself: static geometry,
         * no room to turn, a hull standing for good. And a standing hull so
         * close that the turn this one needs would sweep into it, whether or
         * not that hull means to move on: having pulled up against it, the
         * only way to get the room for the turn is back. */
        const int j = a->blocker ? gm_agent_index(agents, count, a->blocker) : -1;
        const int hard = a->v_des > 0.3f
            && (blocked || (traffic && (j < 0 || (agents[j].stationary && (!agents[j].mobile || a->turn_boxed)))));
        c->hard_since_ms = hard ? (c->hard_since_ms ? c->hard_since_ms : ms) : 0;
        /* Two hulls waiting on each other at a standstill, nose to nose or
         * across each other's way: nothing moves until one makes room. The
         * one with farther to go does, and goes on doing so while the other
         * follows up. Queueing behind a hull that faces the same way is not
         * this: there waiting, and after a while a route round it, is right. */
        const int yields = j >= 0 && (a->remaining > agents[j].remaining
                                      || (a->remaining == agents[j].remaining && v->id > agents[j].v->id));
        const int jammed = traffic && a->v_des > 0.3f && j >= 0 && agents[j].stationary && agents[j].mobile && yields
            && gm_dot(agents[j].fwd, a->fwd) < 0.3f && ms - c->progress_ms >= GM_JAM_MS;

        /* Ordered forward, or to turn, and nothing happens: wedged against
         * something that neither the obstacle cells nor the game's sweep
         * know, or immobile. */
        /* The local planner also backs up: pushing either way counts. */
        const int push_dir = local && a->cmd_speed < 0.0f ? -1 : 1;
        const int pushing = (a->state == GM_DRIVING && (local ? fabsf(a->cmd_speed) : a->cmd_speed) >= 0.3f
                             && fabsf(a->speed) < 0.5f)
            || (a->state == GM_TURNING && fabsf(a->cmd_yaw) > 0.1f);
        if (!pushing || !c->stall_since_ms || gm_dist(a->p, c->stall_anchor) > GM_WEDGE_MOVE
            || fabsf(gm_angle(v->heading - c->stall_heading)) > GM_WEDGE_TURN
            || (local && c->lp_push_dir != push_dir)) {
            /* (The local planner: a push the other way is a new push.) */
            c->stall_since_ms = pushing ? ms : 0;
            c->stall_anchor = a->p;
            c->stall_heading = v->heading;
            c->stall_reported = 0;
        }
        if (local)
            c->lp_push_dir = push_dir;
        const int wedged = pushing && ms - c->stall_since_ms >= GM_WEDGE_MS;
        if (wedged && routed && !c->stall_reported) {
            /* Whatever holds the nose is real: the route layer keeps clear of it. */
            gm_nav_report_blocked(state->nav, v, gm_add(a->p, gm_scale(a->fwd, (float)push_dir * (a->half_length + 1.0f))), 2.0f);
            c->stall_reported = 1;
        }
        if (local) {
            /* s13: what holds the hull is not in the grid. A veto point there
             * for a while: the planner itself finds the reverse or the
             * manoeuvre (and the old back-off below only runs if it stands). */
            const int side = push_dir > 0 ? 0 : 1;
            if (wedged && c->lp_veto_until_ms[side] <= ms) {
                c->lp_veto[side] = gm_add(a->p, gm_scale(a->fwd, (float)push_dir * (a->half_length + 0.3f)));
                c->lp_veto_until_ms[side] = ms + LP_VETO_MS;
                if (snapshot->local_log) {
                    char line[160];
                    snprintf(line, sizeof line, "%llu wedge id %llu at (%.1f,%.1f) heading %.2f %s, veto (%.1f,%.1f) for %u ms",
                             (unsigned long long)ms, (unsigned long long)v->id, (double)a->p.x, (double)a->p.z,
                             (double)v->heading, push_dir > 0 ? "forward" : "reverse", (double)c->lp_veto[side].x,
                             (double)c->lp_veto[side].z, LP_VETO_MS);
                    snapshot->local_log(snapshot->local_context, 1, line);
                }
                c->lp_wedge_open |= 1 << side;
            }
            for (int k = 0; k < 2; ++k)
                if ((c->lp_wedge_open >> k & 1) && gm_dist(a->p, c->lp_veto[k]) > a->half_length + 2.5f) {
                    if (snapshot->local_log) {
                        char line[160];
                        snprintf(line, sizeof line, "%llu recovered id %llu at (%.1f,%.1f), %.1f s after the %s veto",
                                 (unsigned long long)ms, (unsigned long long)v->id, (double)a->p.x, (double)a->p.z,
                                 0.001 * (double)(ms + LP_VETO_MS - c->lp_veto_until_ms[k]), k ? "rear" : "front");
                        snapshot->local_log(snapshot->local_context, 1, line);
                    }
                    c->lp_wedge_open &= ~(1 << k);
                }
        }

        if (a->region) {
            /* Inside the region and held up, by traffic or with no way on to
             * the slot: park here rather than jam it. And at the rim behind
             * hulls that stand for good there is no getting further in: this
             * is as close as it gets. */
            const float from_centre = gm_dist(a->p, v->goal_region_center);
            const int inside = from_centre < v->goal_region_radius - 3.0f;
            const int rim = from_centre < v->goal_region_radius + GM_RIM_PARK;
            const int waited = traffic && ms - c->traffic_since_ms >= GM_PARK_WAIT_MS;
            const int stopped = blocked && ms - c->static_since_ms >= GM_PARK_WAIT_MS;
            const uint64_t idle = ms - c->progress_ms;
            /* A hull length from the slot and facing the wrong way: turning
             * for it would only creep it away again. */
            const int beside = c->slot_valid && gm_dist(a->p, c->slot) < GM_PARK_NEAR
                && fabsf(gm_angle(a->cmd_heading - v->heading)) > 0.8f;
            if (!inside && c->slot_phase == 2 && (traffic || blocked) && idle >= GM_RESLOT_MS
                && c->abandoned_count < GM_ABANDONED_SLOTS) {
                /* Held up short of the region: outside it a tank captures
                 * nothing, so before it settles out here it tries for another
                 * slot, which lies another way. */
                gm_abandon_slot(c, c->slot);
                gm_slot_candidates(nav, snapshot, agents, i, c);
                if (c->slot_count) {
                    /* The old slot stays its goal until the search has bound
                     * another: nothing is gained by aiming at the centre of
                     * the region for the second in between. */
                    c->slot_phase = 1;
                    gm_new_key(c, v);
                    c->replan_ms = ms;
                }
                c->progress_ms = ms;
                c->progress_anchor = a->p;
            } else if ((inside && (waited || stopped || wedged || beside)) || (idle >= GM_SETTLE_MS && rim)
                       || (waited && idle >= GM_RESLOT_MS && j >= 0 && agents[j].stationary && !agents[j].mobile && rim)) {
                c->parked = 1;
                c->slot = a->p;
                c->slot_valid = 1;
                c->slot_phase = 2;
            }
        }
        /* Close enough to a goal that a standing hull occupies: this is
         * arrival. Occupies: its hull leaves no room for this one on the goal;
         * a hull parked on its own place beside the way there is just to be
         * passed. So is getting nowhere for a good while within sight of the
         * goal, and standing beside it facing the wrong way. */
        const int goal_taken = j >= 0 && agents[j].stationary && !agents[j].mobile
            && fabsf(gm_dot(gm_sub(a->goal, agents[j].p), agents[j].fwd)) < agents[j].half_length + a->half_width + GM_SIDE_GAP
            && fabsf(gm_det(agents[j].fwd, gm_sub(a->goal, agents[j].p))) < agents[j].half_width + a->half_width + GM_SIDE_GAP;
        if (!a->region
            && ((traffic && ms - c->traffic_since_ms >= GM_PARK_WAIT_MS && goal_taken
                 && gm_dist(a->p, a->goal) < v->arrival_radius + 2.5f * a->radius)
                || (ms - c->progress_ms >= GM_SETTLE_MS && gm_dist(a->p, a->goal) < GM_SETTLE_RANGE)
                || (gm_dist(a->p, a->goal) < GM_PARK_NEAR && fabsf(gm_angle(a->cmd_heading - v->heading)) > 0.8f)))
            c->arrived = c->arrived_short = 1;

        if (routed && !unreachable && !c->replan && ms >= c->replan_ms + GM_REPLAN_COOLDOWN_MS) {
            if (j >= 0 && agents[j].body && a->pref_blocked && c->replanned_body != a->blocker) {
                /* The way ahead runs into a hull that stands for good (parked,
                 * wrecked, not ours): ask for a route around it right away. */
                c->replanned_body = a->blocker;
                c->replan = 1;
            } else if (traffic && ms - c->progress_ms >= GM_STUCK_MS && j >= 0 && agents[j].stationary) {
                /* Stuck behind a hull that is not moving: route around it. */
                c->body_id = a->blocker;
                c->body_until_ms = ms + GM_BODY_MS;
                c->replan = 1;
            } else if (blocked && ms - c->static_since_ms >= 2500u) {
                c->replan = 1;
            } else if (a->following && a->cross > GM_LOST_ROUTE) {
                c->replan = 1;
            }
        }
        if (c->ex_replan_pending) {
            /* s16: it left the spawn by another way than its route's: one new
             * route from here, once the cooldown allows; none if it is back on
             * the route or a search is under way already. */
            if (c->replan || !(a->following && a->cross > EX_OFF_ROUTE_M)) {
                c->ex_replan_pending = 0;
            } else if (routed && !unreachable && ms >= c->replan_ms + GM_REPLAN_COOLDOWN_MS) {
                c->replan = 1;
                c->ex_replan_pending = 0;
                c->ex_replan_done = 1;
                sq_log(snapshot, "%llu queue: id %llu new route after the exit, %.1f m off the old one",
                       (unsigned long long)ms, (unsigned long long)v->id, (double)a->cross);
            }
        }

        /* Back off: reverse straight a few metres and ask for a route from
         * there. Only when nothing else is left: the hull is wedged, or for
         * some seconds it could neither go on nor turn (nose to a wall, a
         * hull that stands for good in the way). Not among the spawn points,
         * and not twice at the same place. An unreachable route holds it back
         * only where the hull is clear of the route layer's margin: inside it
         * no search could start (Fulda seats 16, 26, 30 stood 30-97 s beside
         * buildings that way), and backing off is the way out. */
        if (c->escape_until_ms) {
            const int timed_out = ms >= c->escape_until_ms;
            if (timed_out || gm_dist(a->p, c->escape_from) >= GM_ESCAPE_BACK
                || !gm_escape_clear(snapshot, nav, agents, count, i)) {
                if (timed_out && gm_dist(a->p, c->escape_from) < GM_STALL_MOVE)
                    c->disabled_until_ms = ms + GM_DISABLED_MS; /* neither forwards nor back: immobile */
                c->escape_until_ms = 0;
                c->escape_done_ms = ms;
                c->stall_since_ms = 0;
                c->replan = 1;
            }
        } else if ((wedged || jammed
                    || (hard && ms - c->hard_since_ms >= GM_HARD_WAIT_MS
                        && (!unreachable || gm_nav_overlap(nav, v, &(gm_pose_t){a->p.x, a->p.z, v->heading, 0.0f, 0.0f},
                                                           GM_NAV_MARGIN) > 0)))
                   && !a->among && !(local && a->lp_moving)
                   && (jammed || !c->escape_done_ms || ms >= c->escape_done_ms + GM_ESCAPE_REPEAT_MS
                       || gm_dist(a->p, c->escape_from) > GM_ESCAPE_SAME_PLACE)
                   && gm_escape_clear(snapshot, nav, agents, count, i)) {
            c->escape_until_ms = ms + GM_ESCAPE_MS;
            c->escape_from = a->p;
            if (jammed && !wedged) {
                /* Having made room, it lets the other through before it comes back. */
                c->yield_id = a->blocker;
                c->yield_until_ms = ms + GM_ESCAPE_MS + GM_YIELD_MS;
            }
        } else if (wedged && ms - c->stall_since_ms >= 2u * GM_STALL_MS) {
            c->disabled_until_ms = ms + GM_DISABLED_MS; /* wedged with no way back either */
            c->stall_since_ms = 0;
        }
        if (c->escape_until_ms) {
            a->state = GM_DRIVING;
            a->reversing = 1;
            a->cmd_speed = -fminf(v->reverse_speed, GM_ESCAPE_SPEED);
            a->cmd_yaw = 0.0f;
            a->geometry = GM_CLEAR;
            a->blocker = 0;
            a->lp_moving = 0; /* the old straight back-off: steering centred */
            a->cmd_curvature = 0.0f;
        }
    }

    /* What is known of a vehicle that is missing from this snapshot is kept
     * for a while. In the game a tank whose move order went stale during a
     * hitch is listed for a publication or two as a foreign hull under another
     * id; losing its route, its track and its bookkeeping each time made whole
     * teams start over together. */
    if (snapshot->local_planner || path_mode)
        lp_stats_note(snapshot, lp_snapshot_us, lp_plans, gm_lp_snapshot_poses);
    static gm_commitment_t kept[GM_MAX_VEHICLES];
    int keep = 0;
    for (int j = 0; j < state->count && count + keep < GM_MAX_VEHICLES; ++j) {
        const gm_commitment_t *old = &state->commitments[j];
        if (ms > old->seen_ms + GM_RETAIN_MS || gm_agent_index(agents, count, old->id) >= 0)
            continue;
        kept[keep++] = *old;
    }
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        a->c.last_state = (int)a->state;
        gm_publish(snapshot, a, &plan->vehicles[i]);
        state->commitments[i] = a->c;
    }
    if (keep)
        memcpy(&state->commitments[count], kept, (size_t)keep * sizeof kept[0]);
    state->count = count + keep;
    return 1;
}
