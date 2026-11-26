#ifndef GROUND_LIDAR_H
#define GROUND_LIDAR_H
/* s11 (2026-11-15): live local perception per bot tank, "lidar" + local
 * occupancy grid. s13 (2026-11-16): the motion layer's local trajectory
 * sampler reads it through lidar_block (the read-only API at the end of this
 * header); control and nav do not.
 *
 * Rays per tank and refresh cycle (GL_CYCLE_RAYS = 161), cast with the game's
 * collision ray (gh_ray of ground_host.h) through gl_config_t.trace:
 *   1 centre ray: down from 3 m above the tank, land flags (GH_RAY_GROUND): the
 *     tank's own ground height g.
 *   144 ring rays: 72 world azimuths (5 deg, azimuth i = i * 5 deg from +x
 *     towards +z, the gm_pose_t sense) at two heights above g, 0.6 m (layer
 *     0, hull bottom: low walls, kerb stones, rubble) and 1.6 m (layer 1,
 *     hull top), from the hull centre, flags GH_RAY_STATIC_AND_OBJECTS (static + placed objects, vehicles
 *     are not hit, so the own hull does not block), range range_m (12 m).
 *     A hit at t < GL_INSIDE_M is recorded as GL_RAY_INSIDE (the ray started
 *     inside an obstacle).
 *   16 ground probes: 8 ego directions (k * 45 deg from the heading towards
 *     +z) x 4 m and 8 m, down from g + 3 m with land flags: ground height
 *     there minus g, and the GL_G_* flags (step, drop, steep, deep water,
 *     no ground).
 * A cycle is four slices of 40 rays (slice k: azimuths k, k+4, ..., both
 * layers, then the ground probes of directions 2k and 2k+1) after the centre
 * ray, so a partial cycle still covers the whole circle at 20 deg.
 *
 * Scheduling: each tank wants one cycle per period_ms (100 ms, 10 Hz). The
 * work of all tanks shares one time cap, us_per_tick microseconds of lidar
 * work (rays + grid updates, measured with gh_qpc) per
 * budget window of window_ms (16 ms; the window starts at the first call
 * after the previous one ended, so with game frames of 16 ms or more it is
 * the cap per frame). Within a window tanks are served one slice at a time:
 * first those served fewest times in this window (round robin), among them
 * the most urgent: urgency = (ms - due + 1) x weight, due = start of the
 * running cycle or the due time of the next one, weight 2 for a tank that is
 * slow (< 1 m/s), has a ring hit within 4 m or is in a snapshot state
 * (static_blocked / wait_traffic), else 1. Lateness grows without bound, so
 * nobody starves; a slow or crowded tank only gets its turn earlier.
 *
 * Grid: per tank, world aligned, 96 x 96 cells of 0.25 m (24 m) centred on
 * the tank and scrolled as it moves (a ring buffer: cells that enter the
 * window are cleared to unknown). One int8 log-odds plane per ray layer
 * (low / high), traversed with a 2D DDA from the hull centre: every cell
 * before the hit gets GL_L_FREE, the hit cell GL_L_HIT, clamped to
 * [GL_L_MIN, GL_L_MAX]; every 100 ms each cell decays one step towards 0
 * (unknown). Occupied: L >= GL_L_OCC, free: L <= GL_L_FREE_AT. A per-cell
 * observation counter (saturating, cleared on scroll) tells a cell seen once
 * from one seen many times.
 *
 * Threading: everything here runs on the game thread inside the adapter's
 * ground lock; the API is not thread safe. No allocation after gl_init. */
#include <stdint.h>

#define GL_MAX_SEATS 32
#define GL_AZIMUTHS 72
#define GL_LAYERS 2
#define GL_GROUND_DIRS 8
#define GL_GROUND_DISTS 2
#define GL_SLICES 4
#define GL_RING_RAYS (GL_AZIMUTHS * GL_LAYERS)
#define GL_GROUND_RAYS (GL_GROUND_DIRS * GL_GROUND_DISTS)
#define GL_SLICE_RAYS (GL_RING_RAYS / GL_SLICES + GL_GROUND_RAYS / GL_SLICES)
#define GL_CYCLE_RAYS (1 + GL_RING_RAYS + GL_GROUND_RAYS)
#define GL_GRID 96
#define GL_CELL_M 0.25f
#define GL_INSIDE_M 0.05f
#define GL_LAYER_LOW_M 0.6f
#define GL_LAYER_HIGH_M 1.6f
#define GL_PROBE_UP_M 3.0f
#define GL_PROBE_DOWN_M 6.0f
#define GL_FLAGS_RING GH_RAY_STATIC_AND_OBJECTS
#define GL_FLAGS_LAND GH_RAY_GROUND

/* Log-odds (int8 units). */
#define GL_L_HIT 12
#define GL_L_FREE (-3)
#define GL_L_MAX 48
#define GL_L_MIN (-24)
#define GL_L_OCC 6
#define GL_L_FREE_AT (-3)
#define GL_DECAY_MS 100

/* Ray flags in gl_ring_t.flags. */
#define GL_RAY_CAST 0x01
#define GL_RAY_HIT 0x02
#define GL_RAY_INSIDE 0x04
#define GL_RAY_STATIC 0x08     /* out_kind 2 */
#define GL_RAY_RENDINST 0x10   /* out_kind 4 */

/* Ground probe flags per ego direction. */
#define GL_G_STEP_UP 0x01      /* > 1.5 m up at 4 m, or between 4 m and 8 m */
#define GL_G_DROP 0x02         /* > 1.5 m down likewise */
#define GL_G_STEEP 0x04        /* grade > 0.45 (GT_SLOPE_HARD) over 0-4 m or 4-8 m */
#define GL_G_WATER 0x08        /* map water level more than 1 m (GT_WADE_M) above the ground there */
#define GL_G_NONE 0x10         /* a probe found no ground within 3 m up / 6 m down */
#define GL_G_START_BELOW 0x20  /* a probe started below the ground (> 3 m step up) */

/* lidar_cell results and layer bits. */
#define GL_CELL_NONE (-1)
#define GL_CELL_UNKNOWN 0
#define GL_CELL_FREE 1
#define GL_CELL_OCCUPIED 2
#define GL_LOW_OCC 0x1
#define GL_HIGH_OCC 0x2
#define GL_LOW_FREE 0x4
#define GL_HIGH_FREE 0x8

/* 1 hit (*t = hit distance), 0 no hit within *t, -1 the ray raised (lidar stops). */
typedef int (*gl_trace_fn)(void *context, const float p[3], const float dir[3], float *t, int *kind, int flags);
typedef void (*gl_event_fn)(void *context, const char *message);
typedef double (*gl_clock_fn)(void *context);   /* microseconds; tests only (NULL: gh_qpc) */

typedef struct {
    gl_trace_fn trace;
    void *trace_context;
    gl_event_fn event;
    void *event_context;
    gl_clock_fn clock_us;
    void *clock_context;
    float range_m;              /* default 12 */
    int us_per_tick;            /* default 1500 */
    int window_ms;              /* default 16 */
    int period_ms;              /* default 100 */
    float water;                /* map water level, NAN: none */
    /* WTOBS001 prior (optional), only for the evaluation logs. */
    const uint16_t *prior;
    uint32_t prior_nx, prior_nz;
    float prior_x0, prior_z0, prior_step;
    const char *log_dir;        /* NULL or "": no files */
    const char *file_prefix;    /* prepended to lidar.csv / lidar_snap.bin (model runs) */
} gl_config_t;

typedef struct {
    int seat;
    uint64_t life;              /* a new life clears the seat */
    float x, y, z, heading, speed;
    int state;                  /* the motion state (enum gm_state), logged */
    const char *state_name;
    int snap_state;             /* static_blocked or wait_traffic: grid snapshots after 1 s */
    float target_x, target_z;   /* the plan's current target (route vertex) */
    float goal_x, goal_z;
    float hull_half_length, hull_half_width, hull_forward, hull_right;
} gl_tank_t;

typedef struct {
    uint64_t life;
    float x, z, ground_y, heading;      /* pose of the last centre ray */
    int own_water;                      /* own ground more than 1 m under the water level */
    float hit[GL_LAYERS][GL_AZIMUTHS];  /* metres from the hull centre; INFINITY: none within range; NAN: not cast */
    uint8_t flags[GL_LAYERS][GL_AZIMUTHS];
    uint64_t ms[GL_LAYERS][GL_AZIMUTHS];
    float ground_dh[GL_GROUND_DIRS][GL_GROUND_DISTS]; /* ground there minus ground_y; NAN: none */
    uint8_t ground_flags[GL_GROUND_DIRS];             /* ego direction k = heading + k * 45 deg (towards +z) */
    float ground_heading;                             /* heading the ground probes were cast at */
    uint64_t ground_ms;
    uint32_t cycles;
} gl_ring_t;

typedef struct {
    uint64_t rays, windows, capped_windows, cycles, faults, bad_answers;
    double ray_us, work_us, log_us, max_window_us;
    double ray_p50_us, ray_p99_us;
} gl_stats_t;

/* Adapter side. gl_init copies the config (strings must outlive the battle). */
void gl_init(const gl_config_t *config);
void gl_shutdown(uint64_t ms);          /* end event, files closed; gl_init again for the next battle */
int gl_active(void);                    /* initialised and not faulted */
/* ms: the adapter's clock (logs, scheduling); now_us: a fine monotonic clock
 * for the budget window (gh_qpc in the game, model time in
 * the movement model). tanks: the live bot tanks now. */
void gl_tick(uint64_t ms, uint64_t now_us, const gl_tank_t *tanks, int count);
void gl_stats(gl_stats_t *out);
double gl_refresh_hz(int seat);         /* achieved cycles per second of this life so far */

/* ---- Read-only API for the next stage (the planner); seat 1..31. ---- */
/* State of the cell holding world (x, z): GL_CELL_* (GL_CELL_NONE: no grid or
 * outside its window); *layers (optional) gets GL_LOW_* / GL_HIGH_* bits. A
 * cell is occupied if either layer is (low only: kerb, rubble, low wall;
 * both: wall, building; high only: something overhanging the hull). */
int lidar_cell(int seat, float x, float z, int *layers);
/* Free distance from the hull centre along world heading (0 along +x,
 * positive towards +z) to the first occupied cell, at most the range; -1 no
 * grid. Unknown cells do not stop it. */
float lidar_clearance(int seat, float heading_rad);
/* As lidar_clearance over a corridor of +-half_width; stop_at_unknown also
 * stops at unknown cells. *why (optional): 0 range, 1 occupied, 2 unknown, 3 window edge. */
float lidar_clearance_ex(int seat, float heading_rad, float half_width, int stop_at_unknown, int *why);
/* Latest ring and ground probes, NULL if none. */
const gl_ring_t *lidar_ring(int seat);
/* Age of the oldest ring ray, ms (UINT64_MAX if the ring is incomplete). */
uint64_t lidar_age_ms(int seat, uint64_t ms);
/* s13 (local planner): the cells of world cell columns wc0 .. wc0+n-1 and
 * rows wr0 .. wr0+n-1 (world cell = floor(coordinate / GL_CELL_M)) into
 * out[row * n + col]: GL_BLOCK_OCCUPIED, GL_BLOCK_FREE (free now in a layer
 * that saw it free at least twice: gl_free_twice) or GL_BLOCK_UNKNOWN (also
 * outside the window and seen free only once). Returns the number of cells
 * that are not unknown, -1 without a grid. */
#define GL_BLOCK_UNKNOWN 0
#define GL_BLOCK_FREE 1
#define GL_BLOCK_OCCUPIED 2
int lidar_block(int seat, int wc0, int wr0, int n, uint8_t *out);
#endif
