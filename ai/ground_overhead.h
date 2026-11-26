#ifndef GROUND_OVERHEAD_H
#define GROUND_OVERHEAD_H
/* s17 (2026-11-16): overhead probes into one map-wide shared occupancy cache.
 *
 * Why: the s11 lidar (ground_lidar.c) casts 12 m horizontal rays from each
 * hull: walls occlude them, 5 deg apart they miss thin things beyond ~12 m,
 * and twice the range is half the refresh. Here vertical rays go down onto
 * cell centres instead, so what is seen does not depend on line of sight, and
 * every cell probed serves every tank: the cost depends on the area around
 * the tanks, not on their number. The lidar stays for the near field (thin
 * fences between cell centres, vertical sides, moving vehicles).
 *
 * Cache: world cells of cell_m (0.5 m) metres, world cell = floor(coordinate
 * / cell_m), in tiles of GO_TILE x GO_TILE cells allocated on first write from
 * a pool of max_tiles (1024: 12.6 MB at 0.5 m) found through an open-addressing
 * hash of the tile coordinates. A full pool evicts the tile used least recently
 * (written, or swept by a tank's scheduler). Per cell: state (unknown / free /
 * obstacle / overhead), top height above the ground (obstacle) or clearance of
 * the overhang (overhead), ground height, time of the last probe, the number of
 * agreeing probes in a row, flags (changed, inside, below free).
 *
 * Probe of a cell (go_probe in ground_overhead.c): ray A straight down onto the
 * cell centre with flags_probe (GH_RAY_GROUND_AND_OBJECTS: land, static, placed objects,
 * the flags the aim ray uses) from top_m (8 m; the prior's obstacle height there
 * plus 1.5 m if higher, at most 40 m) above the ground (the cell's ground once
 * known, else the probing tank's height with half a metre per metre of distance
 * up and down for slopes).
 *   out_kind land: the ground; the cell is free.
 *   anything else: ground from the cell, else ray B with flags_land (GH_RAY_GROUND, as
 *   the lidar's ground probes); h = hit height above it:
 *     h < step_m (0.5 m: the obstacle scan's own rule, height code >= 1): free (a floor, a kerb);
 *     h < overhang_m (4 m: the scan's height code < 8): obstacle, top h;
 *     higher: something overhead (bridge deck, canopy, roof): a probe from below
 *     it (0.25, 1, 2 m under the hit, then overhang_m above the ground, while the
 *     start is inside a solid) says what is under it: an obstacle up to
 *     overhang_m is an obstacle, a start inside a solid down to overhang_m above
 *     the ground is an obstacle (a building), otherwise the cell is "overhead"
 *     (flag below free when the probe reached the ground). Overhead cells say
 *     nothing to the planner: the prior and the lidar decide there.
 *   A hit within 5 cm of the start (the start inside a solid) is probed again
 *   from 40 m once; still inside: an obstacle as tall as the start.
 *   No hit at all: the cell stays unknown (tried again after 2 s, from higher).
 * Cells whose centre lies in a known vehicle hull (go_tick's hull list, grown
 * by 0.6 m plus 0.2 s of its speed) are not probed: vehicles never enter the
 * cache (and moving ones are the lidar's business).
 *
 * Scheduler (shared): every tick, work within us_per_tick microseconds per
 * window_ms window (as the lidar). Per tank three sweeps, near to far: its
 * next 30 m of route (corridor of +-2.5 m; unknown cells, or older than
 * route_refresh_ms), its disc of range_m (30 m) for unknown cells (cells
 * ordered by distance, those ahead of a moving tank counted up to 40% nearer),
 * the disc again for stale cells: older than refresh_ms (20 s), fast_refresh_ms
 * (5 s) for a cell that changed state lately, confirm_ms (2 s) for a cell
 * probed only once. All three done: idle 0.5 s. Moving 3 m or turning by a
 * heading bucket restarts the route and unknown sweeps. Tanks are served in
 * batches of GO_BATCH probes, the one with the fewest probes in this window
 * first (stale-sweep probes count double): nobody is starved. A cell probed
 * for one tank serves all.
 *
 * Logs (log_dir): overhead.csv (one row per tank every 2 s), overhead_snap.bin
 * (the cache around a tank every 30 s per tank, at most 400 records; see
 * tools/overhead_report.py), "overhead:" event lines (start, every 60 s, end).
 *
 * Threading: game thread inside the adapter's ground lock, not thread safe.
 * Allocation only in go_init. */
#include <stdint.h>

#define GO_MAX_SEATS 32
#define GO_ROUTE_POINTS 8
#define GO_TILE 32
#define GO_BATCH 8

/* Cell states. */
#define GO_UNKNOWN 0
#define GO_FREE 1
#define GO_OBSTACLE 2
#define GO_OVERHEAD 3

/* Cell flags. */
#define GO_F_CHANGED 0x01      /* flipped between free and obstacle; cleared after 3 agreeing probes */
#define GO_F_INSIDE 0x02       /* the probe started inside a solid */
#define GO_F_BELOW_FREE 0x04   /* overhead: the probe under it reached the ground */
#define GO_F_FLOOR 0x08        /* free on a non-land surface (h < step_m) */
#define GO_F_MISS 0x10         /* the last probe found nothing */

/* go_block values (equal to GM_WORLD_* in ground_motion.h). */
#define GO_BLOCK_UNKNOWN 0
#define GO_BLOCK_FREE 1        /* free, probed free at least twice in a row (and refutes the prior there, see go_block) */
#define GO_BLOCK_OCCUPIED 2
#define GO_BLOCK_FREE_ONCE 3
#define GO_BLOCK_OVERHEAD 4

/* As gl_trace_fn: 1 hit (*t = distance), 0 no hit within *t, -1 the ray raised. */
typedef int (*go_trace_fn)(void *context, const float p[3], const float dir[3], float *t, int *kind, int flags);
typedef void (*go_event_fn)(void *context, const char *message);
typedef double (*go_clock_fn)(void *context);   /* microseconds; tests and model (NULL: gh_qpc) */

typedef struct {
    go_trace_fn trace;
    void *trace_context;
    go_event_fn event;
    void *event_context;
    go_clock_fn clock_us;
    void *clock_context;
    float cell_m;               /* 0.5 */
    float range_m;              /* 30 */
    int us_per_tick;            /* 1000 */
    int window_ms;              /* 16 */
    int refresh_ms;             /* 20000 */
    int fast_refresh_ms;        /* 5000 */
    int route_refresh_ms;       /* 10000 */
    int confirm_ms;             /* 2000 */
    int max_tiles;              /* 1024 */
    float top_m;                /* 8 */
    float step_m;               /* 0.5 */
    float overhang_m;           /* 4 */
    int flags_probe;            /* GH_RAY_GROUND_AND_OBJECTS */
    int flags_land;             /* GH_RAY_GROUND */
    /* WTOBS001 prior (optional): start heights over tall obstacles, snapshots. */
    const uint16_t *prior;
    uint32_t prior_nx, prior_nz;
    float prior_x0, prior_z0, prior_step;
    const char *log_dir;        /* NULL or "": no files */
    const char *file_prefix;
} go_config_t;

typedef struct {
    int seat;                   /* 1 .. GO_MAX_SEATS - 1 */
    uint64_t life;              /* a new life restarts the seat's sweeps */
    float x, y, z, heading, speed;
    int route_count;            /* upcoming route points in travel order */
    float route_x[GO_ROUTE_POINTS], route_z[GO_ROUTE_POINTS];
} go_tank_t;

/* A vehicle hull (any vehicle, wrecks included): position of the track centre
 * as gm_vehicle_t, half extents, centre offset (lp_centre), speed. */
typedef struct {
    float x, z, heading, half_length, half_width, forward, right, speed;
} go_hull_t;

typedef struct {
    uint64_t probes, rays, windows, capped_windows, vehicle_skips, faults, bad_answers, evictions, hot_evictions;
    uint64_t inside_retries, misses, became_obstacle, became_free, lookups, full_pool;
    double ray_us, work_us, log_us, max_window_us;
    int tiles, tiles_peak;
    uint64_t cells_known;
} go_stats_t;

void go_init(const go_config_t *config);
void go_shutdown(uint64_t ms);
int go_active(void);
/* ms: the adapter's clock; now_us: fine clock for the budget window. tanks:
 * the controlled tanks now; hulls: every vehicle the module must not write. */
void go_tick(uint64_t ms, uint64_t now_us, const go_tank_t *tanks, int count, const go_hull_t *hulls, int hull_count);
void go_stats(go_stats_t *out);

/* ---- Read-only API ---- */
/* The cell holding (x, z): GO_UNKNOWN..GO_OVERHEAD, -1 off. Optional outputs:
 * top (m above ground, NAN unknown), count (agreeing probes), age_ms
 * (UINT64_MAX never probed), flags. */
int go_cell(float x, float z, float *top, int *count, uint64_t *age_ms, int *flags);
/* out[row * n + col] = GO_BLOCK_* of the cache cell under the centre of world
 * cell (wc0 + col, wr0 + row) of cell metres (any size). Returns the number
 * of cells not unknown, -1 off. With a prior, GO_BLOCK_FREE (the planner may
 * clear a solid map cell with it) only where the prior cell is solid for its
 * vertical hit alone; free twice elsewhere comes as GO_BLOCK_FREE_ONCE: an edge
 * ray's thin wall is beyond a vertical probe, and a cell the scan calls clear
 * is closed on the map only for water or slope. */
int go_block(float cell, int wc0, int wr0, int n, uint8_t *out);
/* Share of known cells within range of a seat at its last log row (0..1), -1 none. */
double go_coverage(int seat);
#endif
