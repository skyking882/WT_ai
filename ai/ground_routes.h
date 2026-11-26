#ifndef GROUND_ROUTES_H
#define GROUND_ROUTES_H
/* s26 route library (pre-generated offline, stages/s25_routes, layout: the docstring of binary_v4.py and
 * read_check.c there): routes.bin (spawn poses, targets, slots, rank table, routes sampled every 0.5 m with a
 * clearance byte and the curvature per sample) and map.bin (hard cells at 0.25 m, clearance per 0.5 m cell).
 * Plain C and stdio only. gr_load reads both files once into memory; nothing is written. The grid searches below
 * run on the 0.5 m clearance grid of map.bin (cells with clearance >= GR_CLEAR_MIN; cost per metre
 * 1 + clip(1 - (clr - GR_CLEAR_MIN) / 4, 0, 1), the generator's rule), one search at a time (static buffers). */
#include <stddef.h>
#include <stdint.h>

#define GR_CLEAR_MIN 2.3f   /* m: hull half width 1.8 + 0.5 */
#define GR_STEP 0.5f        /* m between curve samples */

typedef struct { char id[9], area[9]; float x, z, heading_deg; } gr_pose_t;
typedef struct {
    char id[17];
    uint8_t kind, army, post_kind, standing; /* kind 0 zone, 1 post; army 0 zone, 1 south, 2 north */
    float x, z, sx, sz, radius, inner_radius, faces_x, faces_z;
} gr_target_t;
typedef struct { uint16_t target, number; float x, z, clearance; } gr_slot_t;
typedef struct {
    char id[33];
    uint8_t kind, rank, flags, fallback; /* kind 0 spawn->zone, 1 spawn->post, 2 zone<->zone, 3 zone<->post */
    uint16_t pose, start_target, end_target;
    uint8_t gear;                        /* start turn gear: 0 none, 1 forward, 2 reverse, 3 blocked */
    float total_s, drive_s, turn_deg, length;
    uint32_t n, first;                   /* samples [first, first + n) of the library's sample arrays */
} gr_route_t;
#define GR_FLAG_TANGENT 1u
#define GR_FLAG_DESIGNATED 2u
#define GR_FLAG_PIVOT_FWD 4u
#define GR_FLAG_PIVOT_REV 8u

/* A route that leads to a target, and which way: reverse 1 = driven from its last sample to its first. */
typedef struct { int route, reverse; } gr_ref_t;

typedef struct gr_lib {
    int loaded;
    uint32_t n_poses, n_targets, n_slots, n_routes, n_samples, n_zones, n_designated, version;
    uint32_t n_poses_noheading;          /* s32: poses whose heading is NaN (unknown spawn heading) */
    gr_pose_t *poses;
    gr_target_t *targets;
    gr_slot_t *slots;
    uint8_t *ranks;
    gr_route_t *routes;
    float *px, *pz, *kappa, *arc;        /* all samples; arc: metres from the route's first sample */
    uint8_t *clr;                        /* 0.1 m steps */
    int *ref_start;                      /* n_targets + 1 offsets into refs */
    gr_ref_t *refs;                      /* per target: the routes that end there (between routes either way) */
    /* map.bin */
    float x0, z0, cell, cell05;
    uint32_t cols, rows, cols05, rows05, nb;
    uint8_t *bits, *clr05;
    size_t bytes;
    double load_ms;
} gr_lib_t;

/* Loads both files (strict magic, version and layout: every byte accounted for). 1 on success; 0 with a reason in
 * msg, the library left empty. */
int gr_load(gr_lib_t *lib, const char *routes_path, const char *map_path, char *msg, size_t msg_size);
void gr_free(gr_lib_t *lib);
/* One line with the counts (poses, targets, zones, slots, routes, designated, samples, MB, load ms). */
void gr_describe(const gr_lib_t *lib, char *out, size_t size);

float gr_clearance(const gr_lib_t *lib, float x, float z); /* m, 0.5 m cell; 0 off the map */
int gr_hard(const gr_lib_t *lib, float x, float z);         /* 0.25 m hard cell; 1 off the map */
/* s29: 1 when a hard cell lies on the straight line between the two points (0.5 m steps, both ends included). */
int gr_line_hard(const gr_lib_t *lib, float x0, float z0, float x1, float z1);
/* Least distance from the hull rectangle (centre, axis, half extents) to a hard cell's centre, searched out to
 * reach m (returns reach when none is nearer). Test / probe use. */
float gr_hull_hard_distance(const gr_lib_t *lib, float cx, float cz, float heading, float hl, float hw, float reach);

/* Library target of a point: the zone whose circle holds it, else the post within post_radius of its point (the
 * log point or the snapped point; nearest wins). army is ignored (the library's post army labels do not match the
 * commander's orders): posts map by geometry alone. -1 none. */
int gr_target_of(const gr_lib_t *lib, float x, float z, int army, float post_radius);
/* The library pose the tank stands on (within dist m, heading within deg), -1 none. */
int gr_pose_of(const gr_lib_t *lib, float x, float z, float heading_rad, float dist, float deg);
/* The designated route (pose, target); without a designated one the best (rank 0, then least total time). -1. */
int gr_designated(const gr_lib_t *lib, int pose, int target);
/* s32: variant 0 = gr_designated; 1 / 2 = the route (pose -> target) with that fallback order (a flank corridor),
 * else the designated one. A pose with a NaN heading matches by position only (gr_pose_of). -1 none. */
int gr_route_variant(const gr_lib_t *lib, int pose, int target, int variant);

/* ---- grid searches (0.5 m clearance grid) ---- */
typedef struct { float cx, cz, fx, fz, hl, hw; } gr_box_t; /* stamped hull rectangle (centre, axis, half extents) */
typedef struct { float x, z; } gr_pt_t;

enum { GR_OK = 0, GR_FAIL = 1, GR_START_BLOCKED = 2, GR_GOAL_BLOCKED = 3, GR_TOO_FAR = 4 };

typedef struct {
    int status, expanded;
    float cost, length;     /* of the cell path */
    double us;              /* wall time of the search (filled by the caller's clock; 0 here) */
} gr_info_t;

/* Effective clearance of a point: the map's 0.5 m cell, and the distance to every stamped box. */
float gr_eff_clearance(const gr_lib_t *lib, float x, float z, const gr_box_t *boxes, int nbox);

/* Bounded Dijkstra from (x, z) out to max_len m of path. relocate: when the start cell has too little clearance,
 * start from the nearest cell with enough within relocate m (0: GR_START_BLOCKED). After it, gr_reach_cost
 * answers the connector cost to a point (INFINITY when not reached) and gr_reach_path the string-pulled,
 * resampled connector from the start to that point. */
int gr_dijkstra(const gr_lib_t *lib, float x, float z, float max_len, float relocate, const gr_box_t *boxes, int nbox,
                gr_info_t *info);
float gr_reach_cost(const gr_lib_t *lib, float x, float z);
/* Connector of the last search from (sx, sz) (the tank's own position) to the reached point (x, z): cell path
 * string-pulled with the clearance check, resampled every 0.5 m, ending exactly at (x, z). Count, 0 on failure. */
int gr_reach_path(const gr_lib_t *lib, float sx, float sz, float x, float z, const gr_box_t *boxes, int nbox,
                  gr_pt_t *out, int cap);
/* A* from (x, z) to (gx, gz); the goal moved to the nearest cell with enough clearance within goal_relocate m;
 * start as gr_dijkstra (relocate). max_straight: GR_TOO_FAR beyond this straight-line distance. On GR_OK the
 * path (start position .. goal) is written as gr_reach_path does; *count its samples; *goal_out the goal used. */
int gr_astar(const gr_lib_t *lib, float x, float z, float gx, float gz, float relocate, float goal_relocate,
             float max_straight, const gr_box_t *boxes, int nbox, gr_pt_t *out, int cap, int *count, gr_pt_t *goal_out,
             gr_info_t *info);

#endif
