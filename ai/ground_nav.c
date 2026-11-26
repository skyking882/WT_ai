/* Static navigation layer: body-valid lattice routes validated by the swept
 * footprint authority. See ground_nav.h for the contract.
 *
 * Structure
 *   grid   : per-cell lower bound of the distance to the nearest BLOCKED or
 *            UNKNOWN cell square (uint8, 1/16 m, capped) + blocked bitmap.
 *            Built time-sliced from map->cell() inside gm_nav_step.
 *   search : one deterministic A* over (lattice node, heading8) at a time,
 *            sparse node pool + serial-stamped hash owned by the instance.
 *   job    : per vehicle route job = legs (via points, then goal set), each
 *            leg first tried as a straight any-angle motion, then by lattice
 *            search; then string pulling; then publication.
 * Every motion that ends up in a published route (pivot at a vertex followed
 * by a straight translation) is validated by: vetoes, stationary bodies, and
 * either the clearance grid (free space proof) or the authority callback. */
#include "ground_nav.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#ifdef GM_NAV_DEBUG
#include <stdio.h>
#endif

#define NAV_PI 3.14159265358979323846f
#define NAV_QUARTER (NAV_PI * 0.25f)
#define NAV_NODES (1 << 19)
#define NAV_HASH (1 << 20)
#define NAV_CACHE (1 << 20)
#define NAV_RAW 16384
#define NAV_SLOTS GM_MAX_VEHICLES
#define NAV_VIA GM_MAX_ROUTE_POINTS
#define NAV_CLASSES 16
#define NAV_VETOES 1024
#define NAV_SEEDS 256
#define NAV_OP 72                /* budget needed for one atomic operation */
#define NAV_SWEEP_UNITS 8
#define NAV_VIA_RADIUS 12.0f
#define NAV_VIA_EXPANSIONS 20000
#define NAV_GOAL_EXPANSIONS 450000
#define NAV_REVERSE_TRAVEL 12.0f
#define NAV_REVERSE_COST 6.0f
#define NAV_ROT_COST 2.0f        /* per 45 degrees */
#define NAV_UNKNOWN_FACTOR 4.0f
#define NAV_BODY_PAD 0.6f        /* default body_clearance */
#define NAV_EXTEND_DISTANCE 150.0f
#define NAV_SHORTCUT_SPAN 96
#define NAV_LOS_GOALS 4
#define NAV_DQ 16.0f             /* distance quantisation: 1/16 m */
#define NAV_DCAP ((uint8_t)(GM_NAV_CLEAR_CAP * NAV_DQ))
#define NAV_OCT_SLACK 1.0824f    /* octile norm <= 1.0824 * euclidean */
#define NAV_WEIGHT 1.5f          /* weighted A*: the 1 m x 8 heading lattice has wide equal-cost plateaus */

/* node flags */
#define NF_CLOSED 1u
#define NF_REV 2u
#define NF_UNK 4u
#define NF_ESC 8u
#define NF_TERM 16u
#define NF_OPEN 32u
#define NF_TIGHT 64u             /* arriving transition valid only at the minimum margin */
#define NF_NOREV 128u            /* seed reached through a pivot here: a REVERSE leg would not be representable */

#define NAV_TIGHT_TRANS 3.0f     /* comfort tier: cost factor of tight translations */
#define NAV_TIGHT_ROT 6.0f       /* comfort tier: cost factor of tight rotations */

#define NAV_BUCKETS 1024         /* Dial queue for the coarse field (> max edge cost: 22 x 3 x 255 / 32) */
#define NAV_CLUTTER_Q 32         /* coarse clutter factor stored as floor(factor * 32), 32..255; 0 = not yet known */
#define NAV_NARROW_FACTOR 3      /* coarse cells without room for the hull width */
#define NAV_FIELD_EXTRA 400      /* 100 m (0.25 m units) settled beyond the leg start */
#define NAV_FIELD_NONE 0xffffu
/* Terrain cost: a cell's factor is stored as (factor - 1) * NAV_COST_Q. */
#define NAV_COST_Q 32.0f
#define NAV_LOS_FLAT 1.25f       /* a straight any-angle leg may cross cells up to this factor; dearer ground is searched */
/* Components of the free cells: 0 = blocked, NAV_COMP_MANY = in a component
 * beyond the label range (counts as reachable from everywhere). */
#define NAV_COMP_MANY 0xffffu
#define NAV_COMP_NEAR 3          /* cells around a blocked start or goal searched for its component */
/* Effort of a goal search: a base plus this many expansions per metre of
 * straight distance. A far goal earns a long search, a near one does not
 * keep everybody else waiting for minutes when it has no way in. */
#define NAV_GOAL_BASE 200000
#define NAV_GOAL_PER_M 1000
/* Freeing the hull at the start: straight forwards or backwards, or a gentle
 * forward curve, in steps of NAV_FREE_STEP up to NAV_FREE_REACH. */
#define NAV_FREE_STEP 0.5f
#define NAV_FREE_REACH 8.0f
#define NAV_FREE_TURN 0.08f      /* rad per step on the curves (a 6 m radius) */
#define NAV_FREE_SKIN 0.05f      /* bare hull growth for "may leave, never enter", as the local layer */
/* Queue order: shortest straight distance first; a waiting request gains
 * this many metres per snapshot so a far one is not starved. */
#define NAV_AGE_M 20.0f

enum { K_TRANS, K_ROT, K_STAT };
enum { J_PRECHECK, J_NEXT_LEG, J_FIELD, J_CONNECT, J_REVERSE, J_SEARCH, J_FINAL, J_SHORTCUT, J_PUBLISH, J_FAIL };
enum { NEED_NONE, NEED_FULL, NEED_EXTEND };

/* tight: the translation arriving here is tight; ptight: the pivot at this vertex is tight */
typedef struct { gm_point_t p; uint8_t rev, tight, ptight; } nav_raw_t;

typedef struct {
    int32_t parent, heap;
    uint16_t kx, kz;
    uint8_t h, flags;
    int16_t aux;
    float g, f, travel;
    uint8_t hin;  /* heading on arrival at this position (before in-place rotations) */
    int8_t rdir;  /* direction of the in-place rotation chain here, 0 = none */
} nav_node_t;

typedef struct { uint32_t serial; int32_t node; } nav_hash_t;
typedef struct { uint64_t key; uint32_t serial; uint8_t value, pad[3]; } nav_cache_t;
typedef struct { int32_t q[7]; float rm; } nav_class_t;
typedef struct { int32_t q[4]; gm_point_t at; float radius; } nav_veto_t;
/* refused motion: rotation arc [s, s + len] (positive direction, dir = turning sign) at a, or translation a -> b */
typedef struct { int32_t q[4]; int rot, dir; gm_point_t a, b; float s, len; } nav_mveto_t;
#define NAV_MVETOES 1024
#define NAV_MVETO_REACH 1.0f
typedef struct { gm_point_t pts[2]; uint8_t rev[2], tight[2], ptight[2], p0tight; int n; } nav_seed_t;
typedef struct { uint16_t label[8]; int n, any; } nav_comps_t; /* components near a point (nav_comps_near) */

typedef struct {
    int used;
    uint64_t id;
    gm_vehicle_t vehicle;
    gm_point_t via[NAV_VIA];
    int via_count;
    gm_point_t goals[GM_NAV_MAX_GOALS];
    float goal_cost[GM_NAV_MAX_GOALS];
    int goal_count;
    float goal_radius, margin, body_pad, comfort, slack;
    int need, published;
    uint32_t queued_tick; /* snapshot at which the present need arose (queue ageing) */
    /* s24 spread: the request's side and price; the polyline stamped on nav->share for that side (0 points: none) */
    int group, stamp_group, stamp_n;
    float share_k;
    gm_point_t stamp_pts[GM_NAV_ROUTE_POINTS + 1];
    /* continuation of a partial route */
    gm_point_t ext_via[NAV_VIA];
    int ext_via_count, ext_via_base;
    float ext_heading;
    gm_nav_route_t route;
} nav_slot_t;

typedef struct {
    int active, slot, stage, extend;
    uint64_t route_key;
    gm_vehicle_t veh, fp, fpc, fpp; /* margin / comfort translation / comfort rotation footprints */
    int cls, tiers;
    int32_t dims[4];
    float rm, margin, body_pad, comfort, slack;
    gm_point_t via[NAV_VIA];
    int via_count, via_base, via_k;
    short via_raw[NAV_VIA];
    gm_point_t goals[GM_NAV_MAX_GOALS];
    float goal_cost[GM_NAV_MAX_GOALS];
    uint8_t goal_ok[GM_NAV_MAX_GOALS];
    int goal_count, goal_index, precheck_i;
    float goal_radius, goal_reff;
    gm_point_t hc;
    float hr, hmin;
    /* current leg start pose */
    gm_point_t start, cur;
    float start_h, cur_h;
    int cur_lat, ckx, ckz, ch;
    int first_leg, escape;
    int free_prefix; /* raw[1..free_prefix] is the validated move that frees the hull at the start */
    int free_tried;
    nav_comps_t start_comps; /* components near the start: a goal or via in none of them has no route */
    int dropped_vias;  /* the goal leg failed after the via legs: searched once more without them */
    int escape0;       /* escape as the precheck found it, for that second try */
    float free_h;      /* heading at the end of the freeing move */
    /* leg / search */
    int target_via;  /* >= 0: via index, -1: goals */
    int los_order[NAV_LOS_GOALS], los_count, los_i, los_done;
    gm_point_t conn_from;
    float conn_h, travel0;
    int conn_rev, conn_i, conn_ring, rev_tried, rev_d, rev_tight;
    nav_seed_t seeds[NAV_SEEDS];
    int seed_count;
    int cap, expansions, end_node;
    /* raw path */
    nav_raw_t raw[NAV_RAW];
    int raw_count, overflow;
    /* shortcut */
    int sc_init, sc_a, sc_j, sc_best, bad_pivots;
    float sc_h;
    uint8_t sc_bpf, sc_blf, start_pflags;
    gm_point_t out_p[NAV_RAW];
    uint8_t out_f[NAV_RAW];
    float out_h[NAV_RAW];
    short out_via[NAV_VIA];
    int out_n;
    /* stats */
    int total_expansions, sweeps;
    int hard;       /* last check was blocked by a body or a veto */
    int last_tight; /* last check passed only at the minimum margin (comfort tier) */
} nav_job_t;

struct gm_nav {
    int has_map, grid_ready, build_phase, build_acc;
    gm_map_t map;
    uint64_t epoch;
    float x0, z0, step, s;
    int cols, rows, kxmax, kzmax;
    size_t cells, cursor;
    uint8_t *dist, *bits;
    uint8_t *cost;       /* terrain cost per cell, NULL without map->cost */
    uint16_t *comp;      /* component label per cell (0 blocked) */
    struct { int32_t *items; size_t head, count, cap; } fill; /* BFS queue of the labelling */
    uint32_t comp_next;  /* next label */
    int comp_ready;
    uint32_t tick;       /* snapshots seen (gm_nav_begin) */
    int stamp_k;
    uint8_t *stamp;
    nav_node_t *nodes;
    int node_count;
    nav_hash_t *hash;
    int32_t *heap;
    int heap_count;
    uint32_t serial;
    nav_cache_t *cache;
    int cache_used;
    nav_class_t classes[NAV_CLASSES];
    int class_count;
    nav_veto_t vetoes[NAV_VETOES];
    nav_mveto_t mvetoes[NAV_MVETOES];
    int mveto_count, mveto_next;
    int veto_count, veto_next;
    gm_nav_body_t bodies[GM_NAV_MAX_BODIES];
    int body_count;
    nav_slot_t *slots;
    int rr;
    nav_job_t *job;
    int *remap;
    /* coarse (2 x 2 cells) obstacle-aware distance field to the leg target,
     * each metre weighed by the least clutter factor of its coarse cells */
    int cw, chh;
    float cs;
    uint8_t *cmax;       /* max clearance lower bound of the sub-cells */
    uint8_t *cclut;      /* least clutter factor over the coarse cell (NAV_CLUTTER_Q), filled on first use */
    struct { const uint8_t *data; int cols, rows; float x0, z0, step, k; } cclut_of; /* table cclut was filled from */
    uint16_t *field;     /* 0.25 m units */
    uint8_t *settled;    /* bitmap */
    struct { int32_t *items; int count, cap; } bucket[NAV_BUCKETS];
    int pending, fcur, fstop, fdone, fstart, facc, fpass, fw1, fw2;
    gm_nav_route_t scratch;
    gm_static_sweep_fn sweep;
    void *ctx;
    int left;
    gm_nav_stats_t stats;
    /* s24 spread: per side, how many of its published routes cover each cell (saturating); the side and price
     * of the search under way (share_group 0: none). Allocated on first use, freed with the grid. */
    uint8_t *share[2];
    int share_group;
    float share_k;
};

static gm_nav_t *nav_shared_instance;

static const int nav_dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int nav_dz[8] = {0, 1, 1, 1, 0, -1, -1, -1};

/* ------------------------------------------------------------------ math */

static float nav_angle(float a) { return remainderf(a, 2.0f * NAV_PI); }
static float nav_hangle(int h) { return nav_angle((float)(h & 7) * NAV_QUARTER); }
static float nav_dist(gm_point_t a, gm_point_t b) { return hypotf(b.x - a.x, b.z - a.z); }
static float nav_dir(gm_point_t a, gm_point_t b) { return atan2f(b.z - a.z, b.x - a.x); }
static float nav_rotcost(float a, float b) { return fabsf(nav_angle(b - a)) / NAV_QUARTER * NAV_ROT_COST; }

static float nav_segdist(gm_point_t p, gm_point_t a, gm_point_t b)
{
    const float ux = b.x - a.x, uz = b.z - a.z, l2 = ux * ux + uz * uz;
    float t = l2 > 1e-12f ? ((p.x - a.x) * ux + (p.z - a.z) * uz) / l2 : 0.0f;
    t = t < 0.0f ? 0.0f : t > 1.0f ? 1.0f : t;
    return hypotf(a.x + ux * t - p.x, a.z + uz * t - p.z);
}

static float nav_octile(gm_point_t a, gm_point_t b)
{
    const float dx = fabsf(b.x - a.x), dz = fabsf(b.z - a.z);
    const float lo = dx < dz ? dx : dz, hi = dx < dz ? dz : dx;
    return hi - lo + 1.41421356f * lo;
}

static gm_pose_t nav_pose(gm_point_t p, float h)
{
    gm_pose_t pose;
    pose.x = p.x;
    pose.z = p.z;
    pose.heading = h;
    pose.speed = 0.0f;
    pose.yaw_rate = 0.0f;
    return pose;
}

static gm_point_t nav_lp(const gm_nav_t *nav, int kx, int kz)
{
    gm_point_t p;
    p.x = nav->x0 + (float)kx * nav->s;
    p.z = nav->z0 + (float)kz * nav->s;
    return p;
}

static int nav_in_lattice(const gm_nav_t *nav, int kx, int kz)
{
    return kx >= 0 && kz >= 0 && kx <= nav->kxmax && kz <= nav->kzmax;
}

static int32_t nav_q(float v) { return (int32_t)floorf(v * 100.0f + 0.5f); }

/* ------------------------------------------------------------------ grid */

static void nav_free_grid(gm_nav_t *nav)
{
    free(nav->dist);
    free(nav->bits);
    free(nav->stamp);
    free(nav->cmax);
    free(nav->cclut);
    free(nav->field);
    free(nav->settled);
    free(nav->cost);
    free(nav->comp);
    free(nav->fill.items);
    free(nav->share[0]);
    free(nav->share[1]);
    nav->share[0] = nav->share[1] = NULL;
    nav->share_group = 0;
    nav->dist = nav->bits = nav->stamp = nav->cmax = nav->cclut = nav->settled = nav->cost = NULL;
    nav->field = NULL;
    nav->comp = NULL;
    memset(&nav->fill, 0, sizeof nav->fill);
    nav->comp_ready = 0;
    nav->comp_next = 0;
    nav->grid_ready = 0;
}

#define NAV_SHARE_CAP 3          /* s24 spread: teammates' routes counted on a cell at most */
#define NAV_SHARE_CELLS 65536    /* cells one stamped route may cover */

/* Terrain cost factor of the cell containing p (1 outside the map or without costs); s24: times the spread
 * price of the search under way (nav->share_group). */
static float nav_cell_cost(const gm_nav_t *nav, gm_point_t p)
{
    if (!nav->cost && !nav->share_group)
        return 1.0f;
    const float fx = (p.x - nav->x0) / nav->step, fz = (p.z - nav->z0) / nav->step;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)nav->cols || fz >= (float)nav->rows)
        return 1.0f;
    const size_t i = (size_t)(int)fz * (size_t)nav->cols + (size_t)(int)fx;
    const float terrain = nav->cost ? 1.0f + (float)nav->cost[i] / NAV_COST_Q : 1.0f;
    if (!nav->share_group)
        return terrain;
    const int k = nav->share[nav->share_group - 1][i];
    return terrain * (1.0f + nav->share_k * (float)(k < NAV_SHARE_CAP ? k : NAV_SHARE_CAP));
}

static int nav_int_less(const void *a, const void *b)
{
    const int x = *(const int *)a, y = *(const int *)b;
    return (x > y) - (x < y);
}

/* s24 spread: add (+1) or take off (-1) one polyline on its side's counts: every cell within one cell of a
 * point sampled each metre along it, each cell once. */
static void nav_share_apply(gm_nav_t *nav, int group, const gm_point_t *pts, int n, int sign)
{
    static int cells[NAV_SHARE_CELLS];
    if (group < 1 || group > 2 || n < 1 || !nav->cells)
        return;
    uint8_t **share = &nav->share[group - 1];
    if (!*share) {
        if (sign < 0)
            return;
        *share = (uint8_t *)calloc(nav->cells, 1);
        if (!*share)
            return;
    }
    int count = 0;
    for (int i = 0; i < n; ++i) {
        const gm_point_t a = pts[i > 0 ? i - 1 : 0], b = pts[i];
        const float len = nav_dist(a, b);
        const int steps = i > 0 ? (int)ceilf(len) : 0;
        for (int s = 0; s <= steps; ++s) {
            const float t = steps ? (float)s / (float)steps : 0.0f;
            const int c = (int)floorf((a.x + (b.x - a.x) * t - nav->x0) / nav->step);
            const int r = (int)floorf((a.z + (b.z - a.z) * t - nav->z0) / nav->step);
            for (int dr = -1; dr <= 1; ++dr)
                for (int dc = -1; dc <= 1; ++dc) {
                    const int cc = c + dc, rr = r + dr;
                    if (cc >= 0 && rr >= 0 && cc < nav->cols && rr < nav->rows && count < NAV_SHARE_CELLS)
                        cells[count++] = rr * nav->cols + cc;
                }
        }
    }
    qsort(cells, (size_t)count, sizeof cells[0], nav_int_less);
    for (int i = 0; i < count; ++i) {
        if (i && cells[i] == cells[i - 1])
            continue;
        uint8_t *v = &(*share)[cells[i]];
        if (sign > 0 && *v < 255)
            ++*v;
        else if (sign < 0 && *v > 0)
            --*v;
    }
}

/* s24 spread: take the slot's stamped route off its side's counts. */
static void nav_share_unstamp(gm_nav_t *nav, nav_slot_t *slot)
{
    if (slot->stamp_n)
        nav_share_apply(nav, slot->stamp_group, slot->stamp_pts, slot->stamp_n, -1);
    slot->stamp_n = 0;
    slot->stamp_group = 0;
}

/* Clutter factor 1 + k * clutter / 255 of the clutter cell containing p (1
 * outside the table or without one). gm_nav_begin checked the table. */
static float nav_clutter(const gm_nav_t *nav, gm_point_t p)
{
    const gm_map_t *m = &nav->map;
    if (!m->clutter)
        return 1.0f;
    const float fx = (p.x - m->clutter_x0) / m->clutter_step, fz = (p.z - m->clutter_z0) / m->clutter_step;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)m->clutter_columns || fz >= (float)m->clutter_rows)
        return 1.0f;
    const uint8_t q = m->clutter[(size_t)(int)fz * (size_t)m->clutter_columns + (size_t)(int)fx];
    return 1.0f + m->clutter_k * (float)q * (1.0f / 255.0f);
}

/* Cost of driving straight from a to b: its length weighed by the cells it
 * crosses (terrain x clutter), sampled every metre. *peak (optional) is the
 * dearest factor met. */
static float nav_path_cost(const gm_nav_t *nav, gm_point_t a, gm_point_t b, float *peak)
{
    const float len = hypotf(b.x - a.x, b.z - a.z);
    if (peak)
        *peak = 1.0f;
    if ((!nav->cost && !nav->map.clutter && !nav->share_group) || len < 1e-4f)
        return len;
    const int n = (int)ceilf(len);
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float t = ((float)i + 0.5f) / (float)n;
        const gm_point_t p = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        const float f = nav_cell_cost(nav, p) * nav_clutter(nav, p);
        sum += f;
        if (peak && f > *peak)
            *peak = f;
    }
    return len * sum / (float)n;
}

static int nav_bit(const gm_nav_t *nav, size_t i) { return (nav->bits[i >> 3] >> (i & 7)) & 1; }

/* Lower bound of the clearance at p (cell based), 0 outside the map. */
static float nav_lb(const gm_nav_t *nav, gm_point_t p)
{
    const float fx = (p.x - nav->x0) / nav->step, fz = (p.z - nav->z0) / nav->step;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)nav->cols || fz >= (float)nav->rows)
        return 0.0f;
    const int i = (int)fx, j = (int)fz;
    int e = i;
    if (j < e) e = j;
    if (nav->cols - 1 - i < e) e = nav->cols - 1 - i;
    if (nav->rows - 1 - j < e) e = nav->rows - 1 - j;
    const float d = (float)nav->dist[(size_t)j * (size_t)nav->cols + (size_t)i] / NAV_DQ;
    const float edge = (float)e * nav->step;
    return d < edge ? d : edge;
}

int gm_nav_grid(const gm_nav_t *nav, float *x0, float *z0, float *step, int *cols, int *rows)
{
    if (!nav || !nav->has_map || !nav->grid_ready)
        return 0;
    *x0 = nav->x0;
    *z0 = nav->z0;
    *step = nav->step;
    *cols = nav->cols;
    *rows = nav->rows;
    return 1;
}

int gm_nav_cell_solid(const gm_nav_t *nav, int col, int row)
{
    if (col < 0 || row < 0 || col >= nav->cols || row >= nav->rows)
        return 1;
    return nav_bit(nav, (size_t)row * (size_t)nav->cols + (size_t)col);
}

float gm_nav_clearance(const gm_nav_t *nav, gm_point_t p)
{
    if (!nav || !nav->has_map || !nav->grid_ready)
        return 0.0f;
    const float fx = (p.x - nav->x0) / nav->step, fz = (p.z - nav->z0) / nav->step;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)nav->cols || fz >= (float)nav->rows)
        return 0.0f;
    const int i = (int)fx, j = (int)fz;
    const float xmax = nav->x0 + (float)nav->cols * nav->step, zmax = nav->z0 + (float)nav->rows * nav->step;
    float best = GM_NAV_CLEAR_CAP;
    const float edges[4] = {p.x - nav->x0, xmax - p.x, p.z - nav->z0, zmax - p.z};
    for (int k = 0; k < 4; ++k)
        if (edges[k] < best) best = edges[k];
    const uint8_t q = nav->dist[(size_t)j * (size_t)nav->cols + (size_t)i];
    if (q >= NAV_DCAP)
        return best < 0.0f ? 0.0f : best;
    const float lb = (float)q / NAV_DQ;
    const int k = (int)ceilf((lb + 1.5f * nav->step) / nav->step) + 1;
    for (int r = j - k; r <= j + k; ++r) {
        if (r < 0 || r >= nav->rows)
            continue;
        for (int c = i - k; c <= i + k; ++c) {
            if (c < 0 || c >= nav->cols || !nav_bit(nav, (size_t)r * (size_t)nav->cols + (size_t)c))
                continue;
            const float cx0 = nav->x0 + (float)c * nav->step, cz0 = nav->z0 + (float)r * nav->step;
            const float dx = p.x < cx0 ? cx0 - p.x : p.x > cx0 + nav->step ? p.x - cx0 - nav->step : 0.0f;
            const float dz = p.z < cz0 ? cz0 - p.z : p.z > cz0 + nav->step ? p.z - cz0 - nav->step : 0.0f;
            const float d = hypotf(dx, dz);
            if (d < best)
                best = d;
        }
    }
    return best < 0.0f ? 0.0f : best;
}

/* How many obstacle cells (blocked or never scanned, as whole squares) the
 * hull at this pose overlaps when grown by `margin` on every side. Outside
 * the map counts as an obstacle. The scan marks a cell when anything in it
 * collides, so the whole square is taken as solid: the game's own sweep only
 * calls a hull over such a cell "unknown", which is no reason to drive there. */
int gm_nav_overlap(const gm_nav_t *nav, const gm_vehicle_t *v, const gm_pose_t *pose, float margin)
{
    if (!nav || !nav->has_map || !nav->grid_ready)
        return 0;
    const float hl = 0.5f * v->length + margin, hw = 0.5f * v->width + margin;
    const float fx = cosf(pose->heading), fz = sinf(pose->heading);
    gm_point_t c;
    c.x = pose->x + fx * v->hull_forward - fz * v->hull_right;
    c.z = pose->z + fz * v->hull_forward + fx * v->hull_right;
    if (nav_lb(nav, c) >= hypotf(hl, hw))
        return 0; /* nothing within reach of any corner */
    const float ax = fabsf(fx), az = fabsf(fz);
    const float ex = hl * ax + hw * az, ez = hl * az + hw * ax, half = 0.5f * nav->step;
    const int c0 = (int)floorf((c.x - ex - nav->x0) / nav->step), c1 = (int)floorf((c.x + ex - nav->x0) / nav->step);
    const int r0 = (int)floorf((c.z - ez - nav->z0) / nav->step), r1 = (int)floorf((c.z + ez - nav->z0) / nav->step);
    int hits = 0;
    for (int r = r0; r <= r1; ++r) {
        for (int col = c0; col <= c1; ++col) {
            if (col < 0 || r < 0 || col >= nav->cols || r >= nav->rows) {
                ++hits;
                continue;
            }
            if (!nav_bit(nav, (size_t)r * (size_t)nav->cols + (size_t)col))
                continue;
            /* The cell square already meets the hull's bounding box; the two
             * hull axes decide whether it meets the hull itself. */
            const float dx = nav->x0 + ((float)col + 0.5f) * nav->step - c.x;
            const float dz = nav->z0 + ((float)r + 0.5f) * nav->step - c.z;
            const float reach = half * (ax + az);
            if (fabsf(dx * fx + dz * fz) < hl + reach && fabsf(dz * fx - dx * fz) < hw + reach)
                ++hits;
        }
    }
    return hits;
}

/* The same along a motion (a turn on the spot or a straight translation):
 * GM_BLOCKED when the grown hull meets an obstacle cell anywhere on it,
 * GM_CLEAR otherwise, GM_UNKNOWN while there is no grid. */
enum gm_geometry gm_nav_solid(const gm_nav_t *nav, const gm_vehicle_t *v, const gm_pose_t *from, const gm_pose_t *to,
                              float margin)
{
    if (!nav || !nav->has_map || !nav->grid_ready)
        return GM_UNKNOWN;
    const float turn = nav_angle(to->heading - from->heading);
    const float dist = hypotf(to->x - from->x, to->z - from->z);
    int steps = (int)ceilf(fmaxf(dist / (0.5f * nav->step), fabsf(turn) / 0.12f));
    if (steps < 1)
        steps = 1;
    for (int k = 0; k <= steps; ++k) {
        const float f = (float)k / (float)steps;
        gm_pose_t pose = *from;
        pose.x += f * (to->x - from->x);
        pose.z += f * (to->z - from->z);
        pose.heading += f * turn;
        if (gm_nav_overlap(nav, v, &pose, margin))
            return GM_BLOCKED;
    }
    return GM_CLEAR;
}

/* The components of the free cells within NAV_COMP_NEAR cells of p: a hull
 * against a wall or a goal in a building has its centre in a blocked cell,
 * and the nearest free cell can be a pocket between two obstacles that is not
 * where it really is, so every component close by counts. 0 labels: none
 * near, or the labels are not built; NAV_COMP_MANY among them counts as any. */
static nav_comps_t nav_comps_near(const gm_nav_t *nav, gm_point_t p)
{
    nav_comps_t s;
    memset(&s, 0, sizeof s);
    if (!nav || !nav->has_map || !nav->grid_ready || !nav->comp_ready)
        return s;
    const int c = (int)floorf((p.x - nav->x0) / nav->step), r = (int)floorf((p.z - nav->z0) / nav->step);
    for (int dr = -NAV_COMP_NEAR; dr <= NAV_COMP_NEAR; ++dr)
        for (int dc = -NAV_COMP_NEAR; dc <= NAV_COMP_NEAR; ++dc) {
            const int cc = c + dc, rr = r + dr;
            if (cc < 0 || rr < 0 || cc >= nav->cols || rr >= nav->rows)
                continue;
            const uint16_t label = nav->comp[(size_t)rr * (size_t)nav->cols + (size_t)cc];
            if (!label)
                continue;
            s.any |= label == NAV_COMP_MANY;
            int k = 0;
            while (k < s.n && s.label[k] != label)
                ++k;
            if (k == s.n) {
                if (s.n == 8) {
                    s.any = 1; /* a maze of pockets: do not decide here */
                    continue;
                }
                s.label[s.n++] = label;
            }
        }
    return s;
}

static int nav_comps_has(const nav_comps_t *s, uint16_t label)
{
    for (int k = 0; k < s->n; ++k)
        if (s->label[k] == label)
            return 1;
    return 0;
}

/* Could a route join the two neighbourhoods? Unknown counts as yes. */
static int nav_comps_meet(const nav_comps_t *a, const nav_comps_t *b)
{
    if (!a->n || !b->n || a->any || b->any)
        return 1;
    for (int k = 0; k < b->n; ++k)
        if (nav_comps_has(a, b->label[k]))
            return 1;
    return 0;
}

int gm_nav_reach(const gm_nav_t *nav, gm_point_t from, gm_point_t goal, float need, gm_point_t *out)
{
    if (out)
        *out = goal;
    const nav_comps_t own = nav_comps_near(nav, from);
    if (!own.n || own.any)
        return 1;
    const nav_comps_t there = nav_comps_near(nav, goal);
    /* No free cell at all near the goal (deep in a lake, inside a block):
     * as unreachable as one in another component. */
    if (there.n && nav_comps_meet(&own, &there))
        return 1;
    if (!out)
        return 0; /* the question was only whether it is reachable */
    /* Rings of cells around the goal, nearest first: the first ring holding
     * a cell of the own component with room for the hull decides. */
    const int c0 = (int)floorf((goal.x - nav->x0) / nav->step), r0 = (int)floorf((goal.z - nav->z0) / nav->step);
    const int rings = (int)ceilf(GM_NAV_SNAP_RANGE / nav->step);
    for (int k = 1; k <= rings; ++k) {
        float best_d = INFINITY;
        gm_point_t best = goal;
        for (int dr = -k; dr <= k; ++dr)
            for (int dc = -k; dc <= k; ++dc) {
                if (abs(dr) != k && abs(dc) != k)
                    continue;
                const int cc = c0 + dc, rr = r0 + dr;
                if (cc < 0 || rr < 0 || cc >= nav->cols || rr >= nav->rows
                    || !nav_comps_has(&own, nav->comp[(size_t)rr * (size_t)nav->cols + (size_t)cc]))
                    continue;
                const gm_point_t q = {nav->x0 + ((float)cc + 0.5f) * nav->step, nav->z0 + ((float)rr + 0.5f) * nav->step};
                const float d = nav_dist(goal, q);
                if (d < best_d && gm_nav_clearance(nav, q) >= need) {
                    best_d = d;
                    best = q;
                }
            }
        if (isfinite(best_d)) {
            if (out)
                *out = best;
            return 2;
        }
    }
    return 0;
}

static int nav_grid_alloc(gm_nav_t *nav)
{
    nav->cells = (size_t)nav->cols * (size_t)nav->rows;
    nav->dist = (uint8_t *)malloc(nav->cells);
    nav->bits = (uint8_t *)calloc((nav->cells + 7) / 8, 1);
    nav->stamp_k = (int)ceilf(GM_NAV_CLEAR_CAP / nav->step) + 1;
    const int w = 2 * nav->stamp_k + 1;
    nav->stamp = (uint8_t *)malloc((size_t)w * (size_t)w);
    nav->cw = (nav->cols + 1) / 2;
    nav->chh = (nav->rows + 1) / 2;
    nav->cs = 2.0f * nav->step;
    const size_t coarse = (size_t)nav->cw * (size_t)nav->chh;
    nav->cmax = (uint8_t *)calloc(coarse, 1);
    nav->cclut = (uint8_t *)calloc(coarse, 1);
    memset(&nav->cclut_of, 0, sizeof nav->cclut_of);
    nav->field = (uint16_t *)malloc(coarse * sizeof(uint16_t));
    nav->settled = (uint8_t *)calloc((coarse + 7) / 8, 1);
    nav->cost = nav->map.cost ? (uint8_t *)calloc(nav->cells, 1) : NULL;
    nav->comp = (uint16_t *)calloc(nav->cells, sizeof(uint16_t));
    if (!nav->dist || !nav->bits || !nav->stamp || !nav->cmax || !nav->cclut || !nav->field || !nav->settled
        || (nav->map.cost && !nav->cost) || !nav->comp) {
        nav_free_grid(nav);
        return 0;
    }
    for (int a = -nav->stamp_k; a <= nav->stamp_k; ++a)
        for (int b = -nav->stamp_k; b <= nav->stamp_k; ++b) {
            const int ia = abs(a) - 1 > 0 ? abs(a) - 1 : 0, ib = abs(b) - 1 > 0 ? abs(b) - 1 : 0;
            const float d = nav->step * hypotf((float)ia, (float)ib) * NAV_DQ;
            nav->stamp[(size_t)(a + nav->stamp_k) * (size_t)w + (size_t)(b + nav->stamp_k)]
                = d >= (float)NAV_DCAP ? NAV_DCAP : (uint8_t)floorf(d);
        }
    nav->build_phase = 0;
    nav->cursor = 0;
    nav->build_acc = 0;
    return 1;
}

static int nav_interior(const gm_nav_t *nav, int c, int r)
{
    if (c <= 0 || r <= 0 || c >= nav->cols - 1 || r >= nav->rows - 1)
        return 0;
    for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
            if (!nav_bit(nav, (size_t)(r + dr) * (size_t)nav->cols + (size_t)(c + dc)))
                return 0;
    return 1;
}

static void nav_build_step(gm_nav_t *nav)
{
    const int w = 2 * nav->stamp_k + 1;
    if (nav->build_phase == 0) {
        while (nav->left > 0 && nav->cursor < nav->cells) {
            size_t end = nav->cursor + 64;
            if (end > nav->cells)
                end = nav->cells;
            for (size_t i = nav->cursor; i < end; ++i) {
                const int c = (int)(i % (size_t)nav->cols), r = (int)(i / (size_t)nav->cols);
                const enum gm_geometry g = nav->map.cell(nav->map.context, c, r);
                if (g != GM_CLEAR) {
                    nav->bits[i >> 3] |= (uint8_t)(1u << (i & 7));
                    nav->dist[i] = 0;
                } else {
                    nav->dist[i] = NAV_DCAP;
                    if (nav->cost) {
                        const float f = nav->map.cost(nav->map.context, c, r);
                        const float q = isfinite(f) && f > 1.0f ? (f - 1.0f) * NAV_COST_Q + 0.5f : 0.0f;
                        nav->cost[i] = (uint8_t)(q > 255.0f ? 255.0f : q);
                    }
                }
            }
            nav->cursor = end;
            nav->left -= 1;
        }
        if (nav->cursor < nav->cells)
            return;
        nav->build_phase = 1;
        nav->cursor = 0;
    }
    while (nav->build_phase == 1 && nav->left >= 8 && nav->cursor < nav->cells) {
        const size_t i = nav->cursor++;
        nav->build_acc += 1;
        if (nav_bit(nav, i)) {
            const int c = (int)(i % (size_t)nav->cols), r = (int)(i / (size_t)nav->cols);
            if (!nav_interior(nav, c, r)) {
                for (int a = -nav->stamp_k; a <= nav->stamp_k; ++a) {
                    const int rr = r + a;
                    if (rr < 0 || rr >= nav->rows)
                        continue;
                    uint8_t *row = nav->dist + (size_t)rr * (size_t)nav->cols;
                    const uint8_t *st = nav->stamp + (size_t)(a + nav->stamp_k) * (size_t)w;
                    for (int b = -nav->stamp_k; b <= nav->stamp_k; ++b) {
                        const int cc = c + b;
                        if (cc >= 0 && cc < nav->cols && st[b + nav->stamp_k] < row[cc])
                            row[cc] = st[b + nav->stamp_k];
                    }
                }
                nav->build_acc += w * w;
            }
        }
        if (nav->build_acc >= 64) {
            nav->left -= nav->build_acc / 64;
            nav->build_acc %= 64;
        }
    }
    if (nav->build_phase == 1) {
        if (nav->cursor < nav->cells)
            return;
        nav->build_phase = 2;
        nav->cursor = 0;
    }
    const size_t coarse = (size_t)nav->cw * (size_t)nav->chh;
    while (nav->build_phase == 2 && nav->left > 0 && nav->cursor < coarse) {
        size_t end = nav->cursor + 64;
        if (end > coarse)
            end = coarse;
        for (size_t i = nav->cursor; i < end; ++i) {
            const int ci = (int)(i % (size_t)nav->cw), cj = (int)(i / (size_t)nav->cw);
            /* 0: all sub-cells blocked, else 1 + max clearance bound of the clear ones */
            int m = 0;
            for (int s = 0; s < 4; ++s) {
                const int c = 2 * ci + (s & 1), r = 2 * cj + (s >> 1);
                if (c < nav->cols && r < nav->rows) {
                    const size_t k = (size_t)r * (size_t)nav->cols + (size_t)c;
                    if (!nav_bit(nav, k) && nav->dist[k] + 1 > m)
                        m = nav->dist[k] + 1;
                }
            }
            nav->cmax[i] = (uint8_t)(m > 255 ? 255 : m);
        }
        nav->cursor = end;
        nav->left -= 1;
    }
    if (nav->build_phase == 2) {
        if (nav->cursor < coarse)
            return;
        nav->build_phase = 3;
        nav->cursor = 0;
        nav->comp_next = 1;
        nav->fill.head = nav->fill.count = 0;
    }
    /* Components of the free cells, breadth first from each unlabelled free
     * cell in turn. A route can only exist inside one component, so a goal
     * in another is known at once instead of after an exhaustive search. */
    while (nav->build_phase == 3 && nav->left > 0) {
        if (nav->fill.head == nav->fill.count) {
            nav->fill.head = nav->fill.count = 0;
            int seeded = 0;
            size_t end = nav->cursor + 256;
            if (end > nav->cells)
                end = nav->cells;
            for (; nav->cursor < end; ++nav->cursor) {
                const size_t i = nav->cursor;
                if (nav_bit(nav, i) || nav->comp[i])
                    continue;
                if (!nav->fill.cap) {
                    nav->fill.cap = 4096;
                    nav->fill.items = (int32_t *)malloc(sizeof(int32_t) * nav->fill.cap);
                    if (!nav->fill.items) {
                        nav->fill.cap = 0;
                        nav->comp_next = 0;
                        nav->build_phase = 4; /* no memory: every goal counts as reachable */
                        break;
                    }
                }
                nav->comp[i] = (uint16_t)(nav->comp_next < NAV_COMP_MANY ? nav->comp_next : NAV_COMP_MANY);
                if (nav->comp_next < NAV_COMP_MANY)
                    ++nav->comp_next;
                nav->fill.items[nav->fill.count++] = (int32_t)i;
                seeded = 1;
                ++nav->cursor;
                break;
            }
            nav->left -= 1;
            if (!seeded && nav->cursor >= nav->cells)
                nav->build_phase = 4;
            continue;
        }
        int n = 64;
        while (n-- > 0 && nav->fill.head < nav->fill.count) {
            const size_t i = (size_t)nav->fill.items[nav->fill.head++];
            const int c = (int)(i % (size_t)nav->cols), r = (int)(i / (size_t)nav->cols);
            const uint16_t label = nav->comp[i];
            for (int d = 0; d < 8; d += 2) {
                const int cc = c + nav_dx[d], rr = r + nav_dz[d];
                if (cc < 0 || rr < 0 || cc >= nav->cols || rr >= nav->rows)
                    continue;
                const size_t j = (size_t)rr * (size_t)nav->cols + (size_t)cc;
                if (nav_bit(nav, j) || nav->comp[j])
                    continue;
                if (nav->fill.count == nav->fill.cap) {
                    /* drop the expanded part of the queue before growing it */
                    if (nav->fill.head > 0) {
                        memmove(nav->fill.items, nav->fill.items + nav->fill.head,
                                sizeof(int32_t) * (nav->fill.count - nav->fill.head));
                        nav->fill.count -= nav->fill.head;
                        nav->fill.head = 0;
                    }
                    if (nav->fill.count == nav->fill.cap) {
                        int32_t *p = (int32_t *)realloc(nav->fill.items, sizeof(int32_t) * nav->fill.cap * 2);
                        if (!p) {
                            nav->comp_next = 0;
                            nav->build_phase = 4;
                            break;
                        }
                        nav->fill.items = p;
                        nav->fill.cap *= 2;
                    }
                }
                nav->comp[j] = label;
                nav->fill.items[nav->fill.count++] = (int32_t)j;
            }
        }
        nav->left -= 1;
    }
    if (nav->build_phase == 4) {
        free(nav->fill.items);
        memset(&nav->fill, 0, sizeof nav->fill);
        /* Labelling finished, or gave up for lack of memory: then the labels
         * are not used (every goal counts as reachable). */
        nav->comp_ready = nav->cursor >= nav->cells && nav->comp_next > 0;
        nav->build_phase = 5;
    }
    if (nav->build_phase == 5) {
        nav->grid_ready = 1;
        for (int k = 0; k < NAV_SLOTS; ++k)
            if (nav->slots[k].used && nav->slots[k].route.status == GM_NAV_BUILDING)
                nav->slots[k].route.status = GM_NAV_PENDING;
    }
}

/* ------------------------------------------------------------- instance */

static void nav_clear_state(gm_nav_t *nav)
{
    for (int g = 0; g < 2; ++g)
        if (nav->share[g])
            memset(nav->share[g], 0, nav->cells); /* s24: the stamps go with the slots */
    nav->share_group = 0;
    memset(nav->slots, 0, sizeof(nav_slot_t) * NAV_SLOTS);
    memset(nav->cache, 0, sizeof(nav_cache_t) * NAV_CACHE);
    nav->cache_used = 0;
    nav->class_count = 0;
    nav->veto_count = nav->veto_next = 0;
    nav->mveto_count = nav->mveto_next = 0;
    nav->body_count = 0;
    nav->job->active = 0;
    nav->rr = 0;
}

gm_nav_t *gm_nav_shared(void)
{
    if (nav_shared_instance)
        return nav_shared_instance;
    gm_nav_t *nav = (gm_nav_t *)calloc(1, sizeof(gm_nav_t));
    if (!nav)
        return NULL;
    nav->nodes = (nav_node_t *)malloc(sizeof(nav_node_t) * NAV_NODES);
    nav->hash = (nav_hash_t *)calloc(NAV_HASH, sizeof(nav_hash_t));
    nav->heap = (int32_t *)malloc(sizeof(int32_t) * NAV_NODES);
    nav->cache = (nav_cache_t *)calloc(NAV_CACHE, sizeof(nav_cache_t));
    nav->slots = (nav_slot_t *)calloc(NAV_SLOTS, sizeof(nav_slot_t));
    nav->job = (nav_job_t *)calloc(1, sizeof(nav_job_t));
    nav->remap = (int *)malloc(sizeof(int) * NAV_RAW);
    if (!nav->nodes || !nav->hash || !nav->heap || !nav->cache || !nav->slots || !nav->job || !nav->remap) {
        free(nav->nodes);
        free(nav->hash);
        free(nav->heap);
        free(nav->cache);
        free(nav->slots);
        free(nav->job);
        free(nav->remap);
        free(nav);
        return NULL;
    }
    nav->serial = 1;
    nav_shared_instance = nav;
    return nav;
}

void gm_nav_reset(gm_nav_t *nav)
{
    if (nav) {
        nav_clear_state(nav);
        memset(&nav->stats, 0, sizeof nav->stats);
    }
}

enum gm_nav_status gm_nav_begin(gm_nav_t *nav, const gm_map_t *map, uint64_t map_epoch, uint64_t ms)
{
    (void)ms;
    if (!nav)
        return GM_NAV_NO_MAP;
    ++nav->tick;
    if (!map || !map->cell || map->columns <= 0 || map->rows <= 0 || !(map->step > 0.0f)
        || map->columns > 30000 || map->rows > 30000) {
        nav->has_map = 0;
        return GM_NAV_NO_MAP;
    }
    const int same = nav->dist && nav->x0 == map->x0 && nav->z0 == map->z0 && nav->step == map->step
                     && nav->cols == map->columns && nav->rows == map->rows && nav->epoch == map_epoch;
    nav->map = *map;
    nav->has_map = 1;
    /* A clutter table that does not describe a grid, or no weight: none. */
    if (!map->clutter || !(map->clutter_k > 0.0f) || !isfinite(map->clutter_k) || !(map->clutter_step > 0.0f)
        || !isfinite(map->clutter_x0) || !isfinite(map->clutter_z0) || map->clutter_columns <= 0
        || map->clutter_rows <= 0 || map->clutter_columns > 30000 || map->clutter_rows > 30000)
        nav->map.clutter = NULL;
    if (!same) {
        nav_free_grid(nav);
        nav->x0 = map->x0;
        nav->z0 = map->z0;
        nav->step = map->step;
        nav->cols = map->columns;
        nav->rows = map->rows;
        nav->epoch = map_epoch;
        nav->s = map->step * 0.5f;
        nav->kxmax = 2 * map->columns;
        nav->kzmax = 2 * map->rows;
        nav_clear_state(nav);
        if (nav->kxmax > 65535 || nav->kzmax > 65535 || !nav_grid_alloc(nav)) {
            nav->has_map = 0;
            return GM_NAV_NO_MAP;
        }
    }
    const gm_map_t *m = &nav->map;
    if (m->clutter && (nav->cclut_of.data != m->clutter || nav->cclut_of.cols != m->clutter_columns
                       || nav->cclut_of.rows != m->clutter_rows || nav->cclut_of.x0 != m->clutter_x0
                       || nav->cclut_of.z0 != m->clutter_z0 || nav->cclut_of.step != m->clutter_step
                       || nav->cclut_of.k != m->clutter_k)) {
        memset(nav->cclut, 0, (size_t)nav->cw * (size_t)nav->chh);
        nav->cclut_of.data = m->clutter;
        nav->cclut_of.cols = m->clutter_columns;
        nav->cclut_of.rows = m->clutter_rows;
        nav->cclut_of.x0 = m->clutter_x0;
        nav->cclut_of.z0 = m->clutter_z0;
        nav->cclut_of.step = m->clutter_step;
        nav->cclut_of.k = m->clutter_k;
    }
    return nav->grid_ready ? GM_NAV_READY : GM_NAV_BUILDING;
}

void gm_nav_set_bodies(gm_nav_t *nav, const gm_nav_body_t *bodies, int count)
{
    if (!nav || !nav->has_map)
        return;
    if (count < 0 || !bodies)
        count = 0;
    if (count > GM_NAV_MAX_BODIES)
        count = GM_NAV_MAX_BODIES;
    if (count)
        memcpy(nav->bodies, bodies, sizeof(gm_nav_body_t) * (size_t)count);
    nav->body_count = count;
}

static void nav_dims(const gm_vehicle_t *v, int32_t q[4])
{
    q[0] = nav_q(v->length);
    q[1] = nav_q(v->width);
    q[2] = nav_q(v->hull_forward);
    q[3] = nav_q(v->hull_right);
}

static float nav_circum(const gm_vehicle_t *v, float margin)
{
    return hypotf(v->length * 0.5f + fabsf(v->hull_forward) + margin, v->width * 0.5f + fabsf(v->hull_right) + margin);
}

static int nav_class(gm_nav_t *nav, const gm_vehicle_t *v, float margin, float comfort, float slack)
{
    int32_t q[7];
    nav_dims(v, q);
    q[4] = nav_q(margin);
    q[5] = nav_q(comfort);
    q[6] = nav_q(slack);
    if (comfort + slack > margin)
        margin = comfort + slack; /* largest footprint, for cache invalidation radius */
    for (int k = 0; k < nav->class_count; ++k)
        if (!memcmp(nav->classes[k].q, q, sizeof q))
            return k;
    if (nav->class_count >= NAV_CLASSES) {
        memset(nav->cache, 0, sizeof(nav_cache_t) * NAV_CACHE);
        nav->cache_used = 0;
        nav->class_count = 0;
    }
    memcpy(nav->classes[nav->class_count].q, q, sizeof q);
    nav->classes[nav->class_count].rm = nav_circum(v, margin);
    return nav->class_count++;
}

/* drop cached authority answers of this hull near a disc */
static void nav_cache_drop(gm_nav_t *nav, const int32_t q[4], gm_point_t at, float radius)
{
    for (int i = 0; i < NAV_CACHE; ++i) {
        nav_cache_t *e = &nav->cache[i];
        if (!e->key || !e->value)
            continue;
        const int cls = (int)((e->key >> 40) & 0xff);
        if (cls >= nav->class_count || memcmp(nav->classes[cls].q, q, sizeof(int32_t) * 4))
            continue;
        const gm_point_t p = nav_lp(nav, (int)(e->key & 0xffff), (int)((e->key >> 16) & 0xffff));
        if (nav_dist(p, at) <= radius + nav->classes[cls].rm + 2.0f * nav->s + 1.0f)
            e->value = 0;
    }
}

void gm_nav_report_blocked(gm_nav_t *nav, const gm_vehicle_t *vehicle, gm_point_t at, float radius)
{
    if (!nav || !nav->has_map || !vehicle || !(radius > 0.0f) || !isfinite(at.x) || !isfinite(at.z))
        return;
    nav_veto_t *veto = &nav->vetoes[nav->veto_next];
    nav_dims(vehicle, veto->q);
    veto->at = at;
    veto->radius = radius;
    nav->veto_next = (nav->veto_next + 1) % NAV_VETOES;
    if (nav->veto_count < NAV_VETOES)
        ++nav->veto_count;
    nav_cache_drop(nav, veto->q, at, radius);
}

void gm_nav_report_blocked_motion(gm_nav_t *nav, const gm_vehicle_t *vehicle, const gm_pose_t *from, const gm_pose_t *to)
{
    if (!nav || !nav->has_map || !vehicle || !from || !to || !isfinite(from->x) || !isfinite(from->z)
        || !isfinite(to->x) || !isfinite(to->z) || !isfinite(from->heading) || !isfinite(to->heading))
        return;
    nav_mveto_t v;
    memset(&v, 0, sizeof v);
    nav_dims(vehicle, v.q);
    v.a.x = from->x;
    v.a.z = from->z;
    v.b.x = to->x;
    v.b.z = to->z;
    const float len = nav_dist(v.a, v.b);
    if (len < 1e-3f) {
        const float turn = nav_angle(to->heading - from->heading);
        if (fabsf(turn) < 1e-4f)
            return; /* zero motion */
        v.rot = 1;
        v.dir = turn > 0.0f ? 1 : -1;
        v.s = turn > 0.0f ? from->heading : from->heading + turn;
        v.len = fabsf(turn);
    }
    /* bounded ring: beyond NAV_MVETOES the oldest report is evicted */
    nav->mvetoes[nav->mveto_next] = v;
    nav->mveto_next = (nav->mveto_next + 1) % NAV_MVETOES;
    if (nav->mveto_count < NAV_MVETOES)
        ++nav->mveto_count;
    const gm_point_t mid = {0.5f * (v.a.x + v.b.x), 0.5f * (v.a.z + v.b.z)};
    nav_cache_drop(nav, v.q, mid, 0.5f * len + NAV_MVETO_REACH);
}

/* ----------------------------------------------------------- validation */

static int nav_vetoed(const gm_nav_t *nav, const nav_job_t *job, const gm_pose_t *from, const gm_pose_t *to)
{
    const gm_point_t a = {from->x, from->z}, b = {to->x, to->z};
    for (int k = 0; k < nav->veto_count; ++k) {
        const nav_veto_t *v = &nav->vetoes[k];
        if (memcmp(v->q, job->dims, sizeof v->q))
            continue;
        const float d0 = nav_dist(a, v->at), dmin = nav_segdist(v->at, a, b);
        if (dmin < v->radius && dmin < d0 - 1e-3f)
            return 1;
    }
    if (!nav->mveto_count)
        return 0;
    const float len = nav_dist(a, b);
    const float turn = nav_angle(to->heading - from->heading);
    const int rot = len < 1e-4f;
    if (rot && fabsf(turn) < 1e-5f)
        return 0; /* stationary */
    for (int k = 0; k < nav->mveto_count; ++k) {
        const nav_mveto_t *v = &nav->mvetoes[k];
        if (v->rot != rot || memcmp(v->q, job->dims, sizeof v->q))
            continue;
        if (rot) {
            if ((turn > 0.0f ? 1 : -1) != v->dir || nav_dist(a, v->a) > NAV_MVETO_REACH)
                continue;
            const float s = turn > 0.0f ? from->heading : from->heading + turn, l = fabsf(turn);
            const float two_pi = 2.0f * NAV_PI;
            const float d1 = fmodf(fmodf(s - v->s, two_pi) + two_pi, two_pi);
            const float d2 = fmodf(fmodf(v->s - s, two_pi) + two_pi, two_pi);
            if (d1 < v->len - 1e-3f || d2 < l - 1e-3f)
                return 1;
            continue;
        }
        const float vl = nav_dist(v->a, v->b);
        if (vl < 1e-4f)
            continue;
        const float ux = (v->b.x - v->a.x) / vl, uz = (v->b.z - v->a.z) / vl;
        const float mx = (b.x - a.x) / len, mz = (b.z - a.z) / len;
        if (mx * ux + mz * uz < 0.7071f)
            continue; /* not the same direction of travel */
        const float t0 = (a.x - v->a.x) * ux + (a.z - v->a.z) * uz, t1 = (b.x - v->a.x) * ux + (b.z - v->a.z) * uz;
        const float lo = t0 > 0.0f ? t0 : 0.0f, hi = t1 < vl ? t1 : vl;
        if (hi - lo < 0.05f)
            continue;
        float lat[2];
        for (int e = 0; e < 2; ++e) {
            const float tt = e ? hi : lo, f = (tt - t0) / (t1 - t0);
            const float px = a.x + (b.x - a.x) * f - v->a.x, pz = a.z + (b.z - a.z) * f - v->a.z;
            lat[e] = ux * pz - uz * px;
        }
        if (fabsf(lat[0]) <= NAV_MVETO_REACH || fabsf(lat[1]) <= NAV_MVETO_REACH || (lat[0] > 0.0f) != (lat[1] > 0.0f))
            return 1;
    }
    return 0;
}

typedef struct { float x, z, c, s, hl, hw; } nav_rect_t;

static int nav_rect_overlap(const nav_rect_t *a, const nav_rect_t *b)
{
    const float dx = b->x - a->x, dz = b->z - a->z;
    const float ax[4][2] = {{a->c, a->s}, {-a->s, a->c}, {b->c, b->s}, {-b->s, b->c}};
    for (int i = 0; i < 4; ++i) {
        const float ux = ax[i][0], uz = ax[i][1];
        const float ra = a->hl * fabsf(a->c * ux + a->s * uz) + a->hw * fabsf(-a->s * ux + a->c * uz);
        const float rb = b->hl * fabsf(b->c * ux + b->s * uz) + b->hw * fabsf(-b->s * ux + b->c * uz);
        if (fabsf(dx * ux + dz * uz) >= ra + rb)
            return 0;
    }
    return 1;
}

static gm_point_t nav_hull_centre(float x, float z, float h, float fwd, float right)
{
    const float c = cosf(h), s = sinf(h);
    gm_point_t p;
    p.x = x + c * fwd - s * right;
    p.z = z + s * fwd + c * right;
    return p;
}

static void nav_rect_corners(const nav_rect_t *r, gm_point_t p[4])
{
    for (int k = 0; k < 4; ++k) {
        const float fx = (k == 1 || k == 2) ? r->hl : -r->hl, fz = (k >= 2) ? r->hw : -r->hw;
        p[k].x = r->x + r->c * fx - r->s * fz;
        p[k].z = r->z + r->s * fx + r->c * fz;
    }
}

/* Euclidean distance between two oriented rectangles, 0 when they overlap. */
static float nav_rect_distance(const nav_rect_t *a, const nav_rect_t *b)
{
    if (nav_rect_overlap(a, b))
        return 0.0f;
    gm_point_t pa[4], pb[4];
    nav_rect_corners(a, pa);
    nav_rect_corners(b, pb);
    float best = INFINITY;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            const float d1 = nav_segdist(pa[i], pb[j], pb[(j + 1) & 3]);
            const float d2 = nav_segdist(pb[i], pa[j], pa[(j + 1) & 3]);
            if (d1 < best) best = d1;
            if (d2 < best) best = d2;
        }
    return best;
}

static nav_rect_t nav_vehicle_rect(const gm_vehicle_t *v, float x, float z, float h, float pad)
{
    const gm_point_t hc = nav_hull_centre(x, z, h, v->hull_forward, v->hull_right);
    nav_rect_t r = {hc.x, hc.z, cosf(h), sinf(h), v->length * 0.5f + pad, v->width * 0.5f + pad};
    return r;
}

static nav_rect_t nav_body_rect(const gm_nav_body_t *b, float pad)
{
    const gm_point_t bc = nav_hull_centre(b->pose.x, b->pose.z, b->pose.heading, b->hull_forward, b->hull_right);
    nav_rect_t r = {bc.x, bc.z, cosf(b->pose.heading), sinf(b->pose.heading), b->half_length + pad, b->half_width + pad};
    return r;
}

/* 1 when the vehicle's REAL hull at pose p overlaps any body's REAL hull. */
static int nav_body_overlap_real(const gm_nav_t *nav, const nav_job_t *job, const gm_pose_t *p)
{
    const nav_rect_t ra = nav_vehicle_rect(&job->veh, p->x, p->z, p->heading, 0.0f);
    for (int k = 0; k < nav->body_count; ++k) {
        if (nav->bodies[k].id == job->veh.id)
            continue;
        const nav_rect_t rb = nav_body_rect(&nav->bodies[k], 0.0f);
        if (nav_rect_overlap(&ra, &rb))
            return 1;
    }
    return 0;
}

/* Stationary bodies. Normal rule: the swept hull (+margin) must not overlap a
 * body inflated by the request's body_clearance. Escape rule, for every body whose inflated
 * zone the FROM pose already violates: the real hull-to-hull distance must stay
 * positive and never decrease along the motion (flags |= TIGHT). The rule is
 * stateless, so it applies exactly while a route is still inside an inflated
 * zone (start escape prefix) and nowhere else. */
static int nav_body_hit(const gm_nav_t *nav, const nav_job_t *job, const gm_pose_t *from, const gm_pose_t *to,
                        uint8_t *flags)
{
    if (!nav->body_count)
        return 0;
    const gm_vehicle_t *v = &job->veh;
    const float hl = v->length * 0.5f + job->margin, hw = v->width * 0.5f + job->margin;
    const float mr = hypotf(hl, hw);
    const gm_point_t c0 = nav_hull_centre(from->x, from->z, from->heading, v->hull_forward, v->hull_right);
    const gm_point_t c1 = nav_hull_centre(to->x, to->z, to->heading, v->hull_forward, v->hull_right);
    const float turn = nav_angle(to->heading - from->heading);
    const float len = hypotf(to->x - from->x, to->z - from->z);
    for (int k = 0; k < nav->body_count; ++k) {
        const gm_nav_body_t *b = &nav->bodies[k];
        if (b->id == v->id)
            continue;
        nav_rect_t rb;
        const gm_point_t bc = nav_hull_centre(b->pose.x, b->pose.z, b->pose.heading, b->hull_forward, b->hull_right);
        rb.x = bc.x;
        rb.z = bc.z;
        rb.c = cosf(b->pose.heading);
        rb.s = sinf(b->pose.heading);
        rb.hl = b->half_length + job->body_pad;
        rb.hw = b->half_width + job->body_pad;
        const float br = hypotf(rb.hl, rb.hw);
        /* hull centre path is within the rotation-centre segment inflated by the offset */
        const float off = hypotf(v->hull_forward, v->hull_right);
        const gm_point_t a = {from->x, from->z}, e = {to->x, to->z};
        if (nav_segdist(bc, a, e) > br + mr + off + 0.01f && nav_segdist(bc, c0, c1) > br + mr + 0.01f)
            continue;
        /* dense enough that no hull point moves more than ~0.05 m between samples */
        int n = (int)ceilf(len / 0.05f);
        const int nr = (int)ceilf(fabsf(turn) * mr / 0.05f);
        if (nr > n) n = nr;
        if (n < 1) n = 1;
        const nav_rect_t ra0 = nav_vehicle_rect(v, from->x, from->z, from->heading, job->margin);
        if (nav_rect_overlap(&ra0, &rb)) {
            /* escape rule against the real hulls */
            const nav_rect_t real = nav_body_rect(b, 0.0f);
            const nav_rect_t va = nav_vehicle_rect(v, from->x, from->z, from->heading, 0.0f);
            float prev = nav_rect_distance(&va, &real);
            if (prev <= 0.0f)
                return 1;
            for (int i = 1; i <= n; ++i) {
                const float t = (float)i / (float)n;
                const nav_rect_t vi = nav_vehicle_rect(v, from->x + (to->x - from->x) * t, from->z + (to->z - from->z) * t,
                                                       from->heading + turn * t, 0.0f);
                const float d = nav_rect_distance(&vi, &real);
                if (d <= 0.0f || d < prev - 1e-4f)
                    return 1;
                prev = d;
            }
            if (flags)
                *flags |= GM_NAV_TIGHT;
            continue;
        }
        for (int i = 0; i <= n; ++i) {
            const float t = (float)i / (float)n;
            const float h = from->heading + turn * t;
            const gm_point_t hc = nav_hull_centre(from->x + (to->x - from->x) * t, from->z + (to->z - from->z) * t,
                                                  h, v->hull_forward, v->hull_right);
            nav_rect_t ra = {hc.x, hc.z, cosf(h), sinf(h), hl, hw};
            if (nav_rect_overlap(&ra, &rb))
                return 1;
        }
    }
    return 0;
}

/* Free-space proof from the clearance grid. */
static int nav_free(const gm_nav_t *nav, const nav_job_t *job, const gm_pose_t *from, const gm_pose_t *to)
{
    const gm_point_t a = {from->x, from->z}, b = {to->x, to->z};
    const float len = nav_dist(a, b);
    if (len < 1e-4f)
        return nav_lb(nav, a) >= job->rm;
    const int n = (int)ceilf(len);
    const float need = job->rm + 0.5f * len / (float)n;
    for (int i = 0; i <= n; ++i) {
        const float t = (float)i / (float)n;
        const gm_point_t p = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        if (nav_lb(nav, p) < need)
            return 0;
    }
    return 1;
}

static enum gm_geometry nav_authority(gm_nav_t *nav, nav_job_t *job, const gm_vehicle_t *fp, const gm_pose_t *from,
                                      const gm_pose_t *to)
{
    /* The obstacle cells decide first, exactly and without a callback. A
     * caller that passes no sweep routes on them alone: live evidence then
     * reaches the route layer as reported vetoes. */
    nav->left -= 1;
    if (gm_nav_solid(nav, fp, from, to, 0.0f) == GM_BLOCKED)
        return GM_BLOCKED;
    if (!nav->sweep)
        return GM_CLEAR;
    nav->left -= NAV_SWEEP_UNITS;
    ++job->sweeps;
    const enum gm_geometry g = nav->sweep(nav->ctx, fp, from, to);
    return g == GM_CLEAR || g == GM_BLOCKED ? g : GM_UNKNOWN;
}

static int nav_is_rotation(const gm_pose_t *from, const gm_pose_t *to)
{
    return hypotf(to->x - from->x, to->z - from->z) < 1e-4f && fabsf(nav_angle(to->heading - from->heading)) > 1e-5f;
}

/* Any motion (pure pivot or pure translation). Bodies and vetoes first (both
 * tiers), then the comfort tier (free-space proof or authority at comfort
 * inflation), then the minimum margin (tight: job->last_tight, flags TIGHT).
 * Without the tier: TIGHT = the authority was needed. */
static enum gm_geometry nav_motion(gm_nav_t *nav, nav_job_t *job, const gm_pose_t *from, const gm_pose_t *to,
                                   uint8_t *flags)
{
    job->hard = 0;
    job->last_tight = 0;
    if (nav_vetoed(nav, job, from, to) || nav_body_hit(nav, job, from, to, flags)) {
        job->hard = 1; /* bodies and vetoes are never relaxed by the static escape */
        return GM_BLOCKED;
    }
    if (nav_free(nav, job, from, to))
        return GM_CLEAR;
    enum gm_geometry g;
    if (!job->tiers) {
        g = nav_authority(nav, job, &job->fp, from, to);
        if (flags) {
            *flags |= GM_NAV_TIGHT;
            if (g == GM_UNKNOWN)
                *flags |= GM_NAV_UNKNOWN;
        }
        return g;
    }
    g = nav_authority(nav, job, nav_is_rotation(from, to) ? &job->fpp : &job->fpc, from, to);
    if (g != GM_BLOCKED) {
        if (flags && g == GM_UNKNOWN)
            *flags |= GM_NAV_UNKNOWN;
        return g;
    }
    g = nav_authority(nav, job, &job->fp, from, to);
    if (g == GM_BLOCKED)
        return g;
    job->last_tight = 1;
    if (flags) {
        *flags |= GM_NAV_TIGHT;
        if (g == GM_UNKNOWN)
            *flags |= GM_NAV_UNKNOWN;
    }
    return g;
}
static enum gm_geometry nav_pivot(gm_nav_t *nav, nav_job_t *job, gm_point_t p, float a, float b, uint8_t *flags)
{
    job->hard = 0;
    job->last_tight = 0;
    if (fabsf(nav_angle(b - a)) < 1e-5f)
        return GM_CLEAR;
    const gm_pose_t from = nav_pose(p, a), to = nav_pose(p, b);
    return nav_motion(nav, job, &from, &to, flags);
}

static enum gm_geometry nav_trans(gm_nav_t *nav, nav_job_t *job, gm_point_t p, gm_point_t q, float h, uint8_t *flags)
{
    job->hard = 0;
    job->last_tight = 0;
    if (nav_dist(p, q) < 1e-4f)
        return GM_CLEAR;
    const gm_pose_t from = nav_pose(p, h), to = nav_pose(q, h);
    return nav_motion(nav, job, &from, &to, flags);
}

static enum gm_geometry nav_worst(enum gm_geometry a, enum gm_geometry b)
{
    if (a == GM_BLOCKED || b == GM_BLOCKED)
        return GM_BLOCKED;
    return a == GM_UNKNOWN || b == GM_UNKNOWN ? GM_UNKNOWN : GM_CLEAR;
}

/* pivot at p from heading h to the direction of q, then translate to q */
static enum gm_geometry nav_leg(gm_nav_t *nav, nav_job_t *job, gm_point_t p, float h, gm_point_t q, uint8_t *flags)
{
    if (nav_dist(p, q) < 1e-4f)
        return GM_CLEAR;
    const float d = nav_dir(p, q);
    const enum gm_geometry a = nav_pivot(nav, job, p, h, d, flags);
    if (a == GM_BLOCKED)
        return a;
    return nav_worst(a, nav_trans(nav, job, p, q, d, flags));
}

/* Straight any-angle leg (pivot + translation). With the comfort tier only a
 * fully comfortable leg is accepted: a tight straight leg would hide a short
 * tight spot inside a long leg, so the lattice search isolates it instead. */
static int nav_los(gm_nav_t *nav, nav_job_t *job, gm_point_t p, float h, gm_point_t q)
{
    uint8_t fl = 0;
    /* A straight leg over steep, wet or built-up (clutter) ground is not
     * taken on sight: the search decides whether going round is cheaper. */
    float peak;
    nav_path_cost(nav, p, q, &peak);
    if (peak > NAV_LOS_FLAT)
        return 0;
    if (nav_leg(nav, job, p, h, q, &fl) == GM_BLOCKED)
        return 0;
    return !job->tiers || !(fl & GM_NAV_TIGHT);
}

/* ----------------------------------------------------------------- cache */

static uint64_t nav_cache_key(int cls, int kind, int kx, int kz, int h)
{
    return (1ull << 63) | ((uint64_t)(cls & 0xff) << 40) | ((uint64_t)kind << 36) | ((uint64_t)(h & 7) << 32)
           | ((uint64_t)kz << 16) | (uint64_t)kx;
}

static nav_cache_t *nav_cache_find(gm_nav_t *nav, uint64_t key, int create)
{
    uint64_t x = key * 0x9E3779B97F4A7C15ull;
    size_t i = (size_t)(x >> 44) & (NAV_CACHE - 1);
    for (;;) {
        nav_cache_t *e = &nav->cache[i];
        if (e->key == key)
            return e;
        if (!e->key) {
            if (!create)
                return NULL;
            if (nav->cache_used >= NAV_CACHE / 10 * 7) {
                memset(nav->cache, 0, sizeof(nav_cache_t) * NAV_CACHE);
                nav->cache_used = 0;
                return nav_cache_find(nav, key, 1);
            }
            ++nav->cache_used;
            e->key = key;
            e->value = 0;
            return e;
        }
        i = (i + 1) & (NAV_CACHE - 1);
    }
}

/* cached authority answer for a lattice transition at one tier (1 comfort, 0 minimum) */
static enum gm_geometry nav_lattice_auth(gm_nav_t *nav, nav_job_t *job, int kind, int kx, int kz, int h, int tier,
                                         const gm_vehicle_t *fp, const gm_pose_t *from, const gm_pose_t *to)
{
    const uint64_t key = nav_cache_key(job->cls, kind, kx, kz, h) | ((uint64_t)tier << 39);
    nav_cache_t *e = nav_cache_find(nav, key, 1);
    if (e->value && ((e->value & 3) != (uint8_t)(GM_UNKNOWN + 1) || e->serial == nav->serial))
        return (enum gm_geometry)((e->value & 3) - 1);
    const enum gm_geometry g = nav_authority(nav, job, fp, from, to);
    /* the cache may have been cleared by a nested insert: look it up again */
    e = nav_cache_find(nav, key, 1);
    e->value = (uint8_t)(g + 1);
    e->serial = nav->serial;
    return g;
}

/* Lattice transition check: K_TRANS (one step along h from (kx,kz)),
 * K_ROT (rotation h <-> h+1 at (kx,kz)), K_STAT (stationary footprint).
 * reverse: the translation is driven from the far end back to (kx,kz). */
static enum gm_geometry nav_lattice(gm_nav_t *nav, nav_job_t *job, int kind, int kx, int kz, int h, int reverse,
                                    uint8_t *flags)
{
    h &= 7;
    job->last_tight = 0;
    const gm_point_t a = nav_lp(nav, kx, kz);
    gm_pose_t from, to;
    if (kind == K_TRANS) {
        if (!nav_in_lattice(nav, kx + nav_dx[h], kz + nav_dz[h]))
            return GM_BLOCKED;
        const gm_point_t b = nav_lp(nav, kx + nav_dx[h], kz + nav_dz[h]);
        from = nav_pose(reverse ? b : a, nav_hangle(h));
        to = nav_pose(reverse ? a : b, nav_hangle(h));
    } else {
        /* K_ROT reverse: the actual rotation is h+1 -> h (direction matters for the body escape rule) */
        const int rev_rot = kind == K_ROT && reverse;
        from = nav_pose(a, rev_rot ? nav_hangle(h + 1) : nav_hangle(h));
        to = nav_pose(a, kind == K_ROT ? (rev_rot ? nav_hangle(h) : nav_hangle(h + 1)) : nav_hangle(h));
    }
    job->hard = 0;
    if (nav_vetoed(nav, job, &from, &to) || nav_body_hit(nav, job, &from, &to, flags)) {
        job->hard = 1;
        return GM_BLOCKED;
    }
    if (nav_free(nav, job, &from, &to))
        return GM_CLEAR;
    if (reverse && kind != K_STAT) {
        /* canonical direction for the authority (same swept set) */
        const gm_pose_t t = from;
        from = to;
        to = t;
    }
    enum gm_geometry g;
    if (!job->tiers) {
        g = nav_lattice_auth(nav, job, kind, kx, kz, h, 0, &job->fp, &from, &to);
        if (flags) {
            *flags |= GM_NAV_TIGHT;
            if (g == GM_UNKNOWN)
                *flags |= GM_NAV_UNKNOWN;
        }
        return g;
    }
    g = nav_lattice_auth(nav, job, kind, kx, kz, h, 1, kind == K_ROT ? &job->fpp : &job->fpc, &from, &to);
    if (g != GM_BLOCKED) {
        if (flags && g == GM_UNKNOWN)
            *flags |= GM_NAV_UNKNOWN;
        return g;
    }
    g = nav_lattice_auth(nav, job, kind, kx, kz, h, 0, &job->fp, &from, &to);
    if (g == GM_BLOCKED)
        return g;
    job->last_tight = 1;
    if (flags) {
        *flags |= GM_NAV_TIGHT;
        if (g == GM_UNKNOWN)
            *flags |= GM_NAV_UNKNOWN;
    }
    return g;
}
/* ---------------------------------------------------------------- search */

static uint32_t nav_hkey(int kx, int kz, int h)
{
    uint32_t x = (uint32_t)kx * 0x9E3779B1u ^ (uint32_t)kz * 0x85EBCA77u ^ (uint32_t)h * 0xC2B2AE3Du;
    x ^= x >> 15;
    x *= 0x2C1B3C6Du;
    x ^= x >> 12;
    return x;
}

/* returns node index, -1 not found, -2 pool full */
static int nav_node(gm_nav_t *nav, int kx, int kz, int h, int create)
{
    size_t i = nav_hkey(kx, kz, h) & (NAV_HASH - 1);
    for (;;) {
        nav_hash_t *e = &nav->hash[i];
        if (e->serial != nav->serial) {
            if (!create)
                return -1;
            if (nav->node_count >= NAV_NODES)
                return -2;
            const int n = nav->node_count++;
            nav_node_t *node = &nav->nodes[n];
            memset(node, 0, sizeof *node);
            node->parent = -1;
            node->heap = -1;
            node->aux = -1;
            node->kx = (uint16_t)kx;
            node->kz = (uint16_t)kz;
            node->h = (uint8_t)h;
            node->g = INFINITY;
            e->serial = nav->serial;
            e->node = n;
            return n;
        }
        const nav_node_t *node = &nav->nodes[e->node];
        if (node->kx == kx && node->kz == kz && node->h == h)
            return e->node;
        i = (i + 1) & (NAV_HASH - 1);
    }
}

static int nav_less(const gm_nav_t *nav, int a, int b)
{
    const nav_node_t *x = &nav->nodes[a], *y = &nav->nodes[b];
    if (x->f != y->f)
        return x->f < y->f;
    if (x->g != y->g)
        return x->g > y->g;
    return a < b;
}

static void nav_heap_set(gm_nav_t *nav, int pos, int n)
{
    nav->heap[pos] = n;
    nav->nodes[n].heap = pos;
}

static void nav_heap_up(gm_nav_t *nav, int pos)
{
    const int n = nav->heap[pos];
    while (pos > 0) {
        const int parent = (pos - 1) / 2;
        if (!nav_less(nav, n, nav->heap[parent]))
            break;
        nav_heap_set(nav, pos, nav->heap[parent]);
        pos = parent;
    }
    nav_heap_set(nav, pos, n);
}

static void nav_heap_down(gm_nav_t *nav, int pos)
{
    const int n = nav->heap[pos];
    for (;;) {
        int c = 2 * pos + 1;
        if (c >= nav->heap_count)
            break;
        if (c + 1 < nav->heap_count && nav_less(nav, nav->heap[c + 1], nav->heap[c]))
            ++c;
        if (!nav_less(nav, nav->heap[c], n))
            break;
        nav_heap_set(nav, pos, nav->heap[c]);
        pos = c;
    }
    nav_heap_set(nav, pos, n);
}

static void nav_heap_push(gm_nav_t *nav, int n)
{
    if (nav->nodes[n].heap >= 0) {
        nav_heap_up(nav, nav->nodes[n].heap);
        return;
    }
    nav->heap[nav->heap_count] = n;
    nav->nodes[n].heap = nav->heap_count++;
    nav_heap_up(nav, nav->heap_count - 1);
}

static int nav_heap_pop(gm_nav_t *nav)
{
    const int n = nav->heap[0];
    nav->nodes[n].heap = -1;
    if (--nav->heap_count > 0) {
        nav_heap_set(nav, 0, nav->heap[nav->heap_count]);
        nav_heap_down(nav, 0);
    }
    return n;
}

/* ------------------------------------------------------- coarse field
 * Dial-queue Dijkstra from the leg target over 2 x 2-cell coarse cells.
 * A coarse cell is passable when any sub-cell is clear; cells without room
 * for the hull width cost NAV_NARROW_FACTOR times more. It runs until the
 * leg start is settled plus NAV_FIELD_EXTRA, so its result depends only on
 * the leg (deterministic). It only guides the search: validity is always
 * decided by the authority. */

static int nav_settled(const gm_nav_t *nav, int c) { return (nav->settled[c >> 3] >> (c & 7)) & 1; }

static int nav_bpush(gm_nav_t *nav, int cell, int cost)
{
    int32_t **items = &nav->bucket[cost % NAV_BUCKETS].items;
    int *count = &nav->bucket[cost % NAV_BUCKETS].count, *cap = &nav->bucket[cost % NAV_BUCKETS].cap;
    if (*count == *cap) {
        const int ncap = *cap ? *cap * 2 : 1024;
        int32_t *p = (int32_t *)realloc(*items, sizeof(int32_t) * (size_t)ncap);
        if (!p)
            return 0;
        *items = p;
        *cap = ncap;
    }
    (*items)[(*count)++] = cell;
    ++nav->pending;
    return 1;
}

static int nav_coarse_index(const gm_nav_t *nav, gm_point_t p)
{
    const float fx = (p.x - nav->x0) / nav->cs, fz = (p.z - nav->z0) / nav->cs;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)nav->cw || fz >= (float)nav->chh)
        return -1;
    return (int)fz * nav->cw + (int)fx;
}

/* Least clutter factor of the clutter cells a coarse cell overlaps, as
 * floor(factor * NAV_CLUTTER_Q): no metre driven inside the coarse cell costs
 * less, so the field stays a lower bound of the search's costs. */
static int nav_coarse_clutter(gm_nav_t *nav, int c)
{
    if (nav->cclut[c])
        return nav->cclut[c];
    const gm_map_t *m = &nav->map;
    const float xa = nav->x0 + (float)(c % nav->cw) * nav->cs, za = nav->z0 + (float)(c / nav->cw) * nav->cs;
    const float fx0 = (xa - m->clutter_x0) / m->clutter_step, fx1 = (xa + nav->cs - m->clutter_x0) / m->clutter_step;
    const float fz0 = (za - m->clutter_z0) / m->clutter_step, fz1 = (za + nav->cs - m->clutter_z0) / m->clutter_step;
    int least = 255;
    /* any part outside the table counts as no clutter */
    if (!(fx0 >= 0.0f) || !(fz0 >= 0.0f) || fx1 > (float)m->clutter_columns || fz1 > (float)m->clutter_rows)
        least = 0;
    else {
        const int i0 = (int)fx0, j0 = (int)fz0;
        const int i1 = (int)ceilf(fx1) - 1, j1 = (int)ceilf(fz1) - 1;
        for (int j = j0; j <= j1 && least; ++j)
            for (int i = i0; i <= i1; ++i) {
                const int v = m->clutter[(size_t)j * (size_t)m->clutter_columns + (size_t)i];
                if (v < least)
                    least = v;
            }
    }
    const float q = floorf((1.0f + m->clutter_k * (float)least * (1.0f / 255.0f)) * NAV_CLUTTER_Q);
    nav->cclut[c] = (uint8_t)(q > 255.0f ? 255.0f : q < NAV_CLUTTER_Q ? NAV_CLUTTER_Q : q);
    return nav->cclut[c];
}

static void nav_field_seed(gm_nav_t *nav, gm_point_t t, float radius)
{
    const float r = radius + nav->cs * 0.7072f;
    const int i0 = (int)floorf((t.x - r - nav->x0) / nav->cs), i1 = (int)floorf((t.x + r - nav->x0) / nav->cs);
    const int j0 = (int)floorf((t.z - r - nav->z0) / nav->cs), j1 = (int)floorf((t.z + r - nav->z0) / nav->cs);
    for (int j = j0 < 0 ? 0 : j0; j <= j1 && j < nav->chh; ++j)
        for (int i = i0 < 0 ? 0 : i0; i <= i1 && i < nav->cw; ++i) {
            const gm_point_t c = {nav->x0 + ((float)i + 0.5f) * nav->cs, nav->z0 + ((float)j + 0.5f) * nav->cs};
            const int k = j * nav->cw + i;
            if (nav_dist(c, t) <= r && nav->field[k] != 0) {
                nav->field[k] = 0;
                nav_bpush(nav, k, 0);
            }
        }
}

static void nav_field_begin(gm_nav_t *nav, const nav_job_t *job)
{
    const size_t coarse = (size_t)nav->cw * (size_t)nav->chh;
    memset(nav->field, 0xff, coarse * sizeof(uint16_t));
    memset(nav->settled, 0, (coarse + 7) / 8);
    for (int b = 0; b < NAV_BUCKETS; ++b)
        nav->bucket[b].count = 0;
    nav->pending = 0;
    nav->fcur = 0;
    nav->fstop = 0x7fffffff;
    nav->fdone = 0;
    nav->facc = 0;
    nav->fw1 = (int)floorf(nav->cs * 4.0f + 0.5f);
    nav->fw2 = (int)floorf(nav->cs * 4.0f * 1.41421356f);
    float thr = job->veh.width * 0.5f + job->margin - 0.5f * nav->step;
    thr = thr < 0.0f ? 0.0f : thr;
    nav->fpass = 1 + (int)floorf(thr * NAV_DQ);
    nav->fstart = nav_coarse_index(nav, job->cur);
    if (job->target_via >= 0)
        nav_field_seed(nav, job->via[job->target_via], NAV_VIA_RADIUS);
    else
        for (int i = 0; i < job->goal_count; ++i)
            if (job->goal_ok[i])
                nav_field_seed(nav, job->goals[i], job->goal_reff);
}

static void nav_field_step(gm_nav_t *nav)
{
    while (!nav->fdone && nav->left > 0) {
        if (!nav->pending || nav->fcur > nav->fstop || nav->fcur >= 65000) {
            nav->fdone = 1;
            break;
        }
        int *count = &nav->bucket[nav->fcur % NAV_BUCKETS].count;
        if (!*count) {
            ++nav->fcur;
            continue;
        }
        const int c = nav->bucket[nav->fcur % NAV_BUCKETS].items[--*count];
        --nav->pending;
        if (nav->field[c] != nav->fcur || nav_settled(nav, c))
            continue;
        nav->settled[c >> 3] |= (uint8_t)(1u << (c & 7));
        if (c == nav->fstart)
            nav->fstop = nav->fcur + NAV_FIELD_EXTRA;
        if (++nav->facc >= 16) {
            nav->facc = 0;
            nav->left -= 1;
        }
        const int ci = c % nav->cw, cj = c / nav->cw;
        /* The search pays clutter: the field counts the least of it on each
         * edge (the smaller of the two cells' least factors), or the search
         * fans out over every lane a straight line offers. */
        const int qa = nav->map.clutter ? nav_coarse_clutter(nav, c) : NAV_CLUTTER_Q;
        for (int d = 0; d < 8; ++d) {
            const int ni = ci + nav_dx[d], nj = cj + nav_dz[d];
            if (ni < 0 || nj < 0 || ni >= nav->cw || nj >= nav->chh)
                continue;
            const int n = nj * nav->cw + ni;
            if (nav_settled(nav, n) || (!nav->cmax[n] && n != nav->fstart))
                continue;
            int w = (d & 1) ? nav->fw2 : nav->fw1;
            if (nav->cmax[n] < nav->fpass)
                w *= NAV_NARROW_FACTOR;
            if (qa > NAV_CLUTTER_Q) {
                const int qb = nav->cclut[n] ? nav->cclut[n] : nav_coarse_clutter(nav, n);
                w = w * (qa < qb ? qa : qb) / NAV_CLUTTER_Q;
            }
            const int nc = nav->fcur + w;
            if (nc < nav->field[n] && nc < 65000) {
                nav->field[n] = (uint16_t)nc;
                nav_bpush(nav, n, nc);
            }
        }
    }
}

static float nav_field_value(const gm_nav_t *nav, int i, int j)
{
    i = i < 0 ? 0 : i >= nav->cw ? nav->cw - 1 : i;
    j = j < 0 ? 0 : j >= nav->chh ? nav->chh - 1 : j;
    const int c = j * nav->cw + i;
    return (float)(nav_settled(nav, c) ? nav->field[c] : nav->fcur);
}

static float nav_field_h(const gm_nav_t *nav, gm_point_t p)
{
    const float fx = (p.x - nav->x0) / nav->cs - 0.5f, fz = (p.z - nav->z0) / nav->cs - 0.5f;
    const int i = (int)floorf(fx), j = (int)floorf(fz);
    const float tx = fx - (float)i, tz = fz - (float)j;
    const float v = (1.0f - tz) * ((1.0f - tx) * nav_field_value(nav, i, j) + tx * nav_field_value(nav, i + 1, j))
                    + tz * ((1.0f - tx) * nav_field_value(nav, i, j + 1) + tx * nav_field_value(nav, i + 1, j + 1));
    const float h = v * 0.25f - 1.5f * nav->cs;
    return h > 0.0f ? h : 0.0f;
}

static float nav_heuristic(const gm_nav_t *nav, const nav_job_t *job, gm_point_t p)
{
    const float hf = nav_field_h(nav, p);
    if (job->target_via >= 0) {
        const float h = nav_octile(p, job->via[job->target_via]) - NAV_OCT_SLACK * NAV_VIA_RADIUS;
        return h > hf ? h : hf;
    }
    const float h = nav_octile(p, job->hc) - NAV_OCT_SLACK * job->hr;
    return (h > hf ? h : hf) + job->hmin;
}

static void nav_search_begin(gm_nav_t *nav, nav_job_t *job, int cap)
{
    if (++nav->serial == 0)
        nav->serial = 1, memset(nav->hash, 0, sizeof(nav_hash_t) * NAV_HASH);
    nav->node_count = 0;
    nav->heap_count = 0;
    job->seed_count = 0;
    job->cap = cap;
    job->expansions = 0;
    job->end_node = -1;
}

static void nav_seed(gm_nav_t *nav, nav_job_t *job, int kx, int kz, int h, float g, float travel, uint8_t flags,
                     const nav_seed_t *prefix)
{
    const int n = nav_node(nav, kx, kz, h, 1);
    if (n < 0)
        return;
    nav_node_t *node = &nav->nodes[n];
    if (node->g <= g)
        return;
    int aux = -1;
    if (prefix && prefix->n) {
        if (job->seed_count >= NAV_SEEDS)
            return;
        aux = job->seed_count;
        job->seeds[job->seed_count++] = *prefix;
    }
    node->g = g;
    node->travel = travel;
    node->flags = (uint8_t)(flags | NF_OPEN);
    node->aux = (int16_t)aux;
    node->parent = -1;
    node->hin = (uint8_t)h;
    node->rdir = 0;
    node->f = g + NAV_WEIGHT * nav_heuristic(nav, job, nav_lp(nav, kx, kz));
    nav_heap_push(nav, n);
}

/* exact clearance increase rule for escaping a blocked start footprint */
static int nav_escape_ok(const gm_nav_t *nav, gm_point_t from, gm_point_t to)
{
    return gm_nav_clearance(nav, to) > gm_nav_clearance(nav, from) + 1e-4f;
}

/* Clutter factor of one lattice step p -> q, at its midpoint: a step is at
 * most 1.4 m against 4 m clutter cells, so it lies in one or two cells and
 * the midpoint charges each metre by the cell it crosses (the same rule as
 * nav_path_cost, which string pulling compares against). A step that frees
 * the start footprint (escape) is not charged: the shortest way out. */
static float nav_step_clutter(const gm_nav_t *nav, gm_point_t p, gm_point_t q, int esc)
{
    if (esc || !nav->map.clutter)
        return 1.0f;
    const gm_point_t m = {0.5f * (p.x + q.x), 0.5f * (p.z + q.z)};
    return nav_clutter(nav, m);
}

static int nav_relax(gm_nav_t *nav, nav_job_t *job, int from, int kx, int kz, int h, float cost, float travel,
                     uint8_t flags, int hin, int rdir)
{
    const int n = nav_node(nav, kx, kz, h, 1);
    if (n < 0)
        return -1;
    nav_node_t *node = &nav->nodes[n];
    const float g = nav->nodes[from].g + cost;
    if ((node->flags & NF_CLOSED) || g >= node->g)
        return -1;
    node->g = g;
    node->hin = (uint8_t)hin;
    node->rdir = (int8_t)rdir;
    node->f = g + NAV_WEIGHT * nav_heuristic(nav, job, nav_lp(nav, kx, kz));
    node->parent = from;
    node->travel = travel;
    node->flags = (uint8_t)(flags | NF_OPEN);
    nav_heap_push(nav, n);
    return n;
}

/* skip the authority when the successor cannot improve */
static int nav_useless(const gm_nav_t *nav, int kx, int kz, int h, float g)
{
    const int n = nav_node((gm_nav_t *)nav, kx, kz, h, 0);
    return n >= 0 && ((nav->nodes[n].flags & NF_CLOSED) || nav->nodes[n].g <= g);
}

/* one A* expansion: 0 continue, 1 success (job->end_node), -1 failure */
static int nav_expand(gm_nav_t *nav, nav_job_t *job)
{
    if (!nav->heap_count || job->expansions >= job->cap || nav->node_count >= NAV_NODES - 8)
        return -1;
    const int n = nav_heap_pop(nav);
    nav->left -= 1;
    ++job->expansions;
    ++job->total_expansions;
    ++nav->stats.expansions;
    if (job->expansions > nav->stats.max_expansions)
        nav->stats.max_expansions = job->expansions;
    nav_node_t *node = &nav->nodes[n];
    node->flags = (uint8_t)((node->flags | NF_CLOSED) & ~NF_OPEN);
    if (node->flags & NF_TERM) {
        job->end_node = node->parent;
        job->goal_index = node->aux;
        return 1;
    }
    const int kx = node->kx, kz = node->kz, h = node->h;
    const gm_point_t p = nav_lp(nav, kx, kz);
    const int esc = (node->flags & NF_ESC) != 0;
    if (!esc) {
        if (job->target_via >= 0) {
            if (nav_dist(p, job->via[job->target_via]) <= NAV_VIA_RADIUS) {
                job->end_node = n;
                return 1;
            }
        } else {
            int best = -1;
            float cost = 0.0f;
            for (int i = 0; i < job->goal_count; ++i)
                if (job->goal_ok[i] && nav_dist(p, job->goals[i]) <= job->goal_reff
                    && (best < 0 || job->goal_cost[i] < cost))
                    best = i, cost = job->goal_cost[i];
            if (best >= 0 && nav->node_count < NAV_NODES) {
                const int t = nav->node_count++;
                nav_node_t *term = &nav->nodes[t];
                memset(term, 0, sizeof *term);
                term->parent = n;
                term->heap = -1;
                term->aux = (int16_t)best;
                term->flags = NF_TERM | NF_OPEN;
                term->kx = (uint16_t)kx;
                term->kz = (uint16_t)kz;
                term->h = (uint8_t)h;
                term->g = nav->nodes[n].g + cost;
                term->f = term->g;
                nav_heap_push(nav, t);
                node = &nav->nodes[n];
            }
        }
    }
    const float g0 = node->g, travel0 = node->travel;
    const float step = (h & 1) ? nav->s * 1.41421356f : nav->s;
    /* forward */
    {
        const int tx = kx + nav_dx[h], tz = kz + nav_dz[h];
        if (nav_in_lattice(nav, tx, tz) && !nav_useless(nav, tx, tz, h, g0 + step)) {
            uint8_t fl = 0;
            enum gm_geometry r = nav_lattice(nav, job, K_TRANS, kx, kz, h, 0, &fl);
            uint8_t nf = job->last_tight ? NF_TIGHT : 0;
            if (r == GM_BLOCKED && esc && !job->hard && nav_escape_ok(nav, p, nav_lp(nav, tx, tz))) {
                r = GM_UNKNOWN;
                nf |= NF_TIGHT;
                if (nav_lattice(nav, job, K_STAT, tx, tz, h, 0, NULL) == GM_BLOCKED)
                    nf |= NF_ESC;
            }
            if (r != GM_BLOCKED) {
                if (r == GM_UNKNOWN)
                    nf |= NF_UNK;
                /* Steep or wet ground costs the time it takes to cross it,
                 * built-up ground (clutter) the risk of getting stuck there. */
                const gm_point_t q = nav_lp(nav, tx, tz);
                const float terrain = 0.5f * (nav_cell_cost(nav, p) + nav_cell_cost(nav, q));
                nav_relax(nav, job, n, tx, tz, h,
                          step * terrain * nav_step_clutter(nav, p, q, esc)
                              * (r == GM_UNKNOWN ? NAV_UNKNOWN_FACTOR : 1.0f)
                              * ((nf & NF_TIGHT) && job->tiers ? NAV_TIGHT_TRANS : 1.0f),
                          travel0 + step, nf, h, 0);
            }
        }
    }
    /* rotations */
    for (int side = -1; side <= 1; side += 2) {
        const int th = (h + side) & 7;
        /* A vertex route pivots the SHORTEST way from the arrival heading. Keep the
         * rotation chain at one position monotonic and representable: at most
         * 135 deg right or 180 deg left (remainder() resolves +-180 to +180). */
        const int hin = nav->nodes[n].hin, rd = nav->nodes[n].rdir;
        int s = (th - hin) & 7;
        if (s > 4)
            s -= 8;
        if ((rd && rd != side) || (side > 0 ? (s < 1 || s > 4) : (s > -1 || s < -3)))
            continue;
        if (nav_useless(nav, kx, kz, th, nav->nodes[n].g + NAV_ROT_COST))
            continue;
        const enum gm_geometry r = nav_lattice(nav, job, K_ROT, kx, kz, side > 0 ? h : th, side < 0, NULL);
        if (r == GM_BLOCKED)
            continue;
        const int tight = job->last_tight;
        nav_relax(nav, job, n, kx, kz, th,
                  NAV_ROT_COST * (r == GM_UNKNOWN ? NAV_UNKNOWN_FACTOR : 1.0f) * (tight ? NAV_TIGHT_ROT : 1.0f),
                  nav->nodes[n].travel,
                  (uint8_t)((r == GM_UNKNOWN ? NF_UNK : 0) | (esc ? NF_ESC : 0) | (tight ? NF_TIGHT : 0)), hin, side);
    }
    /* reverse (initial escape only) */
    /* REVERSE legs keep the arrival heading: not after an in-place rotation at this position */
    if (job->first_leg && nav->nodes[n].travel + step <= NAV_REVERSE_TRAVEL && !nav->nodes[n].rdir
        && !(nav->nodes[n].flags & NF_NOREV)) {
        const int tx = kx - nav_dx[h], tz = kz - nav_dz[h];
        const float cost = step * NAV_REVERSE_COST;
        if (nav_in_lattice(nav, tx, tz) && !nav_useless(nav, tx, tz, h, nav->nodes[n].g + cost)) {
            enum gm_geometry r = nav_lattice(nav, job, K_TRANS, tx, tz, h, 1, NULL);
            uint8_t nf = (uint8_t)(NF_REV | (job->last_tight ? NF_TIGHT : 0));
            if (r == GM_BLOCKED && esc && !job->hard && nav_escape_ok(nav, p, nav_lp(nav, tx, tz))) {
                r = GM_UNKNOWN;
                nf |= NF_TIGHT;
                if (nav_lattice(nav, job, K_STAT, tx, tz, h, 0, NULL) == GM_BLOCKED)
                    nf |= NF_ESC;
            }
            if (r != GM_BLOCKED) {
                if (r == GM_UNKNOWN)
                    nf |= NF_UNK;
                const gm_point_t q = nav_lp(nav, tx, tz);
                const float terrain = 0.5f * (nav_cell_cost(nav, p) + nav_cell_cost(nav, q));
                nav_relax(nav, job, n, tx, tz, h,
                          cost * terrain * nav_step_clutter(nav, p, q, esc)
                              * (r == GM_UNKNOWN ? NAV_UNKNOWN_FACTOR : 1.0f)
                              * ((nf & NF_TIGHT) && job->tiers ? NAV_TIGHT_TRANS : 1.0f),
                          nav->nodes[n].travel + step, nf, h, 0);
            }
        }
    }
    return 0;
}

/* --------------------------------------------------------------- the job */

/* tight: arriving translation tight; ptight: pivot at p tight (merged into the
 * previous vertex when p repeats it) */
static int nav_raw_push(nav_job_t *job, gm_point_t p, int rev, int tight, int ptight)
{
    if (job->raw_count && nav_dist(job->raw[job->raw_count - 1].p, p) < 1e-4f) {
        job->raw[job->raw_count - 1].ptight |= (uint8_t)(ptight != 0);
        return 1;
    }
    if (job->raw_count >= NAV_RAW - 1) {
        job->overflow = 1;
        return 0;
    }
    job->raw[job->raw_count].p = p;
    job->raw[job->raw_count].rev = (uint8_t)(rev != 0);
    job->raw[job->raw_count].tight = (uint8_t)(tight != 0);
    job->raw[job->raw_count].ptight = (uint8_t)(ptight != 0);
    ++job->raw_count;
    return 1;
}

static int nav_reconstruct(gm_nav_t *nav, nav_job_t *job)
{
    int *chain = nav->heap; /* the open list is not needed any more */
    int count = 0;
    for (int i = job->end_node; i >= 0 && count < NAV_NODES; i = nav->nodes[i].parent)
        chain[count++] = i;
    const int saved = job->raw_count;
    const nav_node_t *seed = &nav->nodes[chain[count - 1]];
    if (seed->aux >= 0) {
        const nav_seed_t *pre = &job->seeds[seed->aux];
        if (job->raw_count)
            job->raw[job->raw_count - 1].ptight |= pre->p0tight;
        for (int k = 0; k < pre->n; ++k)
            if (!nav_raw_push(job, pre->pts[k], pre->rev[k], pre->tight[k], pre->ptight[k]))
                break;
    }
    nav_raw_push(job, nav_lp(nav, seed->kx, seed->kz), 0, 0, 0);
    for (int k = count - 2; k >= 0; --k) {
        const nav_node_t *node = &nav->nodes[chain[k]];
        const gm_point_t q = nav_lp(nav, node->kx, node->kz);
        const int tight = (node->flags & NF_TIGHT) != 0;
        const int moved = nav_dist(job->raw[job->raw_count - 1].p, q) >= 1e-4f;
        if (!nav_raw_push(job, q, node->flags & NF_REV, moved && tight, !moved && tight))
            break;
    }
    if (job->overflow) {
        job->raw_count = saved;
        job->overflow = 0;
        return 0;
    }
    const nav_node_t *end = &nav->nodes[job->end_node];
    job->cur = nav_lp(nav, end->kx, end->kz);
    job->cur_h = nav_hangle(end->h);
    job->cur_lat = 1;
    job->ckx = end->kx;
    job->ckz = end->kz;
    job->ch = end->h;
    return 1;
}

static void nav_leg_moved(nav_job_t *job)
{
    job->first_leg = 0;
    job->escape = 0;
}

static void nav_start_search(gm_nav_t *nav, nav_job_t *job, int via)
{
    job->target_via = via;
    int cap = NAV_VIA_EXPANSIONS;
    if (via < 0) {
        const float d = nav_dist(job->cur, job->hc) - job->hr;
        const float c = (float)NAV_GOAL_BASE + (float)NAV_GOAL_PER_M * (d > 0.0f ? d : 0.0f);
        cap = c < (float)NAV_GOAL_EXPANSIONS ? (int)c : NAV_GOAL_EXPANSIONS;
    }
    nav_search_begin(nav, job, cap);
    ++nav->stats.searches;
    job->conn_from = job->cur;
    job->conn_h = job->cur_h;
    job->conn_rev = 0;
    job->conn_i = 0;
    job->conn_ring = 1;
    job->rev_tried = 0;
    job->travel0 = 0.0f;
    nav_field_begin(nav, job);
    job->stage = J_FIELD;
}

static void nav_job_stage_field(gm_nav_t *nav, nav_job_t *job)
{
    nav_field_step(nav);
    if (!nav->fdone)
        return;
    /* the start is not connected to the target even with optimistic coarse
     * passability: bound the fine search like a via sub-search */
    if (nav->fstart < 0 || !nav_settled(nav, nav->fstart))
        job->cap = job->cap < NAV_VIA_EXPANSIONS ? job->cap : NAV_VIA_EXPANSIONS;
    job->stage = J_CONNECT;
}

static void nav_leg_failed(nav_job_t *job)
{
    if (job->target_via >= 0) {
        job->via_raw[job->target_via] = -1;
        job->via_k = job->target_via + 1;
        job->stage = J_NEXT_LEG;
    } else if (job->via_count && !job->dropped_vias && !job->extend && job->raw_count > job->free_prefix + 1) {
        /* The via legs ended where the goal leg cannot go on: a via point in a
         * pond leaves the hull at its edge facing the water, with no move
         * left (Poland seat 15). The vias are advice: once more from the
         * start, straight for the goal. */
        job->dropped_vias = 1;
        for (int k = 0; k < job->via_count; ++k)
            job->via_raw[k] = -1;
        job->via_k = job->via_count;
        job->raw_count = job->free_prefix + 1;
        job->cur = job->raw[job->free_prefix].p;
        job->cur_h = job->free_prefix ? job->free_h : job->start_h;
        job->cur_lat = 0;
        job->first_leg = 1;
        job->escape = job->free_prefix ? 0 : job->escape0;
        job->los_done = job->los_count = 0;
        job->stage = J_NEXT_LEG;
    } else {
        job->stage = J_FAIL;
    }
}

/* one connection candidate from the off-lattice pose to lattice node (kx,kz) */
static void nav_connect_candidate(gm_nav_t *nav, nav_job_t *job, int kx, int kz)
{
    if (!nav_in_lattice(nav, kx, kz))
        return;
    const gm_point_t P = job->conn_from, n = nav_lp(nav, kx, kz);
    const float d = nav_dist(P, n);
    const float psi = d < 0.05f ? job->conn_h : nav_dir(P, n);
    uint8_t fl = 0;
    int forced = 0, t1 = 0, t2 = 0;
    enum gm_geometry r = nav_pivot(nav, job, P, job->conn_h, psi, &fl);
    t1 = job->last_tight;
    if (r == GM_BLOCKED) {
        if (!job->escape || job->hard || !nav_escape_ok(nav, P, n))
            return;
        forced = t1 = 1;
        r = GM_UNKNOWN;
    }
    enum gm_geometry r2 = d >= 0.05f ? nav_trans(nav, job, P, n, psi, &fl) : GM_CLEAR;
    t2 = d >= 0.05f && job->last_tight;
    if (r2 == GM_BLOCKED) {
        if (!job->escape || job->hard || !nav_escape_ok(nav, P, n))
            return;
        forced = t2 = 1;
        r2 = GM_UNKNOWN;
    }
    r = nav_worst(r, r2);
    const float rot_k = job->tiers && t1 ? NAV_TIGHT_ROT : 1.0f, trans_k = job->tiers && t2 ? NAV_TIGHT_TRANS : 1.0f;
    const float base = nav_rotcost(job->conn_h, psi) * rot_k
                       + nav_path_cost(nav, P, n, NULL) * trans_k * (r == GM_UNKNOWN ? NAV_UNKNOWN_FACTOR : 1.0f);
    const float sector = (psi < 0.0f ? psi + 2.0f * NAV_PI : psi) / NAV_QUARTER;
    const int lo = (int)floorf(sector) & 7;
    const int exact = fabsf(nav_angle(psi - nav_hangle(lo))) < 1e-4f;
    nav_seed_t pre;
    memset(&pre, 0, sizeof pre);
    if (job->conn_rev) {
        /* reverse escape leg to P (no pivot at the start), then the pivot at P */
        pre.pts[pre.n] = P;
        pre.tight[pre.n] = (uint8_t)job->rev_tight;
        pre.ptight[pre.n] = (uint8_t)t1;
        pre.rev[pre.n++] = 1;
    } else {
        pre.p0tight = (uint8_t)t1;
    }
    pre.pts[pre.n] = n;
    pre.tight[pre.n] = (uint8_t)t2;
    pre.rev[pre.n++] = 0;
    for (int k = 0; k < (exact ? 1 : 2); ++k) {
        const int h = (lo + k) & 7;
        uint8_t f3 = 0;
        enum gm_geometry r3 = nav_pivot(nav, job, n, psi, nav_hangle(h), &f3);
        int t3 = job->last_tight;
        int esc = 0;
        if (r3 == GM_BLOCKED) {
            if (!job->escape || job->hard)
                continue;
            esc = t3 = 1;
            r3 = GM_UNKNOWN;
        }
        if ((forced || esc) && nav_lattice(nav, job, K_STAT, kx, kz, h, 0, NULL) == GM_BLOCKED)
            esc = 1;
        else
            esc = 0;
        const enum gm_geometry rr = nav_worst(r, r3);
        const float g = base + nav_rotcost(psi, nav_hangle(h)) * (r3 == GM_UNKNOWN ? NAV_UNKNOWN_FACTOR : 1.0f)
                                   * (job->tiers && t3 ? NAV_TIGHT_ROT : 1.0f);
        pre.ptight[pre.n - 1] = (uint8_t)t3;
        nav_seed(nav, job, kx, kz, h, g, job->travel0 + d,
                 (uint8_t)((rr == GM_UNKNOWN ? NF_UNK : 0) | (esc ? NF_ESC : 0)
                           | (fabsf(nav_angle(nav_hangle(h) - psi)) > 1e-4f ? NF_NOREV : 0)), &pre);
    }
}
static void nav_job_stage_connect(gm_nav_t *nav, nav_job_t *job)
{
    if (job->cur_lat && !job->conn_rev && job->conn_i == 0 && job->conn_ring == 1) {
        int esc = 0;
        if (job->escape) {
            if (nav->left < NAV_OP)
                return;
            esc = nav_lattice(nav, job, K_STAT, job->ckx, job->ckz, job->ch, 0, NULL) == GM_BLOCKED;
        }
        nav_seed(nav, job, job->ckx, job->ckz, job->ch, 0.0f, 0.0f, (uint8_t)(esc ? NF_ESC : 0), NULL);
        job->stage = J_SEARCH;
        return;
    }
    const int kx0 = (int)floorf((job->conn_from.x - nav->x0) / nav->s + 0.5f);
    const int kz0 = (int)floorf((job->conn_from.z - nav->z0) / nav->s + 0.5f);
    for (;;) {
        const int ring = job->conn_ring, w = 2 * ring + 1;
        if (job->conn_i >= w * w) {
            if (nav->heap_count) {
                job->stage = J_SEARCH;
                return;
            }
            if (ring < 2) {
                job->conn_ring = 2;
                job->conn_i = 0;
                continue;
            }
            if (job->conn_rev) {
                job->stage = J_REVERSE; /* next reverse distance */
                return;
            }
            if (job->first_leg && !job->rev_tried) {
                job->rev_tried = 1;
                job->rev_d = 2;
                job->stage = J_REVERSE;
                return;
            }
            nav_leg_failed(job);
            return;
        }
        if (nav->left < NAV_OP)
            return;
        const int i = job->conn_i++;
        const int dx = i % w - ring, dz = i / w - ring;
        if (ring == 2 && abs(dx) < 2 && abs(dz) < 2)
            continue;
        nav_connect_candidate(nav, job, kx0 + dx, kz0 + dz);
    }
}

static void nav_job_stage_reverse(gm_nav_t *nav, nav_job_t *job)
{
    while (job->rev_d <= 6) {
        if (nav->left < NAV_OP)
            return;
        const float d = (float)job->rev_d++;
        const gm_point_t P = job->cur;
        const gm_point_t Q = {P.x - cosf(job->cur_h) * d, P.z - sinf(job->cur_h) * d};
        const enum gm_geometry r = nav_trans(nav, job, P, Q, job->cur_h, NULL);
        if (r == GM_BLOCKED)
            continue;
        job->rev_tight = job->last_tight;
        job->conn_from = Q;
        job->conn_h = job->cur_h;
        job->conn_rev = 1;
        job->conn_i = 0;
        job->conn_ring = 1;
        job->travel0 = d;
        job->stage = J_CONNECT;
        return;
    }
    job->conn_rev = 0;
    nav_leg_failed(job);
}

static int nav_goal_inside(const nav_job_t *job, gm_point_t p, float radius, float *cost)
{
    int best = -1;
    for (int i = 0; i < job->goal_count; ++i)
        if (job->goal_ok[i] && nav_dist(p, job->goals[i]) <= radius && (best < 0 || job->goal_cost[i] < *cost))
            best = i, *cost = job->goal_cost[i];
    return best;
}

static void nav_job_stage_next(gm_nav_t *nav, nav_job_t *job)
{
    for (;;) {
        if (job->via_k < job->via_count) {
            const int k = job->via_k;
            const gm_point_t V = job->via[k];
            if (nav_dist(job->cur, V) <= NAV_VIA_RADIUS) {
                job->via_raw[k] = (short)(job->raw_count - 1);
                ++job->via_k;
                continue;
            }
            const nav_comps_t there = nav_comps_near(nav, V);
            if (!nav_comps_meet(&job->start_comps, &there)) {
                job->via_raw[k] = -1; /* advisory, and in another component: no search for it */
                ++job->via_k;
                continue;
            }
            if (nav->left < NAV_OP)
                return;
            if (nav_los(nav, job, job->cur, job->cur_h, V)) {
                if (!nav_raw_push(job, V, 0, 0, 0)) {
                    job->overflow = 0;
                    job->via_raw[k] = -1;
                    ++job->via_k;
                    continue;
                }
                job->cur_h = nav_dir(job->cur, V);
                job->cur = V;
                job->cur_lat = 0;
                job->via_raw[k] = (short)(job->raw_count - 1);
                ++job->via_k;
                nav_leg_moved(job);
                continue;
            }
            nav_start_search(nav, job, k);
            return;
        }
        /* goal leg */
        float cost = 0.0f;
        const int inside = nav_goal_inside(job, job->cur, job->goal_radius, &cost);
        if (inside >= 0) {
            job->goal_index = inside;
            job->stage = J_SHORTCUT;
            return;
        }
        if (!job->los_done) {
            if (!job->los_count) {
                /* order goals by straight-line lower bound */
                float key[NAV_LOS_GOALS];
                for (int i = 0; i < job->goal_count; ++i) {
                    if (!job->goal_ok[i])
                        continue;
                    const float d = nav_dist(job->cur, job->goals[i]) - job->goal_radius;
                    const float v = (d > 0.0f ? d : 0.0f) + job->goal_cost[i];
                    int pos = job->los_count;
                    while (pos > 0 && key[pos - 1] > v) {
                        if (pos < NAV_LOS_GOALS) {
                            key[pos] = key[pos - 1];
                            job->los_order[pos] = job->los_order[pos - 1];
                        }
                        --pos;
                    }
                    if (pos < NAV_LOS_GOALS) {
                        key[pos] = v;
                        job->los_order[pos] = i;
                        if (job->los_count < NAV_LOS_GOALS)
                            ++job->los_count;
                    }
                }
                job->los_i = 0;
            }
            while (job->los_i < job->los_count) {
                if (nav->left < NAV_OP)
                    return;
                const int i = job->los_order[job->los_i++];
                const gm_point_t G = job->goals[i];
                const float d = nav_dist(job->cur, G);
                gm_point_t e = G;
                if (d > 1e-3f && job->goal_radius > 0.0f) {
                    const float back = job->goal_radius * 0.5f;
                    e.x = G.x + (job->cur.x - G.x) / d * back;
                    e.z = G.z + (job->cur.z - G.z) / d * back;
                }
                if (nav_los(nav, job, job->cur, job->cur_h, e) && nav_raw_push(job, e, 0, 0, 0)) {
                    job->cur_h = nav_dir(job->cur, e);
                    job->cur = e;
                    job->cur_lat = 0;
                    job->goal_index = i;
                    nav_leg_moved(job);
                    job->stage = J_SHORTCUT;
                    return;
                }
                job->overflow = 0;
            }
            job->los_done = 1;
        }
        nav_start_search(nav, job, -1);
        return;
    }
}

static void nav_job_stage_search(gm_nav_t *nav, nav_job_t *job)
{
    while (nav->left >= NAV_OP) {
        const int r = nav_expand(nav, job);
        if (r == 0)
            continue;
        if (r < 0 || !nav_reconstruct(nav, job)) {
#ifdef GM_NAV_DEBUG
            fprintf(stderr, "nav: veh %llu leg via=%d from (%.1f,%.1f) failed after %d expansions, nodes %d heap %d\n",
                    (unsigned long long)job->veh.id, job->target_via, job->cur.x, job->cur.z, job->expansions,
                    nav->node_count, nav->heap_count);
#endif
            nav_leg_failed(job);
            return;
        }
#ifdef GM_NAV_DEBUG
        {
            int s = job->end_node;
            while (nav->nodes[s].parent >= 0)
                s = nav->nodes[s].parent;
            fprintf(stderr, "nav: veh %llu leg via=%d ok: %d expansions, seed h=%.1f g=%.1f, end g=%.1f, field fcur=%d start settled=%d\n",
                    (unsigned long long)job->veh.id, job->target_via, job->expansions,
                    nav_heuristic(nav, job, nav_lp(nav, nav->nodes[s].kx, nav->nodes[s].kz)), nav->nodes[s].g,
                    nav->nodes[job->end_node].g, nav->fcur, nav->fstart >= 0 && nav_settled(nav, nav->fstart));
        }
#endif
        if (job->target_via >= 0) {
            job->via_raw[job->target_via] = (short)(job->raw_count - 1);
            job->via_k = job->target_via + 1;
            nav_leg_moved(job);
            job->stage = J_NEXT_LEG;
            return;
        }
        nav_leg_moved(job);
        job->stage = job->goal_radius < job->goal_reff ? J_FINAL : J_SHORTCUT;
        return;
    }
}

static void nav_job_stage_final(gm_nav_t *nav, nav_job_t *job)
{
    if (nav->left < NAV_OP)
        return;
    const gm_point_t G = job->goals[job->goal_index];
    if (nav_dist(job->cur, G) > job->goal_radius
        && nav_los(nav, job, job->cur, job->cur_h, G) && nav_raw_push(job, G, 0, 0, 0)) {
        job->cur_h = nav_dir(job->cur, G);
        job->cur = G;
        job->cur_lat = 0;
    }
    job->overflow = 0;
    job->stage = J_SHORTCUT;
}

static void nav_collapse(gm_nav_t *nav, nav_job_t *job)
{
    const int m = job->raw_count - 1;
    int *map = nav->remap;
    if (m < 1) {
        for (int i = 0; i <= m; ++i)
            map[i] = i;
    } else {
        int w = 1;
        map[0] = 0;
        for (int i = 1; i < m; ++i) {
            const gm_point_t a = job->raw[w - 1].p, b = job->raw[i].p, c = job->raw[i + 1].p;
            const float ux = b.x - a.x, uz = b.z - a.z, vx = c.x - b.x, vz = c.z - b.z;
            const float cr = ux * vz - uz * vx, dot = ux * vx + uz * vz;
            const float lu = hypotf(ux, uz), lv = hypotf(vx, vz);
            if (fabsf(cr) <= 1e-3f * lu * lv && dot > 0.0f && job->raw[i].rev == job->raw[i + 1].rev
                && job->raw[i].tight == job->raw[i + 1].tight && i > job->free_prefix) {
                map[i] = w;
                continue;
            }
            job->raw[w] = job->raw[i];
            map[i] = w++;
        }
        job->raw[w] = job->raw[m];
        map[m] = w;
        job->raw_count = w + 1;
    }
    for (int k = 0; k < job->via_count; ++k)
        if (job->via_raw[k] >= 0)
            job->via_raw[k] = (short)map[job->via_raw[k]];
    if (job->free_prefix > 0 && job->free_prefix <= m)
        job->free_prefix = map[job->free_prefix]; /* its points are kept, so this is their new index */
}

static int nav_candidate_ok(const nav_job_t *job, int a, int j)
{
    const int m = job->raw_count - 1;
    if (j > m || j - a > NAV_SHORTCUT_SPAN || a < job->free_prefix)
        return 0; /* the move that frees the hull at the start stays as validated */
    if (job->raw[j].rev || (j < m && job->raw[j + 1].rev))
        return 0;
    for (int i = a + 1; i < j; ++i)
        if (job->raw[i].rev)
            return 0;
    for (int k = 0; k < job->via_count; ++k) {
        const int r = job->via_raw[k];
        if (r > a && r < j && nav_segdist(job->via[k], job->raw[a].p, job->raw[j].p) > NAV_VIA_RADIUS)
            return 0;
    }
    return 1;
}

static int nav_shortcut_test(gm_nav_t *nav, nav_job_t *job, int a, float H, int j, uint8_t *pf, uint8_t *lf)
{
    const int m = job->raw_count - 1;
    const gm_point_t P = job->raw[a].p, T = job->raw[j].p;
    const float d = nav_dir(P, T);
    *pf = 0;
    *lf = 0;
    /* comfort tier: never trade a comfortable chain (or pivot) for a tight shortcut,
     * never merge a comfortable leg with a tight one */
    int need_t = 0;
    for (int i = a + 1; i <= j && job->tiers; ++i)
        need_t |= !job->raw[i].tight;
    const int need_p = job->tiers && !job->raw[a].ptight;
    const int need_j = job->tiers && j < m && !job->raw[j].ptight;
    if (nav->cost || nav->map.clutter) {
        /* String pulling must not undo what the terrain and clutter costs made
         * the search do: the straight cut may not cost more than the chain it
         * replaces. */
        float chain = 0.0f;
        for (int i = a + 1; i <= j; ++i)
            chain += nav_path_cost(nav, job->raw[i - 1].p, job->raw[i].p, NULL);
        if (nav_path_cost(nav, P, T, NULL) > chain + 0.25f)
            return 0;
    }
    if (nav_pivot(nav, job, P, H, d, pf) == GM_BLOCKED || (need_p && job->last_tight))
        return 0;
    if (nav_trans(nav, job, P, T, d, lf) == GM_BLOCKED || (need_t && job->last_tight))
        return 0;
    if (j < m) {
        uint8_t f3 = 0;
        if (nav_pivot(nav, job, T, d, nav_dir(T, job->raw[j + 1].p), &f3) == GM_BLOCKED || (need_j && job->last_tight))
            return 0;
    }
    return 1;
}

static void nav_emit(nav_job_t *job, int a, int best, float H, uint8_t pf, uint8_t lf)
{
    const int rev = job->raw[best].rev;
    if (job->out_n == 0)
        job->start_pflags |= pf;
    else
        job->out_f[job->out_n - 1] |= pf;
    job->out_p[job->out_n] = job->raw[best].p;
    job->out_f[job->out_n] = (uint8_t)(lf | (rev ? GM_NAV_REVERSE : 0));
    job->out_h[job->out_n] = rev ? H : nav_dir(job->raw[a].p, job->raw[best].p);
    for (int k = 0; k < job->via_count; ++k)
        if (job->via_raw[k] > a && job->via_raw[k] <= best)
            job->out_via[k] = (short)job->out_n;
    ++job->out_n;
}

static void nav_job_stage_shortcut(gm_nav_t *nav, nav_job_t *job)
{
    if (!job->sc_init) {
        nav_collapse(nav, job);
#ifdef GM_NAV_DEBUG
        fprintf(stderr, "nav: veh %llu raw path %d:", (unsigned long long)job->veh.id, job->raw_count);
        for (int i = 0; i < job->raw_count && i < 40; ++i)
            fprintf(stderr, " (%.1f,%.1f%s%s%s)", job->raw[i].p.x, job->raw[i].p.z, job->raw[i].rev ? " R" : "",
                    job->raw[i].tight ? " T" : "", job->raw[i].ptight ? " P" : "");
        fprintf(stderr, "\n");
#endif
        job->sc_init = 1;
        job->sc_a = 0;
        job->sc_h = job->start_h;
        job->sc_j = 2;
        job->sc_best = -1;
        job->out_n = 0;
        job->start_pflags = 0;
        for (int k = 0; k < job->via_count; ++k)
            job->out_via[k] = (short)(job->via_raw[k] == 0 ? 0 : -1);
    }
    const int m = job->raw_count - 1;
    while (job->sc_a < m) {
        if (nav->left < NAV_OP)
            return;
        const int a = job->sc_a;
        uint8_t pf = 0, lf = 0;
        if (job->sc_j > 0 && nav_candidate_ok(job, a, job->sc_j)) {
            if (nav_shortcut_test(nav, job, a, job->sc_h, job->sc_j, &pf, &lf)) {
                job->sc_best = job->sc_j++;
                job->sc_bpf = pf;
                job->sc_blf = lf;
            } else {
                job->sc_j = 0;
            }
            continue;
        }
        if (job->sc_best < 0 && a < job->free_prefix) {
            /* Checked when it was found, against the bare hull and the rule
             * the local layer drives by: from a pose inside the margin the
             * margin checks below would refuse it. */
            pf = 0;
            lf = GM_NAV_TIGHT;
            job->sc_best = a + 1;
            job->sc_bpf = pf;
            job->sc_blf = lf;
        } else if (job->sc_best < 0) {
            const gm_point_t P = job->raw[a].p, T = job->raw[a + 1].p;
            enum gm_geometry r;
            if (job->raw[a + 1].rev) {
                r = nav_trans(nav, job, P, T, job->sc_h, &lf);
            } else {
                r = nav_pivot(nav, job, P, job->sc_h, nav_dir(P, T), &pf);
                if (r != GM_BLOCKED)
                    r = nav_trans(nav, job, P, T, nav_dir(P, T), &lf);
            }
            if (r == GM_BLOCKED) {
                ++job->bad_pivots;
                lf |= GM_NAV_TIGHT;
            }
            job->sc_best = a + 1;
            job->sc_bpf = pf;
            job->sc_blf = lf;
        }
        nav_emit(job, a, job->sc_best, job->sc_h, job->sc_bpf, job->sc_blf);
        job->sc_h = job->out_h[job->out_n - 1];
        job->sc_a = job->sc_best;
        job->sc_j = job->sc_a + 2;
        job->sc_best = -1;
    }
    if (job->out_n && job->tiers && (job->start_pflags & GM_NAV_TIGHT) && !(job->out_f[0] & GM_NAV_TIGHT)
        && job->out_n < NAV_RAW - 1) {
        /* the pivot at the start pose has no vertex of its own: isolate it into a
         * short first leg (one hull length) so the rest of leg 0 stays non-TIGHT */
        const gm_point_t a = job->start, b = job->out_p[0];
        const float len = nav_dist(a, b), cut = job->veh.length;
        if (len > cut + 0.5f) {
            memmove(job->out_p + 1, job->out_p, sizeof(gm_point_t) * (size_t)job->out_n);
            memmove(job->out_f + 1, job->out_f, (size_t)job->out_n);
            memmove(job->out_h + 1, job->out_h, sizeof(float) * (size_t)job->out_n);
            job->out_p[0].x = a.x + (b.x - a.x) * cut / len;
            job->out_p[0].z = a.z + (b.z - a.z) * cut / len;
            ++job->out_n;
            for (int k = 0; k < job->via_count; ++k)
                if (job->out_via[k] >= 0 && job->via_raw[k] != 0)
                    ++job->out_via[k];
        }
    }
    if (job->out_n)
        job->out_f[0] |= job->start_pflags;
    job->stage = J_PUBLISH;
}

/* ------------------------------------------------------------ publishing */

static void nav_fill_remaining(gm_nav_route_t *route)
{
    float acc = 0.0f;
    for (int i = route->count - 1; i >= 0; --i) {
        route->remaining[i] = acc;
        if (i > 0)
            acc += nav_dist(route->points[i - 1], route->points[i]);
    }
    route->length = acc + (route->count ? nav_dist(route->origin, route->points[0]) : 0.0f);
}

static void nav_publish(gm_nav_t *nav, nav_job_t *job)
{
    nav_slot_t *slot = &nav->slots[job->slot];
    gm_nav_route_t *route = &slot->route;
    /* assemble: kept part of an old route (extension) + new vertices */
    int keep_from = 0, keep = 0;
    gm_point_t origin = job->start;
    if (job->extend) {
        const gm_point_t pos = {slot->vehicle.x, slot->vehicle.z};
        float best = INFINITY;
        for (int i = 0; i < route->count; ++i) {
            const gm_point_t a = i ? route->points[i - 1] : route->origin;
            const float d = nav_segdist(pos, a, route->points[i]);
            if (d < best)
                best = d, keep_from = i;
        }
        origin = keep_from ? route->points[keep_from - 1] : route->origin;
        keep = route->count - keep_from;
    }
    static const int cap = GM_NAV_ROUTE_POINTS;
    const int total = keep + job->out_n;
    int cut = total;
    int partial = 0;
    if (total > cap) {
        partial = 1;
        cut = cap;
        int boundary = -1;
        for (int k = 0; k < job->via_count; ++k) {
            const int v = job->out_via[k] >= 0 ? job->out_via[k] + keep : -1;
            if (v >= 0 && v < cap && v + 1 > boundary)
                boundary = v + 1;
        }
        if (boundary > keep)
            cut = boundary;
    }
    gm_nav_route_t *np = &nav->scratch; /* heap: keep the DLL thread stack small */
    memset(np, 0, sizeof *np);
    np->status = GM_NAV_READY;
    np->revision = route->revision + 1;
    np->route_key = job->route_key;
    np->origin = origin;
    np->count = cut;
    for (int i = 0; i < cut; ++i) {
        if (i < keep) {
            np->points[i] = route->points[keep_from + i];
            np->flags[i] = route->flags[keep_from + i];
        } else {
            np->points[i] = job->out_p[i - keep];
            np->flags[i] = job->out_f[i - keep];
        }
    }
    for (int k = 0; k < NAV_VIA; ++k)
        np->via_vertex[k] = -1;
    if (job->extend) {
        np->via_count = route->via_count;
        for (int k = 0; k < job->via_base && k < NAV_VIA; ++k)
            np->via_vertex[k] = (short)(route->via_vertex[k] >= 0
                                             ? (route->via_vertex[k] - keep_from >= 0 ? route->via_vertex[k] - keep_from : 0)
                                             : -1);
    } else {
        np->via_count = job->via_count;
    }
    int last_via = -1;
    for (int k = 0; k < job->via_count; ++k) {
        const int gk = job->via_base + k;
        if (gk >= NAV_VIA)
            break;
        const int v = job->out_via[k] >= 0 ? job->out_via[k] + keep : -1;
        if (v >= 0 && v < cut) {
            np->via_vertex[gk] = (short)v;
            last_via = k;
        }
    }
    if (!partial)
        last_via = job->via_count - 1;
    np->partial = partial;
    np->goal_index = partial ? -1 : job->goal_index;
    nav_fill_remaining(np);
    if (partial) {
        /* vias after the last honoured one inside the cut stay for the extension */
        slot->ext_via_count = 0;
        for (int k = last_via + 1; k < job->via_count; ++k)
            slot->ext_via[slot->ext_via_count++] = job->via[k];
        slot->ext_via_base = job->via_base + last_via + 1;
        slot->ext_heading = cut - keep - 1 >= 0 ? job->out_h[cut - keep - 1] : job->start_h;
    }
    *route = *np;
    slot->published = 1;
}

static void nav_job_finish(gm_nav_t *nav, nav_job_t *job, int ok)
{
    nav_slot_t *slot = &nav->slots[job->slot];
    nav->share_group = 0; /* s24: the spread price belongs to this search only */
    if (ok) {
        nav_publish(nav, job);
        if (slot->group >= 1 && slot->group <= 2 && slot->share_k > 0.0f && slot->route.count > 0) {
            /* s24 spread: stamped once, as published; taken off when its next search starts or it is forgotten */
            nav_share_unstamp(nav, slot);
            slot->stamp_pts[0] = slot->route.origin;
            memcpy(slot->stamp_pts + 1, slot->route.points, sizeof(gm_point_t) * (size_t)slot->route.count);
            slot->stamp_n = slot->route.count + 1;
            slot->stamp_group = slot->group;
            nav_share_apply(nav, slot->stamp_group, slot->stamp_pts, slot->stamp_n, 1);
        }
    } else {
        slot->route.status = GM_NAV_UNREACHABLE;
        slot->route.count = 0;
        slot->route.length = 0.0f;
        slot->route.partial = 0;
        slot->route.goal_index = -1;
    }
    slot->need = NEED_NONE;
    job->active = 0;
}

static void nav_job_start(gm_nav_t *nav, int index)
{
    nav_slot_t *slot = &nav->slots[index];
    nav_job_t *job = nav->job;
    const int extend = slot->need == NEED_EXTEND;
    memset(job, 0, offsetof(nav_job_t, raw));
    ++nav->stats.jobs;
    job->raw_count = 0;
    job->overflow = 0;
    job->sc_init = 0;
    job->out_n = 0;
    job->bad_pivots = 0;
    job->total_expansions = 0;
    job->sweeps = 0;
    job->active = 1;
    job->slot = index;
    job->extend = extend;
    /* s24 spread: its own old route no longer counts against it; the others of its side do */
    nav_share_unstamp(nav, slot);
    nav->share_group = slot->group >= 1 && slot->group <= 2 && slot->share_k > 0.0f && nav->share[slot->group - 1]
                           ? slot->group : 0;
    nav->share_k = slot->share_k;
    job->route_key = slot->route.route_key;
    job->veh = slot->vehicle;
    job->margin = slot->margin;
    job->body_pad = slot->body_pad;
    job->fp = job->veh;
    job->fp.length += 2.0f * job->margin;
    job->fp.width += 2.0f * job->margin;
    job->tiers = slot->comfort > job->margin;
    job->comfort = job->tiers ? slot->comfort : job->margin;
    job->slack = job->tiers ? slot->slack : 0.0f;
    job->fpc = job->veh;
    job->fpc.length += 2.0f * job->comfort;
    job->fpc.width += 2.0f * job->comfort;
    job->fpp = job->veh;
    job->fpp.length += 2.0f * (job->comfort + job->slack);
    job->fpp.width += 2.0f * (job->comfort + job->slack);
    job->cls = nav_class(nav, &job->veh, job->margin, job->comfort, job->slack);
    nav_dims(&job->veh, job->dims);
    /* free-space proofs establish the comfort tier for translations and rotations */
    job->rm = nav_circum(&job->veh, job->comfort + job->slack);
    if (extend) {
        job->via_count = slot->ext_via_count;
        memcpy(job->via, slot->ext_via, sizeof(gm_point_t) * (size_t)job->via_count);
        job->via_base = slot->ext_via_base;
        job->start = slot->route.points[slot->route.count - 1];
        job->start_h = slot->ext_heading;
    } else {
        job->via_count = slot->via_count;
        memcpy(job->via, slot->via, sizeof(gm_point_t) * (size_t)job->via_count);
        job->via_base = 0;
        job->start.x = slot->vehicle.x;
        job->start.z = slot->vehicle.z;
        job->start_h = slot->vehicle.heading;
    }
    for (int k = 0; k < job->via_count; ++k)
        job->via_raw[k] = -2;
    job->goal_count = slot->goal_count;
    memcpy(job->goals, slot->goals, sizeof(gm_point_t) * (size_t)job->goal_count);
    memcpy(job->goal_cost, slot->goal_cost, sizeof(float) * (size_t)job->goal_count);
    job->goal_radius = slot->goal_radius;
    job->goal_reff = job->goal_radius > 0.75f * nav->s ? job->goal_radius : 0.75f * nav->s;
    job->goal_index = -1;
    job->cur = job->start;
    job->cur_h = job->start_h;
    job->cur_lat = 0;
    job->first_leg = !extend;
    job->escape = 0;
    job->precheck_i = -1;
    nav_raw_push(job, job->start, 0, 0, 0);
    job->stage = J_PRECHECK;
}

/* The hull starts inside the search margin of an obstacle: parked at a post
 * beside a building, nose against a wall after a pivot crept it there. The
 * search's own escape from such a pose only accepts moves that raise the
 * clearance at the rotation centre, and it has no escape for turns: straight
 * back from a wall works only when no forced pivot seed was found first, and
 * on along a wall the centre clearance stays the same, so the search found
 * nothing and the vehicle stood with an unreachable route. Here the short way
 * out is looked for directly, by the rule the local layer drives by (the bare
 * hull may leave obstacle cells, never enter one; bodies and vetoes as
 * everywhere): straight back, straight on, or a gentle curve either way, up
 * to NAV_FREE_REACH, until the hull is clear at the search margin. The
 * shortest one (towards the first target on a tie) becomes the start of the
 * route; the search starts where it ends. */
static int nav_free_hull(gm_nav_t *nav, nav_job_t *job)
{
    gm_point_t target = job->hc;
    if (job->via_count)
        target = job->via[0];
    gm_point_t best_pts[8];
    int best_n = 0, best_rev = 0;
    float best_score = INFINITY, best_h = job->start_h;
    const int steps = (int)(NAV_FREE_REACH / NAV_FREE_STEP);
    for (int option = 0; option < 4; ++option) {
        const int rev = option == 0;
        const float turn = option == 2 ? NAV_FREE_TURN : option == 3 ? -NAV_FREE_TURN : 0.0f;
        const float way = rev ? -1.0f : 1.0f;
        gm_pose_t pose = nav_pose(job->start, job->start_h);
        int allowed = gm_nav_overlap(nav, &job->veh, &pose, NAV_FREE_SKIN);
        gm_point_t pts[8];
        int n = 0, done = 0, k = 1;
        for (; k <= steps && !done; ++k) {
            gm_pose_t next = pose;
            const float mid = pose.heading + 0.5f * turn;
            next.heading = pose.heading + turn;
            next.x += way * cosf(mid) * NAV_FREE_STEP;
            next.z += way * sinf(mid) * NAV_FREE_STEP;
            if (nav_vetoed(nav, job, &pose, &next) || nav_body_hit(nav, job, &pose, &next, NULL))
                break;
            const int overlap = gm_nav_overlap(nav, &job->veh, &next, NAV_FREE_SKIN);
            if (overlap > allowed)
                break;
            allowed = overlap;
            pose = next;
            done = nav_motion(nav, job, &pose, &pose, NULL) != GM_BLOCKED;
            /* A curve is published as short straight legs, one per 2 m. */
            if (turn != 0.0f && !done && k % 4 == 0 && n < 7)
                pts[n++] = (gm_point_t){pose.x, pose.z};
        }
        if (!done)
            continue;
        pts[n++] = (gm_point_t){pose.x, pose.z};
        const float score = (float)(k - 1) * NAV_FREE_STEP + 0.1f * nav_dist(pts[n - 1], target);
        if (score < best_score) {
            best_score = score;
            best_n = n;
            best_rev = rev;
            best_h = pose.heading;
            memcpy(best_pts, pts, sizeof(gm_point_t) * (size_t)n);
        }
    }
    if (!best_n)
        return 0;
    for (int i = 0; i < best_n; ++i)
        if (!nav_raw_push(job, best_pts[i], best_rev, 1, 0))
            return 0;
    job->free_prefix = job->raw_count - 1;
    job->cur = best_pts[best_n - 1];
    job->cur_h = job->free_h = nav_angle(best_h);
    job->cur_lat = 0;
    return 1;
}

static void nav_job_stage_precheck(gm_nav_t *nav, nav_job_t *job)
{
    const float xmax = nav->x0 + (float)nav->cols * nav->step, zmax = nav->z0 + (float)nav->rows * nav->step;
    if (job->precheck_i < 0) {
        if (nav->left < NAV_OP)
            return;
        {
            const gm_pose_t pose = nav_pose(job->cur, job->cur_h);
            if (job->first_leg && nav_body_overlap_real(nav, job, &pose)) {
                job->stage = J_FAIL; /* physically overlaps a body: no escape */
                return;
            }
            job->escape = job->first_leg && nav_motion(nav, job, &pose, &pose, NULL) == GM_BLOCKED;
            job->escape0 = job->escape;
        }
        job->start_comps = nav_comps_near(nav, job->cur);
        job->precheck_i = 0;
    }
    while (job->precheck_i < job->goal_count) {
        const int i = job->precheck_i;
        const gm_point_t G = job->goals[i];
        const nav_comps_t there = nav_comps_near(nav, G);
        job->goal_ok[i] = G.x >= nav->x0 && G.z >= nav->z0 && G.x <= xmax && G.z <= zmax && isfinite(job->goal_cost[i])
                          && (job->goal_radius > 2.0f * nav->s || nav_comps_meet(&job->start_comps, &there));
        if (job->goal_ok[i] && job->goal_radius <= 2.0f * nav->s) {
            if (nav->left < NAV_OP)
                return;
            int any = 0;
            for (int h = 0; h < 8 && !any; ++h) {
                const gm_pose_t pose = nav_pose(G, nav_hangle(h));
                any = nav_motion(nav, job, &pose, &pose, NULL) != GM_BLOCKED;
            }
            job->goal_ok[i] = (uint8_t)any;
        }
        ++job->precheck_i;
    }
    /* admissible goal-set heuristic: bounding circle of the usable goals */
    int n = 0;
    float minx = 0, maxx = 0, minz = 0, maxz = 0, minc = 0;
    for (int i = 0; i < job->goal_count; ++i) {
        if (!job->goal_ok[i])
            continue;
        const gm_point_t G = job->goals[i];
        if (!n || G.x < minx) minx = G.x;
        if (!n || G.x > maxx) maxx = G.x;
        if (!n || G.z < minz) minz = G.z;
        if (!n || G.z > maxz) maxz = G.z;
        if (!n || job->goal_cost[i] < minc) minc = job->goal_cost[i];
        ++n;
    }
    if (!n) {
        job->stage = J_FAIL;
        return;
    }
    job->hc.x = 0.5f * (minx + maxx);
    job->hc.z = 0.5f * (minz + maxz);
    job->hr = 0.0f;
    for (int i = 0; i < job->goal_count; ++i)
        if (job->goal_ok[i]) {
            const float d = nav_dist(job->hc, job->goals[i]) + job->goal_reff;
            if (d > job->hr)
                job->hr = d;
        }
    job->hmin = minc;
    if (job->escape && !job->free_tried) {
        if (nav->left < 4 * NAV_OP)
            return;
        job->free_tried = 1;
        nav->left -= 4 * NAV_OP;
        if (nav_free_hull(nav, job))
            job->escape = 0;
    }
    job->stage = J_NEXT_LEG;
}

static void nav_job_run(gm_nav_t *nav)
{
    nav_job_t *job = nav->job;
    for (;;) {
        const int stage = job->stage;
        const int left = nav->left;
        switch (stage) {
        case J_PRECHECK: nav_job_stage_precheck(nav, job); break;
        case J_NEXT_LEG: nav_job_stage_next(nav, job); break;
        case J_FIELD: nav_job_stage_field(nav, job); break;
        case J_CONNECT: nav_job_stage_connect(nav, job); break;
        case J_REVERSE: nav_job_stage_reverse(nav, job); break;
        case J_SEARCH: nav_job_stage_search(nav, job); break;
        case J_FINAL: nav_job_stage_final(nav, job); break;
        case J_SHORTCUT: nav_job_stage_shortcut(nav, job); break;
        case J_PUBLISH: nav_job_finish(nav, job, job->bad_pivots == 0); return; /* never publish an unvalidated motion */
        default: nav_job_finish(nav, job, 0); return;
        }
        if (job->stage == stage && nav->left == left)
            return; /* out of budget */
        if (nav->left < NAV_OP && job->stage != J_PUBLISH && job->stage != J_FAIL)
            return;
    }
}

/* ------------------------------------------------------------- requests */

static nav_slot_t *nav_find_slot(const gm_nav_t *nav, uint64_t id)
{
    for (int k = 0; k < NAV_SLOTS; ++k)
        if (nav->slots[k].used && nav->slots[k].id == id)
            return &nav->slots[k];
    return NULL;
}

static void nav_cancel(gm_nav_t *nav, const nav_slot_t *slot)
{
    if (nav->job->active && &nav->slots[nav->job->slot] == slot) {
        nav->job->active = 0;
        nav->share_group = 0;
    }
}

static float nav_route_to_end(const gm_nav_route_t *route, gm_point_t pos)
{
    float best = INFINITY, rest = 0.0f;
    for (int i = 0; i < route->count; ++i) {
        const gm_point_t a = i ? route->points[i - 1] : route->origin;
        const float d = nav_segdist(pos, a, route->points[i]);
        if (d < best)
            best = d, rest = nav_dist(pos, route->points[i]) + route->remaining[i];
    }
    return route->count ? rest : 0.0f;
}

void gm_nav_request(gm_nav_t *nav, const gm_nav_request_t *request)
{
    if (!nav || !nav->has_map || !request || !request->vehicle)
        return;
    const gm_vehicle_t *v = request->vehicle;
    nav_slot_t *slot = nav_find_slot(nav, v->id);
    int fresh = 0;
    if (!slot) {
        for (int k = 0; k < NAV_SLOTS && !slot; ++k)
            if (!nav->slots[k].used)
                slot = &nav->slots[k];
        if (!slot)
            return;
        memset(slot, 0, sizeof *slot);
        slot->used = 1;
        slot->id = v->id;
        fresh = 1;
    }
    slot->vehicle = *v;
    slot->vehicle.path = NULL;
    slot->vehicle.path_count = 0;
    slot->vehicle.pending_commands = NULL;
    slot->vehicle.pending_command_count = 0;
    slot->via_count = request->via && request->via_count > 0
                          ? (request->via_count < NAV_VIA ? request->via_count : NAV_VIA) : 0;
    if (slot->via_count)
        memcpy(slot->via, request->via, sizeof(gm_point_t) * (size_t)slot->via_count);
    int goals = request->goals && request->goal_count > 0 ? request->goal_count : 0;
    if (goals > GM_NAV_MAX_GOALS)
        goals = GM_NAV_MAX_GOALS;
    slot->goal_count = goals;
    for (int i = 0; i < goals; ++i) {
        slot->goals[i] = request->goals[i];
        slot->goal_cost[i] = request->goal_cost ? request->goal_cost[i] : 0.0f;
    }
    slot->goal_radius = request->goal_radius > 0.0f ? request->goal_radius : 0.0f;
    slot->margin = request->margin > 0.0f ? request->margin : 0.0f;
    slot->body_pad = request->body_clearance > 0.0f ? request->body_clearance : NAV_BODY_PAD;
    slot->comfort = request->comfort_margin > 0.0f ? request->comfort_margin : 0.0f;
    slot->slack = request->pivot_slack > 0.0f ? request->pivot_slack : 0.0f;
    slot->group = request->group;
    slot->share_k = request->share_cost > 0.0f && isfinite(request->share_cost) ? request->share_cost : 0.0f;

    gm_nav_route_t *route = &slot->route;
    const int pending = nav->grid_ready ? GM_NAV_PENDING : GM_NAV_BUILDING;
    const int job_here = nav->job->active && &nav->slots[nav->job->slot] == slot;
    const int idle = slot->need == NEED_NONE;
    if (fresh || route->route_key != request->route_key) {
        nav_cancel(nav, slot);
        const unsigned revision = route->revision;
        memset(route, 0, sizeof *route);
        route->revision = revision;
        route->route_key = request->route_key;
        route->goal_index = -1;
        route->status = (enum gm_nav_status)pending;
        slot->published = 0;
        slot->need = NEED_FULL;
    } else if (request->replan) {
        if (job_here && nav->job->extend)
            nav_cancel(nav, slot);
        if (!(job_here && !nav->job->extend))
            slot->need = NEED_FULL;
        if (route->status != GM_NAV_READY)
            route->status = (enum gm_nav_status)pending;
    } else if (route->status == GM_NAV_READY && route->partial && slot->need == NEED_NONE && !job_here) {
        const gm_point_t pos = {v->x, v->z};
        if (nav_route_to_end(route, pos) < NAV_EXTEND_DISTANCE)
            slot->need = NEED_EXTEND;
    }
    if (!goals && slot->need != NEED_NONE) {
        nav_cancel(nav, slot);
        nav_share_unstamp(nav, slot);
        slot->need = NEED_NONE;
        route->status = GM_NAV_UNREACHABLE;
        route->count = 0;
    }
    if (idle && slot->need != NEED_NONE)
        slot->queued_tick = nav->tick;
}

/* Straight distance from a slot's vehicle to what it asks for: the queue
 * serves the shortest first, so one far or hopeless request does not hold
 * up all the near ones. */
static float nav_queue_key(const gm_nav_t *nav, const nav_slot_t *slot)
{
    const gm_point_t p = {slot->vehicle.x, slot->vehicle.z};
    float d = INFINITY;
    if (slot->via_count)
        d = nav_dist(p, slot->via[0]);
    else
        for (int i = 0; i < slot->goal_count; ++i) {
            const float g = nav_dist(p, slot->goals[i]);
            if (g < d)
                d = g;
        }
    if (!isfinite(d))
        d = 0.0f;
    return d - NAV_AGE_M * (float)(nav->tick - slot->queued_tick);
}

void gm_nav_forget(gm_nav_t *nav, uint64_t vehicle_id)
{
    if (!nav || !nav->has_map)
        return;
    nav_slot_t *slot = nav_find_slot(nav, vehicle_id);
    if (!slot)
        return;
    nav_cancel(nav, slot);
    nav_share_unstamp(nav, slot);
    memset(slot, 0, sizeof *slot);
}

/* Vehicles without any route yet first, and among them (and then among the
 * rest) the shortest request, aged by its waiting time. */
static int nav_pick(gm_nav_t *nav)
{
    for (int pass = 0; pass < 2; ++pass) {
        int best = -1;
        float best_key = INFINITY;
        for (int k = 0; k < NAV_SLOTS; ++k) {
            const int i = (nav->rr + k) % NAV_SLOTS;
            const nav_slot_t *slot = &nav->slots[i];
            if (!slot->used || !slot->need || (pass == 0 && slot->published))
                continue;
            const float key = nav_queue_key(nav, slot);
            if (key < best_key) {
                best_key = key;
                best = i;
            }
        }
        if (best >= 0)
            return best;
    }
    return -1;
}

int gm_nav_step(gm_nav_t *nav, gm_static_sweep_fn sweep, void *context, int work_units)
{
    if (!nav || !nav->has_map || work_units <= 0)
        return 0;
    nav->left = work_units;
    nav->sweep = sweep;
    nav->ctx = context;
    if (!nav->grid_ready) {
        nav_build_step(nav);
        if (!nav->grid_ready)
            return work_units - nav->left;
    }
    while (nav->left >= NAV_OP) {
        if (!nav->job->active) {
            const int i = nav_pick(nav);
            if (i < 0)
                break;
            nav->rr = (i + 1) % NAV_SLOTS;
            nav_job_start(nav, i);
        }
        const int before = nav->left;
        nav_job_run(nav);
        if (nav->job->active && nav->left == before)
            break;
    }
    nav->sweep = NULL;
    nav->ctx = NULL;
    return work_units - nav->left;
}

const gm_nav_route_t *gm_nav_route(const gm_nav_t *nav, uint64_t vehicle_id)
{
    if (!nav || !nav->has_map)
        return NULL;
    const nav_slot_t *slot = nav_find_slot(nav, vehicle_id);
    return slot ? &slot->route : NULL;
}

void gm_nav_stats(const gm_nav_t *nav, gm_nav_stats_t *out)
{
    if (!out)
        return;
    if (nav)
        *out = nav->stats;
    else
        memset(out, 0, sizeof *out);
}
