/* Layout-specific 2 m obstacle samples and local native collision evidence.
 * G32 and G10 occupancy flags are planning data. They have no wall semantics
 * here. All callers under the ground lock see the same evidence cache. */
#include "ground_terrain.h"

enum geometry_source { GEO_MISSING, GEO_SCAN_2M, GEO_LIVE };
typedef struct {
    enum nav_geometry state;
    enum geometry_source source;
} geometry_evidence_t;

#define GEO_SCAN_KNOWN 0x8000u
#define GEO_SCAN_VERTICAL 0x073fu
#define GEO_SCAN_FIXED 0x0200u
#define GEO_SCAN_X_EDGE 0x0040u
#define GEO_SCAN_Z_EDGE 0x0080u
#define GEO_LIVE_CACHE 8192u
#define GEO_LIVE_RAYS 10
#define GEO_PLANNING_RAYS 80
#define GEO_SLICE_MS 20
#define GEO_PLANNING_US 170
#define GEO_PLANNING_TILES 6
#define GEO_LIVE_REACH_M 52.0f
#define GEO_STATIONARY_CACHE 2048u
#define GEO_STATIONARY_RAYS 240
#define GEO_SUPPORT_GRADE 0.4877f   /* 26-degree route-design evidence limit */

typedef struct {
    int x, z;
    uint64_t valid_until_ms;
    float ground;
    float hit[GEO_LIVE_RAYS - 1];
    int supported;
} geometry_tile_t;

static struct {
    const u8 *scan;
    u8 *host, *game;
    uint32_t nx, nz;
    float x0, z0, step;
    int opened;
    uint32_t scan_generation; /* +1 whenever geometry.scan is mapped or unmapped (map_epoch) */
    uint64_t ms, slice_ms, native_retry_ms;
    int planning_budget, focus_budget, focused, has_tiles;
    float focus_x, focus_y, focus_z;
    long long planning_ticks_left, planning_ticks;
} geometry;
static geometry_tile_t geometry_live[GEO_LIVE_CACHE];

typedef struct {
    uint64_t until_ms;
    float x, y, z, fx, fz, length, width;
    enum nav_geometry evidence;
} geometry_stationary_t;

static geometry_stationary_t geometry_stationary_cache[GEO_STATIONARY_CACHE];
static int geometry_stationary_rays;
static long long geometry_stationary_deadline;


/* ---- Terrain layer (ground_terrain.h): water and slope from the level's
 * decoded heightfield, bot_trees/terrain/<mission>.bin. No file: v11
 * behaviour. */
static struct {
    const u8 *base;
    gt_terrain_t t;
    int opened;
    /* The route layer asks for a cell's state and then its cost: one judgement. */
    int cache_valid, cache_c, cache_r, cache_closed;
    float cache_cost;
    uint32_t cache_generation;
} terrain;

/* bot_trees/terrain/<mission>.bin beside bot_trees/obstacles: derived from
 * the scan path the launcher configures (geometry_file, or scan_file while
 * the scan is still to be made). */
static void terrain_open(void *host)
{
    if (terrain.opened)
        return;
    terrain.opened = 1;
    const char *from = cfg.geometry_file[0] ? cfg.geometry_file : cfg.scan_file;
    if (!from[0] || !cfg.mission[0])
        return;
    char path[MAX_PATH];
    size_t n = strlen(from);
    int cuts = 0;
    while (n > 0 && cuts < 2) {
        --n;
        if (from[n] == '/' || from[n] == '\\')
            ++cuts;
    }
    if (cuts < 2 || n + 9 + strlen(cfg.mission) + 5 >= sizeof path)
        return;
    memcpy(path, from, n + 1);
    path[n + 1] = 0;
    strcat(path, "terrain\\");
    const size_t at = strlen(path);
    for (size_t i = 0; cfg.mission[i] && at + i + 1 < sizeof path; ++i) {
        path[at + i] = (char)tolower((unsigned char)cfg.mission[i]);
        path[at + i + 1] = 0;
    }
    strcat(path, ".bin");
    void *file = gh_file_open(path);
    if (gh_file_failed(file)) {
        event(host, "terrain: layout %s has no terrain file (%s): no water or slope rule", cfg.mission, path);
        return;
    }
    long long size;
    void *map = gh_file_size(file, &size) && size >= 48 && size <= 200000000
                 ? gh_file_map(file) : NULL;
    gh_handle_close(file);
    if (!map)
        return;
    const u8 *data = (const u8 *)gh_file_view(map);
    gh_handle_close(map);
    if (!data)
        return;
    if (!gt_parse(&terrain.t, data, (uint64_t)size)) {
        gh_file_unview(data);
        event(host, "terrain: layout %s terrain file is not valid WTTER001", cfg.mission);
        return;
    }
    terrain.base = data;
    ++geometry.scan_generation; /* the cells behind the map changed: the route layer rebuilds */
    const gt_terrain_t *t = &terrain.t;
    event(host, "terrain: layout %s %u x %u samples at %.1f m from (%.0f,%.0f), water %s%.1f m, decks %s, "
          "slope limit %.0f%%, wading %.1f m", cfg.mission, t->nx, t->nz, t->step, t->x0, t->z0,
          isfinite(t->water) ? "" : "none ", isfinite(t->water) ? t->water : 0.0f, t->flags ? "flagged" : "none",
          100.0f * GT_SLOPE_HARD, GT_WADE_M);
}

/* ---- Clutter table (v13 spec section 3): how built-up the ground is, per
 * 4 m cell, for the route layer's cost. bot_trees/clutter/<mission>.bin
 * (clutter_file, written by ground_brain.py beside geometry_file). Little
 * endian WTCLT001: 0 magic, 8 u32 nx, 12 u32 nz, 16 f32 x0, 20 f32 z0,
 * 24 f32 step (4), 28 u8[nz][nx] (row = +Z), value = round(255 * clutter),
 * road-corridor discount applied, 255 also on deep water (the terrain layer
 * closes deep water cells anyway). No file, a bad file or clutter_k <= 0:
 * no clutter cost, as before. Mapped once for the process, like the terrain. */
static struct {
    const u8 *base;
    uint32_t nx, nz;
    float x0, z0, step;
    int opened;
} clutter;

static void clutter_open(void *host)
{
    if (clutter.opened)
        return;
    clutter.opened = 1;
    if (!(cfg.clutter_k > 0.0f) || !isfinite(cfg.clutter_k)) {
        event(host, "clutter: clutter_k %.2f: no clutter cost", cfg.clutter_k);
        return;
    }
    if (!cfg.clutter_file[0]) {
        event(host, "clutter: layout %s has no clutter table configured: no clutter cost", cfg.mission);
        return;
    }
    void *file = gh_file_open(cfg.clutter_file);
    if (gh_file_failed(file)) {
        event(host, "clutter: layout %s has no clutter table (%s): no clutter cost", cfg.mission, cfg.clutter_file);
        return;
    }
    long long size;
    void *map = gh_file_size(file, &size) && size > 28 && size <= 100000028
                 ? gh_file_map(file) : NULL;
    gh_handle_close(file);
    const u8 *data = map ? (const u8 *)gh_file_view(map) : NULL;
    if (map)
        gh_handle_close(map);
    const uint32_t nx = data ? (*(uint32_t *)((u8 *)(data) + (8))) : 0, nz = data ? (*(uint32_t *)((u8 *)(data) + (12))) : 0;
    const float x0 = data ? (*(float *)((u8 *)(data) + (16))) : NAN, z0 = data ? (*(float *)((u8 *)(data) + (20))) : NAN;
    const float step = data ? (*(float *)((u8 *)(data) + (24))) : 0.0f;
    if (!data || memcmp(data, "WTCLT001", 8) || !nx || !nz || nx > 20000 || nz > 20000
        || (uint64_t)nx * nz + 28 != (uint64_t)size || !isfinite(x0) || !isfinite(z0)
        || fabsf(x0) > 100000.0f || fabsf(z0) > 100000.0f || step != 4.0f) {
        if (data)
            gh_file_unview(data);
        event(host, "clutter: layout %s clutter table %s is not valid WTCLT001: no clutter cost", cfg.mission,
              cfg.clutter_file);
        return;
    }
    clutter.base = data;
    clutter.nx = nx;
    clutter.nz = nz;
    clutter.x0 = x0;
    clutter.z0 = z0;
    clutter.step = step;
    ++geometry.scan_generation; /* routes searched without it are searched again */
    event(host, "clutter: layout %s %u x %u cells at %.0f m from (%.0f,%.0f), route cost x(1 + %.2f clutter)",
          cfg.mission, nx, nz, step, x0, z0, cfg.clutter_k);
}

static void geometry_open_scan(void *host)
{
    geometry.opened = 1;
    terrain_open(host);
    clutter_open(host);
    if (!cfg.geometry_file[0]) {
        event(host, "geometry: layout %s uses local native queries, no 2 m scan configured", cfg.mission);
        return;
    }
    void *file = gh_file_open(cfg.geometry_file);
    if (gh_file_failed(file)) {
        event(host, "geometry: layout %s obstacle evidence could not be opened", cfg.mission);
        return;
    }
    long long size;
    void *map = gh_file_size(file, &size) && size >= 28 && size <= 40000028
                 ? gh_file_map(file) : NULL;
    gh_handle_close(file);
    if (!map)
        return;
    const u8 *data = (const u8 *)gh_file_view(map);
    gh_handle_close(map);
    if (!data)
        return;
    const uint32_t nx = (*(uint32_t *)((u8 *)(data) + (8))), nz = (*(uint32_t *)((u8 *)(data) + (12)));
    const float x0 = (*(float *)((u8 *)(data) + (16))), z0 = (*(float *)((u8 *)(data) + (20))), step = (*(float *)((u8 *)(data) + (24)));
    if (memcmp(data, "WTOBS001", 8) || !nx || !nz || nx > 10000 || nz > 10000
        || (uint64_t)nx * nz * 2 + 28 != (uint64_t)size
        || !isfinite(x0) || !isfinite(z0) || fabsf(x0) > 100000.0f || fabsf(z0) > 100000.0f || step != 2.0f) {
        gh_file_unview(data);
        event(host, "geometry: layout %s has no valid 2 m obstacle evidence", cfg.mission);
        return;
    }
    geometry.scan = data + 28;
    ++geometry.scan_generation;
    geometry.nx = nx;
    geometry.nz = nz;
    geometry.x0 = x0;
    geometry.z0 = z0;
    geometry.step = step;
    event(host, "geometry: layout %s obstacle samples %u x %u at %.0f m, native local refinement",
          cfg.mission, nx, nz, step);
}

/* The one-off scan this battle made is complete and renamed into place: the
 * route layer takes it now. It used to wait for the next battle, so the first
 * battle on a new map was played without routes (Japan, 2026-11-14). The
 * next geometry_context maps it (the terrain stays as it is). */
static void geometry_scan_completed(void *host)
{
    if (geometry.scan || !cfg.scan_file[0])
        return;
    strncpy(cfg.geometry_file, cfg.scan_file, sizeof cfg.geometry_file - 1);
    cfg.geometry_file[sizeof cfg.geometry_file - 1] = 0;
    geometry.opened = 0;
    event(host, "geometry: the scan of layout %s is complete and used from now on", cfg.mission);
}

/* WTOBS001 samples are row-major. Any positive obstacle observation
 * conservatively excludes the cell from strategic searches. Fine hull
 * sweeps still use the physical hit evidence and native refinement. */
typedef struct {
    gm_map_t map;
    uint64_t scene_epoch;
} geometry_map_view_t;

static enum gm_geometry geometry_map_cell(void *context, int column, int row)
{
    const geometry_map_view_t *view = context;
    const gm_map_t *map = &view->map;
    if (view->scene_epoch != ground_scene_epoch || !geometry.scan || !geometry.game
        || column < 0 || row < 0 || column >= map->columns || row >= map->rows)
        return GM_UNKNOWN;
    const size_t index = (size_t)row * (size_t)map->columns + (size_t)column;
    const uint16_t sample = (*(uint16_t *)((u8 *)(geometry.scan) + (index * sizeof(uint16_t))));
    if (!(sample & GEO_SCAN_KNOWN))
        return GM_UNKNOWN;
    /* What stops a tank, as far as the scan can tell: a blocked edge ray (at
     * 1.2 m: a wall, and the walls of every building), static-collision geometry,
     * or something between half a metre and 3.5 m tall under the vertical ray
     * (a barrier, a container, a low wall). A hit higher up is a tree crown, a
     * roof or something overhead; tanks pass under or through those. Counting
     * every hit as an obstacle closed the forest and gantry stretches of the
     * tactical routes; counting none (the hull sweep only calls them unknown)
     * sent routes through buildings. Checked on Spaceport run 20261114_173005:
     * 8 of 1252 tactical route segments and 5 of 7242 moving samples lie in
     * cells this calls solid, against 60 and 92 for "any hit". */
    if (gt_scan_solid(sample) > 0)
        return GM_BLOCKED;
    if (!terrain.t.h)
        return GM_CLEAR;
    /* Water and slope: the same cells for the route layer and the local
     * layer (both read this grid), so a tank never cuts across a pond its
     * route goes round, and one that stands in such a cell may leave it. */
    if (!terrain.cache_valid || terrain.cache_c != column || terrain.cache_r != row
        || terrain.cache_generation != geometry.scan_generation) {
        terrain.cache_valid = 1;
        terrain.cache_c = column;
        terrain.cache_r = row;
        terrain.cache_generation = geometry.scan_generation;
        terrain.cache_closed = gt_judge(&terrain.t, map->x0 + ((float)column + 0.5f) * map->step,
                                        map->z0 + ((float)row + 0.5f) * map->step, sample, &terrain.cache_cost, NULL, NULL);
    }
    return terrain.cache_closed ? GM_BLOCKED : GM_CLEAR;
}

/* Travel cost of a free cell (only asked for cells geometry_map_cell called clear). */
static float geometry_map_cost(void *context, int column, int row)
{
    if (geometry_map_cell(context, column, row) != GM_CLEAR || !terrain.t.h)
        return 1.0f;
    return terrain.cache_cost;
}

/* The view exists only during this scene's locked planning call. */
static const gm_map_t *geometry_map_snapshot(geometry_map_view_t *view)
{
    if (!geometry.game || !geometry.scan)
        return NULL;
    view->scene_epoch = ground_scene_epoch;
    view->map = (gm_map_t){.x0 = geometry.x0, .z0 = geometry.z0, .step = geometry.step,
        .columns = (int)geometry.nx, .rows = (int)geometry.nz, .cell = geometry_map_cell, .context = view,
        .cost = terrain.t.h ? geometry_map_cost : NULL};
    if (clutter.base) {
        view->map.clutter = clutter.base + 28;
        view->map.clutter_columns = (int)clutter.nx;
        view->map.clutter_rows = (int)clutter.nz;
        view->map.clutter_x0 = clutter.x0;
        view->map.clutter_z0 = clutter.z0;
        view->map.clutter_step = clutter.step;
        view->map.clutter_k = cfg.clutter_k;
    }
    return &view->map;
}

/* A raw scan is bound to the configured scene. A replacement game instance
 * needs fresh native evidence because this host cannot identify its layout. */
static void geometry_context(void *host, uint64_t ms)
{
    u8 *game = host ? gh_game(host) : NULL;
    if (game != geometry.game || host != geometry.host) {
        if (geometry.game) {
            if (geometry.scan) {
                gh_file_unview(geometry.scan - 28);
                geometry.scan = NULL;
                ++geometry.scan_generation;
                geometry.nx = geometry.nz = 0;
                geometry.x0 = geometry.z0 = geometry.step = 0.0f;
            }
            cfg.native_control |= cfg.lanes;
            cfg.lanes = 0;
            event(host, "geometry: game instance changed, using the native objective and fresh local evidence");
        }
        ++ground_scene_epoch;
        next_scan = 0;
        motion_reset();
        memset(geometry_live, 0, sizeof geometry_live);
        geometry.has_tiles = 0;
        memset(geometry_stationary_cache, 0, sizeof geometry_stationary_cache);
        geometry.native_retry_ms = geometry.slice_ms = 0;
    }
    geometry.host = host;
    geometry.game = game;
    geometry.ms = ms;
    geometry.focused = 0;
    if (game && !geometry.opened)
        geometry_open_scan(host);
    if (!geometry.planning_ticks) {
        long long frequency;
        if (gh_qpc_frequency(&frequency)) {
            geometry.planning_ticks = frequency * GEO_PLANNING_US / 1000000;
        }
    }
    if (ms >= geometry.slice_ms + GEO_SLICE_MS || !geometry.slice_ms) {
        geometry.slice_ms = ms;
        geometry.planning_ticks_left = geometry.planning_ticks;
        geometry.planning_budget = GEO_PLANNING_RAYS;
    }
}

static void geometry_focus(float x, float y, float z)
{
    geometry.focus_x = x;
    geometry.focus_y = y;
    geometry.focus_z = z;
    geometry.focused = isfinite(x) && isfinite(y) && isfinite(z);
    geometry.focus_budget = GEO_PLANNING_TILES;
}

static int geometry_bounds(const traffic_body_t *pose, float x0, float z0, float step,
                           int *c0, int *r0, int *c1, int *r1)
{
    const float ex = pose->length * fabsf(pose->fx) + pose->width * fabsf(pose->fz);
    const float ez = pose->length * fabsf(pose->fz) + pose->width * fabsf(pose->fx);
    if (!isfinite(ex) || !isfinite(ez) || !isfinite(pose->x) || !isfinite(pose->z)
        || ex > 32.0f || ez > 32.0f || fabsf(pose->x) > 100000.0f || fabsf(pose->z) > 100000.0f)
        return 0;
    *c0 = (int)floorf((pose->x - ex - x0) / step);
    *c1 = (int)floorf((pose->x + ex - x0) / step);
    *r0 = (int)floorf((pose->z - ez - z0) / step);
    *r1 = (int)floorf((pose->z + ez - z0) / step);
    return 1;
}

static int geometry_box_hits(const traffic_body_t *pose, float x, float z, float half)
{
    const float dx = x - pose->x, dz = z - pose->z;
    const float fx = fabsf(pose->fx), fz = fabsf(pose->fz);
    return fabsf(dx) <= pose->length * fx + pose->width * fz + half
        && fabsf(dz) <= pose->length * fz + pose->width * fx + half
        && fabsf(dx * pose->fx + dz * pose->fz) <= pose->length + half * (fx + fz)
        && fabsf(dz * pose->fx - dx * pose->fz) <= pose->width + half * (fx + fz);
}

static int geometry_clip_axis(float x, float dx, float extent, float *enter, float *exit)
{
    if (fabsf(dx) < 0.0001f)
        return fabsf(x) <= extent;
    const float a = (-extent - x) / dx, b = (extent - x) / dx;
    *enter = fmaxf(*enter, fminf(a, b));
    *exit = fminf(*exit, fmaxf(a, b));
    return *enter <= *exit;
}

static int geometry_clip_ray(const traffic_body_t *pose, float x, float z, float dx, float dz,
                             float *enter, float *exit)
{
    const float rx = x - pose->x, rz = z - pose->z;
    return geometry_clip_axis(rx * pose->fx + rz * pose->fz, dx * pose->fx + dz * pose->fz,
                              pose->length, enter, exit)
           && geometry_clip_axis(rz * pose->fx - rx * pose->fz, dz * pose->fx - dx * pose->fz,
                                 pose->width, enter, exit);
}

static enum nav_geometry geometry_scan_evidence(const traffic_body_t *pose)
{
    if (!geometry.scan || !geometry.game)
        return NAV_UNKNOWN;
    const float cs = geometry.step, half = cs * 0.5f;
    int c0, c1, r0, r1;
    if (!geometry_bounds(pose, geometry.x0, geometry.z0, cs, &c0, &r0, &c1, &r1)
        || c0 < 0 || r0 < 0 || c1 >= (int)geometry.nx || r1 >= (int)geometry.nz)
        return NAV_UNKNOWN;
    int uncertain = 0;
    for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
            const uint16_t cell = (*(uint16_t *)((u8 *)(geometry.scan) + (((size_t)r * geometry.nx + c) * 2)));
            const float x = geometry.x0 + ((float)c + 0.5f) * cs;
            const float z = geometry.z0 + ((float)r + 0.5f) * cs;
            if (!(cell & GEO_SCAN_KNOWN)) {
                uncertain |= geometry_box_hits(pose, x, z, half);
                continue;
            }
            /* The vertical sample has a known hit point. A fixed collision
             * object there can veto a candidate. The raw edge flags report
             * a hit somewhere on an interval and need native refinement. */
            if ((cell & GEO_SCAN_VERTICAL) && geometry_box_hits(pose, x, z, 0.18f)) {
                if ((cell & GEO_SCAN_FIXED) && (cell & 0x3fu) < 8u)
                    return NAV_BLOCKED;
                uncertain = 1;
            }
            float enter = 0.0f, exit = cs;
            if ((cell & GEO_SCAN_X_EDGE) && geometry_clip_ray(pose, x - half, z, 1.0f, 0.0f, &enter, &exit))
                uncertain = 1;
            enter = 0.0f;
            exit = cs;
            if ((cell & GEO_SCAN_Z_EDGE) && geometry_clip_ray(pose, x, z - half, 0.0f, 1.0f, &enter, &exit))
                uncertain = 1;
        }
    }
    return uncertain ? NAV_UNKNOWN : NAV_CLEAR;
}

static int geometry_scan_support(float x, float z)
{
    if (!strat.m || !strat.g10 || !(strat.m->g10_cell > 0.0f))
        return 0;
    const int i = cover_cell_of(x, z, NULL, NULL);
    if (i < 0 || i / strat.n10x < 1 || i / strat.n10x >= strat.n10z - 1
        || i % strat.n10x < 1 || i % strat.n10x >= strat.n10x - 1)
        return 0;
    const float centre = (*(float *)((u8 *)(strat.g10) + ((size_t)i * 8)));
    const float west = (*(float *)((u8 *)(strat.g10) + ((size_t)(i - 1) * 8)));
    const float east = (*(float *)((u8 *)(strat.g10) + ((size_t)(i + 1) * 8)));
    const float north = (*(float *)((u8 *)(strat.g10) + ((size_t)(i - strat.n10x) * 8)));
    const float south = (*(float *)((u8 *)(strat.g10) + ((size_t)(i + strat.n10x) * 8)));
    const float gx = fmaxf(fabsf(west - centre), fabsf(east - centre)) / strat.m->g10_cell;
    const float gz = fmaxf(fabsf(north - centre), fabsf(south - centre)) / strat.m->g10_cell;
    return isfinite(centre) && isfinite(west) && isfinite(east) && isfinite(north) && isfinite(south)
           && hypotf(gx, gz) <= GEO_SUPPORT_GRADE;
}

typedef struct { float x, z, dx, dz, length; } geometry_ray_t;
static const geometry_ray_t geometry_tile_rays[] = {
    {-0.5f, 0.0f, 1.0f, 0.0f, 1.0f},
    {0.0f, -0.5f, 0.0f, 1.0f, 1.0f},
    {-0.5f, -0.5f, 0.7071068f, 0.7071068f, 1.4142136f},
    {-0.5f, 0.5f, 0.7071068f, -0.7071068f, 1.4142136f},
};

static const geometry_tile_t *geometry_tile(int c, int r, int probe)
{
    const uint32_t key = ((uint32_t)c * 0x9e3779b1u) ^ ((uint32_t)r * 0x85ebca6bu);
    geometry_tile_t *tile = &geometry_live[(key ^ (key >> 16)) & (GEO_LIVE_CACHE - 1)];
    if (tile->valid_until_ms > geometry.ms && tile->x == c && tile->z == r)
        return tile;
    if (!probe || !geometry.focused || !geometry.game || geometry.ms < geometry.native_retry_ms
        || geometry.focus_budget <= 0)
        return NULL;
    const float x = (float)c + 0.5f, z = (float)r + 0.5f;
    if (dist2d(geometry.focus_x, geometry.focus_z, x, z) > GEO_LIVE_REACH_M)
        return NULL;
    if (geometry.planning_budget < GEO_LIVE_RAYS || geometry.planning_ticks_left <= 0)
        return NULL;
    --geometry.focus_budget;
    geometry.planning_budget -= GEO_LIVE_RAYS;
    geometry.has_tiles = 1;
    long long begin, end;
    gh_qpc(&begin);

    const float down[3] = {0.0f, -1.0f, 0.0f};
    float start[3] = {x, geometry.focus_y + 12.0f, z};
    float t = 24.0f;
    int kind;
    int hit = trace_with_flags(geometry.host, start, down, &t, &kind, GH_RAY_LAND_MESH);
    const int support = hit == 1 && isfinite(t) && t >= 0.0f && t < 24.0f;
    *tile = (geometry_tile_t){.x = c, .z = r, .valid_until_ms = geometry.ms + (support ? 1700 : 250),
                             .supported = support, .ground = support ? start[1] - t : 0.0f};
    if (support) {
        int slot = 0;
        for (int band = 0; band < 2 && hit >= 0; ++band) {
            for (unsigned i = 0; i < sizeof geometry_tile_rays / sizeof geometry_tile_rays[0] && hit >= 0; ++i) {
                const geometry_ray_t ray = geometry_tile_rays[i];
                const float p[3] = {x + ray.x, tile->ground + (band ? 1.35f : 0.55f), z + ray.z};
                const float dir[3] = {ray.dx, 0.0f, ray.dz};
                t = ray.length;
                hit = trace_with_flags(geometry.host, p, dir, &t, &kind, GH_RAY_STATIC_AND_OBJECTS);
                if (hit == 1 && (!isfinite(t) || t < 0.0f || t > ray.length))
                    hit = -1;
                tile->hit[slot++] = hit == 1 ? t : -1.0f;
            }
        }
        if (hit >= 0) {
            start[1] = tile->ground + 1.85f;
            t = 1.6f;
            hit = trace_with_flags(geometry.host, start, down, &t, &kind, GH_RAY_STATIC_AND_OBJECTS);
            if (hit == 1 && (!isfinite(t) || t < 0.0f || t > 1.6f))
                hit = -1;
            tile->hit[GEO_LIVE_RAYS - 2] = hit == 1 ? t : -1.0f;
        }
    }
    if (support && hit >= 0) {
        int obstacle = 0;
        for (int i = 0; i < GEO_LIVE_RAYS - 1; ++i)
            obstacle |= tile->hit[i] >= 0.0f;
        tile->valid_until_ms = geometry.ms + (obstacle ? 1000 : 4000);
    }
    if (hit < 0) {
        geometry.native_retry_ms = geometry.ms + 250;
        tile->supported = 0;
        tile->valid_until_ms = geometry.ms + 250;
    }
    gh_qpc(&end);
    const long long elapsed = end - begin;
    geometry.planning_ticks_left -= elapsed;
    return tile;
}

static geometry_evidence_t geometry_pose_check(traffic_body_t pose, int refine)
{
    if (refine) {
        pose.length += MOTION_FOOTPRINT_MARGIN;
        pose.width += MOTION_FOOTPRINT_MARGIN;
    }
    const enum nav_geometry scan = geometry_scan_evidence(&pose);
    if (scan == NAV_BLOCKED || (scan == NAV_CLEAR && geometry_scan_support(pose.x, pose.z)))
        return (geometry_evidence_t){scan, GEO_SCAN_2M};
    if (!refine && !geometry.has_tiles)
        return (geometry_evidence_t){NAV_UNKNOWN, GEO_MISSING};
    int c0, r0, c1, r1;
    if (!geometry_bounds(&pose, 0.0f, 0.0f, 1.0f, &c0, &r0, &c1, &r1))
        return (geometry_evidence_t){NAV_UNKNOWN, GEO_MISSING};
    int complete = 1, obstruction_unknown = 0, support_unknown = 0, slope_unknown = 0;
    float min_height = INFINITY, max_height = -INFINITY;
    float above[66];
    for (int c = 0; c <= c1 - c0; ++c)
        above[c] = NAN;
    for (int r = r0; r <= r1; ++r) {
        float left = NAN;
        for (int c = c0; c <= c1; ++c) {
            const float x = (float)c + 0.5f, z = (float)r + 0.5f;
            const int col = c - c0;
            const float previous_row = above[col];
            above[col] = NAN;
            if (!geometry_box_hits(&pose, x, z, 0.5f)) {
                left = NAN;
                continue;
            }
            const geometry_tile_t *tile = geometry_tile(c, r, refine);
            if (!tile || !tile->supported) {
                complete = 0;
                support_unknown |= tile != NULL;
                left = NAN;
                continue;
            }
            if ((isfinite(previous_row) && fabsf(tile->ground - previous_row) > GEO_SUPPORT_GRADE)
                || (isfinite(left) && fabsf(tile->ground - left) > GEO_SUPPORT_GRADE))
                slope_unknown = 1;
            left = above[col] = tile->ground;
            min_height = fminf(min_height, tile->ground);
            max_height = fmaxf(max_height, tile->ground);
            if (tile->hit[GEO_LIVE_RAYS - 2] >= 0.0f && geometry_box_hits(&pose, x, z, 0.0f))
                return (geometry_evidence_t){NAV_BLOCKED, GEO_LIVE};
            for (unsigned i = 0; i < sizeof geometry_tile_rays / sizeof geometry_tile_rays[0]; ++i) {
                const geometry_ray_t ray = geometry_tile_rays[i];
                float enter = 0.0f, exit = ray.length;
                if (!geometry_clip_ray(&pose, x + ray.x, z + ray.z, ray.dx, ray.dz, &enter, &exit))
                    continue;
                for (unsigned band = 0; band < 2; ++band) {
                    const float hit = tile->hit[band * (sizeof geometry_tile_rays / sizeof geometry_tile_rays[0]) + i];
                    if (hit < 0.0f || hit > exit)
                        continue;
                    if (hit >= enter)
                        return (geometry_evidence_t){NAV_BLOCKED, GEO_LIVE};
                    obstruction_unknown = 1;
                }
            }
        }
    }
    if (obstruction_unknown || support_unknown || slope_unknown)
        return (geometry_evidence_t){NAV_UNKNOWN, GEO_LIVE};
    if (complete && isfinite(min_height) && isfinite(max_height)) {
        /* Native terrain evidence cannot certify a different deck/floor. */
        const float distance = dist2d(pose.x, pose.z, geometry.focus_x, geometry.focus_z);
        const int grounded = !geometry.focused
                             || fabsf(geometry.focus_y - 0.5f * (max_height + min_height))
                                <= 3.0f + GEO_SUPPORT_GRADE * distance;
        return (geometry_evidence_t){grounded ? NAV_CLEAR : NAV_UNKNOWN, GEO_LIVE};
    }
    return (geometry_evidence_t){NAV_UNKNOWN, GEO_MISSING};
}

static void geometry_stationary_begin(void)
{
    long long now;
    gh_qpc(&now);
    geometry_stationary_deadline = now + 4 * geometry.planning_ticks;
    geometry_stationary_rays = GEO_STATIONARY_RAYS;
}

static int geometry_stationary_trace(const float start[3], const float end[3], int flags, float *range)
{
    float dx = end[0] - start[0], dy = end[1] - start[1], dz = end[2] - start[2];
    const float length = sqrtf(dx * dx + dy * dy + dz * dz);
    if (length < 0.01f || !isfinite(length))
        return -1;
    const float direction[3] = {dx / length, dy / length, dz / length};
    *range = length;
    int kind;
    --geometry_stationary_rays;
    const int hit = trace_with_flags(geometry.host, start, direction, range, &kind, flags);
    return hit < 0 || !isfinite(*range) || *range < 0.0f || *range > length ? -1 : hit;
}

/* Passage and parking discovery request stationary evidence. Refine a
 * sampled footprint once, in the central observation budget. Dynamic
 * candidate sweeps consume observed evidence without issuing native rays. */
static enum nav_geometry geometry_stationary_probe(const traffic_body_t *pose)
{
    if (!geometry.game || !geometry.host)
        return NAV_UNKNOWN;
    const int col = (int)floorf(pose->x * 2.0f), row = (int)floorf(pose->z * 2.0f);
    const unsigned key = (unsigned)col * 0x9e3779b1u ^ (unsigned)row * 0x85ebca6bu;
    geometry_stationary_t *cache = &geometry_stationary_cache[(key ^ (key >> 16)) & (GEO_STATIONARY_CACHE - 1)];
    if (cache->until_ms > geometry.ms && dist2d(pose->x, pose->z, cache->x, cache->z) < 0.08f
        && fabsf(pose->y - cache->y) < 1.0f && fabsf(pose->length - cache->length) < 0.04f
        && fabsf(pose->width - cache->width) < 0.04f
        && fabsf(pose->fx * cache->fx + pose->fz * cache->fz) > 0.999f)
        return cache->evidence;

    long long now;
    gh_qpc(&now);
    if (geometry_stationary_rays < 6 || now >= geometry_stationary_deadline)
        return NAV_UNKNOWN;

    const float span = pose->length;
    float low = INFINITY, high = -INFINITY;
    for (int k = -1; k <= 1; ++k) {
        const float x = pose->x + (float)k * span * pose->fx;
        const float z = pose->z + (float)k * span * pose->fz;
        const float start[3] = {x, pose->y + 8.0f, z};
        const float end[3] = {x, pose->y - 8.0f, z};
        float t;
        if (geometry_stationary_trace(start, end, GH_RAY_LAND_MESH, &t) != 1)
            return NAV_UNKNOWN;
        low = fminf(low, start[1] - t);
        high = fmaxf(high, start[1] - t);
    }
    if (high - low > 2.0f * span * GEO_SUPPORT_GRADE)
        return NAV_UNKNOWN;

    enum nav_geometry result = NAV_CLEAR;
    const float y = (low + high) * 0.5f + 0.85f;
    for (int k = -1; k <= 1; ++k) {
        const float side = (float)k * pose->width;
        const float start[3] = {pose->x - span * pose->fx - side * pose->fz, y,
                                pose->z - span * pose->fz + side * pose->fx};
        const float end[3] = {pose->x + span * pose->fx + side * pose->fz, y,
                              pose->z + span * pose->fz - side * pose->fx};
        float t;
        const int hit = geometry_stationary_trace(start, end, GH_RAY_STATIC_AND_OBJECTS, &t);
        if (hit != 0) {
            result = hit == 1 ? NAV_BLOCKED : NAV_UNKNOWN;
            break;
        }
    }
    *cache = (geometry_stationary_t){geometry.ms + 4000, pose->x, pose->y, pose->z, pose->fx, pose->fz,
                                    pose->length, pose->width, result};
    return result;
}

/* The connector is a geometric corridor query. At the observed position,
 * include the measured hull angle and the envelope through the entrance.
 * Native control still has to supply a feasible time-ordered turn. */
static enum nav_geometry geometry_connector(const lane_seat_t *s, float x, float z, float tx, float tz)
{
    const traffic_hull_t *hull = s ? &traffic_hulls[s - lseat] : NULL;
    const float dx = tx - x, dz = tz - z, length = hypotf(dx, dz);
    if (!isfinite(length) || length > 3500.0f)
        return NAV_UNKNOWN;
    const int current_hull = hull && hull->unit == s->unit && hull->life == s->born_ms;
    const int measured = s && dist2d(s->x, s->z, x, z) < 1.0f && hypotf(s->fx, s->fz) > 0.5f;
    const float start_angle = measured ? atan2f(s->fz, s->fx) : 0.0f;
    const float angle = length > 0.05f ? atan2f(dz, dx) : start_angle;
    const float turn = measured ? wrap180((angle - start_angle) * 57.29578f) / 57.29578f : 0.0f;
    const float entry = fminf(length, 6.0f);
    const int steps = (int)fmaxf(1.0f, ceilf(length / 0.85f));
    int unknown = 0;
    for (int k = 0; k <= steps; ++k) {
        const float f = (float)k / (float)steps;
        const float turn_f = entry > 0.05f ? fminf(1.0f, f * length / entry) : 1.0f;
        const float a = angle - (1.0f - turn_f) * turn;
        const float fx = cosf(a), fz = sinf(a);
        traffic_body_t pose = {
            .x = x + f * dx, .z = z + f * dz, .fx = fx, .fz = fz,
            .length = current_hull ? hull->length : 3.1f, .width = current_hull ? hull->width : 1.65f
        };
        if (current_hull) {
            pose.x += fx * hull->forward - fz * hull->right;
            pose.z += fz * hull->forward + fx * hull->right;
        }
        const geometry_evidence_t sample = geometry_pose_check(pose, s != NULL);
        if (sample.state == NAV_BLOCKED)
            return NAV_BLOCKED;
        if (sample.state == NAV_UNKNOWN) {
            unknown = 1;
            if (!geometry.focused || dist2d(pose.x, pose.z, geometry.focus_x, geometry.focus_z) > GEO_LIVE_REACH_M)
                return NAV_UNKNOWN;
        }
    }
    return unknown ? NAV_UNKNOWN : NAV_CLEAR;
}


static enum nav_geometry control_static_connector(const lane_seat_t *s, float x, float z, float tx, float tz)
{
    return geometry_connector(s, x, z, tx, tz);
}

static enum nav_geometry control_connector_prefix(const lane_seat_t *s, float tx, float tz)
{
    const float dx = tx - s->x, dz = tz - s->z, length = hypotf(dx, dz);
    if (!isfinite(length))
        return NAV_UNKNOWN;
    const float f = length > 8.0f ? 8.0f / length : 1.0f;
    return geometry_connector(s, s->x, s->z, s->x + f * dx, s->z + f * dz);
}
