/* s17 overhead probes and the map-wide shared occupancy cache. See
 * ground_overhead.h for the design. Self-contained like ground_lidar.c: the
 * adapter (ground_overhead_adapter.inc), the movement model and the unit test
 * (tests/overhead_unit_test.c) link this file and give it their own ray. */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ground_overhead.h"
#include "ground_host.h"   /* s49: the performance counter (gh_qpc) */
#ifndef MAX_PATH
#define MAX_PATH 260       /* the log path buffers; the Windows SDK value */
#endif

#define GO_PI 3.14159265358979f
#define GO_CELLS (GO_TILE * GO_TILE)
#define GO_NO_GROUND INT16_MIN
#define GO_INSIDE_M 0.05f          /* a hit this close to the start: the start was inside a solid */
#define GO_TOP_MAX_M 40.0f         /* highest probe start above the ground (the obstacle scan's SCAN_TOP_M) */
#define GO_PRIOR_MARGIN_M 1.5f     /* above the prior's obstacle height there */
#define GO_MISS_RETRY_MS 2000u
#define GO_IDLE_MS 500u
#define GO_RESET_M 3.0f            /* moved this far: the route and unknown sweeps start again from the hull */
#define GO_ROUTE_HALF_M 2.5f       /* route corridor half width */
#define GO_ROUTE_CELLS 2048
#define GO_ROUTE_MOVED_M 2.0f
#define GO_AHEAD_GAIN 0.4f         /* a cell straight ahead of a moving tank counts this much nearer */
#define GO_MOVING_MPS 1.0f
#define GO_BUCKETS 9               /* 8 headings (moving) + 1 all round (standing; the stale sweep) */
#define GO_BATCH_LOOKUPS 2048
#define GO_HULL_MARGIN_M 0.6f
#define GO_HULL_LEAD_S 0.2f
#define GO_NEAR_HULLS 256
#define GO_LOG_MS 2000u
#define GO_STATS_MS 60000u
#define GO_SNAP_GAP_MS 30000u
#define GO_SNAP_MAX 300
#define GO_SNAP_N 128
#define GO_SNAP_PRIOR 40
#define GO_COVER_STRIDE 3
#define GO_IDLE_END_MS 5000u
#define GO_MAX_RADIUS_CELLS 128

/* Sweep phases of a seat. */
enum { GO_PH_ROUTE, GO_PH_UNKNOWN, GO_PH_STALE, GO_PH_IDLE };

typedef struct {
    uint8_t state, count, flags, misses;
    int16_t top_cm;            /* obstacle: top above the ground; overhead: the first hit above the ground */
    int16_t ground;            /* ground height, 5 cm units; GO_NO_GROUND: unknown */
    uint32_t t;                /* ms + 1 of the last probe; 0 never */
} go_cell_t;

typedef struct {
    int32_t tx, tz;
    uint64_t used_ms;
    uint32_t known, live;
    go_cell_t c[GO_CELLS];
} go_tile_t;

typedef struct { int16_t dc, dr; } go_off_t;
typedef struct { float cx, cz, fx, fz, hl, hw; } go_box_t;   /* a hull, centred, grown by the margin */
#define GO_MAX_HULLS 256

typedef struct {
    int used, present;
    go_tank_t tank;
    int phase, route_i, disc0, disc1, bucket, nroute;
    float ox, oz, rx, rz;
    int has_route_ref;
    uint64_t idle_until_ms;
    int served;
    int32_t route_c[GO_ROUTE_CELLS][2];
    int near_n;
    int near_hull[GO_NEAR_HULLS];
    double coverage, coverage_ahead;
    uint64_t next_log_ms, last_snap_ms, probes;
    int has_snap, has_coverage;
} go_seat_t;

static go_seat_t go_seat[GO_MAX_SEATS];

static struct {
    int on, faulted, running;
    go_config_t cfg;
    char log_dir[MAX_PATH], prefix[96];
    FILE *csv, *snap;
    double freq;
    go_tile_t *tiles;
    int32_t *hash;
    uint32_t hmask;
    int ntiles;
    go_off_t *off[GO_BUCKETS];
    int noff;
    /* the tick */
    uint64_t ms;
    const go_hull_t *hulls;
    int hull_count;
    go_box_t box[GO_MAX_HULLS];
    int box_count;
    /* budget window */
    int window_open, window_capped;
    uint64_t window_start_us;
    double window_used_us, probe_ema_us;
    int rr;
    /* totals */
    go_stats_t total;
    uint64_t first_active_ms, last_active_ms, stats_ms, csv_flush_ms;
    int has_active;
    /* interval (GO_STATS_MS) */
    uint64_t i_probes, i_rays, i_windows, i_capped, i_skips, i_evict, i_obst, i_free;
    double i_ray_us, i_work_us, i_max_window_us;
    /* log interval (GO_LOG_MS) */
    uint64_t l_ms, l_probes, l_rays, l_windows, l_capped;
    double l_work_us;
    double r_probes, r_rays, r_work, r_capped;
    uint32_t snaps, csv_rows;
    int snap_rr;
} go;

/* ---- helpers ---------------------------------------------------------- */
static void go_event(const char *fmt, ...)
{
    if (!go.cfg.event)
        return;
    char msg[640];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    go.cfg.event(go.cfg.event_context, msg);
}

static double go_now(void)
{
    if (go.cfg.clock_us)
        return go.cfg.clock_us(go.cfg.clock_context);
    long long c;
    gh_qpc(&c);
    return (double)c * go.freq;
}

static int go_fdiv(int a)
{
    return a >= 0 ? a / GO_TILE : -((-a + GO_TILE - 1) / GO_TILE);
}

static float go_wrap(float a)
{
    while (a > GO_PI)
        a -= 2.0f * GO_PI;
    while (a <= -GO_PI)
        a += 2.0f * GO_PI;
    return a;
}

static int go_wc(float x)
{
    return (int)floorf(x / go.cfg.cell_m);
}

static float go_centre(int w)
{
    return ((float)w + 0.5f) * go.cfg.cell_m;
}

static uint32_t go_stamp(uint64_t ms)
{
    return (uint32_t)(ms + 1u);
}

static uint64_t go_age(const go_cell_t *c, uint64_t ms)
{
    if (!c->t)
        return UINT64_MAX;
    return (uint64_t)(uint32_t)(go_stamp(ms) - c->t);
}

static float go_ground_of(const go_cell_t *c)
{
    return (float)c->ground * 0.05f;
}

static void go_set_ground(go_cell_t *c, float y)
{
    const float q = roundf(y * 20.0f);
    c->ground = q > -32767.0f && q < 32767.0f ? (int16_t)q : GO_NO_GROUND;
}

/* ---- tiles ------------------------------------------------------------ */
static uint32_t go_hash(int tx, int tz)
{
    uint32_t h = (uint32_t)tx * 0x9e3779b1u ^ (uint32_t)tz * 0x85ebca6bu;
    h ^= h >> 15;
    h *= 0x2c1b3c6du;
    h ^= h >> 12;
    return h & go.hmask;
}

static int go_find(int tx, int tz)
{
    for (uint32_t i = go_hash(tx, tz), n = 0; n <= go.hmask; i = (i + 1) & go.hmask, ++n) {
        const int32_t k = go.hash[i];
        if (k < 0)
            return -1;
        if (go.tiles[k].tx == tx && go.tiles[k].tz == tz)
            return k;
    }
    return -1;
}

static void go_hash_insert(int k)
{
    uint32_t i = go_hash(go.tiles[k].tx, go.tiles[k].tz);
    while (go.hash[i] >= 0)
        i = (i + 1) & go.hmask;
    go.hash[i] = k;
}

/* Backward-shift deletion of tile k from the linear-probing table. */
static void go_hash_remove(int k)
{
    uint32_t i = go_hash(go.tiles[k].tx, go.tiles[k].tz);
    while (go.hash[i] != k) {
        if (go.hash[i] < 0)
            return;
        i = (i + 1) & go.hmask;
    }
    uint32_t j = i;
    for (;;) {
        go.hash[i] = -1;
        for (;;) {
            j = (j + 1) & go.hmask;
            const int32_t e = go.hash[j];
            if (e < 0)
                return;
            const uint32_t home = go_hash(go.tiles[e].tx, go.tiles[e].tz);
            /* e may move to i when its home is not cyclically in (i, j]. */
            const int stays = i <= j ? (home > i && home <= j) : (home > i || home <= j);
            if (!stays) {
                go.hash[i] = e;
                i = j;
                break;
            }
        }
    }
}

static void go_tile_reset(go_tile_t *t, int tx, int tz, uint64_t ms)
{
    t->tx = tx;
    t->tz = tz;
    t->used_ms = ms;
    t->known = 0;
    t->live = 1;
    memset(t->c, 0, sizeof t->c);
    for (int i = 0; i < GO_CELLS; ++i)
        t->c[i].ground = GO_NO_GROUND;
}

static go_tile_t *go_tile(int tx, int tz, int create)
{
    const int k = go_find(tx, tz);
    if (k >= 0) {
        go.tiles[k].used_ms = go.ms;
        return &go.tiles[k];
    }
    if (!create)
        return NULL;
    int slot;
    if (go.ntiles < go.cfg.max_tiles) {
        slot = go.ntiles++;
    } else {
        /* Evict the tile used least recently. */
        slot = 0;
        for (int i = 1; i < go.ntiles; ++i)
            if (go.tiles[i].used_ms < go.tiles[slot].used_ms)
                slot = i;
        ++go.total.evictions;
        ++go.i_evict;
        if (go.ms - go.tiles[slot].used_ms < 2000u)
            ++go.total.hot_evictions;
        go.total.cells_known -= go.tiles[slot].known;
        go_hash_remove(slot);
    }
    go_tile_reset(&go.tiles[slot], tx, tz, go.ms);
    go_hash_insert(slot);
    go.total.tiles = go.ntiles;
    if (go.ntiles > go.total.tiles_peak)
        go.total.tiles_peak = go.ntiles;
    return &go.tiles[slot];
}

static go_cell_t *go_cell_ptr(int wc, int wr, int create, go_tile_t **tile_out)
{
    const int tx = go_fdiv(wc), tz = go_fdiv(wr);
    go_tile_t *t = go_tile(tx, tz, create);
    if (tile_out)
        *tile_out = t;
    if (!t)
        return NULL;
    ++go.total.lookups;
    return &t->c[(wr - tz * GO_TILE) * GO_TILE + (wc - tx * GO_TILE)];
}

/* ---- probes ----------------------------------------------------------- */
static float go_top_at(float x, float z)
{
    float top = go.cfg.top_m;
    if (go.cfg.prior) {
        const int c = (int)floorf((x - go.cfg.prior_x0) / go.cfg.prior_step);
        const int r = (int)floorf((z - go.cfg.prior_z0) / go.cfg.prior_step);
        if (c >= 0 && r >= 0 && c < (int)go.cfg.prior_nx && r < (int)go.cfg.prior_nz) {
            const unsigned w = go.cfg.prior[(size_t)r * go.cfg.prior_nx + (size_t)c];
            if (w & 0x8000u)
                top = fmaxf(top, (float)(w & 0x3fu) * 0.5f + GO_PRIOR_MARGIN_M);
        }
    }
    return fminf(top, GO_TOP_MAX_M);
}

/* One ray straight down from p for len metres. 1 hit (*t), 0 none, -1 raised. */
static int go_ray(const float p[3], float len, int flags, float *t, int *kind, double *ray_us)
{
    static const float down[3] = {0.0f, -1.0f, 0.0f};
    *t = len;
    *kind = 0;
    const double a0 = go_now();
    const int hit = go.cfg.trace(go.cfg.trace_context, p, down, t, kind, flags);
    *ray_us += go_now() - a0;
    ++go.total.rays;
    ++go.i_rays;
    if (hit < 0)
        return -1;
    if (hit == 1 && (!isfinite(*t) || *t < 0.0f || *t > len + 0.01f)) {
        ++go.total.bad_answers;
        return 0;
    }
    return hit == 1;
}

static void go_note(go_tile_t *tile, go_cell_t *c, int state, float top, int flags, uint64_t ms)
{
    const int old = c->state;
    if (old == state) {
        if (c->count < 255)
            ++c->count;
        if (c->count >= 3)
            c->flags &= (uint8_t)~GO_F_CHANGED;
    } else {
        if (old == GO_FREE && state == GO_OBSTACLE) {
            c->flags |= GO_F_CHANGED;
            ++go.total.became_obstacle;
            ++go.i_obst;
        } else if (old == GO_OBSTACLE && state == GO_FREE) {
            c->flags |= GO_F_CHANGED;
            ++go.total.became_free;
            ++go.i_free;
        }
        if (old == GO_UNKNOWN) {
            ++tile->known;
            ++go.total.cells_known;
        } else if (state == GO_UNKNOWN) {
            --tile->known;
            --go.total.cells_known;
        }
        c->state = (uint8_t)state;
        c->count = 1;
    }
    c->flags = (uint8_t)((c->flags & GO_F_CHANGED) | (flags & ~GO_F_CHANGED));
    const float cm = roundf(top * 100.0f);
    c->top_cm = (int16_t)(cm < 0.0f ? 0.0f : cm > 32000.0f ? 32000.0f : cm);
    c->t = go_stamp(ms);
    c->misses = 0;
}

static void go_miss(go_cell_t *c, uint64_t ms)
{
    if (c->misses < 255)
        ++c->misses;
    c->flags |= GO_F_MISS;
    c->t = go_stamp(ms);
    ++go.total.misses;
}

/* What lies under an overhang whose top (the first hit) is at hy over ground
 * g: see ground_overhead.h. At most 6 rays. Returns -1 on a raised ray. */
static int go_under(go_tile_t *tile, go_cell_t *c, float x, float z, float g, float hy, uint64_t ms, double *ray_us)
{
    const float step = go.cfg.step_m, over = go.cfg.overhang_m;
    int rays = 0;
    for (int layer = 0; layer < 3; ++layer) {
        const float starts[4] = {hy - 0.25f, hy - 1.0f, hy - 2.0f, g + over};
        float last = hy;
        int outside = 0;
        for (int k = 0; k < 4 && rays < 6; ++k) {
            const float s = starts[k];
            if (s >= last || s <= g + step + 0.1f)
                continue;
            last = s;
            const float p[3] = {x, s, z};
            float t;
            int kind;
            ++rays;
            const int r = go_ray(p, s - g + 0.5f, go.cfg.flags_probe, &t, &kind, ray_us);
            if (r < 0)
                return -1;
            if (r == 1 && t < GO_INSIDE_M)
                continue;           /* still inside the overhang (or a solid under it) */
            outside = 1;
            const float y2 = s - t;
            if (r == 0 || (kind & 1) || y2 - g < step) {
                go_note(tile, c, GO_OVERHEAD, hy - g, r == 0 ? 0 : GO_F_BELOW_FREE, ms);
                return 0;
            }
            if (y2 - g < over) {
                go_note(tile, c, GO_OBSTACLE, y2 - g, 0, ms);
                return 0;
            }
            hy = y2;                /* another layer: what is under that */
            break;
        }
        if (!outside) {
            /* Inside a solid all the way down to overhang_m: a building. */
            go_note(tile, c, GO_OBSTACLE, hy - g, GO_F_INSIDE, ms);
            return 0;
        }
        if (rays >= 6)
            break;
    }
    go_note(tile, c, GO_OVERHEAD, hy - g, 0, ms);
    return 0;
}

/* Probe world cell (wc, wr) for seat s. Returns 0, or -1 on a raised ray. */
static int go_probe(const go_seat_t *s, int wc, int wr, uint64_t ms, double *ray_us)
{
    go_tile_t *tile;
    go_cell_t *c = go_cell_ptr(wc, wr, 1, &tile);
    if (!c)
        return 0;
    ++go.total.probes;
    ++go.i_probes;
    const float x = go_centre(wc), z = go_centre(wr);
    const int gknown = c->ground != GO_NO_GROUND;
    float ref, slope;
    if (gknown) {
        ref = go_ground_of(c);
        slope = 0.5f * (1.0f + (float)(c->misses < 3 ? c->misses : 3));
    } else {
        ref = s->tank.y;
        slope = 1.0f + 0.5f * hypotf(x - s->tank.x, z - s->tank.z);
        slope *= 1.0f + (float)(c->misses < 3 ? c->misses : 3);
    }
    const float top = go_top_at(x, z);
    float p[3] = {x, ref + top + slope, z};
    float len = top + 2.0f * slope + 0.5f;
    float t;
    int kind;
    int r = go_ray(p, len, go.cfg.flags_probe, &t, &kind, ray_us);
    if (r < 0)
        return -1;
    if (r == 1 && t < GO_INSIDE_M) {
        ++go.total.inside_retries;
        p[1] = ref + GO_TOP_MAX_M + slope;
        len = GO_TOP_MAX_M + 2.0f * slope + 0.5f;
        r = go_ray(p, len, go.cfg.flags_probe, &t, &kind, ray_us);
        if (r < 0)
            return -1;
        if (r == 1 && t < GO_INSIDE_M) {
            go_note(tile, c, GO_OBSTACLE, p[1] - ref, GO_F_INSIDE, ms);
            return 0;
        }
    }
    if (r == 0) {
        go_miss(c, ms);
        return 0;
    }
    const float hy = p[1] - t;
    if (kind & 1) {
        go_set_ground(c, hy);
        go_note(tile, c, GO_FREE, 0.0f, 0, ms);
        return 0;
    }
    float g;
    if (gknown) {
        g = go_ground_of(c);
    } else {
        float tb;
        int kb;
        const int rb = go_ray(p, len, go.cfg.flags_land, &tb, &kb, ray_us);
        if (rb < 0)
            return -1;
        if (rb == 0 || tb < GO_INSIDE_M) {
            go_miss(c, ms);
            return 0;
        }
        g = p[1] - tb;
        go_set_ground(c, g);
    }
    const float h = hy - g;
    if (h < go.cfg.step_m) {
        go_note(tile, c, GO_FREE, fmaxf(h, 0.0f), GO_F_FLOOR, ms);
        return 0;
    }
    if (h < go.cfg.overhang_m) {
        go_note(tile, c, GO_OBSTACLE, h, 0, ms);
        return 0;
    }
    /* Overhead: the same first hit as last time says the same thing (no
     * probes under it again). */
    if ((c->state == GO_OVERHEAD || (c->state == GO_OBSTACLE && (c->flags & GO_F_INSIDE)))
        && fabsf((float)c->top_cm * 0.01f - h) < 0.3f) {
        go_note(tile, c, c->state, h, c->flags & (GO_F_BELOW_FREE | GO_F_INSIDE), ms);
        return 0;
    }
    return go_under(tile, c, x, z, g, hy, ms, ray_us);
}

/* ---- scheduler -------------------------------------------------------- */
static int go_bucket(const go_tank_t *t)
{
    if (!(fabsf(t->speed) >= GO_MOVING_MPS))
        return GO_BUCKETS - 1;
    const int k = (int)floorf(go_wrap(t->heading) / (GO_PI / 4.0f) + 0.5f);
    return ((k % 8) + 8) % 8;
}

/* The route corridor cells: polyline from the hull through the route points,
 * GO_ROUTE_HALF_M either side, for range_m of path. */
static void go_route_cells(go_seat_t *s)
{
    s->nroute = 0;
    const go_tank_t *t = &s->tank;
    if (t->route_count <= 0)
        return;
    const float cell = go.cfg.cell_m, reach = go.cfg.range_m;
    const int lanes = (int)ceilf(GO_ROUTE_HALF_M / cell);
    float ax = t->x, az = t->z, done = 0.0f;
    int lastc = INT32_MIN, lastr = INT32_MIN;
    for (int k = 0; k < t->route_count && k < GO_ROUTE_POINTS && done < reach; ++k) {
        const float bx = t->route_x[k], bz = t->route_z[k];
        if (!isfinite(bx) || !isfinite(bz))
            break;
        const float len = hypotf(bx - ax, bz - az);
        if (len < 1e-3f)
            continue;
        const float ux = (bx - ax) / len, uz = (bz - az) / len;
        for (float d = 0.0f; d <= len && done + d <= reach; d += cell) {
            const float cx = ax + ux * d, cz = az + uz * d;
            for (int l = 0; l <= 2 * lanes; ++l) {
                const int j = (l + 1) / 2 * (l & 1 ? 1 : -1);  /* 0, +1, -1, +2, -2, ... */
                const float off = (float)j * cell;
                const int wc = go_wc(cx - uz * off), wr = go_wc(cz + ux * off);
                if (l == 0) {
                    if (wc == lastc && wr == lastr)
                        break;
                    lastc = wc;
                    lastr = wr;
                }
                if (s->nroute >= GO_ROUTE_CELLS)
                    return;
                s->route_c[s->nroute][0] = wc;
                s->route_c[s->nroute][1] = wr;
                ++s->nroute;
            }
        }
        done += len;
        ax = bx;
        az = bz;
    }
}

static void go_restart(go_seat_t *s)
{
    s->phase = GO_PH_ROUTE;
    s->route_i = s->disc0 = 0;
    s->ox = s->tank.x;
    s->oz = s->tank.z;
    s->bucket = go_bucket(&s->tank);
    s->has_route_ref = s->tank.route_count > 0;
    s->rx = s->has_route_ref ? s->tank.route_x[0] : 0.0f;
    s->rz = s->has_route_ref ? s->tank.route_z[0] : 0.0f;
    s->idle_until_ms = 0;
    go_route_cells(s);
}

static void go_near_hulls(go_seat_t *s)
{
    s->near_n = 0;
    const float reach = go.cfg.range_m + 10.0f;
    for (int i = 0; i < go.box_count && s->near_n < GO_NEAR_HULLS; ++i) {
        const go_box_t *b = &go.box[i];
        if (fabsf(b->cx - s->tank.x) <= reach && fabsf(b->cz - s->tank.z) <= reach)
            s->near_hull[s->near_n++] = i;
    }
}

static int go_in_hull(const go_seat_t *s, float x, float z)
{
    for (int k = 0; k < s->near_n; ++k) {
        const go_box_t *b = &go.box[s->near_hull[k]];
        const float dx = x - b->cx, dz = z - b->cz;
        if (fabsf(dx * b->fx + dz * b->fz) <= b->hl && fabsf(-dx * b->fz + dz * b->fx) <= b->hw)
            return 1;
    }
    return 0;
}

/* The hulls of this tick as centred boxes grown by the margin. */
static void go_prepare_hulls(void)
{
    int n = 0;
    for (int i = 0; i < go.hull_count && n < GO_MAX_HULLS; ++i) {
        const go_hull_t *h = &go.hulls[i];
        if (!isfinite(h->x) || !isfinite(h->z) || !isfinite(h->heading))
            continue;
        const float fx = cosf(h->heading), fz = sinf(h->heading);
        const float m = GO_HULL_MARGIN_M + GO_HULL_LEAD_S * (isfinite(h->speed) ? fabsf(h->speed) : 0.0f);
        go.box[n++] = (go_box_t){h->x + fx * h->forward - fz * h->right, h->z + fz * h->forward + fx * h->right, fx, fz,
                                 h->half_length + m, h->half_width + m};
    }
    go.box_count = n;
}

static int go_wants(const go_cell_t *c, int phase, uint64_t ms)
{
    if (!c || !c->t)
        return 1;
    const uint64_t age = go_age(c, ms);
    if (c->state == GO_UNKNOWN)
        return age >= GO_MISS_RETRY_MS;
    if (phase == GO_PH_UNKNOWN)
        return 0;
    if (c->count < 2)
        return age >= (uint64_t)go.cfg.confirm_ms;
    uint64_t period = (uint64_t)((c->flags & GO_F_CHANGED) ? go.cfg.fast_refresh_ms : go.cfg.refresh_ms);
    if (phase == GO_PH_ROUTE && (uint64_t)go.cfg.route_refresh_ms < period)
        period = (uint64_t)go.cfg.route_refresh_ms;
    return age >= period;
}

/* One batch of seat s: up to GO_BATCH probes. Returns the weighted number of
 * probes (at least 1), or -1 on a raised ray. *stop: the budget ran out. */
static int go_batch(go_seat_t *s, int seat, uint64_t ms, double budget, double batch0, int *stop)
{
    *stop = 0;
    go_near_hulls(s);
    const int tc = go_wc(s->tank.x), tr = go_wc(s->tank.z);
    int probes = 0, weight = 0, lookups = 0;
    while (probes < GO_BATCH && lookups < GO_BATCH_LOOKUPS) {
        int wc, wr;
        if (s->phase == GO_PH_ROUTE) {
            if (s->route_i >= s->nroute) {
                s->phase = GO_PH_UNKNOWN;
                continue;
            }
            wc = s->route_c[s->route_i][0];
            wr = s->route_c[s->route_i][1];
        } else if (s->phase == GO_PH_UNKNOWN) {
            if (s->disc0 >= go.noff) {
                s->phase = GO_PH_STALE;
                continue;
            }
            const go_off_t o = go.off[s->bucket][s->disc0];
            wc = tc + o.dc;
            wr = tr + o.dr;
        } else if (s->phase == GO_PH_STALE) {
            if (s->disc1 >= go.noff) {
                s->disc1 = 0;
                s->phase = GO_PH_IDLE;
                s->idle_until_ms = ms + GO_IDLE_MS;
                break;
            }
            const go_off_t o = go.off[GO_BUCKETS - 1][s->disc1];
            wc = tc + o.dc;
            wr = tr + o.dr;
        } else {
            break;
        }
        ++lookups;
        const go_cell_t *c = go_cell_ptr(wc, wr, 0, NULL);
        if (!go_wants(c, s->phase, ms)) {
            if (s->phase == GO_PH_ROUTE)
                ++s->route_i;
            else if (s->phase == GO_PH_UNKNOWN)
                ++s->disc0;
            else
                ++s->disc1;
            continue;
        }
        const double used = go.window_used_us + (go_now() - batch0);
        if (used + go.probe_ema_us > budget) {
            *stop = 1;
            break;
        }
        const int phase = s->phase;
        if (phase == GO_PH_ROUTE)
            ++s->route_i;
        else if (phase == GO_PH_UNKNOWN)
            ++s->disc0;
        else
            ++s->disc1;
        if (go_in_hull(s, go_centre(wc), go_centre(wr))) {
            /* Not probed. An unknown cell is passed over for GO_MISS_RETRY_MS
             * (as a miss, without the flag); a known one keeps its age. */
            ++go.total.vehicle_skips;
            ++go.i_skips;
            go_cell_t *hc = go_cell_ptr(wc, wr, 1, NULL);
            if (hc && hc->state == GO_UNKNOWN) {
                hc->t = go_stamp(ms);
            } else if (hc) {
                /* Due again GO_MISS_RETRY_MS from now: the age it shows is
                 * that much short of its period (no fresher than it was). */
                const uint32_t period = (uint32_t)(hc->count < 2 ? go.cfg.confirm_ms
                                                   : (hc->flags & GO_F_CHANGED) ? go.cfg.fast_refresh_ms
                                                   : go.cfg.refresh_ms);
                const uint32_t back = period > GO_MISS_RETRY_MS ? period - GO_MISS_RETRY_MS : 0u;
                const uint32_t now = go_stamp(ms);
                const uint32_t t = now - back;
                if ((int32_t)(t - hc->t) > 0)
                    hc->t = t ? t : 1u;
            }
            continue;
        }
        const double p0 = go_now();
        double ray_us = 0.0;
        if (go_probe(s, wc, wr, ms, &ray_us) < 0) {
            go.faulted = 1;
            ++go.total.faults;
            go_event("overhead: the collision ray raised an exception (seat %d, cell %d,%d): overhead probes off for the rest "
                     "of the battle", seat, wc, wr);
            return -1;
        }
        const double cost = go_now() - p0;
        go.probe_ema_us = 0.95 * go.probe_ema_us + 0.05 * cost;
        go.total.ray_us += ray_us;
        go.i_ray_us += ray_us;
        ++s->probes;
        ++probes;
        weight += phase == GO_PH_STALE ? 2 : 1;
    }
    return weight > 0 ? weight : 1;
}

static int go_pick(uint64_t ms)
{
    int best = -1, best_served = 1 << 30;
    for (int n = 0; n < GO_MAX_SEATS - 1; ++n) {
        const int i = 1 + (go.rr + n) % (GO_MAX_SEATS - 1);
        go_seat_t *s = &go_seat[i];
        if (!s->used || !s->present)
            continue;
        if (s->phase == GO_PH_IDLE) {
            if (ms < s->idle_until_ms)
                continue;
            s->phase = GO_PH_ROUTE;
            s->route_i = s->disc0 = 0;
        }
        if (s->served < best_served) {
            best = i;
            best_served = s->served;
        }
    }
    return best;
}

/* ---- logs ------------------------------------------------------------- */
static FILE *go_open(const char *name, const char *mode)
{
    if (!go.log_dir[0])
        return NULL;
    char path[MAX_PATH * 2];
    snprintf(path, sizeof path, "%s/%s%s", go.log_dir, go.prefix, name);
    return fopen(path, mode);
}

static void go_coverage_of(go_seat_t *s)
{
    const int tc = go_wc(s->tank.x), tr = go_wc(s->tank.z);
    const float fx = cosf(s->tank.heading), fz = sinf(s->tank.heading);
    int n = 0, known = 0, na = 0, ka = 0;
    const go_off_t *list = go.off[GO_BUCKETS - 1];
    for (int i = 0; i < go.noff; i += GO_COVER_STRIDE) {
        const go_off_t o = list[i];
        const int k = go_find(go_fdiv(tc + o.dc), go_fdiv(tr + o.dr));
        int on = 0;
        if (k >= 0) {
            const int wc = tc + o.dc, wr = tr + o.dr;
            const go_tile_t *t = &go.tiles[k];
            on = t->c[(wr - t->tz * GO_TILE) * GO_TILE + (wc - t->tx * GO_TILE)].state != GO_UNKNOWN;
        }
        ++n;
        known += on;
        const float d = hypotf((float)o.dc, (float)o.dr);
        if (d > 0.0f && ((float)o.dc * fx + (float)o.dr * fz) >= 0.7071f * d) {
            ++na;
            ka += on;
        }
    }
    s->coverage = n ? (double)known / n : 0.0;
    s->coverage_ahead = na ? (double)ka / na : 0.0;
    s->has_coverage = 1;
}

static void go_log_rates(uint64_t ms)
{
    if (go.l_ms && ms > go.l_ms) {
        const double secs = (double)(ms - go.l_ms) / 1000.0;
        go.r_probes = (double)(go.total.probes - go.l_probes) / secs;
        go.r_rays = (double)(go.total.rays - go.l_rays) / secs;
        go.r_work = (go.total.work_us - go.l_work_us) / secs;
        const uint64_t w = go.total.windows - go.l_windows;
        go.r_capped = w ? (double)(go.total.capped_windows - go.l_capped) / (double)w : 0.0;
    }
    go.l_ms = ms;
    go.l_probes = go.total.probes;
    go.l_rays = go.total.rays;
    go.l_work_us = go.total.work_us;
    go.l_windows = go.total.windows;
    go.l_capped = go.total.capped_windows;
}

static void go_log_row(go_seat_t *s, int seat, uint64_t ms)
{
    if (!go.csv) {
        go.csv = go_open("overhead.csv", "a");
        if (!go.csv)
            return;
        if (ftell(go.csv) == 0)
            fprintf(go.csv, "ms,seat,life,x,z,heading,speed,phase,coverage,coverage_ahead,tiles,cells_known,"
                            "probes_s,rays_s,work_us_s,capped_share,probes_seat,vehicle_skips,evictions,"
                            "became_obstacle,became_free,misses,faults\n");
    }
    fprintf(go.csv, "%llu,%d,%llu,%.2f,%.2f,%.1f,%.2f,%d,%.4f,%.4f,%d,%llu,%.0f,%.0f,%.0f,%.3f,%llu,%llu,%llu,%llu,%llu,%llu,%llu\n",
            (unsigned long long)ms, seat, (unsigned long long)s->tank.life, s->tank.x, s->tank.z,
            s->tank.heading * 57.29578f, s->tank.speed, s->phase, s->coverage, s->coverage_ahead, go.ntiles,
            (unsigned long long)go.total.cells_known, go.r_probes, go.r_rays, go.r_work, go.r_capped,
            (unsigned long long)s->probes, (unsigned long long)go.total.vehicle_skips,
            (unsigned long long)go.total.evictions, (unsigned long long)go.total.became_obstacle,
            (unsigned long long)go.total.became_free, (unsigned long long)go.total.misses,
            (unsigned long long)go.total.faults);
    ++go.csv_rows;
    if (ms - go.csv_flush_ms >= 2000u) {
        fflush(go.csv);
        go.csv_flush_ms = ms;
    }
}

/* overhead_snap.bin: header, then fixed-size records (tools/overhead_report.py). */
#pragma pack(push, 1)
typedef struct {
    char magic[8];              /* "WTOHS001" */
    uint32_t header_bytes, record_bytes, n, prior_win;
    float cell, range, step_m, overhang_m;
    uint32_t reserved[2];
} go_snap_header_t;
typedef struct {
    char tag[4];                /* "OHSN" */
    uint32_t record_bytes;
    uint64_t ms, life;
    int32_t seat, phase;
    float x, z, y, heading, speed, coverage, coverage_ahead;
    int32_t wc0, wr0;           /* world cell of cells[0][0]; [row = +z][col = +x] */
    /* bits 0-2 state (0 unknown, 1 free, 2 obstacle, 3 overhead), bit 3 probed
     * twice or more in a row, bit 4 changed, bit 5 below free, bit 6 inside,
     * bit 7 floor (free on a non-land surface) */
    uint8_t cells[GO_SNAP_N * GO_SNAP_N];
    uint8_t top_dm[GO_SNAP_N * GO_SNAP_N];  /* obstacle top / overhang height, 0.1 m (255: 25.5 m or more) */
    uint8_t age_s[GO_SNAP_N * GO_SNAP_N];   /* seconds since the last probe (255: never or 255 s or more) */
    int32_t prior_c0, prior_r0;             /* prior cell of prior[0][0] */
    float prior_x0, prior_z0, prior_step;
    uint16_t prior[GO_SNAP_PRIOR * GO_SNAP_PRIOR];  /* raw WTOBS001 words, 0 outside */
} go_snap_record_t;
#pragma pack(pop)

static go_snap_record_t go_snap_buffer;

static void go_snapshot(go_seat_t *s, int seat, uint64_t ms)
{
    if (!go.snap) {
        go.snap = go_open("overhead_snap.bin", "ab");
        if (!go.snap)
            return;
        if (ftell(go.snap) == 0) {
            go_snap_header_t h = {{'W', 'T', 'O', 'H', 'S', '0', '0', '1'}, (uint32_t)sizeof(go_snap_header_t),
                                  (uint32_t)sizeof(go_snap_record_t), GO_SNAP_N, GO_SNAP_PRIOR, go.cfg.cell_m,
                                  go.cfg.range_m, go.cfg.step_m, go.cfg.overhang_m, {0, 0}};
            fwrite(&h, sizeof h, 1, go.snap);
        }
    }
    go_snap_record_t *r = &go_snap_buffer;
    memset(r, 0, sizeof *r);
    memcpy(r->tag, "OHSN", 4);
    r->record_bytes = (uint32_t)sizeof *r;
    r->ms = ms;
    r->life = s->tank.life;
    r->seat = seat;
    r->phase = s->phase;
    r->x = s->tank.x;
    r->z = s->tank.z;
    r->y = s->tank.y;
    r->heading = s->tank.heading;
    r->speed = s->tank.speed;
    r->coverage = (float)s->coverage;
    r->coverage_ahead = (float)s->coverage_ahead;
    r->wc0 = go_wc(s->tank.x) - GO_SNAP_N / 2;
    r->wr0 = go_wc(s->tank.z) - GO_SNAP_N / 2;
    for (int row = 0; row < GO_SNAP_N; ++row)
        for (int col = 0; col < GO_SNAP_N; ++col) {
            const int wc = r->wc0 + col, wr = r->wr0 + row;
            const int k = go_find(go_fdiv(wc), go_fdiv(wr));
            const int i = row * GO_SNAP_N + col;
            r->age_s[i] = 255;
            if (k < 0)
                continue;
            const go_tile_t *t = &go.tiles[k];
            const go_cell_t *c = &t->c[(wr - t->tz * GO_TILE) * GO_TILE + (wc - t->tx * GO_TILE)];
            uint8_t b = (uint8_t)(c->state & 7u);
            b |= c->count >= 2 ? 0x08 : 0;
            b |= (c->flags & GO_F_CHANGED) ? 0x10 : 0;
            b |= (c->flags & GO_F_BELOW_FREE) ? 0x20 : 0;
            b |= (c->flags & GO_F_INSIDE) ? 0x40 : 0;
            b |= (c->flags & GO_F_FLOOR) ? 0x80 : 0;
            r->cells[i] = b;
            const int dm = c->top_cm / 10;
            r->top_dm[i] = (uint8_t)(dm > 255 ? 255 : dm < 0 ? 0 : dm);
            const uint64_t age = go_age(c, ms);
            r->age_s[i] = (uint8_t)(age == UINT64_MAX || age / 1000u > 255u ? 255u : age / 1000u);
        }
    if (go.cfg.prior) {
        const float st = go.cfg.prior_step;
        r->prior_x0 = go.cfg.prior_x0;
        r->prior_z0 = go.cfg.prior_z0;
        r->prior_step = st;
        r->prior_c0 = (int)floorf(((float)r->wc0 * go.cfg.cell_m - go.cfg.prior_x0) / st) - 1;
        r->prior_r0 = (int)floorf(((float)r->wr0 * go.cfg.cell_m - go.cfg.prior_z0) / st) - 1;
        for (int pr = 0; pr < GO_SNAP_PRIOR; ++pr)
            for (int pc = 0; pc < GO_SNAP_PRIOR; ++pc) {
                const int c = r->prior_c0 + pc, rr = r->prior_r0 + pr;
                if (c >= 0 && rr >= 0 && c < (int)go.cfg.prior_nx && rr < (int)go.cfg.prior_nz)
                    r->prior[pr * GO_SNAP_PRIOR + pc] = go.cfg.prior[(size_t)rr * go.cfg.prior_nx + (size_t)c];
            }
    }
    if (fwrite(r, sizeof *r, 1, go.snap) == 1) {
        ++go.snaps;
        fflush(go.snap);
    }
}

static void go_coverage_summary(double *mean, double *lo, int *tanks)
{
    double sum = 0.0;
    int n = 0;
    *lo = 1.0;
    for (int i = 1; i < GO_MAX_SEATS; ++i)
        if (go_seat[i].used && go_seat[i].present && go_seat[i].has_coverage) {
            sum += go_seat[i].coverage;
            *lo = fmin(*lo, go_seat[i].coverage);
            ++n;
        }
    *mean = n ? sum / n : 0.0;
    if (!n)
        *lo = 0.0;
    *tanks = n;
}

static void go_interval_event(uint64_t ms)
{
    const double secs = (double)(ms - go.stats_ms) / 1000.0;
    double mean, lo;
    int tanks;
    go_coverage_summary(&mean, &lo, &tanks);
    go_event("overhead: last %.0f s: %d tanks, %.0f probes/s, %.0f rays/s (%.2f per probe), %.2f us per ray, "
             "work %.0f us/s, cap reached in %llu of %llu windows, max window %.0f us, %d tiles (peak %d, %.1f MB), "
             "%llu cells known, coverage within %.0f m mean %.0f%% (min %.0f%%), %llu vehicle skips, %llu evictions, "
             "%llu new obstacles, %llu cleared",
             secs, tanks, secs > 0 ? (double)go.i_probes / secs : 0.0, secs > 0 ? (double)go.i_rays / secs : 0.0,
             go.i_probes ? (double)go.i_rays / (double)go.i_probes : 0.0, go.i_rays ? go.i_ray_us / (double)go.i_rays : 0.0,
             secs > 0 ? go.i_work_us / secs : 0.0, (unsigned long long)go.i_capped, (unsigned long long)go.i_windows,
             go.i_max_window_us, go.ntiles, go.total.tiles_peak,
             (double)go.ntiles * (double)sizeof(go_tile_t) / 1048576.0, (unsigned long long)go.total.cells_known,
             go.cfg.range_m, 100.0 * mean, 100.0 * lo, (unsigned long long)go.i_skips, (unsigned long long)go.i_evict,
             (unsigned long long)go.i_obst, (unsigned long long)go.i_free);
    go.i_probes = go.i_rays = go.i_windows = go.i_capped = go.i_skips = go.i_evict = go.i_obst = go.i_free = 0;
    go.i_ray_us = go.i_work_us = go.i_max_window_us = 0.0;
    go.stats_ms = ms;
}

static void go_end_event(uint64_t ms, const char *why)
{
    (void)ms;
    const double secs = go.has_active && go.last_active_ms > go.first_active_ms
                        ? (double)(go.last_active_ms - go.first_active_ms) / 1000.0 : 0.0;
    go_event("overhead: end (%s): %llu probes, %llu rays in %.0f s (%.0f probes/s), %.2f us per ray, work %.0f us/s, "
             "log %.0f us/s, cap reached in %llu of %llu windows, max window %.0f us, tiles peak %d, %llu evictions "
             "(%llu hot), %llu vehicle skips, %llu inside retries, %llu misses, %llu new obstacles, %llu cleared, "
             "%u csv rows, %u snapshots, %llu bad answers, %llu faults",
             why, (unsigned long long)go.total.probes, (unsigned long long)go.total.rays, secs,
             secs > 0 ? (double)go.total.probes / secs : 0.0,
             go.total.rays ? go.total.ray_us / (double)go.total.rays : 0.0, secs > 0 ? go.total.work_us / secs : 0.0,
             secs > 0 ? go.total.log_us / secs : 0.0, (unsigned long long)go.total.capped_windows,
             (unsigned long long)go.total.windows, go.total.max_window_us, go.total.tiles_peak,
             (unsigned long long)go.total.evictions, (unsigned long long)go.total.hot_evictions,
             (unsigned long long)go.total.vehicle_skips, (unsigned long long)go.total.inside_retries,
             (unsigned long long)go.total.misses, (unsigned long long)go.total.became_obstacle,
             (unsigned long long)go.total.became_free, go.csv_rows, go.snaps, (unsigned long long)go.total.bad_answers,
             (unsigned long long)go.total.faults);
}

/* ---- adapter API ------------------------------------------------------ */
typedef struct { float key; go_off_t o; } go_sort_t;

static int go_sort_cmp(const void *a, const void *b)
{
    const go_sort_t *x = (const go_sort_t *)a, *y = (const go_sort_t *)b;
    if (x->key != y->key)
        return x->key < y->key ? -1 : 1;
    if (x->o.dr != y->o.dr)
        return x->o.dr < y->o.dr ? -1 : 1;
    return x->o.dc < y->o.dc ? -1 : x->o.dc > y->o.dc ? 1 : 0;
}

static void go_free_all(void)
{
    free(go.tiles);
    free(go.hash);
    for (int b = 0; b < GO_BUCKETS; ++b)
        free(go.off[b]);
}

void go_init(const go_config_t *config)
{
    if (go.csv)
        fclose(go.csv);
    if (go.snap)
        fclose(go.snap);
    go_free_all();
    memset(&go, 0, sizeof go);
    memset(go_seat, 0, sizeof go_seat);
    if (!config || !config->trace)
        return;
    go.cfg = *config;
    go_config_t *c = &go.cfg;
    if (!(c->cell_m >= 0.1f && c->cell_m <= 4.0f))
        c->cell_m = 0.5f;
    if (!(c->range_m > 1.0f))
        c->range_m = 30.0f;
    if (c->range_m > (float)GO_MAX_RADIUS_CELLS * c->cell_m)
        c->range_m = (float)GO_MAX_RADIUS_CELLS * c->cell_m;
    if (c->us_per_tick <= 0)
        c->us_per_tick = 1000;
    if (c->window_ms <= 0)
        c->window_ms = 16;
    if (c->refresh_ms <= 0)
        c->refresh_ms = 20000;
    if (c->fast_refresh_ms <= 0)
        c->fast_refresh_ms = 5000;
    if (c->route_refresh_ms <= 0)
        c->route_refresh_ms = 10000;
    if (c->confirm_ms <= 0)
        c->confirm_ms = 2000;
    if (c->max_tiles <= 0)
        c->max_tiles = 1024;
    if (c->max_tiles > 65536)
        c->max_tiles = 65536;
    if (!(c->top_m > 1.0f))
        c->top_m = 8.0f;
    if (!(c->step_m > 0.05f))
        c->step_m = 0.5f;
    if (!(c->overhang_m > c->step_m))
        c->overhang_m = 4.0f;
    if (!c->flags_probe)
        c->flags_probe = GH_RAY_GROUND_AND_OBJECTS;
    if (!c->flags_land)
        c->flags_land = GH_RAY_GROUND;
    go.log_dir[0] = go.prefix[0] = 0;
    if (config->log_dir)
        strncpy(go.log_dir, config->log_dir, sizeof go.log_dir - 1);
    if (config->file_prefix)
        strncpy(go.prefix, config->file_prefix, sizeof go.prefix - 1);
    c->log_dir = go.log_dir;
    c->file_prefix = go.prefix;
    if (c->prior && (!c->prior_nx || !c->prior_nz || !(c->prior_step > 0.0f)))
        c->prior = NULL;
    long long f;
    gh_qpc_frequency(&f);
    go.freq = 1e6 / (double)f;
    /* Pool, hash, sweep orders. */
    uint32_t hsize = 1;
    while (hsize < 2u * (uint32_t)c->max_tiles)
        hsize <<= 1;
    go.tiles = (go_tile_t *)calloc((size_t)c->max_tiles, sizeof *go.tiles);
    go.hash = (int32_t *)malloc(hsize * sizeof *go.hash);
    const int rad = (int)ceilf(c->range_m / c->cell_m);
    const size_t box = (size_t)(2 * rad + 1) * (size_t)(2 * rad + 1);
    go_sort_t *sort = (go_sort_t *)malloc(box * sizeof *sort);
    int ok = go.tiles && go.hash && sort;
    for (int b = 0; b < GO_BUCKETS && ok; ++b)
        ok = (go.off[b] = (go_off_t *)malloc(box * sizeof(go_off_t))) != NULL;
    if (!ok) {
        free(sort);
        go_free_all();
        memset(&go, 0, sizeof go);
        if (config->event)
            config->event(config->event_context, "overhead: out of memory, overhead probes off");
        return;
    }
    go.hmask = hsize - 1;
    for (uint32_t i = 0; i < hsize; ++i)
        go.hash[i] = -1;
    for (int b = 0; b < GO_BUCKETS; ++b) {
        int n = 0;
        const float hb = (float)b * (GO_PI / 4.0f);
        for (int dr = -rad; dr <= rad; ++dr)
            for (int dc = -rad; dc <= rad; ++dc) {
                const float d = hypotf((float)dc, (float)dr) * c->cell_m;
                if (d > c->range_m)
                    continue;
                float key = d;
                if (b < GO_BUCKETS - 1 && d > 0.0f) {
                    const float ca = cosf(atan2f((float)dr, (float)dc) - hb);
                    key = d * (1.0f - GO_AHEAD_GAIN * (ca > 0.0f ? ca : 0.0f));
                }
                sort[n++] = (go_sort_t){key, {(int16_t)dc, (int16_t)dr}};
            }
        qsort(sort, (size_t)n, sizeof *sort, go_sort_cmp);
        for (int i = 0; i < n; ++i)
            go.off[b][i] = sort[i].o;
        go.noff = n;
    }
    free(sort);
    go.probe_ema_us = 2.0;
    go.on = 1;
    go_event("overhead: on, cells %.2f m, range %.0f m, cap %d us per %d ms window, refresh %d ms (changed %d, route %d, "
             "confirm %d), probes from %.1f m (prior), step %.2f m, overhang %.1f m, flags 0x%x / land 0x%x, %d tiles "
             "of %dx%d at most (%.1f MB), %d disc cells per tank, prior %s %ux%u",
             c->cell_m, c->range_m, c->us_per_tick, c->window_ms, c->refresh_ms, c->fast_refresh_ms, c->route_refresh_ms,
             c->confirm_ms, c->top_m, c->step_m, c->overhang_m, c->flags_probe, c->flags_land, c->max_tiles, GO_TILE,
             GO_TILE, (double)c->max_tiles * (double)sizeof(go_tile_t) / 1048576.0, go.noff,
             c->prior ? "WTOBS001" : "none", c->prior ? c->prior_nx : 0u, c->prior ? c->prior_nz : 0u);
}

void go_shutdown(uint64_t ms)
{
    if (go.on && (go.running || go.total.probes))
        go_end_event(ms, go.faulted ? "faulted" : "shutdown");
    if (go.csv)
        fclose(go.csv);
    if (go.snap)
        fclose(go.snap);
    go.csv = go.snap = NULL;
    go.on = 0;
    go.running = 0;
}

int go_active(void)
{
    return go.on && !go.faulted;
}

void go_stats(go_stats_t *out)
{
    *out = go.total;
    out->tiles = go.ntiles;
}

double go_coverage(int seat)
{
    if (seat < 1 || seat >= GO_MAX_SEATS || !go_seat[seat].used || !go_seat[seat].has_coverage)
        return -1.0;
    return go_seat[seat].coverage;
}

void go_tick(uint64_t ms, uint64_t now_us, const go_tank_t *tanks, int count, const go_hull_t *hulls, int hull_count)
{
    if (!go.on || go.faulted)
        return;
    go.ms = ms;
    go.hulls = hulls;
    go.hull_count = hulls ? hull_count : 0;
    go_prepare_hulls();
    for (int i = 0; i < GO_MAX_SEATS; ++i)
        go_seat[i].present = 0;
    int live = 0;
    for (int n = 0; n < count; ++n) {
        const go_tank_t *t = &tanks[n];
        if (t->seat < 1 || t->seat >= GO_MAX_SEATS || !isfinite(t->x) || !isfinite(t->y) || !isfinite(t->z)
            || !isfinite(t->heading))
            continue;
        go_seat_t *s = &go_seat[t->seat];
        if (!s->used || s->tank.life != t->life) {
            memset(s, 0, sizeof *s);
            s->used = 1;
            s->tank = *t;
            s->next_log_ms = ms;
            go_restart(s);
        }
        s->tank = *t;
        if (s->tank.route_count > GO_ROUTE_POINTS)
            s->tank.route_count = GO_ROUTE_POINTS;
        if (s->tank.route_count < 0)
            s->tank.route_count = 0;
        s->present = 1;
        ++live;
        const int route_moved = (s->tank.route_count > 0) != s->has_route_ref
            || (s->has_route_ref && hypotf(s->tank.route_x[0] - s->rx, s->tank.route_z[0] - s->rz) > GO_ROUTE_MOVED_M);
        if (hypotf(t->x - s->ox, t->z - s->oz) >= GO_RESET_M || go_bucket(t) != s->bucket || route_moved)
            go_restart(s);
    }
    if (live) {
        if (!go.running) {
            go.running = 1;
            if (!go.has_active) {
                go.has_active = 1;
                go.first_active_ms = go.stats_ms = ms;
            }
        }
        go.last_active_ms = ms;
    } else if (go.running && ms - go.last_active_ms > GO_IDLE_END_MS) {
        go.running = 0;
        go_end_event(ms, "no live tanks for 5 s");
    }

    /* Budget window (as the lidar). */
    const uint64_t window_us = (uint64_t)go.cfg.window_ms * 1000u;
    if (!go.window_open || now_us < go.window_start_us || now_us - go.window_start_us >= window_us) {
        go.window_open = 1;
        go.window_start_us = now_us;
        go.window_used_us = 0.0;
        go.window_capped = 0;
        for (int i = 0; i < GO_MAX_SEATS; ++i)
            go_seat[i].served = 0;
        if (live) {
            ++go.total.windows;
            ++go.i_windows;
        }
    }
    const double budget = (double)go.cfg.us_per_tick;
    for (int guard = 0; live && guard < 64 * GO_MAX_SEATS; ++guard) {
        if (go.window_used_us + go.probe_ema_us > budget) {
            if (!go.window_capped) {
                go.window_capped = 1;
                ++go.total.capped_windows;
                ++go.i_capped;
            }
            break;
        }
        const int seat = go_pick(ms);
        if (seat < 0)
            break;
        go.rr = seat % (GO_MAX_SEATS - 1);
        go_seat_t *s = &go_seat[seat];
        int stop = 0;
        const double b0 = go_now();
        const int w = go_batch(s, seat, ms, budget, b0, &stop);
        const double work = go_now() - b0;
        go.window_used_us += work;
        go.total.work_us += work;
        go.i_work_us += work;
        if (w < 0)
            return;
        s->served += w;
        if (stop) {
            if (!go.window_capped) {
                go.window_capped = 1;
                ++go.total.capped_windows;
                ++go.i_capped;
            }
            break;
        }
    }
    go.total.max_window_us = fmax(go.total.max_window_us, go.window_used_us);
    go.i_max_window_us = fmax(go.i_max_window_us, go.window_used_us);

    /* Logs (outside the cap, measured as log_us). */
    const double l0 = go_now();
    int rates = 0;
    for (int seat = 1; seat < GO_MAX_SEATS; ++seat) {
        go_seat_t *s = &go_seat[seat];
        if (!s->used || !s->present || ms < s->next_log_ms)
            continue;
        if (!rates) {
            go_log_rates(ms);
            rates = 1;
        }
        go_coverage_of(s);
        if (go.log_dir[0])
            go_log_row(s, seat, ms);
        s->next_log_ms = (ms - s->next_log_ms < GO_LOG_MS ? s->next_log_ms : ms) + GO_LOG_MS;
    }
    /* One snapshot per tick at most, seats in turn. */
    for (int n = 0; go.log_dir[0] && go.snaps < GO_SNAP_MAX && n < GO_MAX_SEATS - 1; ++n) {
        const int seat = 1 + (go.snap_rr + n) % (GO_MAX_SEATS - 1);
        go_seat_t *s = &go_seat[seat];
        if (!s->used || !s->present || !s->has_coverage || (s->has_snap && ms - s->last_snap_ms < GO_SNAP_GAP_MS)
            || ms - go.first_active_ms < 5000u)
            continue;
        go_snapshot(s, seat, ms);
        s->has_snap = 1;
        s->last_snap_ms = ms;
        go.snap_rr = seat % (GO_MAX_SEATS - 1);
        break;
    }
    go.total.log_us += go_now() - l0;
    if (go.running && ms - go.stats_ms >= GO_STATS_MS)
        go_interval_event(ms);
}

/* ---- read-only API ---------------------------------------------------- */
static const go_cell_t *go_peek(int wc, int wr)
{
    const int k = go_find(go_fdiv(wc), go_fdiv(wr));
    if (k < 0)
        return NULL;
    const go_tile_t *t = &go.tiles[k];
    return &t->c[(wr - t->tz * GO_TILE) * GO_TILE + (wc - t->tx * GO_TILE)];
}

int go_cell(float x, float z, float *top, int *count, uint64_t *age_ms, int *flags)
{
    if (top)
        *top = NAN;
    if (count)
        *count = 0;
    if (age_ms)
        *age_ms = UINT64_MAX;
    if (flags)
        *flags = 0;
    if (!go.on || !isfinite(x) || !isfinite(z))
        return -1;
    const go_cell_t *c = go_peek(go_wc(x), go_wc(z));
    if (!c)
        return GO_UNKNOWN;
    if (top && c->state != GO_UNKNOWN)
        *top = (float)c->top_cm * 0.01f;
    if (count)
        *count = c->count;
    if (age_ms)
        *age_ms = go_age(c, go.ms);
    if (flags)
        *flags = c->flags;
    return c->state;
}

static uint8_t go_block_value(const go_cell_t *c)
{
    switch (c->state) {
    case GO_OBSTACLE:
        return GO_BLOCK_OCCUPIED;
    case GO_FREE:
        return c->count >= 2 ? GO_BLOCK_FREE : GO_BLOCK_FREE_ONCE;
    case GO_OVERHEAD:
        return GO_BLOCK_OVERHEAD;
    default:
        return GO_BLOCK_UNKNOWN;
    }
}

/* With a prior, free twice may clear only what a vertical probe can refute:
 * a prior cell solid for its vertical hit alone (static geometry, or 0.5-3.5 m
 * tall). Not one solid for an edge ray (bits 6-7: a horizontal ray at 1.2 m
 * across the cell was blocked, a thin wall or a fence, which probes on 0.5 m
 * centres cannot see go), nor a cell the scan calls clear (the planner's map
 * closes those only for deep water or a steep slope, which a ground hit says
 * nothing about). Elsewhere it is reported as free once (no override). */
static int go_prior_refutable(int pc, int pr)
{
    if (pc < 0 || pr < 0)
        return 0;
    const unsigned w = go.cfg.prior[(size_t)pr * go.cfg.prior_nx + (size_t)pc];
    const unsigned kind = w & 0x700u, h = w & 0x3fu;
    return (w & 0x8000u) && !(w & 0xc0u) && (kind == 0x200u || (kind && h >= 1u && h < 8u));
}

/* An obstacle cell among the 8 around cache cell (wc, wr)? A probe samples
 * only its cell centre: an obstacle seen next door may reach up to a cell
 * into this one, so this free cell does not clear a solid prior cell (the
 * prior's 2 m margin shrinks to the cache's lattice, not below it). */
static int go_obstacle_beside(int wc, int wr)
{
    for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
            if (!dr && !dc)
                continue;
            const go_cell_t *c = go_peek(wc + dc, wr + dr);
            if (c && c->state == GO_OBSTACLE)
                return 1;
        }
    return 0;
}

/* go_block stages the cache cells under the block (with a one-cell margin)
 * once, so the neighbour rule costs array reads, not hash look-ups. */
#define GO_STAGE_MAX 320
static uint8_t go_stage_val[GO_STAGE_MAX * GO_STAGE_MAX], go_stage_near[GO_STAGE_MAX * GO_STAGE_MAX];

int go_block(float cell, int wc0, int wr0, int n, uint8_t *out)
{
    if (!go.on || go.faulted || n <= 0 || n > 1024 || !out || !(cell > 0.0f))
        return -1;
    static int ccol[1024], ctx[1024], pcol[1024], crow[1024], prow[1024];
    const float cm = go.cfg.cell_m;
    const int prior = go.cfg.prior != NULL;
    for (int k = 0; k < n; ++k) {
        const float x = ((float)(wc0 + k) + 0.5f) * cell, z = ((float)(wr0 + k) + 0.5f) * cell;
        ccol[k] = (int)floorf(x / cm);
        ctx[k] = go_fdiv(ccol[k]);
        crow[k] = (int)floorf(z / cm);
        pcol[k] = prow[k] = -1;
        if (prior) {
            const int pc = (int)floorf((x - go.cfg.prior_x0) / go.cfg.prior_step);
            const int pr = (int)floorf((z - go.cfg.prior_z0) / go.cfg.prior_step);
            pcol[k] = pc >= 0 && pc < (int)go.cfg.prior_nx ? pc : -1;
            prow[k] = pr >= 0 && pr < (int)go.cfg.prior_nz ? pr : -1;
        }
    }
    const int c_lo = ccol[0] - 1, r_lo = crow[0] - 1;
    const int w = ccol[n - 1] + 2 - c_lo, h = crow[n - 1] + 2 - r_lo;
    const int staged = w <= GO_STAGE_MAX && h <= GO_STAGE_MAX;
    if (staged) {
        for (int r = 0; r < h; ++r) {
            const int wr = r_lo + r, tz = go_fdiv(wr);
            const int base = (wr - tz * GO_TILE) * GO_TILE;
            uint8_t *v = go_stage_val + (size_t)r * GO_STAGE_MAX;
            int last_tx = INT32_MIN;
            const go_tile_t *t = NULL;
            for (int c = 0; c < w; ++c) {
                const int wc = c_lo + c, tx = go_fdiv(wc);
                if (tx != last_tx) {
                    last_tx = tx;
                    const int k = go_find(tx, tz);
                    t = k >= 0 ? &go.tiles[k] : NULL;
                }
                v[c] = t ? go_block_value(&t->c[base + (wc - tx * GO_TILE)]) : GO_BLOCK_UNKNOWN;
            }
        }
        for (int r = 1; r < h - 1; ++r)
            for (int c = 1; c < w - 1; ++c) {
                const uint8_t *v = go_stage_val + (size_t)r * GO_STAGE_MAX + c;
                int beside = 0;
                for (int dr = -1; dr <= 1 && !beside; ++dr)
                    for (int dc = -1; dc <= 1; ++dc)
                        if ((dr || dc) && v[dr * GO_STAGE_MAX + dc] == GO_BLOCK_OCCUPIED) {
                            beside = 1;
                            break;
                        }
                go_stage_near[(size_t)r * GO_STAGE_MAX + c] = (uint8_t)beside;
            }
    }
    int known = 0;
    for (int row = 0; row < n; ++row) {
        const int cr = crow[row], tz = go_fdiv(cr);
        const int base = (cr - tz * GO_TILE) * GO_TILE;
        uint8_t *o = out + (size_t)row * (size_t)n;
        int last_tx = INT32_MIN;
        const go_tile_t *t = NULL;
        for (int col = 0; col < n; ++col) {
            uint8_t v;
            int beside;
            if (staged) {
                const size_t i = (size_t)(cr - r_lo) * GO_STAGE_MAX + (size_t)(ccol[col] - c_lo);
                v = go_stage_val[i];
                beside = v == GO_BLOCK_FREE && go_stage_near[i];
            } else {
                if (ctx[col] != last_tx) {
                    last_tx = ctx[col];
                    const int k = go_find(last_tx, tz);
                    t = k >= 0 ? &go.tiles[k] : NULL;
                }
                v = t ? go_block_value(&t->c[base + (ccol[col] - last_tx * GO_TILE)]) : GO_BLOCK_UNKNOWN;
                beside = v == GO_BLOCK_FREE && go_obstacle_beside(ccol[col], cr);
            }
            if (v == GO_BLOCK_FREE && (beside || (prior && !go_prior_refutable(pcol[col], prow[row]))))
                v = GO_BLOCK_FREE_ONCE;
            o[col] = v;
            known += v != GO_BLOCK_UNKNOWN;
        }
    }
    return known;
}
