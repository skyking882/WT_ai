/* s11 live local perception per bot tank: rays, local occupancy grid,
 * evaluation logs. See ground_lidar.h for the design. Self-contained: the
 * adapter (ground_lidar_adapter.inc), the movement model and the unit test
 * all link this file and give it their own ray function. */
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
#include <emmintrin.h>
#include "ground_lidar.h"
#include "ground_host.h"   /* s49: the performance counter (gh_qpc) */
#ifndef MAX_PATH
#define MAX_PATH 260       /* the log path buffers; the Windows SDK value */
#endif

#define GL_N (GL_GRID * GL_GRID)
#define GL_HIST 512            /* 0.1 us bins */
#define GL_HIST_US 0.1
#define GL_LOG_MS 500          /* lidar.csv: 2 Hz per tank */
#define GL_SNAP_AFTER_MS 1000
#define GL_SNAP_GAP_MS 10000
#define GL_SNAP_MAX 600
#define GL_STATS_MS 60000
#define GL_PRIOR_WIN 15        /* prior cells per side in a snapshot (24 m window + alignment) */
#define GL_NEAR_M 4.0f
#define GL_SLOW_MPS 1.0f
#define GL_DISAGREE_M 6.0f
#define GL_IDLE_END_MS 5000
#define GL_STEP_M 1.5f         /* GL_G_STEP_UP / GL_G_DROP */
#define GL_STEEP 0.45f         /* GT_SLOPE_HARD */
#define GL_WADE_M 1.0f         /* GT_WADE_M */
#define GL_PI 3.14159265358979f

typedef struct {
    int used, present;
    gl_tank_t tank;
    char state_name[24];
    gl_ring_t ring;
    int8_t L[GL_LAYERS][GL_N];
    uint8_t nobs[GL_LAYERS][GL_N];
    int ox, oz, grid_valid;
    uint64_t decay_ms;
    int cursor;
    uint64_t cycle_start_ms, cycle_due_ms, next_cycle_ms;
    int served;                         /* slices in this window */
    float nearest;                      /* nearest ring hit at the last complete cycle (priority) */
    uint64_t first_ms, last_ms;         /* this life, ticks present */
    uint32_t cycles_life;
    uint32_t interval_cycles;
    uint64_t interval_first_ms, interval_last_ms;
    int interval_seen;
    uint64_t next_log_ms;
    int snap_prev, snap_prev_state, snap_done, has_snap;
    uint64_t snap_since_ms, last_snap_ms;
} gl_seat_t;

static gl_seat_t gl_seat[GL_MAX_SEATS];

static struct {
    int on, faulted, running;
    gl_config_t cfg;
    char log_dir[MAX_PATH], prefix[96];
    FILE *csv, *snap;
    uint64_t csv_flush_ms, last_active_ms, first_active_ms, stats_ms;
    int has_active;
    int window_open, window_capped;
    uint64_t window_start_us;
    double window_used_us, ray_ema_us;
    int rr;
    double freq;
    /* battle totals */
    gl_stats_t total;
    uint32_t hist[GL_HIST + 1];
    /* interval (GL_STATS_MS) */
    uint64_t i_rays, i_windows, i_capped;
    double i_ray_us, i_work_us, i_max_window_us;
    uint32_t i_hist[GL_HIST + 1];
    uint32_t snaps, csv_rows;
} gl;

/* ---- helpers ---------------------------------------------------------- */
static void gl_event(const char *fmt, ...)
{
    if (!gl.cfg.event)
        return;
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    gl.cfg.event(gl.cfg.event_context, msg);
}

static double gl_now(void)
{
    if (gl.cfg.clock_us)
        return gl.cfg.clock_us(gl.cfg.clock_context);
    long long c;
    gh_qpc(&c);
    return (double)c * gl.freq;
}

static int gl_mod(int a)
{
    const int m = a % GL_GRID;
    return m < 0 ? m + GL_GRID : m;
}

static float gl_wrap(float a)
{
    while (a > GL_PI)
        a -= 2.0f * GL_PI;
    while (a <= -GL_PI)
        a += 2.0f * GL_PI;
    return a;
}

/* Ego sector 0..7 of a world direction: 0 ahead, k = k * 45 deg towards +z. */
static int gl_sector(float world, float heading)
{
    const float e = gl_wrap(world - heading);
    int k = (int)floorf((e + GL_PI / 8.0f) / (GL_PI / 4.0f));
    return ((k % 8) + 8) % 8;
}

static float gl_azimuth(int i)
{
    return (float)i * (2.0f * GL_PI / (float)GL_AZIMUTHS);
}

static void gl_hist_add(uint32_t *hist, double us)
{
    int bin = (int)(us / GL_HIST_US);
    if (bin < 0)
        bin = 0;
    if (bin > GL_HIST)
        bin = GL_HIST;
    ++hist[bin];
}

static double gl_hist_quantile(const uint32_t *hist, double q)
{
    uint64_t n = 0;
    for (int i = 0; i <= GL_HIST; ++i)
        n += hist[i];
    if (!n)
        return 0.0;
    const uint64_t want = (uint64_t)ceil(q * (double)n);
    uint64_t acc = 0;
    for (int i = 0; i <= GL_HIST; ++i) {
        acc += hist[i];
        if (acc >= want)
            return (double)(i + 1) * GL_HIST_US;
    }
    return (double)(GL_HIST + 1) * GL_HIST_US;
}

/* Prior (WTOBS001) solid rule: scanned and (an edge ray blocked, static
 * collision, or a vertical hit 0.5-3.5 m high). 1 solid, 0 free, -1 not
 * scanned or outside. */
static int gl_prior_cell(int c, int r)
{
    if (!gl.cfg.prior || c < 0 || r < 0 || c >= (int)gl.cfg.prior_nx || r >= (int)gl.cfg.prior_nz)
        return -1;
    const unsigned w = gl.cfg.prior[(size_t)r * gl.cfg.prior_nx + (size_t)c];
    if (!(w & 0x8000u))
        return -1;
    const unsigned kind = w & 0x700u, h = w & 0x3fu;
    return (w & 0xc0u) || kind == 0x200u || (kind && h >= 1u && h < 8u) ? 1 : 0;
}

static int gl_prior_at(float x, float z)
{
    if (!gl.cfg.prior || !(gl.cfg.prior_step > 0.0f))
        return -1;
    return gl_prior_cell((int)floorf((x - gl.cfg.prior_x0) / gl.cfg.prior_step),
                         (int)floorf((z - gl.cfg.prior_z0) / gl.cfg.prior_step));
}

/* ---- grid ------------------------------------------------------------- */
static void gl_grid_clear(gl_seat_t *s)
{
    memset(s->L, 0, sizeof s->L);
    memset(s->nobs, 0, sizeof s->nobs);
}

static void gl_clear_column(gl_seat_t *s, int col)
{
    for (int r = 0; r < GL_GRID; ++r)
        for (int l = 0; l < GL_LAYERS; ++l) {
            s->L[l][r * GL_GRID + col] = 0;
            s->nobs[l][r * GL_GRID + col] = 0;
        }
}

static void gl_clear_row(gl_seat_t *s, int row)
{
    for (int l = 0; l < GL_LAYERS; ++l) {
        memset(&s->L[l][row * GL_GRID], 0, GL_GRID);
        memset(&s->nobs[l][row * GL_GRID], 0, GL_GRID);
    }
}

/* Centre the window on (x, z). Columns / rows that enter it were someone
 * else's world cells in the ring buffer: cleared to unknown. */
static void gl_recentre(gl_seat_t *s, float x, float z)
{
    if (!isfinite(x) || !isfinite(z))
        return;
    const int nox = (int)floorf(x / GL_CELL_M) - GL_GRID / 2, noz = (int)floorf(z / GL_CELL_M) - GL_GRID / 2;
    if (!s->grid_valid) {
        gl_grid_clear(s);
        s->ox = nox;
        s->oz = noz;
        s->grid_valid = 1;
        return;
    }
    const int dx = nox - s->ox, dz = noz - s->oz;
    if (!dx && !dz)
        return;
    if (abs(dx) >= GL_GRID || abs(dz) >= GL_GRID) {
        gl_grid_clear(s);
    } else {
        if (dx > 0)
            for (int wc = s->ox + GL_GRID; wc < nox + GL_GRID; ++wc)
                gl_clear_column(s, gl_mod(wc));
        else
            for (int wc = nox; wc < s->ox; ++wc)
                gl_clear_column(s, gl_mod(wc));
        if (dz > 0)
            for (int wr = s->oz + GL_GRID; wr < noz + GL_GRID; ++wr)
                gl_clear_row(s, gl_mod(wr));
        else
            for (int wr = noz; wr < s->oz; ++wr)
                gl_clear_row(s, gl_mod(wr));
    }
    s->ox = nox;
    s->oz = noz;
}

/* Storage index of world cell (wc, wr), -1 outside the window. */
static int gl_index(const gl_seat_t *s, int wc, int wr)
{
    if (!s->grid_valid || wc < s->ox || wr < s->oz || wc >= s->ox + GL_GRID || wr >= s->oz + GL_GRID)
        return -1;
    return gl_mod(wr) * GL_GRID + gl_mod(wc);
}

static void gl_mark(gl_seat_t *s, int layer, int wc, int wr, int delta)
{
    const int i = gl_index(s, wc, wr);
    if (i < 0)
        return;
    int v = s->L[layer][i] + delta;
    v = v > GL_L_MAX ? GL_L_MAX : v < GL_L_MIN ? GL_L_MIN : v;
    s->L[layer][i] = (int8_t)v;
    if (s->nobs[layer][i] < 255)
        ++s->nobs[layer][i];
}

/* 2D DDA (Amanatides-Woo) over 0.25 m cells from (x, z) along (dx, dz) for
 * len metres: free up to the end cell; the end cell occupied if hit. */
static void gl_integrate(gl_seat_t *s, int layer, float x, float z, float dx, float dz, float len, int hit)
{
    const float inv = 1.0f / GL_CELL_M;
    const float fx = x * inv, fz = z * inv;
    /* A hit on a cell edge belongs to the obstacle behind it. */
    const float reach = (len + (hit ? 0.02f : 0.0f)) * inv;
    const float ex_f = fx + dx * reach, ez_f = fz + dz * reach;
    int cx = (int)floorf(fx), cz = (int)floorf(fz);
    const int ex = (int)floorf(ex_f), ez = (int)floorf(ez_f);
    const int sx = dx > 0.0f ? 1 : -1, sz = dz > 0.0f ? 1 : -1;
    const float adx = fabsf(dx), adz = fabsf(dz);
    float tmx = adx > 1e-6f ? ((sx > 0 ? (float)(cx + 1) - fx : fx - (float)cx) / adx) : INFINITY;
    float tmz = adz > 1e-6f ? ((sz > 0 ? (float)(cz + 1) - fz : fz - (float)cz) / adz) : INFINITY;
    const float tdx = adx > 1e-6f ? 1.0f / adx : INFINITY, tdz = adz > 1e-6f ? 1.0f / adz : INFINITY;
    for (int guard = 0; guard < 4 * GL_GRID; ++guard) {
        if (cx == ex && cz == ez)
            break;
        gl_mark(s, layer, cx, cz, GL_L_FREE);
        if (tmx < tmz) {
            if (tmx > reach)
                break;
            tmx += tdx;
            cx += sx;
        } else {
            if (tmz > reach)
                break;
            tmz += tdz;
            cz += sz;
        }
    }
    gl_mark(s, layer, ex, ez, hit ? GL_L_HIT : GL_L_FREE);
}

static void gl_decay(gl_seat_t *s, uint64_t ms)
{
    if (!s->decay_ms || ms < s->decay_ms) {
        s->decay_ms = ms;
        return;
    }
    const uint64_t steps64 = (ms - s->decay_ms) / GL_DECAY_MS;
    if (!steps64)
        return;
    s->decay_ms += steps64 * GL_DECAY_MS;
    if (steps64 >= (uint64_t)GL_L_MAX) {
        memset(s->L, 0, sizeof s->L);
        return;
    }
    /* v -> sign(v) * max(|v| - steps, 0), 16 cells at a time: with u = v
     * as offset binary (v ^ 0x80), P = max(u - 128, 0) and Q = max(128 - u, 0)
     * are |v| on either side; the result is (P - steps)+ - (Q - steps)+. */
    const __m128i bias = _mm_set1_epi8(-128), step = _mm_set1_epi8((char)steps64);
    __m128i *p = (__m128i *)&s->L[0][0];
    _Static_assert(sizeof s->L % 16 == 0, "grid planes in 16-byte blocks");
    for (size_t i = 0; i < sizeof s->L / 16; ++i) {
        const __m128i u = _mm_xor_si128(_mm_loadu_si128(p + i), bias);
        const __m128i pos = _mm_subs_epu8(_mm_subs_epu8(u, bias), step);
        const __m128i neg = _mm_subs_epu8(_mm_subs_epu8(bias, u), step);
        _mm_storeu_si128(p + i, _mm_sub_epi8(pos, neg));
    }
}

static int gl_layer_state(const gl_seat_t *s, int layer, int i)
{
    const int v = s->L[layer][i];
    return v >= GL_L_OCC ? GL_CELL_OCCUPIED : v <= GL_L_FREE_AT ? GL_CELL_FREE : GL_CELL_UNKNOWN;
}

static int gl_merged(const gl_seat_t *s, int i, int *layers)
{
    const int lo = gl_layer_state(s, 0, i), hi = gl_layer_state(s, 1, i);
    if (layers)
        *layers = (lo == GL_CELL_OCCUPIED ? GL_LOW_OCC : 0) | (hi == GL_CELL_OCCUPIED ? GL_HIGH_OCC : 0)
                  | (lo == GL_CELL_FREE ? GL_LOW_FREE : 0) | (hi == GL_CELL_FREE ? GL_HIGH_FREE : 0);
    if (lo == GL_CELL_OCCUPIED || hi == GL_CELL_OCCUPIED)
        return GL_CELL_OCCUPIED;
    return lo == GL_CELL_FREE || hi == GL_CELL_FREE ? GL_CELL_FREE : GL_CELL_UNKNOWN;
}

/* Free and seen at least twice in a free layer, none occupied. */
static int gl_free_twice(const gl_seat_t *s, int i)
{
    if (gl_merged(s, i, NULL) != GL_CELL_FREE)
        return 0;
    return (gl_layer_state(s, 0, i) == GL_CELL_FREE && s->nobs[0][i] >= 2)
           || (gl_layer_state(s, 1, i) == GL_CELL_FREE && s->nobs[1][i] >= 2);
}

/* ---- seats ------------------------------------------------------------ */
static void gl_seat_reset(gl_seat_t *s, uint64_t life, uint64_t ms)
{
    memset(s, 0, sizeof *s);
    s->used = 1;
    s->ring.life = life;
    for (int l = 0; l < GL_LAYERS; ++l)
        for (int a = 0; a < GL_AZIMUTHS; ++a)
            s->ring.hit[l][a] = NAN;
    for (int k = 0; k < GL_GROUND_DIRS; ++k)
        for (int d = 0; d < GL_GROUND_DISTS; ++d)
            s->ring.ground_dh[k][d] = NAN;
    s->ring.ground_y = NAN;
    s->nearest = INFINITY;
    s->next_cycle_ms = ms;
    s->first_ms = s->last_ms = ms;
    s->decay_ms = ms;
}

static float gl_nearest_hit(const gl_seat_t *s)
{
    float nearest = INFINITY;
    for (int l = 0; l < GL_LAYERS; ++l)
        for (int a = 0; a < GL_AZIMUTHS; ++a)
            if (s->ring.hit[l][a] < nearest)
                nearest = s->ring.hit[l][a];
    return nearest;
}

static void gl_fault(int seat, int cursor)
{
    gl.faulted = 1;
    ++gl.total.faults;
    gl_event("lidar: the collision ray raised an exception (seat %d, ray %d of the cycle): lidar off for the rest of the battle",
             seat, cursor);
}

/* One ray of seat s's cycle (cursor). Returns 0, or -1 on a fault. */
static int gl_ray(gl_seat_t *s, int seat, uint64_t ms, double *ray_us)
{
    const gl_tank_t *t = &s->tank;
    const float range = gl.cfg.range_m;
    const int c = s->cursor;
    float p[3], dir[3], dist;
    int kind = 0, flags, ring = -1, layer = 0, az = 0, gdir = 0, gdist = 0;
    if (c == 0) {
        p[0] = t->x;
        p[1] = t->y + GL_PROBE_UP_M;
        p[2] = t->z;
        dist = GL_PROBE_UP_M + GL_PROBE_DOWN_M;
        flags = GL_FLAGS_LAND;
        dir[0] = 0.0f;
        dir[1] = -1.0f;
        dir[2] = 0.0f;
    } else {
        const int j = c - 1, k = j / GL_SLICE_RAYS, r = j % GL_SLICE_RAYS;
        const int ring_per_slice = GL_RING_RAYS / GL_SLICES;
        const float g = isfinite(s->ring.ground_y) ? s->ring.ground_y : t->y;
        if (r < ring_per_slice) {
            ring = 1;
            az = k + GL_SLICES * (r / GL_LAYERS);
            layer = r % GL_LAYERS;
            const float a = gl_azimuth(az);
            p[0] = t->x;
            p[1] = g + (layer ? GL_LAYER_HIGH_M : GL_LAYER_LOW_M);
            p[2] = t->z;
            dir[0] = cosf(a);
            dir[1] = 0.0f;
            dir[2] = sinf(a);
            dist = range;
            flags = GL_FLAGS_RING;
        } else {
            ring = 0;
            const int q = r - ring_per_slice;
            gdir = 2 * k + q / GL_GROUND_DISTS;
            gdist = q % GL_GROUND_DISTS;
            const float a = s->ring.ground_heading + (float)gdir * (GL_PI / 4.0f);
            const float d = gdist ? 8.0f : 4.0f;
            p[0] = t->x + d * cosf(a);
            p[1] = g + GL_PROBE_UP_M;
            p[2] = t->z + d * sinf(a);
            dir[0] = 0.0f;
            dir[1] = -1.0f;
            dir[2] = 0.0f;
            dist = GL_PROBE_UP_M + GL_PROBE_DOWN_M;
            flags = GL_FLAGS_LAND;
        }
    }
    const float limit = dist;
    const double a0 = gl_now();
    const int hit = gl.cfg.trace(gl.cfg.trace_context, p, dir, &dist, &kind, flags);
    const double a1 = gl_now();
    *ray_us = a1 - a0;
    if (hit < 0) {
        gl_fault(seat, c);
        return -1;
    }
    int ok = hit == 1;
    if (ok && (!isfinite(dist) || dist < 0.0f || dist > limit + 0.01f)) {
        ok = 0;
        ++gl.total.bad_answers;
    }
    if (c == 0) {
        s->ring.ground_y = ok ? p[1] - dist : t->y;
        s->ring.x = t->x;
        s->ring.z = t->z;
        s->ring.heading = t->heading;
        s->ring.ground_heading = t->heading;
        s->ring.own_water = isfinite(gl.cfg.water) && gl.cfg.water - s->ring.ground_y > GL_WADE_M;
    } else if (ring) {
        uint8_t f = GL_RAY_CAST;
        if (ok) {
            f |= GL_RAY_HIT;
            if (dist < GL_INSIDE_M)
                f |= GL_RAY_INSIDE;
            if (kind & 2)
                f |= GL_RAY_STATIC;
            if (kind & 4)
                f |= GL_RAY_RENDINST;
        }
        s->ring.hit[layer][az] = ok ? dist : INFINITY;
        s->ring.flags[layer][az] = f;
        s->ring.ms[layer][az] = ms;
        gl_integrate(s, layer, p[0], p[2], dir[0], dir[2], ok ? dist : range, ok);
    } else {
        float dh = NAN;
        if (ok)
            dh = dist < GL_INSIDE_M ? GL_PROBE_UP_M : (p[1] - dist) - s->ring.ground_y;
        s->ring.ground_dh[gdir][gdist] = dh;
        if (gdist == 0)
            s->ring.ground_flags[gdir] = 0;
        uint8_t *gf = &s->ring.ground_flags[gdir];
        if (!ok)
            *gf |= GL_G_NONE;
        else if (dist < GL_INSIDE_M)
            *gf |= GL_G_START_BELOW | GL_G_STEP_UP;
        if (ok && isfinite(gl.cfg.water) && gl.cfg.water - (p[1] - dist) > GL_WADE_M)
            *gf |= GL_G_WATER;
        if (gdist == GL_GROUND_DISTS - 1) {
            const float d4 = s->ring.ground_dh[gdir][0], d8 = s->ring.ground_dh[gdir][1];
            if (isfinite(d4)) {
                if (d4 > GL_STEP_M)
                    *gf |= GL_G_STEP_UP;
                if (d4 < -GL_STEP_M)
                    *gf |= GL_G_DROP;
                if (fabsf(d4) / 4.0f > GL_STEEP)
                    *gf |= GL_G_STEEP;
            }
            if (isfinite(d4) && isfinite(d8)) {
                if (d8 - d4 > GL_STEP_M)
                    *gf |= GL_G_STEP_UP;
                if (d8 - d4 < -GL_STEP_M)
                    *gf |= GL_G_DROP;
                if (fabsf(d8 - d4) / 4.0f > GL_STEEP)
                    *gf |= GL_G_STEEP;
            }
            s->ring.ground_ms = ms;
        }
    }
    return 0;
}

/* ---- logs ------------------------------------------------------------- */
static FILE *gl_open(const char *name, const char *mode)
{
    if (!gl.log_dir[0])
        return NULL;
    char path[MAX_PATH * 2];
    snprintf(path, sizeof path, "%s/%s%s", gl.log_dir, gl.prefix, name);
    return fopen(path, mode);
}

static void gl_log_row(gl_seat_t *s, int seat, uint64_t ms)
{
    if (!gl.csv) {
        gl.csv = gl_open("lidar.csv", "a");
        if (!gl.csv)
            return;
        if (ftell(gl.csv) == 0)
            fprintf(gl.csv, "ms,seat,life,x,z,heading,speed,plan_state,cycles,hz,age_ms,"
                            "hit_0,hit_p45,hit_p90,hit_p135,hit_180,hit_m135,hit_m90,hit_m45,low_only_mask,inside_mask,"
                            "prior_0,prior_p45,prior_p90,prior_p135,prior_180,prior_m135,prior_m90,prior_m45,"
                            "prior_n,prior_confirmed,prior_phantom,pf_cells,lo_cells,occ_low_only,occ_high_only,occ_both,"
                            "gflags,own_water\n");
    }
    const gl_tank_t *t = &s->tank;
    float hit[8], lo[8], hi[8], prior[8];
    unsigned inside = 0, low_only = 0;
    for (int k = 0; k < 8; ++k)
        hit[k] = lo[k] = hi[k] = prior[k] = INFINITY;
    for (int l = 0; l < GL_LAYERS; ++l)
        for (int a = 0; a < GL_AZIMUTHS; ++a) {
            const float d = s->ring.hit[l][a];
            if (!(d < INFINITY))
                continue;
            const int k = gl_sector(gl_azimuth(a), t->heading);
            hit[k] = fminf(hit[k], d);
            if (l)
                hi[k] = fminf(hi[k], d);
            else
                lo[k] = fminf(lo[k], d);
            if (s->ring.flags[l][a] & GL_RAY_INSIDE)
                inside |= 1u << k;
        }
    for (int k = 0; k < 8; ++k)
        if (lo[k] < INFINITY && !(hi[k] < lo[k] + 0.5f))
            low_only |= 1u << k;
    /* Prior: nearest point of each solid 2 m cell within range; the cell
     * counts (within 6 m) by cell centre. */
    int prior_n = 0, confirmed = 0, phantom = 0;
    if (gl.cfg.prior && gl.cfg.prior_step > 0.0f) {
        const float st = gl.cfg.prior_step;
        const int pc = (int)floorf((t->x - gl.cfg.prior_x0) / st), pr = (int)floorf((t->z - gl.cfg.prior_z0) / st);
        const int reach = (int)ceilf(gl.cfg.range_m / st) + 1;
        for (int r = pr - reach; r <= pr + reach; ++r)
            for (int c = pc - reach; c <= pc + reach; ++c) {
                if (gl_prior_cell(c, r) != 1)
                    continue;
                const float x0 = gl.cfg.prior_x0 + (float)c * st, z0 = gl.cfg.prior_z0 + (float)r * st;
                const float nx = fmaxf(x0, fminf(t->x, x0 + st)), nz = fmaxf(z0, fminf(t->z, z0 + st));
                const float d = hypotf(nx - t->x, nz - t->z);
                const float cxm = x0 + 0.5f * st, czm = z0 + 0.5f * st;
                if (d <= gl.cfg.range_m) {
                    const int k = d > 0.0f ? gl_sector(atan2f(czm - t->z, cxm - t->x), t->heading) : 0;
                    prior[k] = fminf(prior[k], d);
                }
                if (hypotf(cxm - t->x, czm - t->z) > GL_DISAGREE_M)
                    continue;
                ++prior_n;
                int occ = 0, seen_free = 0;
                const int sub = (int)(st / GL_CELL_M + 0.5f);
                const int wc0 = (int)floorf(x0 / GL_CELL_M + 0.001f), wr0 = (int)floorf(z0 / GL_CELL_M + 0.001f);
                for (int rr = 0; rr < sub; ++rr)
                    for (int cc = 0; cc < sub; ++cc) {
                        const int i = gl_index(s, wc0 + cc, wr0 + rr);
                        if (i < 0)
                            continue;
                        occ += gl_merged(s, i, NULL) == GL_CELL_OCCUPIED;
                        seen_free += gl_free_twice(s, i);
                    }
                if (occ)
                    ++confirmed;
                else if (seen_free * 4 >= sub * sub)
                    ++phantom;
            }
    }
    /* 0.25 m cells within 6 m. */
    int pf = 0, lonly = 0, occ_lo = 0, occ_hi = 0, occ_both = 0;
    const int rad = (int)ceilf(GL_DISAGREE_M / GL_CELL_M);
    const int tc = (int)floorf(t->x / GL_CELL_M), tr = (int)floorf(t->z / GL_CELL_M);
    for (int r = tr - rad; r <= tr + rad; ++r)
        for (int c = tc - rad; c <= tc + rad; ++c) {
            const float x = ((float)c + 0.5f) * GL_CELL_M, z = ((float)r + 0.5f) * GL_CELL_M;
            if (hypotf(x - t->x, z - t->z) > GL_DISAGREE_M)
                continue;
            const int i = gl_index(s, c, r);
            if (i < 0)
                continue;
            int layers = 0;
            const int m = gl_merged(s, i, &layers);
            if (m == GL_CELL_OCCUPIED) {
                const int both = (layers & GL_LOW_OCC) && (layers & GL_HIGH_OCC);
                occ_both += both;
                occ_lo += !both && (layers & GL_LOW_OCC);
                occ_hi += !both && (layers & GL_HIGH_OCC);
            }
            const int pcell = gl_prior_at(x, z);
            if (pcell == 1 && gl_free_twice(s, i))
                ++pf;
            if (pcell == 0 && m == GL_CELL_OCCUPIED)
                ++lonly;
        }
    uint64_t oldest = ms;
    for (int l = 0; l < GL_LAYERS; ++l)
        for (int a = 0; a < GL_AZIMUTHS; ++a)
            if (s->ring.ms[l][a] < oldest)
                oldest = s->ring.ms[l][a];
    char gflags[GL_GROUND_DIRS * 2 + 1];
    for (int k = 0; k < GL_GROUND_DIRS; ++k)
        snprintf(gflags + 2 * k, 3, "%02x", (unsigned)s->ring.ground_flags[k]);
    const double span = (double)(s->last_ms - s->first_ms) / 1000.0;
    fprintf(gl.csv, "%llu,%d,%llu,%.2f,%.2f,%.1f,%.2f,%s,%u,%.2f,%llu",
            (unsigned long long)ms, seat, (unsigned long long)s->tank.life, t->x, t->z, t->heading * 57.29578f, t->speed,
            s->state_name[0] ? s->state_name : "-", s->cycles_life, span > 0.5 ? (double)s->cycles_life / span : 0.0,
            (unsigned long long)(ms - oldest));
    for (int k = 0; k < 8; ++k)
        fprintf(gl.csv, ",%.2f", hit[k] < INFINITY ? hit[k] : 99.0f);
    fprintf(gl.csv, ",%u,%u", low_only, inside);
    for (int k = 0; k < 8; ++k)
        fprintf(gl.csv, ",%.2f", prior[k] < INFINITY ? prior[k] : 99.0f);
    fprintf(gl.csv, ",%d,%d,%d,%d,%d,%d,%d,%d,%s,%d\n", prior_n, confirmed, phantom, pf, lonly, occ_lo, occ_hi, occ_both,
            gflags, s->ring.own_water);
    ++gl.csv_rows;
    if (ms - gl.csv_flush_ms >= 1000) {
        fflush(gl.csv);
        gl.csv_flush_ms = ms;
    }
}

/* lidar_snap.bin: file header, then fixed-size records (tools/lidar_report.py). */
#pragma pack(push, 1)
typedef struct {
    char magic[8];              /* "WTLSN001" */
    uint32_t header_bytes, record_bytes, grid, prior_win, azimuths, layers;
    float cell;
    uint32_t reserved;
} gl_snap_header_t;
typedef struct {
    char tag[4];                /* "SNAP" */
    uint32_t record_bytes;
    uint64_t ms, life;
    int32_t seat, state;
    float x, z, ground_y, heading, speed;
    float target_x, target_z, goal_x, goal_z;
    float hull_half_length, hull_half_width, hull_forward, hull_right;
    float range;
    int32_t grid_ox, grid_oz;   /* world cell (x / 0.25, z / 0.25) of cells[0][0] */
    char state_name[16];
    /* bits 0-1 low layer (0 unknown, 1 free, 2 occupied), 2-3 high layer,
     * 4 low seen >= 2 times, 5 high seen >= 2 times; [row = +z][col = +x] */
    uint8_t cells[GL_GRID * GL_GRID];
    float hit[GL_LAYERS][GL_AZIMUTHS];   /* 99 none, -1 not cast */
    uint8_t ray_flags[GL_LAYERS][GL_AZIMUTHS];
    float ground_dh[GL_GROUND_DIRS][GL_GROUND_DISTS]; /* -99 none */
    uint8_t ground_flags[GL_GROUND_DIRS];
    int32_t prior_c0, prior_r0; /* prior cell of prior[0][0] */
    uint16_t prior[GL_PRIOR_WIN * GL_PRIOR_WIN];   /* raw WTOBS001 words, 0 outside the map */
} gl_snap_record_t;
#pragma pack(pop)

static gl_snap_record_t gl_snap_buffer;

static void gl_snapshot(gl_seat_t *s, int seat, uint64_t ms)
{
    if (!gl.snap) {
        gl.snap = gl_open("lidar_snap.bin", "ab");
        if (!gl.snap)
            return;
        if (ftell(gl.snap) == 0) {
            gl_snap_header_t h = {{'W', 'T', 'L', 'S', 'N', '0', '0', '1'}, (uint32_t)sizeof(gl_snap_header_t),
                                  (uint32_t)sizeof(gl_snap_record_t), GL_GRID, GL_PRIOR_WIN, GL_AZIMUTHS, GL_LAYERS,
                                  GL_CELL_M, 0};
            fwrite(&h, sizeof h, 1, gl.snap);
        }
    }
    gl_snap_record_t *r = &gl_snap_buffer;
    memset(r, 0, sizeof *r);
    memcpy(r->tag, "SNAP", 4);
    r->record_bytes = (uint32_t)sizeof *r;
    r->ms = ms;
    r->life = s->tank.life;
    r->seat = seat;
    r->state = s->tank.state;
    r->x = s->tank.x;
    r->z = s->tank.z;
    r->ground_y = s->ring.ground_y;
    r->heading = s->tank.heading;
    r->speed = s->tank.speed;
    r->target_x = s->tank.target_x;
    r->target_z = s->tank.target_z;
    r->goal_x = s->tank.goal_x;
    r->goal_z = s->tank.goal_z;
    r->hull_half_length = s->tank.hull_half_length;
    r->hull_half_width = s->tank.hull_half_width;
    r->hull_forward = s->tank.hull_forward;
    r->hull_right = s->tank.hull_right;
    r->range = gl.cfg.range_m;
    r->grid_ox = s->ox;
    r->grid_oz = s->oz;
    strncpy(r->state_name, s->state_name, sizeof r->state_name - 1);
    for (int row = 0; row < GL_GRID; ++row)
        for (int col = 0; col < GL_GRID; ++col) {
            const int i = gl_index(s, s->ox + col, s->oz + row);
            uint8_t b = 0;
            if (i >= 0) {
                b = (uint8_t)(gl_layer_state(s, 0, i) | (gl_layer_state(s, 1, i) << 2));
                b |= (uint8_t)((s->nobs[0][i] >= 2 ? 0x10 : 0) | (s->nobs[1][i] >= 2 ? 0x20 : 0));
            }
            r->cells[row * GL_GRID + col] = b;
        }
    for (int l = 0; l < GL_LAYERS; ++l)
        for (int a = 0; a < GL_AZIMUTHS; ++a) {
            const float d = s->ring.hit[l][a];
            r->hit[l][a] = isnan(d) ? -1.0f : d < INFINITY ? d : 99.0f;
            r->ray_flags[l][a] = s->ring.flags[l][a];
        }
    for (int k = 0; k < GL_GROUND_DIRS; ++k) {
        for (int d = 0; d < GL_GROUND_DISTS; ++d)
            r->ground_dh[k][d] = isfinite(s->ring.ground_dh[k][d]) ? s->ring.ground_dh[k][d] : -99.0f;
        r->ground_flags[k] = s->ring.ground_flags[k];
    }
    if (gl.cfg.prior && gl.cfg.prior_step > 0.0f) {
        const float st = gl.cfg.prior_step;
        r->prior_c0 = (int)floorf(((float)s->ox * GL_CELL_M - gl.cfg.prior_x0) / st) - 1;
        r->prior_r0 = (int)floorf(((float)s->oz * GL_CELL_M - gl.cfg.prior_z0) / st) - 1;
        for (int pr = 0; pr < GL_PRIOR_WIN; ++pr)
            for (int pc = 0; pc < GL_PRIOR_WIN; ++pc) {
                const int c = r->prior_c0 + pc, rr = r->prior_r0 + pr;
                if (c >= 0 && rr >= 0 && c < (int)gl.cfg.prior_nx && rr < (int)gl.cfg.prior_nz)
                    r->prior[pr * GL_PRIOR_WIN + pc] = gl.cfg.prior[(size_t)rr * gl.cfg.prior_nx + (size_t)c];
            }
    }
    if (fwrite(r, sizeof *r, 1, gl.snap) == 1) {
        ++gl.snaps;
        fflush(gl.snap);
    }
}

/* ---- stats ------------------------------------------------------------ */
static void gl_hz_summary(int interval, uint64_t ms, double *mean, double *lo, double *hi, int *tanks)
{
    double sum = 0.0;
    int n = 0;
    *lo = INFINITY;
    *hi = 0.0;
    for (int i = 1; i < GL_MAX_SEATS; ++i) {
        const gl_seat_t *s = &gl_seat[i];
        if (!s->used)
            continue;
        const uint64_t a = interval ? s->interval_first_ms : s->first_ms, b = interval ? s->interval_last_ms : s->last_ms;
        const uint32_t cycles = interval ? s->interval_cycles : s->cycles_life;
        if ((interval && !s->interval_seen) || b <= a + 5000 || (interval && ms - b > GL_STATS_MS))
            continue;
        const double hz = (double)cycles / ((double)(b - a) / 1000.0);
        sum += hz;
        *lo = fmin(*lo, hz);
        *hi = fmax(*hi, hz);
        ++n;
    }
    *mean = n ? sum / n : 0.0;
    if (!n)
        *lo = 0.0;
    *tanks = n;
}

static void gl_interval_event(uint64_t ms)
{
    const double secs = (double)(ms - gl.stats_ms) / 1000.0;
    double mean, lo, hi;
    int tanks;
    gl_hz_summary(1, ms, &mean, &lo, &hi, &tanks);
    gl_event("lidar: last %.0f s: %d tanks, %.0f rays/s, %.2f us per ray (p50 %.1f, p99 %.1f), work %.0f us/s, "
             "refresh %.1f Hz per tank (min %.1f, max %.1f), cap reached in %llu of %llu windows, max window %.0f us",
             secs, tanks, secs > 0 ? (double)gl.i_rays / secs : 0.0, gl.i_rays ? gl.i_ray_us / (double)gl.i_rays : 0.0,
             gl_hist_quantile(gl.i_hist, 0.5), gl_hist_quantile(gl.i_hist, 0.99), secs > 0 ? gl.i_work_us / secs : 0.0,
             mean, lo, hi, (unsigned long long)gl.i_capped, (unsigned long long)gl.i_windows, gl.i_max_window_us);
    gl.i_rays = gl.i_windows = gl.i_capped = 0;
    gl.i_ray_us = gl.i_work_us = gl.i_max_window_us = 0.0;
    memset(gl.i_hist, 0, sizeof gl.i_hist);
    for (int i = 0; i < GL_MAX_SEATS; ++i) {
        gl_seat[i].interval_cycles = 0;
        gl_seat[i].interval_first_ms = gl_seat[i].interval_last_ms = 0;
        gl_seat[i].interval_seen = 0;
    }
    gl.stats_ms = ms;
}

static void gl_end_event(uint64_t ms, const char *why)
{
    double mean, lo, hi;
    int tanks;
    gl_hz_summary(0, ms, &mean, &lo, &hi, &tanks);
    const double secs = gl.has_active && gl.last_active_ms > gl.first_active_ms
                        ? (double)(gl.last_active_ms - gl.first_active_ms) / 1000.0 : 0.0;
    gl_event("lidar: end (%s): %llu rays in %.0f s (%.0f rays/s), %.2f us per ray (p50 %.1f, p99 %.1f), "
             "work %.0f us/s, log %.0f us/s, refresh %.1f Hz per life (min %.1f, max %.1f, %d lives over 5 s), "
             "cap reached in %llu of %llu windows, max window %.0f us, %u csv rows, %u snapshots, %llu bad answers, %llu faults",
             why, (unsigned long long)gl.total.rays, secs, secs > 0 ? (double)gl.total.rays / secs : 0.0,
             gl.total.rays ? gl.total.ray_us / (double)gl.total.rays : 0.0, gl_hist_quantile(gl.hist, 0.5),
             gl_hist_quantile(gl.hist, 0.99), secs > 0 ? gl.total.work_us / secs : 0.0,
             secs > 0 ? gl.total.log_us / secs : 0.0, mean, lo, hi, tanks, (unsigned long long)gl.total.capped_windows,
             (unsigned long long)gl.total.windows, gl.total.max_window_us, gl.csv_rows, gl.snaps,
             (unsigned long long)gl.total.bad_answers, (unsigned long long)gl.total.faults);
}

/* ---- adapter API ------------------------------------------------------ */
void gl_init(const gl_config_t *config)
{
    if (gl.csv)
        fclose(gl.csv);
    if (gl.snap)
        fclose(gl.snap);
    memset(&gl, 0, sizeof gl);
    memset(gl_seat, 0, sizeof gl_seat);
    if (!config || !config->trace)
        return;
    gl.cfg = *config;
    if (!(gl.cfg.range_m > 1.0f))
        gl.cfg.range_m = 12.0f;
    if (gl.cfg.range_m > 0.5f * GL_GRID * GL_CELL_M)    /* the grid holds 12 m around the hull centre */
        gl.cfg.range_m = 0.5f * GL_GRID * GL_CELL_M;
    if (gl.cfg.us_per_tick <= 0)
        gl.cfg.us_per_tick = 1500;
    if (gl.cfg.window_ms <= 0)
        gl.cfg.window_ms = 16;
    if (gl.cfg.period_ms <= 0)
        gl.cfg.period_ms = 100;
    gl.log_dir[0] = gl.prefix[0] = 0;
    if (config->log_dir)
        strncpy(gl.log_dir, config->log_dir, sizeof gl.log_dir - 1);
    if (config->file_prefix)
        strncpy(gl.prefix, config->file_prefix, sizeof gl.prefix - 1);
    gl.cfg.log_dir = gl.log_dir;
    gl.cfg.file_prefix = gl.prefix;
    if (gl.cfg.prior && (!gl.cfg.prior_nx || !gl.cfg.prior_nz || !(gl.cfg.prior_step > 0.0f)))
        gl.cfg.prior = NULL;
    long long f;
    gh_qpc_frequency(&f);
    gl.freq = 1e6 / (double)f;
    gl.ray_ema_us = 5.0;
    gl.on = 1;
    gl_event("lidar: on, range %.1f m, cap %d us per %d ms window, refresh every %d ms, %d rays per cycle "
             "(%d ring at %.1f/%.1f m in %d slices, %d ground probes, 1 centre), grid %dx%d at %.2f m, water %s%.1f, "
             "prior %s %ux%u",
             gl.cfg.range_m, gl.cfg.us_per_tick, gl.cfg.window_ms, gl.cfg.period_ms, GL_CYCLE_RAYS, GL_RING_RAYS,
             GL_LAYER_LOW_M, GL_LAYER_HIGH_M, GL_SLICES, GL_GROUND_RAYS, GL_GRID, GL_GRID, GL_CELL_M,
             isfinite(gl.cfg.water) ? "" : "none ", isfinite(gl.cfg.water) ? gl.cfg.water : 0.0f,
             gl.cfg.prior ? "WTOBS001" : "none", gl.cfg.prior ? gl.cfg.prior_nx : 0u, gl.cfg.prior ? gl.cfg.prior_nz : 0u);
}

void gl_shutdown(uint64_t ms)
{
    if (gl.on && (gl.running || gl.total.rays))
        gl_end_event(ms, gl.faulted ? "faulted" : "shutdown");
    if (gl.csv)
        fclose(gl.csv);
    if (gl.snap)
        fclose(gl.snap);
    gl.csv = gl.snap = NULL;
    gl.on = 0;
    gl.running = 0;
}

int gl_active(void)
{
    return gl.on && !gl.faulted;
}

void gl_stats(gl_stats_t *out)
{
    *out = gl.total;
    out->ray_p50_us = gl_hist_quantile(gl.hist, 0.5);
    out->ray_p99_us = gl_hist_quantile(gl.hist, 0.99);
}

double gl_refresh_hz(int seat)
{
    if (seat < 1 || seat >= GL_MAX_SEATS || !gl_seat[seat].used)
        return 0.0;
    const gl_seat_t *s = &gl_seat[seat];
    return s->last_ms > s->first_ms ? (double)s->cycles_life / ((double)(s->last_ms - s->first_ms) / 1000.0) : 0.0;
}

/* Next tank to serve: fewest slices in this window first, then urgency. */
static int gl_pick(uint64_t ms)
{
    int best = -1, best_served = 1 << 30;
    double best_urgency = -1.0;
    for (int n = 0; n < GL_MAX_SEATS - 1; ++n) {
        const int i = 1 + (gl.rr + n) % (GL_MAX_SEATS - 1);
        const gl_seat_t *s = &gl_seat[i];
        if (!s->used || !s->present || (s->cursor == 0 && ms < s->next_cycle_ms))
            continue;
        const uint64_t due = s->cursor ? s->cycle_due_ms : s->next_cycle_ms;
        const int weighty = s->tank.speed < GL_SLOW_MPS || s->nearest < GL_NEAR_M || s->tank.snap_state;
        const double urgency = (double)(ms - due + 1) * (weighty ? 2.0 : 1.0);
        if (s->served < best_served || (s->served == best_served && urgency > best_urgency)) {
            best = i;
            best_served = s->served;
            best_urgency = urgency;
        }
    }
    return best;
}

void gl_tick(uint64_t ms, uint64_t now_us, const gl_tank_t *tanks, int count)
{
    if (!gl.on || gl.faulted)
        return;
    for (int i = 0; i < GL_MAX_SEATS; ++i)
        gl_seat[i].present = 0;
    int live = 0;
    for (int n = 0; n < count; ++n) {
        const gl_tank_t *t = &tanks[n];
        if (t->seat < 1 || t->seat >= GL_MAX_SEATS || !isfinite(t->x) || !isfinite(t->z) || !isfinite(t->y)
            || !isfinite(t->heading))
            continue;
        gl_seat_t *s = &gl_seat[t->seat];
        if (!s->used || s->tank.life != t->life)
            gl_seat_reset(s, t->life, ms);
        s->tank = *t;
        s->tank.state_name = NULL;
        strncpy(s->state_name, t->state_name ? t->state_name : "", sizeof s->state_name - 1);
        s->present = 1;
        s->last_ms = ms;
        if (!s->interval_seen) {
            s->interval_seen = 1;
            s->interval_first_ms = ms;
        }
        s->interval_last_ms = ms;
        ++live;
    }
    if (live) {
        if (!gl.running) {
            gl.running = 1;
            if (!gl.has_active) {
                gl.has_active = 1;
                gl.first_active_ms = gl.stats_ms = ms;
            }
        }
        gl.last_active_ms = ms;
    } else if (gl.running && ms - gl.last_active_ms > GL_IDLE_END_MS) {
        gl.running = 0;
        gl_end_event(ms, "no live tanks for 5 s");
    }

    /* Budget window. */
    const uint64_t window_us = (uint64_t)gl.cfg.window_ms * 1000u;
    if (!gl.window_open || now_us < gl.window_start_us || now_us - gl.window_start_us >= window_us) {
        gl.window_open = 1;
        gl.window_start_us = now_us;
        gl.window_used_us = 0.0;
        gl.window_capped = 0;
        for (int i = 0; i < GL_MAX_SEATS; ++i)
            gl_seat[i].served = 0;
        if (live) {
            ++gl.total.windows;
            ++gl.i_windows;
        }
    }
    const double budget = (double)gl.cfg.us_per_tick;
    /* Decay towards unknown, every GL_DECAY_MS per present tank. */
    const double d0 = gl_now();
    for (int i = 1; i < GL_MAX_SEATS; ++i)
        if (gl_seat[i].present && gl_seat[i].grid_valid)
            gl_decay(&gl_seat[i], ms);
    const double decay_us = gl_now() - d0;
    gl.window_used_us += decay_us;
    gl.total.work_us += decay_us;
    gl.i_work_us += decay_us;
    for (int guard = 0; live && guard < 4 * GL_MAX_SEATS * GL_SLICES; ++guard) {
        const int seat = gl_pick(ms);
        if (seat < 0)
            break;
        if (gl.window_used_us + gl.ray_ema_us > budget) {
            if (!gl.window_capped) {
                gl.window_capped = 1;
                ++gl.total.capped_windows;
                ++gl.i_capped;
            }
            break;
        }
        gl_seat_t *s = &gl_seat[seat];
        gl.rr = seat % (GL_MAX_SEATS - 1);
        const double p0 = gl_now();
        if (s->cursor == 0) {
            s->cycle_start_ms = ms;
            s->cycle_due_ms = s->next_cycle_ms;
        }
        gl_recentre(s, s->tank.x, s->tank.z);
        const double prep = gl_now() - p0;
        gl.window_used_us += prep;
        gl.total.work_us += prep;
        gl.i_work_us += prep;
        const int end = s->cursor == 0 ? 1 + GL_SLICE_RAYS : 1 + GL_SLICE_RAYS * ((s->cursor - 1) / GL_SLICE_RAYS + 1);
        while (s->cursor < end) {
            if (gl.window_used_us + gl.ray_ema_us > budget)
                break;
            const double w0 = gl_now();
            double ray_us = 0.0;
            if (gl_ray(s, seat, ms, &ray_us) < 0)
                return;
            ++s->cursor;
            const double work = gl_now() - w0;
            gl.window_used_us += work;
            gl.ray_ema_us = 0.95 * gl.ray_ema_us + 0.05 * work;
            ++gl.total.rays;
            ++gl.i_rays;
            gl.total.ray_us += ray_us;
            gl.i_ray_us += ray_us;
            gl.total.work_us += work;
            gl.i_work_us += work;
            gl_hist_add(gl.hist, ray_us);
            gl_hist_add(gl.i_hist, ray_us);
        }
        ++s->served;
        if (s->cursor >= GL_CYCLE_RAYS) {
            s->cursor = 0;
            /* From the due time, not the start: ticks come in frame steps,
             * and this keeps the long-run rate at 1 / period. More than a
             * period behind: start again from now. */
            s->next_cycle_ms = s->cycle_due_ms + (uint64_t)gl.cfg.period_ms;
            if (s->next_cycle_ms + (uint64_t)gl.cfg.period_ms < ms)
                s->next_cycle_ms = ms;
            ++s->ring.cycles;
            s->nearest = gl_nearest_hit(s);
            ++s->cycles_life;
            ++s->interval_cycles;
            ++gl.total.cycles;
        }
    }
    gl.total.max_window_us = fmax(gl.total.max_window_us, gl.window_used_us);
    gl.i_max_window_us = fmax(gl.i_max_window_us, gl.window_used_us);

    /* Evaluation logs (outside the cap, measured as log_us). */
    const double logs = gl_now();
    for (int seat = 1; seat < GL_MAX_SEATS; ++seat) {
        gl_seat_t *s = &gl_seat[seat];
        if (!s->used || !s->present || !s->ring.cycles)
            continue;
        if (gl.log_dir[0] && ms >= s->next_log_ms) {
            gl_recentre(s, s->tank.x, s->tank.z);
            gl_log_row(s, seat, ms);
            s->next_log_ms = (s->next_log_ms && ms - s->next_log_ms < GL_LOG_MS ? s->next_log_ms : ms) + GL_LOG_MS;
        }
        const int snap = s->tank.snap_state != 0;
        if (snap && (!s->snap_prev || s->snap_prev_state != s->tank.state)) {
            s->snap_since_ms = ms;
            s->snap_done = 0;
        }
        if (snap && !s->snap_done && ms - s->snap_since_ms > GL_SNAP_AFTER_MS
            && (!s->has_snap || ms - s->last_snap_ms >= GL_SNAP_GAP_MS) && gl.snaps < GL_SNAP_MAX && gl.log_dir[0]) {
            gl_recentre(s, s->tank.x, s->tank.z);
            gl_snapshot(s, seat, ms);
            s->snap_done = 1;
            s->has_snap = 1;
            s->last_snap_ms = ms;
        }
        s->snap_prev = snap;
        s->snap_prev_state = s->tank.state;
    }
    const double end = gl_now();
    gl.total.log_us += end - logs;
    if (gl.running && ms - gl.stats_ms >= GL_STATS_MS)
        gl_interval_event(ms);
}

/* ---- read-only API ---------------------------------------------------- */
static const gl_seat_t *gl_grid_of(int seat)
{
    if (!gl.on || seat < 1 || seat >= GL_MAX_SEATS || !gl_seat[seat].used || !gl_seat[seat].grid_valid)
        return NULL;
    return &gl_seat[seat];
}

int lidar_cell(int seat, float x, float z, int *layers)
{
    if (layers)
        *layers = 0;
    const gl_seat_t *s = gl_grid_of(seat);
    if (!s || !isfinite(x) || !isfinite(z))
        return GL_CELL_NONE;
    const int i = gl_index(s, (int)floorf(x / GL_CELL_M), (int)floorf(z / GL_CELL_M));
    return i < 0 ? GL_CELL_NONE : gl_merged(s, i, layers);
}

float lidar_clearance_ex(int seat, float heading_rad, float half_width, int stop_at_unknown, int *why)
{
    if (why)
        *why = 0;
    const gl_seat_t *s = gl_grid_of(seat);
    if (!s || !isfinite(heading_rad))
        return -1.0f;
    const float dx = cosf(heading_rad), dz = sinf(heading_rad);
    const float x0 = s->tank.x, z0 = s->tank.z, range = gl.cfg.range_m;
    const float step = 0.5f * GL_CELL_M;
    const int lanes = half_width > 0.0f ? (int)ceilf(half_width / GL_CELL_M) : 0;
    for (float d = 0.0f; d <= range; d += step) {
        for (int k = -lanes; k <= lanes; ++k) {
            const float off = lanes ? half_width * (float)k / (float)lanes : 0.0f;
            const float x = x0 + d * dx - off * dz, z = z0 + d * dz + off * dx;
            const int i = gl_index(s, (int)floorf(x / GL_CELL_M), (int)floorf(z / GL_CELL_M));
            if (i < 0) {
                if (why)
                    *why = 3;
                return d;
            }
            const int m = gl_merged(s, i, NULL);
            if (m == GL_CELL_OCCUPIED || (stop_at_unknown && m == GL_CELL_UNKNOWN)) {
                if (why)
                    *why = m == GL_CELL_OCCUPIED ? 1 : 2;
                return d;
            }
        }
    }
    return range;
}

int lidar_block(int seat, int wc0, int wr0, int n, uint8_t *out)
{
    const gl_seat_t *s = gl_grid_of(seat);
    if (!s || n <= 0 || !out)
        return -1;
    memset(out, GL_BLOCK_UNKNOWN, (size_t)n * (size_t)n);
    /* The overlap of the asked block with the grid window. */
    const int c0 = wc0 > s->ox ? wc0 : s->ox, c1 = wc0 + n < s->ox + GL_GRID ? wc0 + n : s->ox + GL_GRID;
    const int r0 = wr0 > s->oz ? wr0 : s->oz, r1 = wr0 + n < s->oz + GL_GRID ? wr0 + n : s->oz + GL_GRID;
    int known = 0;
    for (int wr = r0; wr < r1; ++wr) {
        uint8_t *row = out + (size_t)(wr - wr0) * (size_t)n;
        const int base = gl_mod(wr) * GL_GRID;
        for (int wc = c0; wc < c1; ++wc) {
            const int i = base + gl_mod(wc);
            const int lo = gl_layer_state(s, 0, i), hi = gl_layer_state(s, 1, i);
            uint8_t v = GL_BLOCK_UNKNOWN;
            if (lo == GL_CELL_OCCUPIED || hi == GL_CELL_OCCUPIED)
                v = GL_BLOCK_OCCUPIED;
            else if ((lo == GL_CELL_FREE && s->nobs[0][i] >= 2) || (hi == GL_CELL_FREE && s->nobs[1][i] >= 2))
                v = GL_BLOCK_FREE;
            row[wc - wc0] = v;
            known += v != GL_BLOCK_UNKNOWN;
        }
    }
    return known;
}

float lidar_clearance(int seat, float heading_rad)
{
    return lidar_clearance_ex(seat, heading_rad, 0.0f, 0, NULL);
}

const gl_ring_t *lidar_ring(int seat)
{
    if (!gl.on || seat < 1 || seat >= GL_MAX_SEATS || !gl_seat[seat].used)
        return NULL;
    return &gl_seat[seat].ring;
}

#ifdef GL_TESTING
/* Unit-test hooks (tests/lidar_unit_test.c), not in the DLL. */
void gl_debug_recentre(int seat, float x, float z)
{
    gl_recentre(&gl_seat[seat], x, z);
    gl_seat[seat].tank.x = x;
    gl_seat[seat].tank.z = z;
}

void gl_debug_budget(int us)
{
    gl.cfg.us_per_tick = us;
}

int gl_debug_logodds(int seat, int layer, float x, float z)
{
    const int i = gl_index(&gl_seat[seat], (int)floorf(x / GL_CELL_M), (int)floorf(z / GL_CELL_M));
    return i < 0 ? -1000 : gl_seat[seat].L[layer][i];
}

int gl_debug_cursor(int seat)
{
    return gl_seat[seat].cursor;
}
#endif

uint64_t lidar_age_ms(int seat, uint64_t ms)
{
    const gl_ring_t *r = lidar_ring(seat);
    if (!r || !r->cycles)
        return UINT64_MAX;
    uint64_t oldest = ms;
    for (int l = 0; l < GL_LAYERS; ++l)
        for (int a = 0; a < GL_AZIMUTHS; ++a)
            if (r->ms[l][a] < oldest)
                oldest = r->ms[l][a];
    return ms - oldest;
}
