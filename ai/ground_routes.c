/* s26 route library loader and the grid searches on its clearance grid (ground_routes.h). Plain C and stdio. */
#define _CRT_SECURE_NO_WARNINGS
#include "ground_routes.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ---- loading ---- */

typedef struct {
    const uint8_t *p;
    size_t n, at;
    int bad;
} gr_rd_t;

static const uint8_t *gr_take(gr_rd_t *r, size_t k)
{
    if (r->bad || k > r->n - r->at) {
        r->bad = 1;
        return NULL;
    }
    const uint8_t *q = r->p + r->at;
    r->at += k;
    return q;
}

static uint8_t gr_u8(gr_rd_t *r)
{
    const uint8_t *q = gr_take(r, 1);
    return q ? q[0] : 0;
}

static uint16_t gr_u16(gr_rd_t *r)
{
    uint16_t v = 0;
    const uint8_t *q = gr_take(r, 2);
    if (q)
        memcpy(&v, q, 2);
    return v;
}

static uint32_t gr_u32(gr_rd_t *r)
{
    uint32_t v = 0;
    const uint8_t *q = gr_take(r, 4);
    if (q)
        memcpy(&v, q, 4);
    return v;
}

static float gr_f32(gr_rd_t *r)
{
    float v = 0.0f;
    const uint8_t *q = gr_take(r, 4);
    if (q)
        memcpy(&v, q, 4);
    return v;
}

static void gr_str(gr_rd_t *r, char *out, size_t k)
{
    const uint8_t *q = gr_take(r, k);
    memset(out, 0, k + 1);
    if (q)
        memcpy(out, q, k);
}

static uint8_t *gr_slurp(const char *path, size_t *size)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return NULL;
    uint8_t *data = NULL;
    if (fseek(f, 0, SEEK_END) == 0) {
        const long n = ftell(f);
        if (n > 0 && fseek(f, 0, SEEK_SET) == 0) {
            data = (uint8_t *)malloc((size_t)n);
            if (data && fread(data, 1, (size_t)n, f) != (size_t)n) {
                free(data);
                data = NULL;
            }
            *size = (size_t)n;
        }
    }
    fclose(f);
    return data;
}

void gr_free(gr_lib_t *lib)
{
    free(lib->poses);
    free(lib->targets);
    free(lib->slots);
    free(lib->ranks);
    free(lib->routes);
    free(lib->px);
    free(lib->pz);
    free(lib->kappa);
    free(lib->arc);
    free(lib->clr);
    free(lib->ref_start);
    free(lib->refs);
    free(lib->bits);
    free(lib->clr05);
    memset(lib, 0, sizeof *lib);
}

#define GR_FAILS(...)                                          \
    do {                                                       \
        if (msg && msg_size)                                   \
            snprintf(msg, msg_size, __VA_ARGS__);              \
        free(rb);                                              \
        free(mb);                                              \
        gr_free(lib);                                          \
        return 0;                                              \
    } while (0)

int gr_load(gr_lib_t *lib, const char *routes_path, const char *map_path, char *msg, size_t msg_size)
{
    const clock_t c0 = clock();
    memset(lib, 0, sizeof *lib);
    size_t rn = 0, mn = 0;
    uint8_t *rb = gr_slurp(routes_path, &rn), *mb = NULL;
    if (!rb)
        GR_FAILS("cannot read %s", routes_path);
    mb = gr_slurp(map_path, &mn);
    if (!mb)
        GR_FAILS("cannot read %s", map_path);

    /* routes.bin */
    gr_rd_t r = {rb, rn, 0, 0};
    char magic[9];
    gr_str(&r, magic, 8);
    if (r.bad || strcmp(magic, "WTRTE004"))
        GR_FAILS("%s: magic is not WTRTE004", routes_path);
    lib->version = gr_u32(&r);
    if (lib->version != 4)
        GR_FAILS("%s: version %u, only 4 is understood", routes_path, lib->version);
    lib->n_poses = gr_u32(&r);
    lib->n_targets = gr_u32(&r);
    lib->n_slots = gr_u32(&r);
    lib->n_routes = gr_u32(&r);
    if (r.bad || !lib->n_poses || !lib->n_targets || !lib->n_routes || lib->n_poses > 4096 || lib->n_targets > 4096
        || lib->n_slots > 65536 || lib->n_routes > 1000000)
        GR_FAILS("%s: implausible counts %u poses, %u targets, %u slots, %u routes", routes_path, lib->n_poses,
                 lib->n_targets, lib->n_slots, lib->n_routes);
    lib->poses = (gr_pose_t *)calloc(lib->n_poses, sizeof *lib->poses);
    lib->targets = (gr_target_t *)calloc(lib->n_targets, sizeof *lib->targets);
    lib->slots = (gr_slot_t *)calloc(lib->n_slots ? lib->n_slots : 1, sizeof *lib->slots);
    lib->ranks = (uint8_t *)calloc((size_t)lib->n_poses * lib->n_targets, 1);
    lib->routes = (gr_route_t *)calloc(lib->n_routes, sizeof *lib->routes);
    if (!lib->poses || !lib->targets || !lib->slots || !lib->ranks || !lib->routes)
        GR_FAILS("out of memory");
    for (uint32_t i = 0; i < lib->n_poses; ++i) {
        gr_pose_t *p = &lib->poses[i];
        gr_str(&r, p->id, 8);
        gr_str(&r, p->area, 8);
        p->x = gr_f32(&r);
        p->z = gr_f32(&r);
        p->heading_deg = gr_f32(&r);
        /* s32: a pose needs finite x and z; its heading may be NaN (an unknown spawn heading, Eastern Europe) */
        if (!isfinite(p->x) || !isfinite(p->z))
            GR_FAILS("%s: pose %u position not finite", routes_path, i);
        if (!isfinite(p->heading_deg)) {
            p->heading_deg = NAN;       /* one mark for every non-finite heading */
            ++lib->n_poses_noheading;
        }
    }
    for (uint32_t i = 0; i < lib->n_targets; ++i) {
        gr_target_t *t = &lib->targets[i];
        gr_str(&r, t->id, 16);
        t->kind = gr_u8(&r);
        t->army = gr_u8(&r);
        t->post_kind = gr_u8(&r);
        t->standing = gr_u8(&r);
        t->x = gr_f32(&r);
        t->z = gr_f32(&r);
        t->sx = gr_f32(&r);
        t->sz = gr_f32(&r);
        t->radius = gr_f32(&r);
        t->inner_radius = gr_f32(&r);
        t->faces_x = gr_f32(&r);
        t->faces_z = gr_f32(&r);
        if (t->kind > 1 || t->army > 2 || t->post_kind > 4 || !isfinite(t->x) || !isfinite(t->z)
            || (t->kind == 0 && !(t->radius > 0.0f)))
            GR_FAILS("%s: target %u (%s) does not fit the layout (kind %u army %u post kind %u radius %g)", routes_path, i,
                     t->id, t->kind, t->army, t->post_kind, (double)t->radius);
        lib->n_zones += t->kind == 0;
    }
    for (uint32_t i = 0; i < lib->n_slots; ++i) {
        gr_slot_t *s = &lib->slots[i];
        s->target = gr_u16(&r);
        s->number = gr_u16(&r);
        s->x = gr_f32(&r);
        s->z = gr_f32(&r);
        s->clearance = gr_f32(&r);
        if (!r.bad && (s->target >= lib->n_targets || !isfinite(s->x) || !isfinite(s->z)))
            GR_FAILS("%s: slot %u does not fit the layout (target %u)", routes_path, i, s->target);
    }
    {
        const uint8_t *q = gr_take(&r, (size_t)lib->n_poses * lib->n_targets);
        if (q)
            memcpy(lib->ranks, q, (size_t)lib->n_poses * lib->n_targets);
    }
    if (r.bad)
        GR_FAILS("%s: short before the routes", routes_path);
    /* first pass: route headers and the sample count */
    const size_t routes_at = r.at;
    uint64_t total = 0;
    for (uint32_t i = 0; i < lib->n_routes; ++i) {
        gr_take(&r, 60);
        const uint32_t n = gr_u32(&r);
        if (r.bad || n < 2 || n > 200000)
            GR_FAILS("%s: route %u has %u samples or the file is short", routes_path, i, n);
        gr_take(&r, (size_t)n * 13u);
        total += n;
    }
    if (r.bad || r.at != r.n)
        GR_FAILS("%s: layout mismatch: %zu of %zu bytes accounted for (a newer or older layout under the same "
                 "magic?)", routes_path, r.at, r.n);
    lib->n_samples = (uint32_t)total;
    lib->px = (float *)malloc(sizeof(float) * total);
    lib->pz = (float *)malloc(sizeof(float) * total);
    lib->kappa = (float *)malloc(sizeof(float) * total);
    lib->arc = (float *)malloc(sizeof(float) * total);
    lib->clr = (uint8_t *)malloc(total);
    if (!lib->px || !lib->pz || !lib->kappa || !lib->arc || !lib->clr)
        GR_FAILS("out of memory for %llu samples", (unsigned long long)total);
    r.at = routes_at;
    uint32_t first = 0;
    for (uint32_t i = 0; i < lib->n_routes; ++i) {
        gr_route_t *t = &lib->routes[i];
        gr_str(&r, t->id, 32);
        t->kind = gr_u8(&r);
        t->rank = gr_u8(&r);
        t->flags = gr_u8(&r);
        t->fallback = gr_u8(&r);
        t->pose = gr_u16(&r);
        t->start_target = gr_u16(&r);
        t->end_target = gr_u16(&r);
        t->gear = gr_u8(&r);
        gr_u8(&r);
        t->total_s = gr_f32(&r);
        t->drive_s = gr_f32(&r);
        t->turn_deg = gr_f32(&r);
        t->length = gr_f32(&r);
        t->n = gr_u32(&r);
        t->first = first;
        const int spawn = t->kind <= 1;
        if (t->kind > 3 || t->end_target >= lib->n_targets || t->gear > 3
            || (spawn && t->pose >= lib->n_poses) || (!spawn && t->start_target >= lib->n_targets))
            GR_FAILS("%s: route %u (%s) does not fit the layout (kind %u pose %u start %u end %u gear %u)", routes_path,
                     i, t->id, t->kind, t->pose, t->start_target, t->end_target, t->gear);
        lib->n_designated += (t->flags & GR_FLAG_DESIGNATED) != 0;
        for (uint32_t k = 0; k < t->n; ++k) {
            lib->px[first + k] = gr_f32(&r);
            lib->pz[first + k] = gr_f32(&r);
        }
        const uint8_t *q = gr_take(&r, t->n);
        if (q)
            memcpy(lib->clr + first, q, t->n);
        for (uint32_t k = 0; k < t->n; ++k)
            lib->kappa[first + k] = gr_f32(&r);
        float arc = 0.0f;
        for (uint32_t k = 0; k < t->n; ++k) {
            const uint32_t j = first + k;
            if (!isfinite(lib->px[j]) || !isfinite(lib->pz[j]))
                GR_FAILS("%s: route %u sample %u not finite", routes_path, i, k);
            if (!isfinite(lib->kappa[j]))
                lib->kappa[j] = 0.0f;
            if (k)
                arc += hypotf(lib->px[j] - lib->px[j - 1], lib->pz[j] - lib->pz[j - 1]);
            lib->arc[j] = arc;
        }
        first += t->n;
    }
    if (r.bad || r.at != r.n)
        GR_FAILS("%s: layout mismatch in the second pass", routes_path);
    /* per target: the routes that end there */
    lib->ref_start = (int *)calloc(lib->n_targets + 1u, sizeof(int));
    lib->refs = (gr_ref_t *)calloc(2u * lib->n_routes, sizeof(gr_ref_t));
    if (!lib->ref_start || !lib->refs)
        GR_FAILS("out of memory");
    for (uint32_t i = 0; i < lib->n_routes; ++i) {
        ++lib->ref_start[lib->routes[i].end_target + 1];
        if (lib->routes[i].kind >= 2)
            ++lib->ref_start[lib->routes[i].start_target + 1];
    }
    for (uint32_t t = 0; t < lib->n_targets; ++t)
        lib->ref_start[t + 1] += lib->ref_start[t];
    {
        int *fill = (int *)calloc(lib->n_targets, sizeof(int));
        if (!fill)
            GR_FAILS("out of memory");
        for (uint32_t i = 0; i < lib->n_routes; ++i) {
            const gr_route_t *t = &lib->routes[i];
            lib->refs[lib->ref_start[t->end_target] + fill[t->end_target]++] = (gr_ref_t){(int)i, 0};
            if (t->kind >= 2)
                lib->refs[lib->ref_start[t->start_target] + fill[t->start_target]++] = (gr_ref_t){(int)i, 1};
        }
        free(fill);
    }

    /* map.bin */
    gr_rd_t m = {mb, mn, 0, 0};
    gr_str(&m, magic, 8);
    if (m.bad || strcmp(magic, "WTMAP004"))
        GR_FAILS("%s: magic is not WTMAP004", map_path);
    lib->x0 = gr_f32(&m);
    lib->z0 = gr_f32(&m);
    lib->cell = gr_f32(&m);
    lib->cell05 = gr_f32(&m);
    lib->cols = gr_u32(&m);
    lib->rows = gr_u32(&m);
    lib->cols05 = gr_u32(&m);
    lib->rows05 = gr_u32(&m);
    lib->nb = (lib->cols + 7u) / 8u;
    if (m.bad || !(lib->cell > 0.0f) || !(lib->cell05 > 0.0f) || !lib->cols || !lib->rows || !lib->cols05
        || !lib->rows05 || lib->cols > 100000 || lib->rows > 100000 || lib->cols05 > 50000 || lib->rows05 > 50000
        || fabsf(lib->cell05 - GR_STEP) > 1e-4f)
        GR_FAILS("%s: implausible grid (%u x %u at %g, %u x %u at %g)", map_path, lib->cols, lib->rows,
                 (double)lib->cell, lib->cols05, lib->rows05, (double)lib->cell05);
    const size_t nbits = (size_t)lib->rows * lib->nb, nclr = (size_t)lib->rows05 * lib->cols05;
    if (m.at + nbits + nclr != m.n)
        GR_FAILS("%s: layout mismatch: %zu bytes, the header promises %zu", map_path, m.n, m.at + nbits + nclr);
    lib->bits = (uint8_t *)malloc(nbits);
    lib->clr05 = (uint8_t *)malloc(nclr);
    if (!lib->bits || !lib->clr05)
        GR_FAILS("out of memory for the map");
    memcpy(lib->bits, mb + m.at, nbits);
    memcpy(lib->clr05, mb + m.at + nbits, nclr);
    lib->bytes = (size_t)total * 17u + nbits + nclr + lib->n_routes * sizeof(gr_route_t) + 2u * lib->n_routes * sizeof(gr_ref_t);
    free(rb);
    free(mb);
    lib->loaded = 1;
    lib->load_ms = 1000.0 * (double)(clock() - c0) / (double)CLOCKS_PER_SEC;
    if (msg && msg_size)
        msg[0] = 0;
    return 1;
}

void gr_describe(const gr_lib_t *lib, char *out, size_t size)
{
    uint32_t posts = lib->n_targets - lib->n_zones;
    snprintf(out, size, "%u poses (%u without a heading: matched by position, no start turn), %u targets (%u zones, %u posts), "
             "%u slots, %u routes, %u designated, %u samples, %.1f MB, loaded in %.0f ms", lib->n_poses,
             lib->n_poses_noheading, lib->n_targets, lib->n_zones, posts, lib->n_slots, lib->n_routes, lib->n_designated,
             lib->n_samples, (double)lib->bytes / 1048576.0, lib->load_ms);
}

/* ---- map lookups ---- */

float gr_clearance(const gr_lib_t *lib, float x, float z)
{
    const float fc = floorf((x - lib->x0) / lib->cell05), fr = floorf((z - lib->z0) / lib->cell05);
    if (!(fc >= 0.0f) || !(fr >= 0.0f) || fc >= (float)lib->cols05 || fr >= (float)lib->rows05)
        return 0.0f;
    return 0.1f * (float)lib->clr05[(size_t)fr * lib->cols05 + (size_t)fc];
}

int gr_hard(const gr_lib_t *lib, float x, float z)
{
    const float fc = floorf((x - lib->x0) / lib->cell), fr = floorf((z - lib->z0) / lib->cell);
    if (!(fc >= 0.0f) || !(fr >= 0.0f) || fc >= (float)lib->cols || fr >= (float)lib->rows)
        return 1;
    const size_t c = (size_t)fc, r = (size_t)fr;
    return (lib->bits[r * lib->nb + (c >> 3)] >> (c & 7u)) & 1;
}

/* s29: a hard cell on the straight line (x0, z0) -> (x1, z1), the hard bit mask marched in 0.5 m steps (both ends
 * included; off the map counts as hard, as gr_hard has it). 1 blocked. */
int gr_line_hard(const gr_lib_t *lib, float x0, float z0, float x1, float z1)
{
    const float len = hypotf(x1 - x0, z1 - z0);
    if (!isfinite(len))
        return 1;
    const int n = (int)ceilf(len / 0.5f);
    for (int k = 0; k <= n; ++k) {
        const float t = n ? (float)k / (float)n : 0.0f;
        if (gr_hard(lib, x0 + (x1 - x0) * t, z0 + (z1 - z0) * t))
            return 1;
    }
    return 0;
}

static float gr_point_rect(float qx, float qz, float cx, float cz, float fx, float fz, float hl, float hw)
{
    const float dx = qx - cx, dz = qz - cz;
    const float u = fmaxf(fabsf(dx * fx + dz * fz) - hl, 0.0f), v = fmaxf(fabsf(fx * dz - fz * dx) - hw, 0.0f);
    return hypotf(u, v);
}

float gr_hull_hard_distance(const gr_lib_t *lib, float cx, float cz, float heading, float hl, float hw, float reach)
{
    const float fx = cosf(heading), fz = sinf(heading);
    const float ex = fabsf(fx) * hl + fabsf(fz) * hw + reach, ez = fabsf(fz) * hl + fabsf(fx) * hw + reach;
    float best = reach;
    for (float z = cz - ez; z <= cz + ez; z += lib->cell)
        for (float x = cx - ex; x <= cx + ex; x += lib->cell) {
            /* the cell centre nearest (x, z) */
            const float c = floorf((x - lib->x0) / lib->cell), r = floorf((z - lib->z0) / lib->cell);
            const float qx = lib->x0 + (c + 0.5f) * lib->cell, qz = lib->z0 + (r + 0.5f) * lib->cell;
            if (!gr_hard(lib, qx, qz))
                continue;
            const float d = gr_point_rect(qx, qz, cx, cz, fx, fz, hl, hw);
            if (d < best)
                best = d;
        }
    return best;
}

/* ---- targets, poses ---- */

int gr_target_of(const gr_lib_t *lib, float x, float z, int army, float post_radius)
{
    /* army is ignored: the army labels of the library's post table do not match the orders the commander gives
     * (battles b5f6 / ee99), so a post maps by geometry alone, the nearest within post_radius */
    (void)army;
    int best = -1;
    float bd = INFINITY;
    for (uint32_t i = 0; i < lib->n_targets; ++i) {
        const gr_target_t *t = &lib->targets[i];
        if (t->kind != 0)
            continue;
        const float d = hypotf(x - t->x, z - t->z);
        if (d <= t->radius && d < bd) {
            bd = d;
            best = (int)i;
        }
    }
    if (best >= 0)
        return best;
    for (uint32_t i = 0; i < lib->n_targets; ++i) {
        const gr_target_t *t = &lib->targets[i];
        if (t->kind != 1)
            continue;
        float d = hypotf(x - t->x, z - t->z);
        if (isfinite(t->sx) && isfinite(t->sz))
            d = fminf(d, hypotf(x - t->sx, z - t->sz));
        if (d <= post_radius && d < bd) {
            bd = d;
            best = (int)i;
        }
    }
    return best;
}

int gr_pose_of(const gr_lib_t *lib, float x, float z, float heading_rad, float dist, float deg)
{
    int best = -1;
    float bd = INFINITY;
    for (uint32_t i = 0; i < lib->n_poses; ++i) {
        const gr_pose_t *p = &lib->poses[i];
        const float d = hypotf(x - p->x, z - p->z);
        /* s32: a pose whose heading is unknown (NaN, Eastern Europe's spawns) matches by position only */
        const float e = isfinite(p->heading_deg)
                        ? fabsf(remainderf(heading_rad - p->heading_deg * 0.017453293f, 6.2831853f)) * 57.29578f : 0.0f;
        if (d <= dist && e <= deg && d < bd) {
            bd = d;
            best = (int)i;
        }
    }
    return best;
}

int gr_designated(const gr_lib_t *lib, int pose, int target)
{
    int best = -1, flagged = 0;
    for (uint32_t i = 0; i < lib->n_routes; ++i) {
        const gr_route_t *t = &lib->routes[i];
        if (t->kind > 1 || t->pose != (uint32_t)pose || t->end_target != (uint32_t)target)
            continue;
        const int f = (t->flags & GR_FLAG_DESIGNATED) != 0;
        if (best < 0 || (f && !flagged)
            || (f == flagged && (t->rank < lib->routes[best].rank
                                 || (t->rank == lib->routes[best].rank && t->total_s < lib->routes[best].total_s)))) {
            best = (int)i;
            flagged = f;
        }
    }
    return best;
}

/* s32: variant 0 = gr_designated (the recommended corridor); 1 / 2 = the spawn route (pose -> target) whose fallback
 * order is that number (a flank corridor entering the zone from another side), else the designated one. -1 none. */
int gr_route_variant(const gr_lib_t *lib, int pose, int target, int variant)
{
    if (variant == 1 || variant == 2) {
        int best = -1;
        for (uint32_t i = 0; i < lib->n_routes; ++i) {
            const gr_route_t *t = &lib->routes[i];
            if (t->kind > 1 || t->pose != (uint32_t)pose || t->end_target != (uint32_t)target || t->fallback != variant)
                continue;
            if (best < 0 || t->total_s < lib->routes[best].total_s)
                best = (int)i;
        }
        if (best >= 0)
            return best;
    }
    return gr_designated(lib, pose, target);
}

/* ---- grid searches ---- */

#define GR_WMAX 640
#define GR_WCELLS (GR_WMAX * GR_WMAX)
static float gs_g[GR_WCELLS], gs_len[GR_WCELLS], gs_clr[GR_WCELLS];
static uint32_t gs_stamp[GR_WCELLS], gs_clr_stamp[GR_WCELLS], gs_closed[GR_WCELLS];
static uint8_t gs_dir[GR_WCELLS];
static int gs_heap[GR_WCELLS], gs_pos[GR_WCELLS];
static float gs_key[GR_WCELLS];
static int gs_heap_n;
static uint32_t gs_gen;
static int gs_wc0, gs_wr0, gs_ww, gs_wh, gs_valid;
static int gs_start_c, gs_start_r;
static const gr_box_t *gs_boxes;
static int gs_nbox;

static const int gs_dc[8] = {1, -1, 0, 0, 1, 1, -1, -1};
static const int gs_dr[8] = {0, 0, 1, -1, 1, -1, 1, -1};

float gr_eff_clearance(const gr_lib_t *lib, float x, float z, const gr_box_t *boxes, int nbox)
{
    float c = gr_clearance(lib, x, z);
    for (int k = 0; k < nbox; ++k)
        c = fminf(c, gr_point_rect(x, z, boxes[k].cx, boxes[k].cz, boxes[k].fx, boxes[k].fz, boxes[k].hl, boxes[k].hw));
    return c;
}

static float gs_cx(const gr_lib_t *lib, int c)
{
    return lib->x0 + ((float)c + 0.5f) * lib->cell05;
}

static float gs_cz(const gr_lib_t *lib, int r)
{
    return lib->z0 + ((float)r + 0.5f) * lib->cell05;
}

/* clearance of a window cell (map cell and the stamped boxes), cached per search */
static float gs_cell_clr(const gr_lib_t *lib, int c, int r)
{
    if (c < 0 || r < 0 || c >= (int)lib->cols05 || r >= (int)lib->rows05)
        return 0.0f;
    const int i = (r - gs_wr0) * gs_ww + (c - gs_wc0);
    if (gs_clr_stamp[i] == gs_gen)
        return gs_clr[i];
    float v = 0.1f * (float)lib->clr05[(size_t)r * lib->cols05 + (size_t)c];
    if (gs_nbox && v >= GR_CLEAR_MIN) {
        const float x = gs_cx(lib, c), z = gs_cz(lib, r);
        for (int k = 0; k < gs_nbox; ++k)
            v = fminf(v, gr_point_rect(x, z, gs_boxes[k].cx, gs_boxes[k].cz, gs_boxes[k].fx, gs_boxes[k].fz,
                                       gs_boxes[k].hl, gs_boxes[k].hw));
    }
    gs_clr_stamp[i] = gs_gen;
    gs_clr[i] = v;
    return v;
}

static int gs_inside(int c, int r)
{
    return c >= gs_wc0 && r >= gs_wr0 && c < gs_wc0 + gs_ww && r < gs_wr0 + gs_wh;
}

static void gs_heap_up(int at)
{
    const int item = gs_heap[at];
    const float key = gs_key[item];
    while (at > 0) {
        const int parent = (at - 1) / 2;
        if (gs_key[gs_heap[parent]] <= key)
            break;
        gs_heap[at] = gs_heap[parent];
        gs_pos[gs_heap[at]] = at;
        at = parent;
    }
    gs_heap[at] = item;
    gs_pos[item] = at;
}

static void gs_heap_down(int at)
{
    const int item = gs_heap[at];
    const float key = gs_key[item];
    for (;;) {
        int child = 2 * at + 1;
        if (child >= gs_heap_n)
            break;
        if (child + 1 < gs_heap_n && gs_key[gs_heap[child + 1]] < gs_key[gs_heap[child]])
            ++child;
        if (gs_key[gs_heap[child]] >= key)
            break;
        gs_heap[at] = gs_heap[child];
        gs_pos[gs_heap[at]] = at;
        at = child;
    }
    gs_heap[at] = item;
    gs_pos[item] = at;
}

static void gs_push(int i, float key)
{
    if (gs_stamp[i] == gs_gen && gs_pos[i] >= 0) {
        gs_key[i] = key;
        gs_heap_up(gs_pos[i]);
        return;
    }
    gs_key[i] = key;
    gs_heap[gs_heap_n] = i;
    gs_pos[i] = gs_heap_n;
    gs_heap_up(gs_heap_n++);
}

static int gs_pop(void)
{
    const int top = gs_heap[0];
    gs_pos[top] = -1;
    if (--gs_heap_n > 0) {
        gs_heap[0] = gs_heap[gs_heap_n];
        gs_pos[gs_heap[0]] = 0;
        gs_heap_down(0);
    }
    return top;
}

static void gs_window(const gr_lib_t *lib, int c0, int r0, int c1, int r1)
{
    if (c0 < 0)
        c0 = 0;
    if (r0 < 0)
        r0 = 0;
    if (c1 > (int)lib->cols05 - 1)
        c1 = (int)lib->cols05 - 1;
    if (r1 > (int)lib->rows05 - 1)
        r1 = (int)lib->rows05 - 1;
    gs_wc0 = c0;
    gs_wr0 = r0;
    gs_ww = c1 - c0 + 1;
    gs_wh = r1 - r0 + 1;
    if (++gs_gen == 0) {
        memset(gs_stamp, 0, sizeof gs_stamp);
        memset(gs_clr_stamp, 0, sizeof gs_clr_stamp);
        memset(gs_closed, 0, sizeof gs_closed);
        gs_gen = 1;
    }
    gs_heap_n = 0;
}

/* nearest cell (window) with clearance >= GR_CLEAR_MIN within radius m of (x, z); 0 none */
static int gs_free_near(const gr_lib_t *lib, float x, float z, float radius, int *oc, int *orr)
{
    const int c = (int)floorf((x - lib->x0) / lib->cell05), r = (int)floorf((z - lib->z0) / lib->cell05);
    const int k = (int)ceilf(radius / lib->cell05);
    float bd = INFINITY;
    for (int dr = -k; dr <= k; ++dr)
        for (int dc = -k; dc <= k; ++dc) {
            const int cc = c + dc, rr = r + dr;
            if (!gs_inside(cc, rr) || gs_cell_clr(lib, cc, rr) < GR_CLEAR_MIN)
                continue;
            const float d = hypotf(gs_cx(lib, cc) - x, gs_cz(lib, rr) - z);
            if (d <= radius && d < bd) {
                bd = d;
                *oc = cc;
                *orr = rr;
            }
        }
    return isfinite(bd);
}

static float gs_factor(float clr)
{
    const float f = 1.0f - (clr - GR_CLEAR_MIN) / 4.0f;
    return 1.0f + (f < 0.0f ? 0.0f : f > 1.0f ? 1.0f : f);
}

/* One search: Dijkstra (no goal, bounded by max_len of path) or A* (goal cell, heuristic). */
static int gs_run(const gr_lib_t *lib, int sc, int sr, int gc, int gr, float max_len, gr_info_t *info)
{
    const int start = (sr - gs_wr0) * gs_ww + (sc - gs_wc0);
    const int goal = gc >= 0 ? (gr - gs_wr0) * gs_ww + (gc - gs_wc0) : -1;
    const float gx = gc >= 0 ? gs_cx(lib, gc) : 0.0f, gz = gc >= 0 ? gs_cz(lib, gr) : 0.0f;
    gs_stamp[start] = gs_gen;
    gs_g[start] = 0.0f;
    gs_len[start] = 0.0f;
    gs_dir[start] = 255;
    gs_pos[start] = -1;
    gs_push(start, goal >= 0 ? hypotf(gs_cx(lib, sc) - gx, gs_cz(lib, sr) - gz) : 0.0f);
    gs_start_c = sc;
    gs_start_r = sr;
    int expanded = 0;
    while (gs_heap_n > 0) {
        const int i = gs_pop();
        if (gs_closed[i] == gs_gen)
            continue;
        gs_closed[i] = gs_gen;
        ++expanded;
        if (i == goal) {
            info->expanded = expanded;
            info->cost = gs_g[i];
            info->length = gs_len[i];
            return GR_OK;
        }
        const int c = gs_wc0 + i % gs_ww, r = gs_wr0 + i / gs_ww;
        for (int d = 0; d < 8; ++d) {
            const int nc = c + gs_dc[d], nr = r + gs_dr[d];
            if (!gs_inside(nc, nr))
                continue;
            const int j = (nr - gs_wr0) * gs_ww + (nc - gs_wc0);
            if (gs_closed[j] == gs_gen)
                continue;
            const float clr = gs_cell_clr(lib, nc, nr);
            if (clr < GR_CLEAR_MIN)
                continue;
            const float step = d < 4 ? lib->cell05 : lib->cell05 * 1.41421356f;
            const float len = gs_len[i] + step;
            if (goal < 0 && len > max_len)
                continue;
            const float g = gs_g[i] + step * 0.5f * (gs_factor(clr) + gs_factor(gs_cell_clr(lib, c, r)));
            if (gs_stamp[j] == gs_gen && gs_g[j] <= g)
                continue;
            if (gs_stamp[j] != gs_gen)
                gs_pos[j] = -1;
            gs_stamp[j] = gs_gen;
            gs_g[j] = g;
            gs_len[j] = len;
            gs_dir[j] = (uint8_t)d;
            gs_push(j, goal >= 0 ? g + hypotf(gs_cx(lib, nc) - gx, gs_cz(lib, nr) - gz) : g);
        }
    }
    info->expanded = expanded;
    return goal >= 0 ? GR_FAIL : GR_OK;
}

static int gs_cell_of(const gr_lib_t *lib, float x, float z, int *c, int *r)
{
    *c = (int)floorf((x - lib->x0) / lib->cell05);
    *r = (int)floorf((z - lib->z0) / lib->cell05);
    return gs_inside(*c, *r);
}

/* start cell: its own, or the nearest free one within relocate m (0: none) */
static int gs_start(const gr_lib_t *lib, float x, float z, float relocate, int *sc, int *sr)
{
    if (!gs_cell_of(lib, x, z, sc, sr))
        return GR_START_BLOCKED;
    if (gs_cell_clr(lib, *sc, *sr) >= GR_CLEAR_MIN)
        return GR_OK;
    if (relocate > 0.0f && gs_free_near(lib, x, z, relocate, sc, sr))
        return GR_OK;
    return GR_START_BLOCKED;
}

int gr_dijkstra(const gr_lib_t *lib, float x, float z, float max_len, float relocate, const gr_box_t *boxes, int nbox,
                gr_info_t *info)
{
    gr_info_t dummy;
    if (!info)
        info = &dummy;
    memset(info, 0, sizeof *info);
    gs_valid = 0;
    gs_boxes = boxes;
    gs_nbox = nbox;
    const int c = (int)floorf((x - lib->x0) / lib->cell05), r = (int)floorf((z - lib->z0) / lib->cell05);
    int k = (int)ceilf((max_len + relocate) / lib->cell05) + 2;
    if (2 * k + 1 > GR_WMAX)
        k = (GR_WMAX - 1) / 2;
    gs_window(lib, c - k, r - k, c + k, r + k);
    int sc, sr;
    info->status = gs_start(lib, x, z, relocate, &sc, &sr);
    if (info->status != GR_OK)
        return info->status;
    info->status = gs_run(lib, sc, sr, -1, -1, max_len, info);
    gs_valid = info->status == GR_OK;
    return info->status;
}

float gr_reach_cost(const gr_lib_t *lib, float x, float z)
{
    int c, r;
    if (!gs_valid || !gs_cell_of(lib, x, z, &c, &r))
        return INFINITY;
    const int i = (r - gs_wr0) * gs_ww + (c - gs_wc0);
    return gs_stamp[i] == gs_gen && gs_closed[i] == gs_gen ? gs_g[i] : INFINITY;
}

/* segment check: every 0.25 m at least thr of clearance */
static int gs_segment_ok(const gr_lib_t *lib, float ax, float az, float bx, float bz, float thr)
{
    const float len = hypotf(bx - ax, bz - az);
    const int n = (int)ceilf(len / 0.25f);
    for (int k = 1; k < n; ++k) {
        const float t = (float)k / (float)n;
        if (gr_eff_clearance(lib, ax + (bx - ax) * t, az + (bz - az) * t, gs_boxes, gs_nbox) < thr)
            return 0;
    }
    return 1;
}

#define GS_CHAIN 4096
static gr_pt_t gs_chain[GS_CHAIN], gs_pulled[GS_CHAIN];

/* cell chain from the start to cell (c, r) (cell centres), with the tank's own position first and (ex, ez) last */
static int gs_build_path(const gr_lib_t *lib, float sx, float sz, int c, int r, float ex, float ez, gr_pt_t *out,
                         int cap)
{
    int n = 0;
    int i = (r - gs_wr0) * gs_ww + (c - gs_wc0);
    if (gs_stamp[i] != gs_gen)
        return 0;
    /* backwards from the end cell */
    gs_chain[n++] = (gr_pt_t){ex, ez};
    for (int guard = 0; guard < GR_WCELLS; ++guard) {
        if (n >= GS_CHAIN - 2)
            return 0;
        gs_chain[n++] = (gr_pt_t){gs_cx(lib, c), gs_cz(lib, r)};
        const uint8_t d = gs_dir[i];
        if (d == 255)
            break;
        c -= gs_dc[d];
        r -= gs_dr[d];
        i = (r - gs_wr0) * gs_ww + (c - gs_wc0);
    }
    gs_chain[n++] = (gr_pt_t){sx, sz};
    /* reverse */
    for (int a = 0, b = n - 1; a < b; ++a, --b) {
        const gr_pt_t t = gs_chain[a];
        gs_chain[a] = gs_chain[b];
        gs_chain[b] = t;
    }
    /* string pull: from each anchor the farthest point reachable by a straight segment with the clearance. A tank
     * that stands where the clearance is less goes straight to its start cell first. */
    int m = 0;
    gs_pulled[m++] = gs_chain[0];
    int a = 0;
    if (n > 2 && gr_eff_clearance(lib, gs_chain[0].x, gs_chain[0].z, gs_boxes, gs_nbox) < GR_CLEAR_MIN) {
        gs_pulled[m++] = gs_chain[1];
        a = 1;
    }
    while (a < n - 1) {
        int best = a + 1;
        for (int j = a + 2; j < n && j < a + 800; ++j) {
            if (gs_segment_ok(lib, gs_chain[a].x, gs_chain[a].z, gs_chain[j].x, gs_chain[j].z, GR_CLEAR_MIN))
                best = j;
            else if (j > best + 6)
                break;
        }
        gs_pulled[m++] = gs_chain[best];
        a = best;
    }
    /* resample every GR_STEP */
    int k = 0;
    out[k++] = gs_pulled[0];
    float carry = 0.0f;
    for (int s = 1; s < m; ++s) {
        const gr_pt_t p = gs_pulled[s - 1], q = gs_pulled[s];
        const float len = hypotf(q.x - p.x, q.z - p.z);
        float at = GR_STEP - carry;
        while (at < len - 1e-4f) {
            if (k >= cap)
                return 0;
            out[k++] = (gr_pt_t){p.x + (q.x - p.x) * at / len, p.z + (q.z - p.z) * at / len};
            at += GR_STEP;
        }
        carry = len - (at - GR_STEP);
    }
    const gr_pt_t last = gs_pulled[m - 1];
    if (hypotf(last.x - out[k - 1].x, last.z - out[k - 1].z) > 0.05f) {
        if (k >= cap)
            return 0;
        out[k++] = last;
    }
    return k;
}

int gr_reach_path(const gr_lib_t *lib, float sx, float sz, float x, float z, const gr_box_t *boxes, int nbox,
                  gr_pt_t *out, int cap)
{
    int c, r;
    if (!gs_valid || !gs_cell_of(lib, x, z, &c, &r))
        return 0;
    gs_boxes = boxes;
    gs_nbox = nbox;
    return gs_build_path(lib, sx, sz, c, r, x, z, out, cap);
}

int gr_astar(const gr_lib_t *lib, float x, float z, float gx, float gz, float relocate, float goal_relocate,
             float max_straight, const gr_box_t *boxes, int nbox, gr_pt_t *out, int cap, int *count, gr_pt_t *goal_out,
             gr_info_t *info)
{
    gr_info_t dummy;
    if (!info)
        info = &dummy;
    memset(info, 0, sizeof *info);
    *count = 0;
    gs_valid = 0;
    gs_boxes = boxes;
    gs_nbox = nbox;
    if (hypotf(gx - x, gz - z) > max_straight)
        return info->status = GR_TOO_FAR;
    const float pad = 40.0f;
    int c0 = (int)floorf((fminf(x, gx) - pad - lib->x0) / lib->cell05), r0 = (int)floorf((fminf(z, gz) - pad - lib->z0) / lib->cell05);
    int c1 = (int)floorf((fmaxf(x, gx) + pad - lib->x0) / lib->cell05), r1 = (int)floorf((fmaxf(z, gz) + pad - lib->z0) / lib->cell05);
    if (c1 - c0 + 1 > GR_WMAX) {
        const int mid = (c0 + c1) / 2;
        c0 = mid - GR_WMAX / 2 + 1;
        c1 = c0 + GR_WMAX - 1;
    }
    if (r1 - r0 + 1 > GR_WMAX) {
        const int mid = (r0 + r1) / 2;
        r0 = mid - GR_WMAX / 2 + 1;
        r1 = r0 + GR_WMAX - 1;
    }
    gs_window(lib, c0, r0, c1, r1);
    int sc, sr, ec, er;
    info->status = gs_start(lib, x, z, relocate, &sc, &sr);
    if (info->status != GR_OK)
        return info->status;
    if (!gs_cell_of(lib, gx, gz, &ec, &er))
        return info->status = GR_GOAL_BLOCKED;
    float ex = gx, ez = gz;
    if (gs_cell_clr(lib, ec, er) < GR_CLEAR_MIN) {
        if (!gs_free_near(lib, gx, gz, goal_relocate, &ec, &er))
            return info->status = GR_GOAL_BLOCKED;
        ex = gs_cx(lib, ec);
        ez = gs_cz(lib, er);
    }
    if (goal_out)
        *goal_out = (gr_pt_t){ex, ez};
    info->status = gs_run(lib, sc, sr, ec, er, 0.0f, info);
    if (info->status != GR_OK)
        return info->status;
    *count = gs_build_path(lib, x, z, ec, er, ex, ez, out, cap);
    if (!*count)
        return info->status = GR_FAIL;
    return GR_OK;
}
