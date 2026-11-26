#ifndef GROUND_TERRAIN_H
#define GROUND_TERRAIN_H
/* Terrain layer: water and slope from the level's decoded heightfield.
 *
 * File bot_trees/terrain/<mission>.bin (WTTER001, tools/terrain/gen_terrain.py):
 * the level's HM2 heights around the battle area, the water level of the map
 * (NaN: none) and a flag per sample where tanks were logged standing well
 * above the heightfield (bridges and decks it does not have). Little endian:
 *   0 "WTTER001", 8 u32 nx, 12 u32 nz, 16 f32 x0, 20 f32 z0, 24 f32 step,
 *   28 f32 offset, 32 f32 scale (Y = offset + u16 * scale), 36 f32 water,
 *   40 u32 flags (bit 0: a u8 flag plane follows the heights), 44 u32 0,
 *   48 u16[nz][nx] heights (row = +Z), then u8[nz][nx] flags (bit 0 deck).
 *
 * The obstacle scan knows nothing about terrain: water has no collision and
 * reads clear, and a slope stops none of its rays. Tanks drowned following
 * routes over ponds (Poland: 5 lives in 295 s) and stood for minutes on
 * 25-110% slopes (Japan). A cell is closed where the water is deeper than
 * GT_WADE_M or the ground is steeper than GT_SLOPE_HARD over the track
 * length; moderate slopes and shallow water cost time instead. The limits
 * come from the battle logs (tools/terrain: slope_evidence.py,
 * water_evidence.py, drownings.py; numbers in the spec). Used by the adapter
 * (ground_geometry.inc) and the offline route test; header only, so both
 * read the same rule. */
#include <math.h>
#include <stdint.h>
#include <string.h>

#define GT_HALF_M 3.0f       /* grade over +-3 m, the length of the tracks: kerbs and crater rims do not count */
#define GT_SLOPE_SOFT 0.15f  /* grade where the extra cost starts (99% of driving at speed was below 24-37%) */
#define GT_SLOPE_HARD 0.45f  /* grade beyond which a cell is closed (Japan's long standstills: 43-78%) */
#define GT_WADE_M 1.0f       /* water deeper than this closes a cell (lived through 1.6 m, drowned at 2.9-3.3 m) */
#define GT_SLOPE_COST 2.0f   /* extra cost factor at the hard grade */
#define GT_WATER_COST 2.0f   /* extra cost factor at the wading depth */

typedef struct {
    const uint16_t *h;
    const uint8_t *flags;
    uint32_t nx, nz;
    float x0, z0, step, offset, scale, water;
} gt_terrain_t;

/* 1 if data[0..size) is a valid WTTER001 file; fills *t pointing into data. */
static int gt_parse(gt_terrain_t *t, const uint8_t *data, uint64_t size)
{
    memset(t, 0, sizeof *t);
    if (size < 48 || memcmp(data, "WTTER001", 8))
        return 0;
    uint32_t nx, nz, flags;
    float x0, z0, step, offset, scale, water;
    memcpy(&nx, data + 8, 4);
    memcpy(&nz, data + 12, 4);
    memcpy(&x0, data + 16, 4);
    memcpy(&z0, data + 20, 4);
    memcpy(&step, data + 24, 4);
    memcpy(&offset, data + 28, 4);
    memcpy(&scale, data + 32, 4);
    memcpy(&water, data + 36, 4);
    memcpy(&flags, data + 40, 4);
    const uint64_t samples = (uint64_t)nx * nz;
    if (nx < 2 || nz < 2 || nx > 20000 || nz > 20000 || 48 + samples * ((flags & 1) ? 3 : 2) != size
        || !isfinite(x0) || !isfinite(z0) || !(step > 0.1f && step <= 8.0f) || !isfinite(offset)
        || !(scale > 0.0f) || !isfinite(scale) || (isfinite(water) && fabsf(water) > 10000.0f))
        return 0;
    t->h = (const uint16_t *)(data + 48);
    t->flags = (flags & 1) ? data + 48 + samples * 2 : NULL;
    t->nx = nx;
    t->nz = nz;
    t->x0 = x0;
    t->z0 = z0;
    t->step = step;
    t->offset = offset;
    t->scale = scale;
    t->water = water;
    return 1;
}

/* Bilinear ground height at (x, z); NaN outside the file. */
static float gt_height(const gt_terrain_t *t, float x, float z)
{
    if (!t->h)
        return NAN;
    const float fx = (x - t->x0) / t->step, fz = (z - t->z0) / t->step;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)(t->nx - 1) || fz >= (float)(t->nz - 1))
        return NAN;
    const uint32_t i = (uint32_t)fx, j = (uint32_t)fz;
    const float tx = fx - (float)i, tz = fz - (float)j;
    const uint16_t *row = t->h + (size_t)j * t->nx + i;
    const float a = (float)row[0], b = (float)row[1], c = (float)row[t->nx], d = (float)row[t->nx + 1];
    return t->offset + t->scale * ((1.0f - tz) * ((1.0f - tx) * a + tx * b) + tz * ((1.0f - tx) * c + tx * d));
}

/* A deck flag on one of the four samples around (x, z). */
static int gt_deck(const gt_terrain_t *t, float x, float z)
{
    if (!t->flags)
        return 0;
    const float fx = (x - t->x0) / t->step, fz = (z - t->z0) / t->step;
    if (!(fx >= 0.0f) || !(fz >= 0.0f) || fx >= (float)(t->nx - 1) || fz >= (float)(t->nz - 1))
        return 0;
    const uint8_t *row = t->flags + (size_t)(uint32_t)fz * t->nx + (uint32_t)fx;
    return ((row[0] | row[1] | row[t->nx] | row[t->nx + 1]) & 1) != 0;
}

/* The rule for the scan cell centred on (x, z); sample is its WTOBS001 word.
 * 1: closed (too deep or too steep). *cost: the factor on the cell's length
 * (1 on level dry ground), *grade and *depth (optional) what it saw. */
static int gt_judge(const gt_terrain_t *t, float x, float z, uint16_t sample, float *cost, float *grade_out,
                    float *depth_out)
{
    *cost = 1.0f;
    if (grade_out)
        *grade_out = 0.0f;
    if (depth_out)
        *depth_out = 0.0f;
    const float h = gt_height(t, x, z);
    if (!isfinite(h))
        return 0;
    const float e = GT_HALF_M;
    /* Where tanks have stood on something the heightfield lacks, its slope
     * and its water say nothing. */
    if (gt_deck(t, x, z) || gt_deck(t, x + e, z) || gt_deck(t, x - e, z) || gt_deck(t, x, z + e) || gt_deck(t, x, z - e))
        return 0;
    const float hx0 = gt_height(t, x - e, z), hx1 = gt_height(t, x + e, z);
    const float hz0 = gt_height(t, x, z - e), hz1 = gt_height(t, x, z + e);
    const float gx = isfinite(hx0) && isfinite(hx1) ? (hx1 - hx0) / (2.0f * e) : 0.0f;
    const float gz = isfinite(hz0) && isfinite(hz1) ? (hz1 - hz0) / (2.0f * e) : 0.0f;
    const float grade = hypotf(gx, gz);
    /* Water, unless the scan's downward ray met something above it there (a
     * bridge deck, a pier): then the scan's own rule decides. */
    const float depth = isfinite(t->water) && !(sample & 0x0700u) ? t->water - h : 0.0f;
    if (grade_out)
        *grade_out = grade;
    if (depth_out)
        *depth_out = depth;
    if (grade > GT_SLOPE_HARD || depth > GT_WADE_M)
        return 1;
    const float steep = (grade - GT_SLOPE_SOFT) / (GT_SLOPE_HARD - GT_SLOPE_SOFT);
    *cost = 1.0f + GT_SLOPE_COST * fminf(1.0f, fmaxf(0.0f, steep))
            + GT_WATER_COST * fminf(1.0f, fmaxf(0.0f, depth / GT_WADE_M));
    return 0;
}

/* The scan's own rule (geometry_map_cell): an edge ray blocked, static
 * geometry, or a hit 0.5-3.5 m high. 1 solid, 0 clear, -1 never scanned. */
static int gt_scan_solid(uint16_t sample)
{
    if (!(sample & 0x8000u))
        return -1;
    const unsigned height = sample & 0x3fu;
    return (sample & 0x02c0u) || ((sample & 0x0700u) && height >= 1u && height < 8u) ? 1 : 0;
}
#endif
