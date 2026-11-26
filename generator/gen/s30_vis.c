/* s30_vis.c: line-of-sight helpers for the offline strategy analysis. No game code.
 *
 * Grid (row-major, H rows along z, W columns along x, square cells of cs metres), per cell:
 *   G   ground height (m)
 *   T   top of the blocking surface: G, or G + height of a hard object (house, wall) on the cell
 *   FB  bottom of foliage (absolute m), FT top of foliage; no foliage where FT <= FB
 *
 * Sight line between Q (height G[Q] + hq) and P (height G[P] + hp), straight in 3-D:
 *   blocked by an intermediate cell i when T[i] rises above the line at i ("max slope" test);
 *   foliage depth = summed ray length over intermediate cells i where FB[i] <= line height < FT[i];
 *   the line counts as seen when it is not blocked, its 2-D length <= range and foliage depth < fol_max.
 * The end cells themselves never block.
 *
 * vs_sweep_*: radial sweep (R2) from Q to every cell of the square window of half-size R: one ray to the centre
 * of every window border cell, cells sampled at every whole step along the major axis (minor axis rounded). A cell
 * is judged on the first ray that reaches it. The foliage test for P uses the exact line Q-P through the cells
 * of that ray: slopes from Q are kept in a Fenwick tree of 2048 angle bins, so a cell's foliage counts for P
 * when slope(FB) <= slope(P) < slope(FT).
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define NB 2048
#define HALF_PI 1.57079632679f

static inline int abin(float s)
{
    int b = (int)((atanf(s) / HALF_PI + 1.0f) * 0.5f * NB);
    return b < 0 ? 0 : (b >= NB ? NB - 1 : b);
}

/* Fenwick on reversed bins: suffix sums over bins > b. */
static inline void fw_add(float *f, int b, float w)
{
    for (int i = NB - 1 - b + 1; i <= NB; i += i & -i)
        f[i - 1] += w;
}

static inline float fw_above(const float *f, int b)
{
    /* sum of bins b+1 .. NB-1  == reversed indices 0 .. NB-2-b */
    float s = 0.0f;
    for (int i = NB - 1 - b; i > 0; i -= i & -i)
        s += f[i - 1];
    return s;
}

typedef struct {
    const float *G, *T, *FB, *FT;
    int H, W;
    float cs;
} grid_t;

/* mode 0: accumulate (exp[idx] += dt, tf[idx] = min(tf, t)), mode 1: query (min A, count A <= thr). */
typedef struct {
    int mode;
    float *exp, *tf;
    float dt, t;
    const float *A;
    float thr, minA;
    int64_t cnt;
} sink_t;

static void sweep(const grid_t *g, int qi, int qj, float hq, float hp, int R, float range, float fol_max,
                  int32_t *stamp, int32_t sv, float *fw, int *touch_b, float *touch_w, sink_t *k)
{
    const int W = g->W, H = g->H;
    int i0 = qi - R < 0 ? 0 : qi - R, i1 = qi + R >= W ? W - 1 : qi + R;
    int j0 = qj - R < 0 ? 0 : qj - R, j1 = qj + R >= H ? H - 1 : qj + R;
    const float eye = g->G[(size_t)qj * W + qi] + hq;
    stamp[(size_t)qj * W + qi] = sv;
    int nper = 2 * (i1 - i0 + 1) + 2 * (j1 - j0 - 1 > 0 ? j1 - j0 - 1 : 0);
    for (int e = 0; e < nper; e++) {
        int ex, ey;
        int nx = i1 - i0 + 1;
        if (e < nx) { ex = i0 + e; ey = j0; }
        else if (e < 2 * nx) { ex = i0 + e - nx; ey = j1; }
        else {
            int r = e - 2 * nx, ny = j1 - j0 - 1;
            if (r < ny) { ex = i0; ey = j0 + 1 + r; }
            else { ex = i1; ey = j0 + 1 + r - ny; }
        }
        int dx = ex - qi, dy = ey - qj;
        int n = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
        if (n == 0)
            continue;
        double sx = (double)dx / n, sy = (double)dy / n;
        float sl = (float)(sqrt(sx * sx + sy * sy) * g->cs);
        float maxs = -1e30f;
        int nt = 0;
        for (int s = 1; s <= n; s++) {
            int x = qi + (int)lround(sx * s), y = qj + (int)lround(sy * s);
            float d = s * sl;
            if (d > range)
                break;
            size_t idx = (size_t)y * W + x;
            if (stamp[idx] != sv) {
                stamp[idx] = sv;
                float sp = (g->G[idx] + hp - eye) / d;
                int vis = sp > maxs;
                if (vis && nt) {
                    float fd = fw_above(fw, abin(sp));
                    vis = fd < fol_max;
                }
                if (vis) {
                    if (k->mode == 0) {
                        k->exp[idx] += k->dt;
                        if (k->t < k->tf[idx])
                            k->tf[idx] = k->t;
                    } else {
                        float a = k->A[idx];
                        if (a < k->minA)
                            k->minA = a;
                        if (a <= k->thr)
                            k->cnt++;
                    }
                }
            }
            float st = (g->T[idx] - eye) / d;
            if (st > maxs)
                maxs = st;
            if (g->FT[idx] > g->FB[idx]) {
                int bt = abin((g->FT[idx] - eye) / d), bb = abin((g->FB[idx] - eye) / d);
                if (bt != bb) {
                    /* count where slope(FB) <= sP < slope(FT): +w for sP < slope(FT) , -w for sP < slope(FB) */
                    fw_add(fw, bt, sl);
                    fw_add(fw, bb, -sl);
                    touch_b[nt] = bt; touch_w[nt++] = sl;
                    touch_b[nt] = bb; touch_w[nt++] = -sl;
                }
            }
        }
        for (int t = 0; t < nt; t++)
            fw_add(fw, touch_b[t], -touch_w[t]);
    }
}

/* Targets grouped: group g owns targets gstart[g] .. gstart[g+1]-1 and writes exp/tf slice g (N = H*W each).
 * Per target: cell index, time t (s), weight dt. exp must be zeroed and tf filled with a large value. */
EXPORT void vs_accumulate(const float *G, const float *T, const float *FB, const float *FT, int H, int W, float cs,
                          int ngroups, const int32_t *gstart, const int32_t *cell, const float *tt, const float *tdt,
                          float hq, float hp, float range, float fol_max, float *exp, float *tf)
{
    grid_t g = {G, T, FB, FT, H, W, cs};
    int R = (int)ceilf(range / cs) + 1;
    size_t N = (size_t)H * W;
#pragma omp parallel
    {
        int32_t *stamp = malloc(sizeof(int32_t) * N);
        float *fw = calloc(NB, sizeof(float));
        int *tb = malloc(sizeof(int) * (8 * (size_t)R + 16));
        float *tw = malloc(sizeof(float) * (8 * (size_t)R + 16));
        for (size_t i = 0; i < N; i++)
            stamp[i] = -1;
        int gi;
#pragma omp for schedule(dynamic, 1)
        for (gi = 0; gi < ngroups; gi++) {
            sink_t k = {0};
            k.exp = exp + (size_t)gi * N;
            k.tf = tf + (size_t)gi * N;
            for (int t = gstart[gi]; t < gstart[gi + 1]; t++) {
                k.dt = tdt[t];
                k.t = tt[t];
                sweep(&g, cell[t] % W, cell[t] / W, hq, hp, R, range, fol_max, stamp, t, fw, tb, tw, &k);
            }
        }
        free(stamp);
        free(fw);
        free(tb);
        free(tw);
    }
}

/* For each target: minimum of A over the cells that see it, and the number of seeing cells with A <= thr[t]. */
EXPORT void vs_query(const float *G, const float *T, const float *FB, const float *FT, int H, int W, float cs,
                     int ntg, const int32_t *cell, const float *thr, float hq, float hp, float range, float fol_max,
                     const float *A, float *minA, int64_t *cnt)
{
    grid_t g = {G, T, FB, FT, H, W, cs};
    int R = (int)ceilf(range / cs) + 1;
    size_t N = (size_t)H * W;
#pragma omp parallel
    {
        int32_t *stamp = malloc(sizeof(int32_t) * N);
        float *fw = calloc(NB, sizeof(float));
        int *tb = malloc(sizeof(int) * (8 * (size_t)R + 16));
        float *tw = malloc(sizeof(float) * (8 * (size_t)R + 16));
        for (size_t i = 0; i < N; i++)
            stamp[i] = -1;
        int t;
#pragma omp for schedule(dynamic, 1)
        for (t = 0; t < ntg; t++) {
            sink_t k = {0};
            k.mode = 1;
            k.A = A;
            k.thr = thr[t];
            k.minA = 1e30f;
            sweep(&g, cell[t] % W, cell[t] / W, hq, hp, R, range, fol_max, stamp, t, fw, tb, tw, &k);
            minA[t] = k.minA;
            cnt[t] = k.cnt;
        }
        free(stamp);
        free(fw);
        free(tb);
        free(tw);
    }
}

/* Exact single lines Q -> P (same stepping as the sweep, ray aimed at P itself).
 * out[3*i]: 1 if not blocked by T, out[3*i+1]: 1 if not blocked by the ground G alone,
 * out[3*i+2]: foliage depth (m) along the line. */
EXPORT void los_pairs(const float *G, const float *T, const float *FB, const float *FT, int H, int W, float cs,
                      int n, const int32_t *qcell, const float *hq, const int32_t *pcell, const float *hp, float *out)
{
    int i;
#pragma omp parallel for schedule(static)
    for (i = 0; i < n; i++) {
        int qi = qcell[i] % W, qj = qcell[i] / W, pi = pcell[i] % W, pj = pcell[i] / W;
        int dx = pi - qi, dy = pj - qj;
        int m = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
        float eye = G[(size_t)qj * W + qi] + hq[i];
        float tgt = G[(size_t)pj * W + pi] + hp[i];
        int okT = 1, okG = 1;
        float fd = 0.0f;
        if (m > 0) {
            double sx = (double)dx / m, sy = (double)dy / m;
            float sl = (float)(sqrt(sx * sx + sy * sy) * cs);
            for (int s = 1; s < m; s++) {
                int x = qi + (int)lround(sx * s), y = qj + (int)lround(sy * s);
                size_t idx = (size_t)y * W + x;
                float z = eye + (tgt - eye) * (float)s / m;
                if (T[idx] > z)
                    okT = 0;
                if (G[idx] > z)
                    okG = 0;
                if (FT[idx] > FB[idx] && FB[idx] <= z && z < FT[idx])
                    fd += sl;
            }
        }
        out[3 * i] = (float)okT;
        out[3 * i + 1] = (float)okG;
        out[3 * i + 2] = fd;
    }
}
