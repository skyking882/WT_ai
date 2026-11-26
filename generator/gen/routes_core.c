/* routes_core.c: grid helpers for the offline route generator.
 * Built as a DLL by build_core.py, used through ctypes. No game code.
 *
 *   edt()       exact Euclidean distance transform (Felzenszwalb & Huttenlocher),
 *               distance in cells from each cell centre to the nearest marked cell centre.
 *   dijkstra()  8-connected shortest paths over a cost grid; edge cost = metric length
 *               (1 or sqrt 2 cells) * mean of the two cell costs; cost <= 0 or not finite = closed.
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
#define BIG 1e20f

/* 1-D squared distance transform of sampled function f (Felzenszwalb & Huttenlocher 2012). */
static void edt1(const double *f, int n, double *d, int *v, double *z)
{
    int k = 0;
    v[0] = 0;
    z[0] = -1e300;
    z[1] = 1e300;
    for (int q = 1; q < n; q++) {
        double s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) / (2.0 * (q - v[k]));
        while (s <= z[k]) {
            k--;
            s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) / (2.0 * (q - v[k]));
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = 1e300;
    }
    k = 0;
    for (int q = 0; q < n; q++) {
        while (z[k + 1] < q)
            k++;
        double dq = (double)(q - v[k]);
        d[q] = dq * dq + f[v[k]];
    }
}

/* mark: 1 = feature cell. out: distance in cells from the cell centre to the nearest feature
 * cell centre (about 1e6 where the grid has no feature). */
EXPORT void edt(const uint8_t *mark, int H, int W, float *out)
{
    int n = H > W ? H : W;
    double *f = malloc(sizeof(double) * n), *d = malloc(sizeof(double) * n), *z = malloc(sizeof(double) * (n + 1));
    double *tmp = malloc(sizeof(double) * (size_t)H * W);
    int *v = malloc(sizeof(int) * n);
    for (int x = 0; x < W; x++) {
        for (int y = 0; y < H; y++)
            f[y] = mark[(size_t)y * W + x] ? 0.0 : 1e12;
        edt1(f, H, d, v, z);
        for (int y = 0; y < H; y++)
            tmp[(size_t)y * W + x] = d[y];
    }
    for (int y = 0; y < H; y++) {
        memcpy(f, tmp + (size_t)y * W, sizeof(double) * W);
        edt1(f, W, d, v, z);
        for (int x = 0; x < W; x++)
            out[(size_t)y * W + x] = (float)sqrt(d[x] < 1e12 ? d[x] : 1e12);
    }
    free(tmp);
    free(f);
    free(d);
    free(z);
    free(v);
}

typedef struct {
    float k;
    int32_t i;
} node_t;

static node_t *heap;
static size_t hn, hcap;

static void hpush(float k, int32_t i)
{
    if (hn == hcap) {
        hcap = hcap ? hcap * 2 : 1 << 20;
        heap = realloc(heap, hcap * sizeof(node_t));
    }
    size_t c = hn++;
    while (c) {
        size_t p = (c - 1) / 2;
        if (heap[p].k <= k)
            break;
        heap[c] = heap[p];
        c = p;
    }
    heap[c].k = k;
    heap[c].i = i;
}

static node_t hpop(void)
{
    node_t top = heap[0], last = heap[--hn];
    size_t c = 0;
    for (;;) {
        size_t l = 2 * c + 1;
        if (l >= hn)
            break;
        if (l + 1 < hn && heap[l + 1].k < heap[l].k)
            l++;
        if (heap[l].k >= last.k)
            break;
        heap[c] = heap[l];
        c = l;
    }
    if (hn)
        heap[c] = last;
    return top;
}

/* Geodesic discs: for each centre cell, every open cell (cost > 0) reachable from it within rcells of
 * 8-connected driving (edge length 1 or sqrt 2, no corner cutting) gets mark = 1. Local window search. */
EXPORT void geodisc(const float *cost, int H, int W, const int32_t *centres, int n, float rcells, uint8_t *mark)
{
    int R = (int)ceilf(rcells) + 1, w = 2 * R + 1;
    float *d = malloc(sizeof(float) * w * w);
    uint8_t *cl = malloc((size_t)w * w);
    static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int c = 0; c < n; c++) {
        int cy = centres[c] / W, cx = centres[c] % W;
        if (centres[c] < 0 || !(cost[centres[c]] > 0.0f))
            continue;
        for (int i = 0; i < w * w; i++) {
            d[i] = BIG;
            cl[i] = 0;
        }
        hn = 0;
        int li = R * w + R;
        d[li] = 0.0f;
        hpush(0.0f, li);
        while (hn) {
            node_t t = hpop();
            if (cl[t.i])
                continue;
            cl[t.i] = 1;
            int ly = t.i / w, lx = t.i % w, gy = cy + ly - R, gx = cx + lx - R;
            mark[(size_t)gy * W + gx] = 1;
            for (int k = 0; k < 8; k++) {
                int nx = lx + dx[k], ny = ly + dy[k], gnx = gx + dx[k], gny = gy + dy[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= w || gnx < 0 || gny < 0 || gnx >= W || gny >= H)
                    continue;
                if (!(cost[(size_t)gny * W + gnx] > 0.0f))
                    continue;
                if (k >= 4 && (!(cost[(size_t)gy * W + gnx] > 0.0f) || !(cost[(size_t)gny * W + gx] > 0.0f)))
                    continue;
                float nd = t.k + (k >= 4 ? 1.41421356f : 1.0f);
                int j = ny * w + nx;
                if (nd <= rcells && nd < d[j]) {
                    d[j] = nd;
                    hpush(nd, j);
                }
            }
        }
    }
    free(d);
    free(cl);
}

/* cost: per-cell factor (closed if !(cost > 0) or not finite). sources: nsrc cell indices.
 * goal: stop when this cell is settled (-1: none). maxd: stop beyond this distance (<=0: none).
 * dist (out): BIG where unreached. parent (out): -1 for sources/unreached.
 * Returns the number of settled cells. */
/* hscale > 0 with goal >= 0: A* with heuristic hscale * straight-line distance (cells) to the goal
 * (admissible when every cell cost >= hscale). */
EXPORT int64_t dijkstra(const float *cost, int H, int W, const int32_t *src, int nsrc, int32_t goal, float maxd,
                        float hscale, float *dist, int32_t *parent)
{
    size_t N = (size_t)H * W;
    uint8_t *closed = calloc(N, 1);
    for (size_t i = 0; i < N; i++) {
        dist[i] = BIG;
        parent[i] = -1;
    }
    const int gy = goal >= 0 ? goal / W : 0, gx = goal >= 0 ? goal % W : 0;
    const int astar = goal >= 0 && hscale > 0;
#define HEUR(idx) (astar ? hscale * sqrtf((float)(((idx) / W - gy) * ((idx) / W - gy) + ((idx) % W - gx) * ((idx) % W - gx))) : 0.0f)
    hn = 0;
    for (int s = 0; s < nsrc; s++) {
        int32_t i = src[s];
        if (i < 0 || (size_t)i >= N)
            continue;
        dist[i] = 0.0f;
        hpush(HEUR(i), i);
    }
    static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float SQ2 = 1.41421356f;
    int64_t settled = 0;
    while (hn) {
        node_t t = hpop();
        if (closed[t.i])
            continue;
        closed[t.i] = 1;
        settled++;
        if (t.i == goal)
            break;
        if (maxd > 0 && dist[t.i] > maxd)
            break;
        int y = t.i / W, x = t.i % W;
        float ca = cost[t.i];
        for (int k = 0; k < 8; k++) {
            int nx = x + dx[k], ny = y + dy[k];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                continue;
            int32_t j = ny * W + nx;
            float cb = cost[j];
            if (!(cb > 0.0f) || !isfinite(cb))
                continue;
            if (k >= 4) { /* diagonal: both side cells must be open (no corner cutting) */
                float s1 = cost[y * W + nx], s2 = cost[ny * W + x];
                if (!(s1 > 0.0f) || !isfinite(s1) || !(s2 > 0.0f) || !isfinite(s2))
                    continue;
            }
            if (closed[j])
                continue;
            float nd = dist[t.i] + (k >= 4 ? SQ2 : 1.0f) * 0.5f * (ca + cb);
            if (nd < dist[j]) {
                dist[j] = nd;
                parent[j] = t.i;
                hpush(nd + HEUR(j), j);
            }
        }
    }
    free(closed);
    return settled;
}
