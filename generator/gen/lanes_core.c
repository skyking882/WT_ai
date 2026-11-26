/* lanes_core.c: the cover-graph search for lanes. No game code.
 *
 * cover_search(): bounded Dijkstra over the 0.5 m grid from a set of source cells.
 *   cost[i]   driving time per cell (route cost / speed; <= 0 closed), the same grid as the routes;
 *   open[i]   1 where the cell is in the open for this army (seen from more than the allowed share of the
 *             enemy's likely positions), 0 where it is hidden;
 *   catchid[i] node id of a cover node's catch disc, -1 elsewhere.
 * The search key is time x (1 + open_pen x open), so paths prefer cover; along every tree path it keeps the true
 * time, the metres in the open, the current open run (metres since the last hidden cell) and the longest open run.
 * A path whose open run would exceed run_lim is not extended (pruned). A settled cell of another node's catch disc
 * is reported once per node (its first settled cell) and not expanded: edges join neighbouring cover nodes only.
 * Work arrays are reused between calls with a stamp (stamp[i] == 2*sv: touched, 2*sv+1: settled).
 *
 * chain(): parent chain from a cell back to its source (cells written in that order), returns the length.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

typedef struct {
    float k;
    int32_t i;
} lnode_t;

static lnode_t *lheap = NULL;
static size_t ln = 0, lcap = 0;

static void lpush(float k, int32_t i)
{
    if (ln == lcap) {
        lcap = lcap ? lcap * 2 : 1 << 20;
        lheap = realloc(lheap, lcap * sizeof(lnode_t));
    }
    size_t c = ln++;
    while (c) {
        size_t p = (c - 1) / 2;
        if (lheap[p].k <= k)
            break;
        lheap[c] = lheap[p];
        c = p;
    }
    lheap[c].k = k;
    lheap[c].i = i;
}

static lnode_t lpop(void)
{
    lnode_t top = lheap[0], last = lheap[--ln];
    size_t c = 0;
    for (;;) {
        size_t l = 2 * c + 1;
        if (l >= ln)
            break;
        if (l + 1 < ln && lheap[l + 1].k < lheap[l].k)
            l++;
        if (lheap[l].k >= last.k)
            break;
        lheap[c] = lheap[l];
        c = l;
    }
    if (ln)
        lheap[c] = last;
    return top;
}

/* No hard cap: key = time + open_pen x seconds in the open + dash_b x sum over dashes of
 * max(0, dash length - dash_l0)^2, accumulated as the dash grows (a dash that ends adds nothing more).
 * run_lim stays as a plain search bound (set it beyond any dash that can occur). hit_key = key at the hit. */
static inline float sq0(float r, float l0)
{
    float e = r - l0;
    return e > 0.0f ? e * e : 0.0f;
}

EXPORT int cover_search(const float *cost, const uint8_t *open, const int32_t *catchid, int H, int W,
                        const int32_t *src, int nsrc, int32_t self_id, float maxt, float run_lim, float open_pen,
                        float dash_l0, float dash_b,
                        int32_t sv, int32_t *stamp, float *key, float *tt, int32_t *par, float *run, float *mrun,
                        float *om, int max_hits, int32_t *hit_id, int32_t *hit_cell, float *hit_t, float *hit_open,
                        float *hit_mrun, float *hit_key)
{
    static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int32_t TOUCH = 2 * sv, DONE = 2 * sv + 1;
    int nh = 0;
    ln = 0;
    for (int s = 0; s < nsrc; s++) {
        int32_t i = src[s];
        if (i < 0 || i >= H * W || !(cost[i] > 0.0f))
            continue;
        stamp[i] = TOUCH;
        key[i] = 0.0f;
        tt[i] = 0.0f;
        par[i] = -1;
        run[i] = 0.0f;
        mrun[i] = 0.0f;
        om[i] = 0.0f;
        lpush(0.0f, i);
    }
    while (ln) {
        lnode_t t = lpop();
        int32_t a = t.i;
        if (stamp[a] == DONE || t.k > key[a])
            continue;
        stamp[a] = DONE;
        if (tt[a] > maxt)
            continue;
        int32_t id = catchid[a];
        if (id >= 0 && id != self_id) {
            int seen = 0;
            for (int h = 0; h < nh; h++)
                if (hit_id[h] == id) {
                    seen = 1;
                    break;
                }
            if (!seen && nh < max_hits) {
                hit_id[nh] = id;
                hit_cell[nh] = a;
                hit_t[nh] = tt[a];
                hit_open[nh] = om[a];
                hit_mrun[nh] = mrun[a];
                hit_key[nh] = key[a];
                nh++;
            }
            continue;
        }
        int y = a / W, x = a % W;
        float ca = cost[a];
        for (int k = 0; k < 8; k++) {
            int nx = x + dx[k], ny = y + dy[k];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                continue;
            int32_t j = ny * W + nx;
            float cb = cost[j];
            if (!(cb > 0.0f))
                continue;
            if (k >= 4 && (!(cost[y * W + nx] > 0.0f) || !(cost[ny * W + x] > 0.0f)))
                continue;
            if (stamp[j] == DONE)
                continue;
            float len = k >= 4 ? 0.70710678f : 0.5f;
            float dt = (k >= 4 ? 1.41421356f : 1.0f) * 0.5f * (ca + cb);
            float nrun = open[j] ? run[a] + len : 0.0f;
            if (nrun > run_lim)
                continue;
            float nk = key[a] + dt * (1.0f + open_pen * (float)open[j]);
            if (open[j])
                nk += dash_b * (sq0(nrun, dash_l0) - sq0(run[a], dash_l0));
            if (stamp[j] != TOUCH || nk < key[j]) {
                stamp[j] = TOUCH;
                key[j] = nk;
                tt[j] = tt[a] + dt;
                par[j] = a;
                run[j] = nrun;
                mrun[j] = nrun > mrun[a] ? nrun : mrun[a];
                om[j] = om[a] + (open[j] ? len : 0.0f);
                lpush(nk, j);
            }
        }
    }
    return nh;
}

EXPORT int chain(const int32_t *par, int32_t start, int32_t *out, int maxlen)
{
    int n = 0;
    int32_t i = start;
    while (i >= 0 && n < maxlen) {
        out[n++] = i;
        i = par[i];
    }
    return n;
}
