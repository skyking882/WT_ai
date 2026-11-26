/* s38 road network loader, spatial index and search (ground_network.h). Plain C and stdio. */
#define _CRT_SECURE_NO_WARNINGS
#include "ground_network.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define GN_HEADER 52u           /* magic 8 + 7 x u32 + 2 x f32 + 2 x u32 (the brief's "48 B" does not add up) */
#define GN_NODE_BYTES 16u
#define GN_EDGE_BYTES 28u
#define GN_CELL 4.0f            /* m: spatial index cell */
#define GN_MAX_CELLS 16000000u  /* the index grows its cell rather than exceed this */
#define GN_BOX_MARGIN 0.5f      /* m: a sample this close to a box's rectangle is inside it (as gl_in_closed) */
#define GN_DUP 0.05f            /* m: a chain sample this close to the previous one is not written again */

static uint32_t gn_u32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

static uint16_t gn_u16(const uint8_t *p)
{
    uint16_t v;
    memcpy(&v, p, 2);
    return v;
}

static float gn_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

static uint8_t *gn_slurp(const char *path, size_t *size)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return NULL;
    uint8_t *data = NULL;
    *size = 0;
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

void gn_free(gn_net_t *net)
{
    free(net->nodes);
    free(net->edges);
    free(net->adj);
    free(net->target_first);
    free(net->target_nodes);
    free(net->sx);
    free(net->sz);
    free(net->clr);
    free(net->s_edge);
    free(net->s_arc);
    free(net->e_arc);
    free(net->h_in);
    free(net->h_out);
    free(net->cell_first);
    free(net->cell_items);
    free(net->st_dist);
    free(net->st_prev);
    free(net->st_stamp);
    free(net->st_done);
    free(net->st_heap);
    free(net->st_pos);
    free(net->n_goal);
    free(net->turn_deg);
    free(net->turn_at);
    free(net->turn_node);
    free(net->e_block);
    free(net->e_mark);
    free(net->e_exit_a);
    free(net->e_exit_b);
    free(net->e_exit_sa);
    free(net->e_exit_sb);
    free(net->e_touched);
    free(net->s_ent);
    free(net->s_ext);
    free(net->s_ent_stamp);
    free(net->s_ext_stamp);
    free(net->near_buf);
    free(net->ent_buf);
    free(net->ext_buf);
    free(net->pieces);
    memset(net, 0, sizeof *net);
}

#define GN_FAILS(...)                                          \
    do {                                                       \
        if (msg && msg_size)                                   \
            snprintf(msg, msg_size, __VA_ARGS__);              \
        free(fb);                                              \
        gn_free(net);                                          \
        return 0;                                              \
    } while (0)

int gn_load(gn_net_t *net, const char *path, const gr_lib_t *lib, char *msg, size_t msg_size)
{
    const clock_t c0 = clock();
    memset(net, 0, sizeof *net);
    size_t fn = 0;
    uint8_t *fb = gn_slurp(path, &fn);
    if (!fb)
        GN_FAILS("cannot read %s", path);
    if (fn < GN_HEADER)
        GN_FAILS("%s: %zu bytes, shorter than the 52 byte header", path, fn);
    if (memcmp(fb, "WTNET001", 8))
        GN_FAILS("%s: magic is not WTNET001", path);
    const uint8_t *h = fb + 8;
    net->version = gn_u32(h);
    net->n_nodes = gn_u32(h + 4);
    net->n_edges = gn_u32(h + 8);
    net->n_samples = gn_u32(h + 12);
    net->n_adj = gn_u32(h + 16);
    net->n_targets = gn_u32(h + 20);
    net->n_target_nodes = gn_u32(h + 24);
    net->step = gn_f32(h + 28);
    net->tol_m = gn_f32(h + 32);
    net->lib_routes = gn_u32(h + 36);
    net->lib_samples = gn_u32(h + 40);
    if (net->version != 1)
        GN_FAILS("%s: version %u, only 1 is understood", path, net->version);
    if (!net->n_nodes || !net->n_edges || !net->n_samples || !net->n_targets || net->n_nodes > 50000000u
        || net->n_edges > 50000000u || net->n_samples > 200000000u || net->n_adj > 100000000u || net->n_targets > 4096u
        || net->n_target_nodes > 50000000u)
        GN_FAILS("%s: implausible counts %u nodes, %u edges, %u samples, %u adj, %u targets, %u target nodes", path,
                 net->n_nodes, net->n_edges, net->n_samples, net->n_adj, net->n_targets, net->n_target_nodes);
    if (!(fabsf(net->step - GR_STEP) <= 1e-4f) || !isfinite(net->tol_m) || !(net->tol_m > 0.0f) || net->tol_m > 10.0f)
        GN_FAILS("%s: step %g m (0.5 expected) or tolerance %g m not plausible", path, (double)net->step,
                 (double)net->tol_m);
    if (net->n_adj != 2u * net->n_edges)
        GN_FAILS("%s: %u adjacency entries for %u edges (two per edge expected)", path, net->n_adj, net->n_edges);
    if (lib) {
        if (net->n_targets != lib->n_targets)
            GN_FAILS("%s: %u targets, the library has %u", path, net->n_targets, lib->n_targets);
        if (net->lib_routes != lib->n_routes || net->lib_samples != lib->n_samples)
            GN_FAILS("%s: built from a library of %u routes / %u samples, the loaded one has %u / %u", path,
                     net->lib_routes, net->lib_samples, lib->n_routes, lib->n_samples);
    }
    const uint64_t want = (uint64_t)GN_HEADER + (uint64_t)net->n_nodes * GN_NODE_BYTES
                          + (uint64_t)net->n_edges * GN_EDGE_BYTES + (uint64_t)net->n_adj * 4u
                          + ((uint64_t)net->n_targets + 1u) * 4u + (uint64_t)net->n_target_nodes * 4u
                          + (uint64_t)net->n_samples * 9u;
    if (want != (uint64_t)fn)
        GN_FAILS("%s: layout mismatch: %zu bytes, the header promises %llu", path, fn, (unsigned long long)want);

    const uint32_t nn = net->n_nodes, ne = net->n_edges, ns = net->n_samples, nt = net->n_targets;
    net->nodes = (gn_node_t *)calloc(nn, sizeof *net->nodes);
    net->edges = (gn_edge_t *)calloc(ne, sizeof *net->edges);
    net->adj = (uint32_t *)malloc(sizeof(uint32_t) * net->n_adj);
    net->target_first = (uint32_t *)malloc(sizeof(uint32_t) * (nt + 1u));
    net->target_nodes = (uint32_t *)malloc(sizeof(uint32_t) * (net->n_target_nodes ? net->n_target_nodes : 1u));
    net->sx = (float *)malloc(sizeof(float) * ns);
    net->sz = (float *)malloc(sizeof(float) * ns);
    net->clr = (uint8_t *)malloc(ns);
    net->s_edge = (uint32_t *)malloc(sizeof(uint32_t) * ns);
    net->s_arc = (float *)malloc(sizeof(float) * ns);
    net->e_arc = (float *)malloc(sizeof(float) * ne);
    if (!net->nodes || !net->edges || !net->adj || !net->target_first || !net->target_nodes || !net->sx || !net->sz
        || !net->clr || !net->s_edge || !net->s_arc || !net->e_arc)
        GN_FAILS("out of memory for the network");

    const uint8_t *p = fb + GN_HEADER;
    for (uint32_t i = 0; i < nn; ++i, p += GN_NODE_BYTES) {
        gn_node_t *n = &net->nodes[i];
        n->x = gn_f32(p);
        n->z = gn_f32(p + 4);
        n->adj_first = gn_u32(p + 8);
        n->adj_n = gn_u16(p + 12);
        n->flags = gn_u16(p + 14);
        if (!isfinite(n->x) || !isfinite(n->z) || (uint64_t)n->adj_first + n->adj_n > net->n_adj || (n->flags & ~7u))
            GN_FAILS("%s: node %u does not fit the layout (adj %u + %u of %u, flags 0x%x)", path, i, n->adj_first,
                     (unsigned)n->adj_n, net->n_adj, (unsigned)n->flags);
        net->n_spawn_nodes += (n->flags & GN_NODE_SPAWN) != 0;
        net->n_pivot_nodes += (n->flags & GN_NODE_PIVOT) != 0;
    }
    for (uint32_t i = 0; i < ne; ++i, p += GN_EDGE_BYTES) {
        gn_edge_t *e = &net->edges[i];
        e->node_a = gn_u32(p);
        e->node_b = gn_u32(p + 4);
        e->first = gn_u32(p + 8);
        e->n = gn_u32(p + 12);
        e->length_m = gn_f32(p + 16);
        e->time_s = gn_f32(p + 20);
        e->uses = gn_u16(p + 24);
        e->min_clr = p[26];
        e->flags = p[27];
        if (e->node_a >= nn || e->node_b >= nn || e->n < 2 || (uint64_t)e->first + e->n > ns || !isfinite(e->length_m)
            || !(e->length_m > 0.0f) || !isfinite(e->time_s) || !(e->time_s > 0.0f))
            GN_FAILS("%s: edge %u does not fit the layout (nodes %u %u, samples %u + %u of %u, %g m, %g s)", path, i,
                     e->node_a, e->node_b, e->first, e->n, ns, (double)e->length_m, (double)e->time_s);
    }
    for (uint32_t i = 0; i < net->n_adj; ++i, p += 4)
        net->adj[i] = gn_u32(p);
    for (uint32_t i = 0; i <= nt; ++i, p += 4)
        net->target_first[i] = gn_u32(p);
    for (uint32_t i = 0; i < net->n_target_nodes; ++i, p += 4)
        net->target_nodes[i] = gn_u32(p);
    for (uint32_t i = 0; i < ns; ++i, p += 8) {
        net->sx[i] = gn_f32(p);
        net->sz[i] = gn_f32(p + 4);
        if (!isfinite(net->sx[i]) || !isfinite(net->sz[i]))
            GN_FAILS("%s: sample %u not finite", path, i);
    }
    memcpy(net->clr, p, ns);
    p += ns;
    if ((size_t)(p - fb) != fn)
        GN_FAILS("%s: %zu of %zu bytes read", path, (size_t)(p - fb), fn);

    /* every sample belongs to exactly one edge; the edge's ends lie on its nodes; its length is its samples' arc */
    for (uint32_t i = 0; i < ns; ++i)
        net->s_edge[i] = UINT32_MAX;
    for (uint32_t i = 0; i < ne; ++i) {
        const gn_edge_t *e = &net->edges[i];
        float arc = 0.0f;
        for (uint32_t k = 0; k < e->n; ++k) {
            const uint32_t j = e->first + k;
            if (net->s_edge[j] != UINT32_MAX)
                GN_FAILS("%s: sample %u belongs to edge %u and edge %u", path, j, net->s_edge[j], i);
            net->s_edge[j] = i;
            if (k)
                arc += hypotf(net->sx[j] - net->sx[j - 1], net->sz[j] - net->sz[j - 1]);
            net->s_arc[j] = arc;
        }
        net->e_arc[i] = arc;
        net->total_km += arc / 1000.0;
        const gn_node_t *a = &net->nodes[e->node_a], *b = &net->nodes[e->node_b];
        const uint32_t j0 = e->first, j1 = e->first + e->n - 1u;
        const float da = hypotf(net->sx[j0] - a->x, net->sz[j0] - a->z), db = hypotf(net->sx[j1] - b->x, net->sz[j1] - b->z);
        if (da > net->tol_m + 0.05f || db > net->tol_m + 0.05f)
            GN_FAILS("%s: edge %u does not run from node %u to node %u (ends %.2f m and %.2f m off, tolerance %.2f m)", path,
                     i, e->node_a, e->node_b, (double)da, (double)db, (double)net->tol_m);
        if (!(arc > 0.0f) || fabsf(arc - e->length_m) > 0.02f * arc + 0.5f)
            GN_FAILS("%s: edge %u length %.2f m, its samples run %.2f m", path, i, (double)e->length_m, (double)arc);
    }
    for (uint32_t i = 0; i < ns; ++i)
        if (net->s_edge[i] == UINT32_MAX)
            GN_FAILS("%s: sample %u belongs to no edge", path, i);
    /* adjacency: each entry names an edge that has this node at that end; each edge end listed exactly once */
    {
        uint8_t *seen = (uint8_t *)calloc(ne, 1);
        if (!seen)
            GN_FAILS("out of memory");
        for (uint32_t i = 0; i < nn; ++i) {
            const gn_node_t *n = &net->nodes[i];
            for (uint32_t q = n->adj_first; q < n->adj_first + n->adj_n; ++q) {
                const uint32_t ei = net->adj[q] >> 1, end = net->adj[q] & 1u;
                if (ei >= ne || (end ? net->edges[ei].node_b : net->edges[ei].node_a) != i || (seen[ei] & (1u << end))) {
                    free(seen);
                    GN_FAILS("%s: adjacency entry %u of node %u (edge %u end %u) does not fit", path, q, i, ei, end);
                }
                seen[ei] |= (uint8_t)(1u << end);
            }
        }
        for (uint32_t i = 0; i < ne; ++i)
            if (seen[i] != 3u) {
                free(seen);
                GN_FAILS("%s: edge %u is not listed at both its nodes", path, i);
            }
        free(seen);
    }
    /* targets */
    if (net->target_first[0] != 0 || net->target_first[nt] != net->n_target_nodes)
        GN_FAILS("%s: target offsets run %u .. %u, 0 .. %u expected", path, net->target_first[0], net->target_first[nt],
                 net->n_target_nodes);
    for (uint32_t t = 0; t < nt; ++t) {
        if (net->target_first[t + 1] < net->target_first[t])
            GN_FAILS("%s: target offsets decrease at target %u", path, t);
        net->n_targets_without += net->target_first[t + 1] == net->target_first[t];
    }
    for (uint32_t i = 0; i < net->n_target_nodes; ++i) {
        const uint32_t v = net->target_nodes[i];
        if (v >= nn || !(net->nodes[v].flags & GN_NODE_TARGET))
            GN_FAILS("%s: target node entry %u (node %u) is not a node flagged as a target end", path, i, v);
    }
    for (uint32_t i = 0; i < nn; ++i)
        net->n_target_ends += (net->nodes[i].flags & GN_NODE_TARGET) != 0;

    /* spatial index */
    {
        float x0 = INFINITY, z0 = INFINITY, x1 = -INFINITY, z1 = -INFINITY;
        for (uint32_t i = 0; i < ns; ++i) {
            x0 = fminf(x0, net->sx[i]);
            z0 = fminf(z0, net->sz[i]);
            x1 = fmaxf(x1, net->sx[i]);
            z1 = fmaxf(z1, net->sz[i]);
        }
        float cell = GN_CELL;
        uint32_t cols, rows;
        for (;;) {
            cols = (uint32_t)floorf((x1 - x0) / cell) + 1u;
            rows = (uint32_t)floorf((z1 - z0) / cell) + 1u;
            if ((uint64_t)cols * rows <= GN_MAX_CELLS)
                break;
            cell *= 2.0f;
        }
        net->gx0 = x0;
        net->gz0 = z0;
        net->cell = cell;
        net->gcols = cols;
        net->grows = rows;
        const size_t nc = (size_t)cols * rows;
        net->cell_first = (uint32_t *)calloc(nc + 1u, sizeof(uint32_t));
        net->cell_items = (uint32_t *)malloc(sizeof(uint32_t) * ns);
        uint32_t *fill = (uint32_t *)calloc(nc, sizeof(uint32_t));
        if (!net->cell_first || !net->cell_items || !fill) {
            free(fill);
            GN_FAILS("out of memory for the sample index");
        }
        for (uint32_t i = 0; i < ns; ++i) {
            const uint32_t c = (uint32_t)((net->sx[i] - x0) / cell), r = (uint32_t)((net->sz[i] - z0) / cell);
            ++net->cell_first[(size_t)(r < rows ? r : rows - 1u) * cols + (c < cols ? c : cols - 1u) + 1u];
        }
        for (size_t c = 0; c < nc; ++c)
            net->cell_first[c + 1] += net->cell_first[c];
        for (uint32_t i = 0; i < ns; ++i) {
            const uint32_t c = (uint32_t)((net->sx[i] - x0) / cell), r = (uint32_t)((net->sz[i] - z0) / cell);
            const size_t k = (size_t)(r < rows ? r : rows - 1u) * cols + (c < cols ? c : cols - 1u);
            net->cell_items[net->cell_first[k] + fill[k]++] = i;
        }
        free(fill);
    }
    /* turn headings per state: f 1 (a -> b): leaving node_a over the first GN_HEAD_SAMPLES steps, arriving at node_b
     * over the last ones; f 0 the reverse (the generator's rule) */
    const size_t nst = 2u * (size_t)ne;
    net->h_in = (float *)malloc(sizeof(float) * nst);
    net->h_out = (float *)malloc(sizeof(float) * nst);
    if (!net->h_in || !net->h_out)
        GN_FAILS("out of memory for the turn headings");
    for (uint32_t i = 0; i < ne; ++i) {
        const gn_edge_t *e = &net->edges[i];
        const uint32_t k = e->n - 1u < (uint32_t)GN_HEAD_SAMPLES ? e->n - 1u : (uint32_t)GN_HEAD_SAMPLES;
        const uint32_t j0 = e->first, j1 = e->first + e->n - 1u;
        const float h0 = atan2f(net->sz[j0 + k] - net->sz[j0], net->sx[j0 + k] - net->sx[j0]);
        const float h1 = atan2f(net->sz[j1] - net->sz[j1 - k], net->sx[j1] - net->sx[j1 - k]);
        net->h_out[2u * i + 1u] = h0;
        net->h_in[2u * i + 1u] = h1;
        net->h_out[2u * i] = h1 + 3.14159265f;
        net->h_in[2u * i] = h0 + 3.14159265f;
    }
    /* search buffers */
    net->st_dist = (float *)malloc(sizeof(float) * nst);
    net->st_prev = (int32_t *)malloc(sizeof(int32_t) * nst);
    net->st_stamp = (uint32_t *)calloc(nst, sizeof(uint32_t));
    net->st_done = (uint32_t *)calloc(nst, sizeof(uint32_t));
    net->st_heap = (int32_t *)malloc(sizeof(int32_t) * nst);
    net->st_pos = (int32_t *)malloc(sizeof(int32_t) * nst);
    net->n_goal = (uint32_t *)calloc(nn, sizeof(uint32_t));
    net->turn_deg = (float *)malloc(sizeof(float) * (nst + 2u));
    net->turn_at = (int32_t *)malloc(sizeof(int32_t) * (nst + 2u));
    net->turn_node = (int32_t *)malloc(sizeof(int32_t) * (nst + 2u));
    net->e_block = (uint32_t *)calloc(ne, sizeof(uint32_t));
    net->e_mark = (uint32_t *)calloc(ne, sizeof(uint32_t));
    net->e_exit_a = (float *)malloc(sizeof(float) * ne);
    net->e_exit_b = (float *)malloc(sizeof(float) * ne);
    net->e_exit_sa = (uint32_t *)malloc(sizeof(uint32_t) * ne);
    net->e_exit_sb = (uint32_t *)malloc(sizeof(uint32_t) * ne);
    net->e_touched = (uint32_t *)malloc(sizeof(uint32_t) * ne);
    net->s_ent = (float *)malloc(sizeof(float) * ns);
    net->s_ext = (float *)malloc(sizeof(float) * ns);
    net->s_ent_stamp = (uint32_t *)calloc(ns, sizeof(uint32_t));
    net->s_ext_stamp = (uint32_t *)calloc(ns, sizeof(uint32_t));
    net->near_buf = (uint32_t *)malloc(sizeof(uint32_t) * ns);
    net->ent_buf = (gn_end_t *)malloc(sizeof(gn_end_t) * ns);
    net->ext_buf = (gn_end_t *)malloc(sizeof(gn_end_t) * ns);
    net->pieces = (int32_t *)malloc(sizeof(int32_t) * 4u * (nst + 2u));
    if (!net->st_dist || !net->st_prev || !net->st_stamp || !net->st_done || !net->n_goal || !net->st_heap
        || !net->st_pos || !net->turn_deg || !net->turn_at || !net->turn_node || !net->e_block || !net->e_mark || !net->e_exit_a || !net->e_exit_b || !net->e_exit_sa || !net->e_exit_sb
        || !net->e_touched || !net->s_ent || !net->s_ext || !net->s_ent_stamp || !net->s_ext_stamp || !net->near_buf
        || !net->ent_buf || !net->ext_buf || !net->pieces)
        GN_FAILS("out of memory for the search buffers");
    net->gen = 0;
    /* every block allocated above */
    net->bytes = (size_t)nn * (sizeof(gn_node_t) + 4u) + (size_t)ne * (sizeof(gn_edge_t) + 4u + 8u * 4u)
                 + nst * (2u * 4u + 6u * 4u + 3u * 4u + 4u * 4u) + (size_t)net->n_adj * 4u
                 + ((size_t)nt + 1u + net->n_target_nodes) * 4u
                 + (size_t)ns * (8u + 1u + 4u + 4u + 4u + 4u + 4u * 4u + 4u + 2u * sizeof(gn_end_t))
                 + ((size_t)net->gcols * net->grows + 1u) * 4u;
    free(fb);
    net->loaded = 1;
    net->load_ms = 1000.0 * (double)(clock() - c0) / (double)CLOCKS_PER_SEC;
    if (msg && msg_size)
        msg[0] = 0;
    return 1;
}

void gn_describe(const gn_net_t *net, char *out, size_t size)
{
    snprintf(out, size, "%u nodes (%u target ends, %u spawn poses, %u pivot free), %u edges (%.1f km), %u samples, %u targets "
             "(%u target end entries, %u targets without one), index %u x %u cells of %g m, built from %u routes / %u "
             "samples, %.1f MB, loaded in %.0f ms", net->n_nodes, net->n_target_ends, net->n_spawn_nodes,
             net->n_pivot_nodes, net->n_edges,
             net->total_km, net->n_samples, net->n_targets, net->n_target_nodes, net->n_targets_without, net->gcols,
             net->grows, (double)net->cell, net->lib_routes, net->lib_samples, (double)net->bytes / 1048576.0,
             net->load_ms);
}

/* ---- spatial queries ---- */

static void gn_cell_range(const gn_net_t *net, float x, float z, float r, int *c0, int *r0, int *c1, int *r1)
{
    *c0 = (int)floorf((x - r - net->gx0) / net->cell);
    *r0 = (int)floorf((z - r - net->gz0) / net->cell);
    *c1 = (int)floorf((x + r - net->gx0) / net->cell);
    *r1 = (int)floorf((z + r - net->gz0) / net->cell);
    if (*c0 < 0)
        *c0 = 0;
    if (*r0 < 0)
        *r0 = 0;
    if (*c1 > (int)net->gcols - 1)
        *c1 = (int)net->gcols - 1;
    if (*r1 > (int)net->grows - 1)
        *r1 = (int)net->grows - 1;
}

int gn_near(const gn_net_t *net, float x, float z, float radius, uint32_t *out, int cap)
{
    if (!net->loaded || !(radius >= 0.0f))
        return 0;
    int c0, r0, c1, r1, n = 0;
    gn_cell_range(net, x, z, radius, &c0, &r0, &c1, &r1);
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c) {
            const size_t k = (size_t)r * net->gcols + (size_t)c;
            for (uint32_t q = net->cell_first[k]; q < net->cell_first[k + 1]; ++q) {
                const uint32_t i = net->cell_items[q];
                if (hypotf(net->sx[i] - x, net->sz[i] - z) <= radius) {
                    if (n < cap)
                        out[n] = i;
                    ++n;
                }
            }
        }
    return n;
}

int gn_nearest(const gn_net_t *net, float x, float z, float *dist)
{
    int best = -1;
    float bd = INFINITY;
    if (!net->loaded)
        return -1;
    const float span = hypotf((float)net->gcols * net->cell, (float)net->grows * net->cell)
                       + hypotf(x - net->gx0, z - net->gz0);
    for (float r = net->cell;; r *= 2.0f) {
        int c0, r0, c1, r1;
        gn_cell_range(net, x, z, r, &c0, &r0, &c1, &r1);
        for (int rr = r0; rr <= r1; ++rr)
            for (int c = c0; c <= c1; ++c) {
                const size_t k = (size_t)rr * net->gcols + (size_t)c;
                for (uint32_t q = net->cell_first[k]; q < net->cell_first[k + 1]; ++q) {
                    const uint32_t i = net->cell_items[q];
                    const float d = hypotf(net->sx[i] - x, net->sz[i] - z);
                    if (d < bd || (d == bd && (int)i < best)) {
                        bd = d;
                        best = (int)i;
                    }
                }
            }
        if (bd <= r || r > span)
            break;
    }
    if (dist)
        *dist = bd;
    return best;
}

/* ---- the search ---- */

static void gn_heap_up(const gn_net_t *net, int at)
{
    const int32_t item = net->st_heap[at];
    const float key = net->st_dist[item];
    while (at > 0) {
        const int parent = (at - 1) / 2;
        if (net->st_dist[net->st_heap[parent]] <= key)
            break;
        net->st_heap[at] = net->st_heap[parent];
        net->st_pos[net->st_heap[at]] = at;
        at = parent;
    }
    net->st_heap[at] = item;
    net->st_pos[item] = at;
}

static void gn_heap_down(gn_net_t *net, int at)
{
    const int32_t item = net->st_heap[at];
    const float key = net->st_dist[item];
    for (;;) {
        int child = 2 * at + 1;
        if (child >= net->heap_n)
            break;
        if (child + 1 < net->heap_n && net->st_dist[net->st_heap[child + 1]] < net->st_dist[net->st_heap[child]])
            ++child;
        if (net->st_dist[net->st_heap[child]] >= key)
            break;
        net->st_heap[at] = net->st_heap[child];
        net->st_pos[net->st_heap[at]] = at;
        at = child;
    }
    net->st_heap[at] = item;
    net->st_pos[item] = at;
}

/* a better label for state v (edge << 1 | f): cost d, came from prev */
static void gn_relax(gn_net_t *net, uint32_t v, float d, int32_t prev)
{
    if (net->st_done[v] == net->gen)
        return;
    if (net->st_stamp[v] == net->gen) {
        if (net->st_dist[v] <= d)
            return;
        net->st_dist[v] = d;
        net->st_prev[v] = prev;
        gn_heap_up(net, net->st_pos[v]);
        return;
    }
    net->st_stamp[v] = net->gen;
    net->st_dist[v] = d;
    net->st_prev[v] = prev;
    net->st_heap[net->heap_n] = (int32_t)v;
    net->st_pos[v] = net->heap_n;
    gn_heap_up(net, net->heap_n++);
}

/* the turn (deg, 0..180) from state s (arrived at its end node) onto edge e2 driven in direction f2 */
static float gn_turn_deg(const gn_net_t *net, uint32_t s, uint32_t e2, uint32_t f2)
{
    return fabsf(remainderf(net->h_out[2u * e2 + f2] - net->h_in[s], 6.2831853f)) * 57.29578f;
}

/* the turn rule at node n: the cost in s, or a negative value when the turn is not allowed there */
static float gn_turn_cost(const gn_net_t *net, uint32_t n, float deg)
{
    if (deg <= GN_TURN_FREE_DEG)
        return 0.0f;
    return (net->nodes[n].flags & GN_NODE_PIVOT) ? deg / GN_TURN_DEG_S : -1.0f;
}

static float gn_point_rect(float qx, float qz, const gr_box_t *b)
{
    const float dx = qx - b->cx, dz = qz - b->cz;
    const float u = fmaxf(fabsf(dx * b->fx + dz * b->fz) - b->hl, 0.0f), v = fmaxf(fabsf(b->fx * dz - b->fz * dx) - b->hw, 0.0f);
    return hypotf(u, v);
}

static void gn_touch(gn_net_t *net, uint32_t e)
{
    if (net->e_mark[e] == net->gen)
        return;
    net->e_mark[e] = net->gen;
    net->e_exit_a[e] = net->e_exit_b[e] = INFINITY;
    net->e_exit_sa[e] = net->e_exit_sb[e] = UINT32_MAX;
    net->e_touched[net->n_touched++] = e;
}

/* local index of an edge's end: 0 for node_a, n - 1 for node_b */
static int32_t gn_end_k(const gn_edge_t *e, uint32_t end)
{
    return end ? (int32_t)e->n - 1 : 0;
}

int gn_search(const gn_net_t *cnet, const gn_end_t *entries, int n_entries, int target, const gn_end_t *exits, int n_exits,
              const gr_box_t *boxes, int nbox, gr_pt_t *out, uint8_t *out_clr, int cap, gn_result_t *res)
{
    gn_net_t *net = (gn_net_t *)cnet; /* the search buffers are scratch; the network data is not written */
    memset(res, 0, sizeof *res);
    res->end_node = -1;
    res->entry = res->exit = UINT32_MAX;
    if (!net->loaded || !entries || n_entries <= 0 || !out || cap <= 0
        || (target >= 0 && (uint32_t)target >= net->n_targets) || (target < 0 && (!exits || n_exits <= 0)))
        return res->status = GN_BAD_ARGS;
    if (++net->gen == 0) {
        memset(net->st_stamp, 0, sizeof(uint32_t) * 2u * net->n_edges);
        memset(net->st_done, 0, sizeof(uint32_t) * 2u * net->n_edges);
        memset(net->n_goal, 0, sizeof(uint32_t) * net->n_nodes);
        memset(net->e_block, 0, sizeof(uint32_t) * net->n_edges);
        memset(net->e_mark, 0, sizeof(uint32_t) * net->n_edges);
        memset(net->s_ent_stamp, 0, sizeof(uint32_t) * net->n_samples);
        memset(net->s_ext_stamp, 0, sizeof(uint32_t) * net->n_samples);
        net->gen = 1;
    }
    const uint32_t gen = net->gen;
    net->heap_n = 0;
    net->n_touched = 0;

    /* edges through a box are not used */
    for (int b = 0; b < nbox; ++b) {
        const float reach = hypotf(boxes[b].hl, boxes[b].hw) + GN_BOX_MARGIN;
        int c0, r0, c1, r1;
        gn_cell_range(net, boxes[b].cx, boxes[b].cz, reach, &c0, &r0, &c1, &r1);
        for (int r = r0; r <= r1; ++r)
            for (int c = c0; c <= c1; ++c) {
                const size_t k = (size_t)r * net->gcols + (size_t)c;
                for (uint32_t q = net->cell_first[k]; q < net->cell_first[k + 1]; ++q) {
                    const uint32_t i = net->cell_items[q];
                    if (gn_point_rect(net->sx[i], net->sz[i], &boxes[b]) < GN_BOX_MARGIN)
                        net->e_block[net->s_edge[i]] = gen;
                }
            }
    }
    /* entries and exits, the least cost per sample */
    int n_ent = 0, n_ext = 0;
    for (int q = 0; q < n_entries; ++q) {
        const uint32_t i = entries[q].sample;
        if (i >= net->n_samples || !isfinite(entries[q].cost) || net->e_block[net->s_edge[i]] == gen)
            continue;
        if (net->s_ent_stamp[i] != gen || entries[q].cost < net->s_ent[i]) {
            n_ent += net->s_ent_stamp[i] != gen;
            net->s_ent_stamp[i] = gen;
            net->s_ent[i] = entries[q].cost;
        }
        gn_touch(net, net->s_edge[i]);
    }
    if (!n_ent)
        return res->status = GN_NO_ENTRY;
    if (target >= 0) {
        for (uint32_t q = net->target_first[target]; q < net->target_first[target + 1]; ++q) {
            net->n_goal[net->target_nodes[q]] = gen;
            ++n_ext;
        }
    } else {
        for (int q = 0; q < n_exits; ++q) {
            const uint32_t i = exits[q].sample;
            if (i >= net->n_samples || !isfinite(exits[q].cost) || net->e_block[net->s_edge[i]] == gen)
                continue;
            if (net->s_ext_stamp[i] != gen || exits[q].cost < net->s_ext[i]) {
                n_ext += net->s_ext_stamp[i] != gen;
                net->s_ext_stamp[i] = gen;
                net->s_ext[i] = exits[q].cost;
            }
            gn_touch(net, net->s_edge[i]);
        }
    }
    if (!n_ext)
        return res->status = GN_NO_EXIT;

    /* per touched edge: the entries label its two states (driven to node_b: f 1, to node_a: f 0; no turn cost); the
     * exits give the best exit from each end; entry and exit on the same edge give a direct candidate */
    float best = INFINITY;
    int kind = 0;               /* 1 direct, 2 from a state onto an exit's edge, 3 a target node */
    uint32_t b_edge = 0, b_in = 0, b_out = 0, b_state = 0, b_f2 = 0;
    for (uint32_t q = 0; q < net->n_touched; ++q) {
        const uint32_t ei = net->e_touched[q];
        const gn_edge_t *e = &net->edges[ei];
        const float L = net->e_arc[ei], rate = e->time_s / L;
        float fin = INFINITY, bin = INFINITY;   /* forward: min(ent - rate*arc); backward: min(ent + rate*arc) */
        uint32_t fin_s = 0, bin_s = 0;
        for (uint32_t k = 0; k < e->n; ++k) {
            const uint32_t i = e->first + k;
            const float arc = net->s_arc[i];
            if (net->s_ent_stamp[i] == gen) {
                const float c = net->s_ent[i];
                gn_relax(net, 2u * ei, c + rate * arc, -1 - (int32_t)i);
                gn_relax(net, 2u * ei + 1u, c + rate * (L - arc), -1 - (int32_t)i);
                if (c - rate * arc < fin) {
                    fin = c - rate * arc;
                    fin_s = i;
                }
            }
            if (target < 0 && net->s_ext_stamp[i] == gen) {
                const float x = net->s_ext[i];
                if (rate * arc + x < net->e_exit_a[ei]) {
                    net->e_exit_a[ei] = rate * arc + x;
                    net->e_exit_sa[ei] = i;
                }
                if (rate * (L - arc) + x < net->e_exit_b[ei]) {
                    net->e_exit_b[ei] = rate * (L - arc) + x;
                    net->e_exit_sb[ei] = i;
                }
                if (fin + rate * arc + x < best) {
                    best = fin + rate * arc + x;
                    kind = 1;
                    b_edge = ei;
                    b_in = fin_s;
                    b_out = i;
                }
            }
        }
        if (target < 0)
            for (uint32_t k = e->n; k-- > 0;) {
                const uint32_t i = e->first + k;
                const float arc = net->s_arc[i];
                if (net->s_ent_stamp[i] == gen && net->s_ent[i] + rate * arc < bin) {
                    bin = net->s_ent[i] + rate * arc;
                    bin_s = i;
                }
                if (net->s_ext_stamp[i] == gen && bin - rate * arc + net->s_ext[i] < best) {
                    best = bin - rate * arc + net->s_ext[i];
                    kind = 1;
                    b_edge = ei;
                    b_in = bin_s;
                    b_out = i;
                }
            }
    }
    /* Dijkstra over the states: state s = (edge << 1) | f, standing at the edge's node_b (f 1) or node_a (f 0) */
    while (net->heap_n > 0) {
        const int32_t u = net->st_heap[0];
        net->st_pos[u] = -1;
        if (--net->heap_n > 0) {
            net->st_heap[0] = net->st_heap[net->heap_n];
            net->st_pos[net->st_heap[0]] = 0;
            gn_heap_down(net, 0);
        }
        const float d = net->st_dist[u];
        if (d >= best)
            break;
        net->st_done[u] = gen;
        ++res->settled;
        const uint32_t ue = (uint32_t)u >> 1;
        const uint32_t node = (u & 1) ? net->edges[ue].node_b : net->edges[ue].node_a;
        if (target >= 0 && net->n_goal[node] == gen) {
            best = d;
            kind = 3;
            b_state = (uint32_t)u;
            break;
        }
        const gn_node_t *nd = &net->nodes[node];
        for (uint32_t q = nd->adj_first; q < nd->adj_first + nd->adj_n; ++q) {
            /* leaving the node over edge ei: from its node_a (end 0) it is driven a -> b (f2 1) */
            const uint32_t ei = net->adj[q] >> 1, f2 = (net->adj[q] & 1u) ^ 1u;
            if (net->e_block[ei] == gen)
                continue;
            const float tc = gn_turn_cost(net, node, gn_turn_deg(net, (uint32_t)u, ei, f2));
            if (tc < 0.0f)
                continue;           /* a turn over GN_TURN_FREE_DEG where the tank cannot turn on the spot */
            if (target < 0 && ei != ue && net->e_mark[ei] == gen) {
                const float x = f2 ? net->e_exit_a[ei] : net->e_exit_b[ei];
                if (d + tc + x < best) {
                    best = d + tc + x;
                    kind = 2;
                    b_state = (uint32_t)u;
                    b_edge = ei;
                    b_f2 = f2;
                    b_out = f2 ? net->e_exit_sa[ei] : net->e_exit_sb[ei];
                }
            }
            gn_relax(net, (ei << 1) | f2, d + tc + net->edges[ei].time_s, u);
        }
    }
    if (!kind)
        return res->status = GN_NOT_JOINED;

    /* the pieces, backwards: (edge, from k, to k, direction f: 1 driven a -> b) */
    int32_t *pc = net->pieces;
    int np = 0;
    uint32_t entry_s = 0;
    if (kind == 1) {
        const uint32_t k0 = b_in - net->edges[b_edge].first, k1 = b_out - net->edges[b_edge].first;
        pc[0] = (int32_t)b_edge;
        pc[1] = (int32_t)k0;
        pc[2] = (int32_t)k1;
        pc[3] = k1 >= k0;
        np = 1;
        entry_s = b_in;
        res->exit = b_out;
    } else {
        if (kind == 2) {
            const gn_edge_t *e = &net->edges[b_edge];
            pc[0] = (int32_t)b_edge;
            pc[1] = gn_end_k(e, b_f2 ^ 1u);   /* driven a -> b (f2 1) it starts at node_a */
            pc[2] = (int32_t)(b_out - e->first);
            pc[3] = (int32_t)b_f2;
            np = 1;
            res->exit = b_out;
        } else {
            res->end_node = (int)((b_state & 1u) ? net->edges[b_state >> 1].node_b : net->edges[b_state >> 1].node_a);
        }
        uint32_t s = b_state;
        for (uint32_t guard = 0;; ++guard) {
            const uint32_t ei = s >> 1, f = s & 1u;
            const gn_edge_t *e = &net->edges[ei];
            const int32_t prev = net->st_prev[s];
            if (guard > 2u * net->n_edges + 1u)
                return res->status = GN_NOT_JOINED; /* a cycle in the labels: cannot happen */
            pc[4 * np] = (int32_t)ei;
            pc[4 * np + 2] = gn_end_k(e, f);
            pc[4 * np + 3] = (int32_t)f;
            if (prev < 0) {
                entry_s = (uint32_t)(-1 - prev);
                pc[4 * np + 1] = (int32_t)(entry_s - e->first);
                ++np;
                break;
            }
            pc[4 * np + 1] = gn_end_k(e, f ^ 1u);
            ++np;
            s = (uint32_t)prev;
        }
        if (kind == 3)
            res->exit = net->edges[pc[0]].first + (uint32_t)pc[2];
    }
    res->entry = entry_s;
    res->entry_cost = net->s_ent[entry_s];
    res->exit_cost = kind == 3 ? 0.0f : net->s_ext[res->exit];
    /* the chain, forwards, and the turns at the joints */
    int n = 0;
    for (int q = np - 1; q >= 0; --q) {
        const uint32_t ei = (uint32_t)pc[4 * q];
        const gn_edge_t *e = &net->edges[ei];
        const int32_t k0 = pc[4 * q + 1], k1 = pc[4 * q + 2], step = k1 >= k0 ? 1 : -1;
        const float part = fabsf(net->s_arc[e->first + (uint32_t)k1] - net->s_arc[e->first + (uint32_t)k0]);
        res->time_s += e->time_s * part / net->e_arc[ei];
        ++res->edges;           /* every piece, the entry and exit pieces too (the generator's count) */
        if (q < np - 1) {
            /* the joint with the previous piece: its state (edge, direction) and the turn onto this one */
            const uint32_t pe = (uint32_t)pc[4 * (q + 1)], pf = (uint32_t)pc[4 * (q + 1) + 3];
            const float deg = gn_turn_deg(net, (pe << 1) | pf, ei, (uint32_t)pc[4 * q + 3]);
            if (deg > GN_TURN_FREE_DEG) {
                const uint32_t node = pf ? net->edges[pe].node_b : net->edges[pe].node_a;
                net->turn_deg[res->n_turns] = deg;
                net->turn_at[res->n_turns] = n - 1;
                net->turn_node[res->n_turns] = (int32_t)node;
                ++res->n_turns;
                res->turn_s += deg / GN_TURN_DEG_S;
            }
        }
        /* a piece after the first starts on the joint node, which the previous piece's last sample already stands for
         * (the generator's assembly: at a crossing node the two end samples may lie up to tol_m apart, and keeping
         * both would put a short backwards step into the chain) */
        for (int32_t k = q < np - 1 && k1 != k0 ? k0 + step : k0;; k += step) {
            const uint32_t i = e->first + (uint32_t)k;
            if (q < np - 1 && k == k0)
                break;          /* a piece of one sample after the first: nothing new */
            if (!n || hypotf(net->sx[i] - out[n - 1].x, net->sz[i] - out[n - 1].z) >= GN_DUP) {
                if (n >= cap)
                    return res->status = GN_TOO_LONG;
                if (n)
                    res->length_m += hypotf(net->sx[i] - out[n - 1].x, net->sz[i] - out[n - 1].z);
                out[n] = (gr_pt_t){net->sx[i], net->sz[i]};
                if (out_clr)
                    out_clr[n] = net->clr[i];
                ++n;
            }
            if (k == k1)
                break;
        }
    }
    res->n = n;
    res->pieces = np;
    res->time_s += res->turn_s;
    res->turn_deg = net->turn_deg;
    res->turn_at = net->turn_at;
    res->turn_node = net->turn_node;
    res->total = best;
    return res->status = GN_OK;
}

int gn_pure(const gn_net_t *net, float x, float z, int target, const gr_lib_t *lib, float gx, float gz, gr_pt_t *out,
            int cap, gn_result_t *res, float *d_entry, float *d_exit)
{
    float de = INFINITY, dx = INFINITY;
    const int s = gn_nearest(net, x, z, &de);
    gn_end_t entry = {(uint32_t)(s < 0 ? 0 : s), 0.0f}, exit = {0u, 0.0f};
    *d_entry = de;
    *d_exit = INFINITY;
    if (s < 0) {
        memset(res, 0, sizeof *res);
        return res->status = GN_NO_ENTRY;
    }
    if (target < 0) {
        const int g = gn_nearest(net, gx, gz, &dx);
        if (g < 0) {
            memset(res, 0, sizeof *res);
            return res->status = GN_NO_EXIT;
        }
        exit.sample = (uint32_t)g;
    }
    const int st = gn_search(net, &entry, 1, target, &exit, 1, NULL, 0, out, NULL, cap, res);
    if (st == GN_OK) {
        if (target < 0)
            *d_exit = dx;
        else if (lib && (uint32_t)target < lib->n_targets && res->end_node >= 0) {
            /* the end node to the target's point (a post: its snapped point when it has one) */
            const gr_target_t *t = &lib->targets[target];
            const int snapped = t->kind == 1 && isfinite(t->sx) && isfinite(t->sz);
            *d_exit = hypotf(net->nodes[res->end_node].x - (snapped ? t->sx : t->x),
                             net->nodes[res->end_node].z - (snapped ? t->sz : t->z));
        }
    }
    return st;
}
