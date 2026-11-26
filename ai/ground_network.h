#ifndef GROUND_NETWORK_H
#define GROUND_NETWORK_H
/* s38 road network over the route library's curves (generator: stages/s38_network, file <mission>.network.bin).
 * Plain C and stdio only. gn_load reads the file once, strictly (magic, version, every byte accounted for, every index
 * in range, the library it was built from); nothing is written. Nodes are curve junctions, ends and street crossings,
 * edges are real pieces of library curves sampled every 0.5 m, every edge usable in both directions.
 *
 * Layout (little endian, no padding):
 *   header 52 B (the design brief said 48 B; its own field list adds up to 52, as the generator writes it):
 *                char[8] "WTNET001", u32 version 1, u32 n_nodes, n_edges, n_samples, n_adj, n_targets,
 *                n_target_nodes, f32 step, f32 tol_m, u32 lib_routes, u32 lib_samples
 *   nodes   n_nodes x {f32 x, z, u32 adj_first, u16 adj_n, u16 flags}                        16 B
 *           flags bit 0 target end, bit 1 spawn pose, bit 2 pivot free (a tank can turn on the spot there)
 *   edges   n_edges x {u32 node_a, node_b, first_sample, n_samples, f32 length_m, time_s,
 *                      u16 uses, u8 min_clr, u8 flags}                                        28 B
 *   adj     n_adj x u32 (edge << 1) | end (0: the node is node_a, 1: node_b)
 *   target_first (n_targets + 1) x u32, target_nodes n_target_nodes x u32
 *   samples n_samples x {f32 x, z}, clr n_samples x u8 (0.1 m)
 *
 * The search (gn_search) is a Dijkstra over the states "arrived at a node over an edge end" (2 x n_edges; binary heap),
 * cost = seconds: edge time_s (a partial edge in proportion to its arc length) plus the turn rule at every node: the
 * angle between the arrival heading and the leaving heading (each over the 2 m of the edge next to the node, 4
 * samples) is free up to GN_TURN_FREE_DEG; a larger turn only at a pivot-free node, costing angle / GN_TURN_DEG_S s.
 * No turn cost at the entry. From entry samples with entry costs to a library target's end nodes or to exit samples
 * with exit costs (the turn rule holds onto the exit's edge). One search at a time: every buffer is allocated at load,
 * nothing per search. */
#include <stddef.h>
#include <stdint.h>

#include "ground_routes.h"

#define GN_NODE_TARGET 1u
#define GN_NODE_SPAWN 2u
#define GN_NODE_PIVOT 4u

#define GN_TURN_FREE_DEG 30.0f  /* a turn at a node up to this: free */
#define GN_TURN_DEG_S 15.0f     /* deg / s of a larger turn (on the spot): the library's start-turn rate */
#define GN_HEAD_SAMPLES 4       /* the turn headings over this many sample steps (2 m) next to the node */

typedef struct { float x, z; uint32_t adj_first; uint16_t adj_n, flags; } gn_node_t;
typedef struct {
    uint32_t node_a, node_b, first, n;
    float length_m, time_s;
    uint16_t uses;
    uint8_t min_clr, flags;
} gn_edge_t;

/* an entry or exit: a network sample and its cost in seconds */
typedef struct { uint32_t sample; float cost; } gn_end_t;

enum { GN_OK = 0, GN_NO_ENTRY = 1, GN_NO_EXIT = 2, GN_NOT_JOINED = 3, GN_TOO_LONG = 4, GN_BAD_ARGS = 5 };

typedef struct {
    int status;
    int n;                   /* samples written (entry sample .. exit sample / end node) */
    int edges;               /* edge pieces (whole or partial) the chain runs on */
    uint32_t entry, exit;    /* the entry sample and the exit sample (target mode: the end node's sample) */
    int end_node;            /* target mode: the end node reached; -1 otherwise */
    float time_s;            /* network seconds incl. the turn costs (entry and exit costs not included) */
    float turn_s;            /* ... of which the counted turns */
    float length_m;          /* arc length of the chain */
    float entry_cost, exit_cost;
    float total;             /* entry cost + time_s + exit cost: what the search minimised */
    int settled;             /* states settled */
    int pieces;              /* net->pieces holds them, last piece first: (edge, from k, to k, direction) each */
    /* the counted turns (> GN_TURN_FREE_DEG), in path order: angle (deg), the chain index of the joint sample, the node;
     * the arrays live in the network's buffers until the next search */
    int n_turns;
    const float *turn_deg;
    const int32_t *turn_at, *turn_node;
} gn_result_t;

typedef struct gn_net {
    int loaded;
    uint32_t version, n_nodes, n_edges, n_samples, n_adj, n_targets, n_target_nodes, lib_routes, lib_samples;
    float step, tol_m;
    gn_node_t *nodes;
    gn_edge_t *edges;
    uint32_t *adj, *target_first, *target_nodes;
    float *sx, *sz;          /* samples */
    uint8_t *clr;            /* 0.1 m */
    /* built at load */
    uint32_t *s_edge;        /* the edge of each sample */
    float *s_arc;            /* metres from the edge's first sample */
    float *e_arc;            /* per edge: the arc length of its samples */
    float *h_in, *h_out;     /* per state (edge << 1 | f, f 1 = driven a -> b): heading arriving at its end node /
                              * leaving its start node (rad) */
    uint32_t n_target_ends, n_spawn_nodes, n_pivot_nodes, n_targets_without;
    double total_km;
    /* spatial index of the samples: cells of `cell` m from (gx0, gz0), cell_first (cols * rows + 1), cell_items */
    float gx0, gz0, cell;
    uint32_t gcols, grows;
    uint32_t *cell_first, *cell_items;
    /* search buffers (load time; one search at a time), per state */
    float *st_dist;
    int32_t *st_prev;        /* the state it came from, or -1 - (the entry sample) */
    uint32_t *st_stamp, *st_done;
    int32_t *st_heap, *st_pos;
    uint32_t *n_goal;        /* per node: a target end node of this search */
    int heap_n;
    uint32_t gen;
    uint32_t *e_block, *e_mark;
    float *e_exit_a, *e_exit_b;            /* best exit from end a / end b of an edge (seconds incl. exit cost) */
    uint32_t *e_exit_sa, *e_exit_sb;       /* ... and its sample */
    uint32_t *e_touched;
    uint32_t n_touched;
    float *s_ent, *s_ext;                  /* entry / exit cost per sample, valid where s_stamp == gen */
    uint32_t *s_ent_stamp, *s_ext_stamp;
    /* caller buffers sized n_samples: for gn_near and for building entry / exit lists without allocating */
    uint32_t *near_buf;
    gn_end_t *ent_buf, *ext_buf;
    int32_t *pieces;         /* the chain's pieces, 4 x (2 n_edges + 2) */
    float *turn_deg;         /* the counted turns of the last result, 2 n_edges + 2 each */
    int32_t *turn_at, *turn_node;
    size_t bytes;
    double load_ms;
} gn_net_t;

/* Loads the network file. lib: the loaded route library it must belong to (n_targets, n_routes, n_samples); NULL skips
 * that check (tests only). 1 on success; 0 with a reason in msg, the network left empty. */
int gn_load(gn_net_t *net, const char *path, const gr_lib_t *lib, char *msg, size_t msg_size);
void gn_free(gn_net_t *net);
/* One line: nodes, edges, km, samples, targets / end nodes, spawn / pivot nodes, index, MB, load ms. */
void gn_describe(const gn_net_t *net, char *out, size_t size);

/* The network samples within radius m of (x, z) (straight distance), written to out (at most cap; net->near_buf has
 * n_samples entries). Returns how many there are (may exceed cap: only cap written). */
int gn_near(const gn_net_t *net, float x, float z, float radius, uint32_t *out, int cap);
/* The network sample nearest to (x, z); *dist its distance. -1 when the network is empty. */
int gn_nearest(const gn_net_t *net, float x, float z, float *dist);

/* The search. entries (n_entries > 0, each a sample and a cost in s, both directions from it, no turn cost);
 * target >= 0: to any of the target's end nodes (exits ignored); target < 0: to the exits (n_exits > 0): onto an exit's
 * edge from a node under the turn rule (not from a state over that same edge), or along the entry's own edge.
 * boxes: no edge with a sample within 0.5 m of a box's rectangle is used (entries and exits on such an edge are
 * dropped). The chain (entry sample .. exit sample or end node) is written to out (and its clearance bytes to out_clr
 * when not NULL), at most cap samples (GN_TOO_LONG beyond). */
int gn_search(const gn_net_t *net, const gn_end_t *entries, int n_entries, int target, const gn_end_t *exits, int n_exits,
              const gr_box_t *boxes, int nbox, gr_pt_t *out, uint8_t *out_clr, int cap, gn_result_t *res);

/* The "pure" query (test harness): entry = the sample nearest to (x, z) (cost 0); to the target's end nodes
 * (target >= 0) or to the sample nearest to (gx, gz) (cost 0). *d_entry / *d_exit: start -> entry and exit -> goal
 * (target mode: the end node -> the target's point, a post's snapped point when it has one). */
int gn_pure(const gn_net_t *net, float x, float z, int target, const gr_lib_t *lib, float gx, float gz, gr_pt_t *out,
            int cap, gn_result_t *res, float *d_entry, float *d_exit);

#endif
