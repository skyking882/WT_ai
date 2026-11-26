/* ground_command.h (s47, s48): the boundary between the command layer (ground_command.c: lanes, zone quotas,
 * area caps, orders, posts, the team planner and the commander's tick) and the game adapter (ground_brain.c).
 * Everything both units use is declared here: the shared types and tables (moved unchanged; tables are defined in
 * exactly one unit), the pure helpers both need (static inline), the brain functions the command unit calls
 * (s49: called directly, game-free; they were the gc_host_* services), and the command functions the adapter
 * calls. Standard C headers, ground_host.h and the game-free project headers only. */
#ifndef GROUND_COMMAND_H
#define GROUND_COMMAND_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <xmmintrin.h>   /* __m128 in strat_state_t */

#include "ground_host.h"   /* s49: the host's neutral access functions (gh_unit_pos for the command unit) */

typedef unsigned char u8;
#ifndef MAX_PATH
#define MAX_PATH 260       /* config_t path fields; the Windows SDK value (the brain has it) */
#endif

#define MAX_SEATS 32
#define MAX_SPOTS 200
typedef struct { float x, z, score; } spot_t;
enum { ROLE_TANK, ROLE_TD, ROLE_HEAVY, ROLE_SPAA };
enum { OBJECTIVE_UNKNOWN, OBJECTIVE_CAPTURE, OBJECTIVE_KILL };
typedef struct {
    int loaded;
    char log_dir[MAX_PATH];
    uint32_t brain_mask;
    int roles[MAX_SEATS];
    spot_t spots[MAX_SPOTS];
    int nspots;
    int direct_every;   /* every Nth seat of a team goes straight for the objective */
    int plan;           /* 0: probes only (fixed-point plan was rejected 2026-11-09) */
    int vis;            /* bot tank vision: engine line of sight instead of the 40 m hearing record */
    float vis_near;     /* always sensed within this many metres (about 35 m) */
    float vis_max;      /* never seen beyond this many metres */
    int los_ms;         /* bot tank line-of-sight re-trace interval, ms (42 = 24 Hz) */
    int los_tier;       /* 20261115: tiered per-pair re-trace (wt_ground_los_interval) instead of los_ms per unit */
    int riex_owner;     /* 20261115: host has no render owner for the scenery-object layers (riex_owner_tick) */
    int native_control; /* native goals with the shared traffic and geometry gate */
    int lanes;          /* approach-lane executor (ground_lanes.py) instead of the old plan */
    int preaim;         /* idle turret pre-aim (lanes only) */
    int react;          /* per-tank reactions: engage stop, under fire, pursue (lanes only) */
    int team;           /* 1 Hz team planner: intercept infiltrators, anchor points (lanes only) */
    char geometry_file[MAX_PATH]; /* layout-specific 2 m collision samples */
    char scan_file[MAX_PATH];   /* one-off obstacle scan output (absent: no scan) */
    char clutter_file[MAX_PATH]; /* bot_trees/clutter/<mission>.bin, WTCLT001 4 m clutter table (route cost) */
    float clutter_k;            /* route cost factor 1 + clutter_k * clutter (default 4, 0: off) */
    float scan_box[4];          /* x0, z0, x1, z1 */
    float scan_step;
    int scan_budget_us;         /* per slice (default SCAN_SLICE_US) */
    int scan_flags;             /* flags of the game's ray test (default 6) */
    int direct;                 /* drive tactical-route hops straight (path count 0), not via the game's own path search */
    float tree_cover;           /* pick weight of the tree-line route variant (fast 1, cover 1.3, flank 0.6) */
    char mission[64];           /* mission name (logs) */
    int objective_mode;         /* from the mission catalog; unknown preserves lane inference */
    char blocked_file[MAX_PATH];    /* bot_trees/blocked/<mission>.csv: where tanks got stuck (T1 planner input) */
    char strategy_file[MAX_PATH];   /* bot_trees/strategy/<mission>.bin (S1 strategy layer, log only) */
    /* review_5003 (2026-11-10) fine switches, written by ground_brain.py from the env. */
    int stab;           /* A1 turret stabiliser feed-forward while the gunner has a target (WT_GROUND_STAB) */
    int gate;           /* A2 own fire gate in the Gun fire entry (WT_GROUND_GATE; the hook needs it too) */
    int face;           /* hull pivot towards the target, reload window only (WT_GROUND_FACE) */
    int shortstop;      /* 0 none, 1 fire stop (A3), 2 the old timed 5 s / 12 s cycle (WT_GROUND_SHORTSTOP) */
    int headbias;       /* nose-to-threat course bias in lane_drive, > 400 m only (WT_GROUND_HEADBIAS) */
    int pursue;         /* chase a lost target's last position (WT_GROUND_PURSUE) */
    int close;          /* B1 close fight: stop and face an enemy seen under CLOSE_M (WT_GROUND_CLOSE) */
    int probe_seat;     /* A7 10 Hz turret probe seat, 0 = first seat whose gunner gets a target */
    int commander;      /* S2 team commander replaces lane_advance / retarget (WT_GROUND_COMMANDER) */
    /* 2026-11-11 stabiliser / hull work. */
    int stab_native;    /* switch the engine's own gun stabiliser on for bot tanks (WT_GROUND_STAB_NATIVE) */
    int movefire;       /* fire on the move only with a working vertical stabiliser (WT_GROUND_MOVEFIRE) */
    int stab_emu;       /* stab2: DLL two-axis stabiliser emulation where the engine does not stabilise (WT_GROUND_STAB_EMU) */
    int rear;           /* B7: stop and turn the hull when the fight is behind it (WT_GROUND_REAR) */
    int cover;          /* 2026-11-11 cover seeking on the G10 hull-down layer (WT_GROUND_COVER) */
    int approach;       /* 2026-11-15 covered approach under the fire-threat field (WT_GROUND_APPROACH, read here) */
    /* s11 2026-11-15: live local perception, logs only (ground_lidar.c; WT_GROUND_LIDAR, read here too). */
    int lidar;
    int lidar_us_per_tick;      /* lidar work cap per 16 ms window */
    float lidar_range_m;
    /* s17 2026-11-16: overhead probes into one shared map-wide cache, fused by
     * the local planner (ground_overhead.c; WT_GROUND_OVERHEAD, read here too;
     * 0: no probes and exactly s16). */
    int overhead;
    int overhead_us_per_tick;   /* overhead work cap per 16 ms window */
    float overhead_range_m;     /* probed around each controlled tank */
    float overhead_cell_m;      /* cache cell */
    /* s13 2026-11-16: local trajectory-sampling planner on the fused lidar /
     * obstacle-cell occupancy (ground_motion.c; WT_GROUND_LOCAL=0 or local=0:
     * the s12 static candidate check, byte for byte). */
    int local;
    /* s16 2026-11-16: exit mode at the spawn, open space first, route after
     * (ground_motion.c; WT_GROUND_SPAWN_EXIT=0 or spawn_exit=0: exactly s15). */
    int spawn_exit;
    int bot_phys_hz;            /* 2026-11-16: bot tank physics tick rate (ground_phys_rate.inc), 0 = game default */
    /* s22 2026-11-17: the WASD driver (ground_drive.inc), its keys written right before the game's input layer
     * on every unit update (ground_control_graft.inc). WT_GROUND_GRAFT=0 or graft=0: no hook, the s15 tracker
     * writes its four bytes on the behaviour-tree tick, as before s20. */
    int graft;
    /* s24 2026-11-17: path mode, only with graft (ground_motion.c gm_path_follow / gm_path_decide): a route from
     * where the tank stands to its target (no via points, spread across a side), the next route vertex as the
     * aim, a stop distance, an avoidance layer idle unless something keeps the stop distance under 2 m, and the
     * driver's keys; the local sampler, the spawn queue / exit mode and the old back-off rules take no part.
     * WT_GROUND_PATH=0 or path=0 (or graft=0): exactly s22. s26: default 0 (s22c); WT_GROUND_PATH=1 gives s24. */
    int path;
    /* s26 2026-11-17: the pre-generated route library (ground_routes.c, ground_motion_library.inc). Library mode is on
     * for a battle only with graft on, routes on (WT_GROUND_ROUTES / routes=, default 1) and both files loaded
     * (routes_file= / routes_map_file=, bot_trees/routes/<mission>.routes.bin / .map.bin, written by
     * ground_brain.py); otherwise s22c (or s24 with path=1). */
    int routes;
    char routes_file[MAX_PATH];
    char routes_map_file[MAX_PATH];
    /* s38: the road network over the library's curves (routes_net_file=, bot_trees/routes/<mission>.network.bin, written
     * by ground_brain.py when the file exists; ground_network.c). Absent or refused: no NET source (s37). */
    char routes_net_file[MAX_PATH];
    /* s26: the commander's nearness term for a tank's first order of its life (cmd_tick; WT_GROUND_NEAR / near=,
     * default 1; 0: the cost as before). */
    int near_first;
    /* s28 2026-11-17 tactics (ground_tactics.inc), each its own switch, default on; 0 = s27 for that step:
     *  sense    (WT_GROUND_SENSE / sense=)       presence within sense_m is a hint only, seeing is the sight table;
     *  alert    (WT_GROUND_ALERT / alert=)       team rear-area alert record that survives a gap in sight;
     *  takeover (WT_GROUND_TAKEOVER / takeover=) readiness, advance gate, target re-evaluation;
     *  waitpeek (WT_GROUND_WAITPEEK / waitpeek=) per-tank reaction record: continue / wait / peek / come out. */
    int sense, alert, takeover, waitpeek;
    float sense_m;
    /* s29: threat facing (WT_GROUND_THREATFACE / threatface=, default 1; 0 = s28): one decision turns the hull to the
     * threat (or first to cover within cover_m, WT_GROUND_COVER_M / cover_m=, default 30) */
    int threatface;
    float cover_m;
    /* s32: the team plan (WT_GROUND_TEAMPLAN / teamplan=, default 1; no plan pack for the mission or 0 = s29) */
    int teamplan;
    /* s34: intel through the command layer (WT_GROUND_INTEL / intel=, default 1; 0 = s33) */
    int intel;
} config_t;
/* T2 (2026-11-10): tactical routes and covered firing spots from build_tactical.py. */
#define MAX_TROUTES 160
#define MAX_TWP 64
#include "ground_navigation.h"
#include "ground_motion.h"
#include "ground_routes.h"
#include "ground_task_order.h"

extern int authored_spawn_seat_army[MAX_SEATS];
#define MAX_TSPOTS 320
/* TV_STREET (v13, stage s10): street_flank, a parallel side street on the road network (spec section 7). */
enum { TV_FAST, TV_COVER, TV_FLANK, TV_TREES, TV_STREET };
typedef struct {
    int army, variant, n;
    float zx, zz, sx, sz;       /* capture point, spawn group */
    float x[MAX_TWP], z[MAX_TWP];
} troute_t;
typedef struct { int army, kind; float zx, zz, x, z, score; } tspot_t;   /* kind 0 terrain, 1 building, 2 trees */
extern troute_t troutes[MAX_TROUTES + MAX_SEATS];
extern int ntroutes;
extern tspot_t tspots[MAX_TSPOTS];
extern int ntspots;

/* v13 road network (SPEC-v13 sections 5-8b, stage s10), written by ground_brain.py from the
 * tactical json; none of these lines (WT_GROUND_ROADS=0 or no data) = the behaviour before.
 *   tchoke=army,id,zx,zz,x,z,value,line,kind        choke point protecting capture (zx, zz)
 *   tchokespot=army,choke_id,x,z,face_x,face_z,score ambush spot of that choke (up to 3)
 *   tow=army,id,x,z,face_x,face_z,score,line,arc_deg,cover   precomputed overwatch position
 * line 1 holds, line 2 is the push line (cmd_tick switches with hysteresis, roads_line_update). */
#define MAX_RCHOKES 64
#define MAX_RCHOKE_SPOTS 192
#define MAX_ROWS 48
typedef struct { int army, id, line, kind; float zx, zz, x, z, value; } rchoke_t;
typedef struct { int army, choke; float x, z, fx, fz, score; } rchoke_spot_t;
typedef struct { int army, id, line; float x, z, fx, fz, score, arc, cover; } rows_t;
extern rchoke_t rchokes[MAX_RCHOKES];
extern int nrchokes;
extern rchoke_spot_t rchoke_spots[MAX_RCHOKE_SPOTS];
extern int nrchoke_spots;
extern rows_t rows[MAX_ROWS];
extern int nrows;
/* Per-vehicle data for the per-task fit (spec 8b): vfit.<seat>= / vfitr.<seat>= (reserve vehicle)
 * depression_deg,elevation_deg,reverse_kph,turret,turret_traverse_dps,hull_traverse_dps,reload_s,
 * length_m,width_m; -1 = missing (a neutral 0.5 term). */
typedef struct { int ok; float dep, elev, rev, turret, tdps, hdps, reload, len, wid; } vfit_t;
extern vfit_t seat_vfit[MAX_SEATS], reserve_vfit[MAX_SEATS];

/* Approach lanes (ground_lanes.py writes them; design: docs/ground_ai_algorithm_20261120.md, section 2.4). */
#define MAX_LANES 32
#define MAX_LANE_PTS 6
typedef struct {
    int army, kind, n;            /* kind 0 target lane (S0..S4), 1 flank lane (S0, F1, F2) */
    float zx, zz, zr;             /* the lane's capture point (flank: the outer point) */
    float x[MAX_LANE_PTS], z[MAX_LANE_PTS];
} lane_t;
extern lane_t lanes[MAX_LANES];
extern int nlanes;
extern int seat_lane[MAX_SEATS], seat_has_lane[MAX_SEATS];
extern float seat_hp[MAX_SEATS];

/* S2 (2026-11-11): vehicle class per seat from bot_trees/vehicle_classes.json
 * (build_vehicle_classes.py), config line cls.<seat>=role,flags,hp_per_t,
 * pen1000_mm,front_ke_mm,shell_speed_m_s,w_flank,w_breach,w_overwatch (x100).
 * role: 0 flank, 1 breach, 2 overwatch, 3 escort (= SR_* below). */
enum {
    VC_LIGHT = 1, VC_MEDIUM = 2, VC_HEAVY = 4, VC_TD = 8, VC_MISSILE = 16, VC_SPAA = 32, VC_WHEELED = 64,
    VC_MBT = 128, VC_ATGM = 256, VC_SCOUT = 512, VC_ERA = 1024, VC_APS = 2048, VC_COMPOSITE = 4096, VC_HE_ONLY = 8192,
    VC_STAB_H = 0x4000, VC_STAB_V = 0x8000      /* main gun gunStabilizer hasHorizontal / hasVertical */
};
/* stab_h_kmh / stab_v_kmh (fields 10, 11; 2026-11-11): -1 no stabiliser on that
 * axis, 0 no speed limit, else its SpeedLimitKPH. Older 9-field lines: -1. */
typedef struct { int ok, role; uint32_t flags; float hp, pen, front, speed, w[3], stab_h_kmh, stab_v_kmh; } vcls_t;
extern vcls_t seat_cls[MAX_SEATS];
extern vcls_t reserve_cls[MAX_SEATS];

extern int human_slot;
extern u8 *slot_unit[64];
extern int slot_alive[64], slot_army[64];
extern uint64_t slot_life[64];
extern config_t cfg;
extern gr_lib_t routes_lib;
extern int routes_lib_state;

/* s48: the team knowledge record of one enemy slot (gc_host_contact). */
typedef struct {
    float x, z, vx, vz;
    uint64_t last_ms, first_ms;     /* last sighting; start of the current run of sightings */
    int by;                         /* last observer slot */
} contact_t;

static inline float dist2d(float ax, float az, float bx, float bz)
{
    return sqrtf((ax - bx) * (ax - bx) + (az - bz) * (az - bz));
}

#define STRAT_RING 8
static inline int ring_count(const uint64_t *r, uint64_t ms, uint64_t window)
{
    int n = 0;
    while (n < STRAT_RING && r[n] && ms - r[n] <= window)
        ++n;
    return n;
}
extern uint64_t hits_taken_ms[64][STRAT_RING], hits_dealt_ms[64][STRAT_RING];
extern int last_hitter[64];
extern uint64_t last_hitter_ms[64];
extern uint64_t last_unit_scan;

enum { LROLE_ASSAULT, LROLE_OVERWATCH, LROLE_FLANKER, LROLE_SCREEN };
/* OB_STAY (stage 2): an immobilised tank fights where it stands; never an objective in the list. */
enum { OB_NONE, OB_CAPTURE, OB_HOLD, OB_SUPPORT, OB_OVERWATCH, OB_PUSH, OB_PRESS, OB_STAY, OB_N };
#define CMD_POST_ID_BASE 100
#define CMD_POST_SELF_DEFENSE_M 200.0f
enum { H_NONE, H_UNTIL_OWNED, H_FOR, H_FOREVER, H_ADVANCE }; /* H_ADVANCE: hold, then attack the nearest point not ours */
#define SLOT_PT (-1)            /* route entry: a slot inside the lane's capture point */
#define MAX_ROUTE 6
#define SPAWN_CALM_MS 15000     /* ... or this soon after it */
typedef struct { int pt, hold; float secs; } route_step_t;
/* ---- lane_seat_t fields by user (s49, written by tools_split_fields.py --per-file --write) ----
 * Comment only: no field is moved or reordered. "written" includes a field handed on by address (&s->f);
 * the groups are files: command unit = ground_command.c and its .inc files; tactics = ground_tactics.inc,
 * ground_threat.inc; approach = ground_approach.inc; order execution = ground_brain.c (lanes, reactions,
 * cover, order application), ground_navigation.inc; driving glue = ground_vehicle_control.inc,
 * ground_control_graft.inc, ground_drive.inc, ground_geometry.inc, the lidar / overhead adapters,
 * ground_phys_rate.inc; fire control = ground_fire.inc; the host = ground_host.c; header helpers = the
 * static inline helpers of this header (used by both units).
 * written by the command unit (44): lane, hold_start, task, task_tgt, task_x, task_z, task_side, task_since,
 *      task_why, spawn_pose1, street_closed_ms, plan_lane1, plan_slot, plan_lane_ms, pusher, task_point, cmd_release,
 *      cmd_type, cmd_id, cmd_lane, cmd_x, cmd_z, cmd_since, cmd_ntype, cmd_nid, cmd_nlane, cmd_nx, cmd_nz,
 *      cmd_apply_ms, cmd_side, post_in_place_ms, post_enemy_ms, post_hit_ms, immobile, imm_flags, imm_x, imm_z,
 *      imm_tick_x, imm_tick_z, imm_ref_ms, imm_need_ms, imm_tick_ms, imm_since, imm_retry_ms
 * read, not written, by the command unit (44): unit, army, n, step, route, sx, sz, born_ms, holding, last_ms,
 *      l1_until, native_until, x, z, fx, fz, vx, vz, motion_state, mode, wants_move, eng, ex, ez, eng_seen_ms,
 *      stopping, task_alert, route_variant, yield_to, pose_observed_ms, scene_epoch, motion_handoff_until,
 *      motion_cmd_ms, nav, direct_off_until, route_turn_until, move_target_x, move_target_z, parked, eng_own,
 *      fight_since, fight_yield, cmd_parked, ap_state
 * not used by the command unit, used only by approach (25): ap_assembly, ap_bounds, ap_final, ap_generation, ap_task,
 *      ap_since, ap_until, ap_next_ms, ap_eval_ms, ap_cool_until, ap_replan_ms, ap_run_x, ap_run_z, ap_end_x,
 *      ap_end_z, ap_face_x, ap_face_z, ap_run_m, ap_route_threat, ap_best, ap_plan_m, ap_odo, ap_px, ap_pz,
 *      ap_progress_ms
 * not used by the command unit, used only by order execution (93): tx, tz, d, gt_next, pursue_until, retreat_until,
 *      retreat_cool_until, rx, rz, gunner_log_next, relief_peer, anchor_log_ms, spot, direct_on, route_drive,
 *      unstuck_n, bias_x, bias_z, bias_until, retarget_next, close_slot, close_seen_ms, close_log_ms, stop_shots,
 *      eng_switch_ms, fight_retry_ms, fight_window_ms, fight_x, fight_z, fight_last_x, fight_last_z, fight_best,
 *      fight_post_x, fight_post_z, fight_post_ox, fight_post_oz, fight_last, fight_post, cmd_replan_ms, post_option,
 *      post_retry, post_wp, post_traffic_failures, post_best, post_progress_ms, flank_lane, flank_step,
 *      flank_attempted, flank_replanned, flank_check_ms, flank_exposure, flank_attempt_x, flank_attempt_z,
 *      flank_generation, rear_since, rear_cool_until, pivot_creep, stab_logged, cov_q, cov_type, cov_shots,
 *      cov_close_tried, cov_x, cov_z, cov_dist, cov_until, cov_since, cov_cool_until, cov_try_ms, cov_hop_best,
 *      cov_hop_progress_ms, cov_fail_count, cov_fail_x, cov_fail_z, cov_fail_origin_x, cov_fail_origin_z,
 *      cov_fail_threat_x, cov_fail_threat_z, cov_fail_route_x, cov_fail_route_z, cov_fail_ms, cov_dest_ok,
 *      cov_dest_q, cov_dest_type, cov_dx, cov_dz, cov_ox, cov_oz, cov_dest_ms, cov_dest_seen, home_set, home_x,
 *      home_z
 * not used by the command unit, used only by driving glue (9): move_log_ms, move_log_peer, move_log_wp,
 *      move_log_reason, move_log_gen, pivot_start, pivot_cool_until, pivot_err0, pivot_tgt
 * not used by the command unit, used by several of those (39): node (order execution, driving glue),
 *      role (approach, order execution, header helpers), ax (approach, order execution, driving glue),
 *      az (approach, order execution, driving glue), anchor_ms (approach, order execution, driving glue),
 *      gt_slot (tactics, order execution, driving glue, fire control),
 *      stop_cool_until (order execution, fire control), face_x (tactics, approach, order execution, driving glue),
 *      face_z (tactics, approach, order execution, driving glue), face (approach, order execution, driving glue),
 *      last_hit_ms (order execution, header helpers), goal_target1 (order execution, driving glue),
 *      tr (tactics, approach, order execution, driving glue), wpi (tactics, approach, order execution, driving glue),
 *      wpn (approach, order execution, driving glue), route_generation (approach, order execution, driving glue),
 *      native_tried (approach, order execution), route_interrupted (approach, order execution, driving glue),
 *      route_native_only (approach, order execution), move_seq (order execution, driving glue),
 *      pivoting (order execution, driving glue), bias_on (order execution, driving glue),
 *      hull_rate (order execution, fire control), close (order execution, driving glue),
 *      fight_life (tactics, order execution), fight_effect_ms (order execution, header helpers),
 *      fight_move_until (order execution, header helpers), fight_exhausted_ms (tactics, order execution),
 *      fight_move (tactics, order execution, driving glue, header helpers),
 *      post_defending (order execution, driving glue), post_gx (order execution, header helpers),
 *      post_gz (order execution, header helpers), rear (order execution, driving glue),
 *      cov_use (order execution, driving glue), cov_state (order execution, driving glue, fire control),
 *      ap_rush (approach, order execution), ap_pick_blind (approach, order execution),
 *      ap_x (approach, order execution), ap_z (approach, order execution)
 * used by none of these files (49): l1x, l1z, motion_contact_owner, relief_progress_ms, after_ms, relief_x, relief_z,
 *      relief_best, relief_reverse_m, relief_turn_best, relief_phase, relief_attempt, motion_leg_serial,
 *      motion_phase_until, motion_candidate_ms, motion_leg_x, motion_leg_z, motion_leg_dx, motion_leg_dz,
 *      motion_leg_m, motion_resume_x, motion_resume_z, motion_exit_extended, motion_resume_native,
 *      motion_candidate_type, motion_candidate_id, native_limit_ms, traffic_replan, recover_generation, recover_wp,
 *      recover_x, recover_z, recover_start_x, recover_start_z, move_log_owner, move_log_attempt, recovery_plan_kind,
 *      recovery_plan_serial, slow_target_x, slow_target_z, slow_start_angle, route_brake, slow_since, unstuck_x,
 *      unstuck_z, pivot_checked, cmd_hx, cmd_hz, cmd_dest_ms
 * ---- end of the lane_seat_t field users ---- */
typedef struct {
    u8 *unit, *node;
    int army, lane, role, n, step;
    route_step_t route[MAX_ROUTE];
    float sx, sz;               /* where this life started (S0..S2 are computed from it) */
    uint64_t born_ms;
    float tx, tz;               /* current command point */
    int holding;
    uint64_t hold_start;
    uint64_t last_ms, l1_until, native_until;
    float l1x, l1z;
    float x, z, d;              /* last position and distance to the command point */
    float fx, fz;               /* heading */
    float vx, vz;               /* smoothed ground velocity, m/s */
    enum gm_state motion_state;
    float ax, az;               /* anchor: where it last moved MOVE_M from */
    uint64_t anchor_ms;
    /* Reactions (2026-11-10, plan L2). */
    int mode;                   /* M_* */
    int wants_move;             /* 0 while holding or stopped on purpose (contact solver) */
    int gt_slot;                /* gunner's target slot, -1 none (read at 5 Hz) */
    uint64_t gt_next;
    int eng;                    /* slot engaged (gunner or own eyes), -1 */
    float ex, ez;               /* where it was last seen */
    uint64_t eng_seen_ms;
    int stopping;               /* engage short stop in progress */
    uint64_t stop_cool_until;
    float face_x, face_z;       /* hull facing target while stopped */
    int face;
    uint64_t pursue_until;
    uint64_t last_hit_ms, retreat_until, retreat_cool_until;
    float rx, rz;
    uint64_t gunner_log_next;
    /* Team task (L1): intercept enemy slot task_tgt. */
    int task, task_tgt;
    float task_x, task_z, task_side;
    uint64_t task_since;
    const char *task_why;
    int task_alert;             /* s28 alert=1: 1 + the rear-area alert record this task hangs on (0: none) */
    /* s32 teamplan: the accepted order's route variant (to the driving side), and 1 + this life's spawn pose (0 unknown) */
    int route_variant, spawn_pose1;
    uint64_t street_closed_ms;  /* s34: the driving side last reported "no other street" */
    /* s35 lanes: 1 + the plan lane this life was given and its end slot; 1 + the library target of the accepted order's
     * slot (the driving side takes that target instead of mapping the goal) */
    int plan_lane1, plan_slot, goal_target1;
    uint64_t plan_lane_ms;
    /* Contact solver (P1). */
    int yield_to;
    int motion_contact_owner;
    uint64_t relief_progress_ms, after_ms;
    float relief_x, relief_z, relief_best, relief_reverse_m, relief_turn_best;
    int relief_peer, relief_phase, relief_attempt;
    uint64_t motion_leg_serial, pose_observed_ms, scene_epoch;
    uint64_t motion_phase_until, motion_handoff_until, motion_cmd_ms, motion_candidate_ms;
    float motion_leg_x, motion_leg_z, motion_leg_dx, motion_leg_dz, motion_leg_m;
    float motion_resume_x, motion_resume_z;
    int motion_exit_extended, motion_resume_native, motion_candidate_type, motion_candidate_id;
    uint64_t anchor_log_ms;
    /* T2 tactical route */
    int tr, wpi, wpn, spot;
    nav_state_t nav;
    uint64_t direct_off_until, native_limit_ms, route_turn_until;
    unsigned route_generation;
    int native_tried, route_interrupted, route_native_only, traffic_replan;
    unsigned recover_generation;
    int recover_wp;
    float recover_x, recover_z, recover_start_x, recover_start_z;
    uint64_t move_seq, move_log_ms;
    int move_log_owner, move_log_peer, move_log_wp, move_log_attempt, move_log_reason;
    int recovery_plan_kind;
    uint64_t recovery_plan_serial;
    unsigned move_log_gen;
    float move_target_x, move_target_z, slow_target_x, slow_target_z, slow_start_angle;
    int direct_on, route_brake;
    /* Drive fixes (2026-11-10 21:00). */
    int parked;                 /* in position on an intercept task (hull may pivot) */
    int pusher;                 /* nearest tank to a point that is not ours: no short stops, no pursuit */
    int route_drive;            /* follow_troute commanded a waypoint this tick */
    uint64_t slow_since;    /* fast unstuck (wt_ground_after) */
    float unstuck_x, unstuck_z;
    int unstuck_n;
    int pivoting, pivot_checked;                /* hull pivot (wt_ground_after) */
    uint64_t pivot_start, pivot_cool_until;
    float pivot_err0;
    int task_point;                             /* the task defends a point of ours */
    int bias_on;                                /* nose-to-threat detour point (lane_drive) */
    float bias_x, bias_z;
    uint64_t bias_until, retarget_next;
    /* review_5003 (2026-11-10). */
    float hull_rate;                            /* |hull yaw rate|, deg/s, smoothed (preaim) */
    int close, close_slot;                      /* B1 close fight with close_slot */
    uint64_t close_seen_ms, close_log_ms;
    int stop_shots;                             /* A3: shots of the seat when the fire stop began */
    int pivot_tgt;                              /* target when the pivot began (B3 self-check) */
    int eng_own;                                /* eng is the gunner's target or in own sight now */
    uint64_t eng_switch_ms;                     /* B6: last engagement target switch */
    /* Combat manoeuvre commitment survives loss of sight of the same target. */
    uint64_t fight_life;
    uint64_t fight_since, fight_effect_ms, fight_move_until, fight_retry_ms, fight_exhausted_ms, fight_window_ms;
    float fight_x, fight_z, fight_last_x, fight_last_z, fight_best;
    float fight_post_x, fight_post_z, fight_post_ox, fight_post_oz;
    int fight_move, fight_last, fight_yield, fight_post;
    int cmd_release;                            /* S2 withdraws discretionary parking */
    /* S2 commander order (2026-11-11): OB_* type, id (lane / human slot / band),
     * lane of the point (CAPTURE / HOLD), destination; the next order waits
     * until cmd_apply_ms (1-3 s command delay). OB_NONE: none yet this life. */
    int cmd_type, cmd_id, cmd_lane;
    float cmd_x, cmd_z;
    uint64_t cmd_since;
    int cmd_ntype, cmd_nid, cmd_nlane;
    float cmd_nx, cmd_nz;
    uint64_t cmd_apply_ms;
    int cmd_side, cmd_parked;                   /* SUPPORT slot (0-2); parked at the destination */
    float cmd_hx, cmd_hz;                       /* SUPPORT: where the human was when the destination was set */
    uint64_t cmd_dest_ms, cmd_replan_ms;
    int post_option, post_retry, post_wp, post_defending, post_traffic_failures;
    float post_gx, post_gz, post_best;          /* gate bearing and remaining distance on this leg */
    uint64_t post_progress_ms;
    /* v13 choke / overwatch posts (roads_post_check): first tick in position at this spot, last
     * enemy detected from there, the newest hit already judged. */
    uint64_t post_in_place_ms, post_enemy_ms, post_hit_ms;
    /* A mapped connection within the current flank lane step. */
    int flank_lane, flank_step, flank_attempted, flank_replanned;
    uint64_t flank_check_ms;
    float flank_exposure, flank_attempt_x, flank_attempt_z;
    unsigned flank_generation;
    /* 2026-11-11: B7 rear guard, pivot creep, stabiliser log. */
    int rear;                                   /* stopped to turn the hull to a fight behind it */
    uint64_t rear_since, rear_cool_until;
    int pivot_creep;                            /* the pivot did not turn on the spot: some throttle */
    int stab_logged;
    /* 2026-11-11 cover seeking (cover_find): a move into a cover cell (COV_FIRE /
     * COV_CLOSE / COV_RETREAT), state 1 driving to it, 2 in it; the cell is also
     * this seat's claim. Order stop spot (HOLD / SUPPORT / PUSH / PRESS): cov_d*,
     * computed for the destination cov_o*, claimed while cov_dest_seen is recent. */
    int cov_use, cov_state, cov_q, cov_type, cov_shots, cov_close_tried;
    float cov_x, cov_z, cov_dist;
    uint64_t cov_until, cov_since, cov_cool_until, cov_try_ms;
    float cov_hop_best;
    uint64_t cov_hop_progress_ms;
    int cov_fail_count;
    float cov_fail_x[2], cov_fail_z[2], cov_fail_origin_x, cov_fail_origin_z;
    float cov_fail_threat_x, cov_fail_threat_z, cov_fail_route_x, cov_fail_route_z;
    uint64_t cov_fail_ms[2];
    int cov_dest_ok, cov_dest_q, cov_dest_type;
    float cov_dx, cov_dz, cov_ox, cov_oz;
    uint64_t cov_dest_ms, cov_dest_seen;
    /* Stage 2 (2026-11-15), written by nav_report each task-layer tick
     * (cmd_observe_mobility): immobility evidence and the last tick's pose. */
    int immobile, imm_flags;                    /* cannot go anywhere now; times flagged this life */
    float imm_x, imm_z, imm_tick_x, imm_tick_z; /* start of the no-progress stretch; pose at the last tick */
    uint64_t imm_ref_ms, imm_need_ms, imm_tick_ms, imm_since, imm_retry_ms;
    /* Stage 3 (2026-11-15), covered approach (ground_approach.inc): AP_* state on
     * the route of generation ap_generation under order ap_task. The stop is at
     * (ap_x, ap_z) before the exposed stretch ap_run -> ap_end (ap_run_m long,
     * ap_final: it reaches the order's goal, ap_rush: it ends in cover, so it
     * is crossed without fire stops), fire expected from ap_face. */
    int ap_state, ap_assembly, ap_bounds, ap_final, ap_rush, ap_pick_blind;
    unsigned ap_generation;
    uint64_t ap_task, ap_since, ap_until, ap_next_ms, ap_eval_ms, ap_cool_until, ap_replan_ms;
    float ap_x, ap_z, ap_run_x, ap_run_z, ap_end_x, ap_end_z, ap_face_x, ap_face_z, ap_run_m;
    float ap_route_threat;                      /* threat part of the route's cost when it was last judged */
    float ap_best;                              /* way into the cover: least distance left, and when it last shrank */
    float ap_plan_m, ap_odo, ap_px, ap_pz;      /* crossing: metres planned, metres driven, where it was last tick */
    uint64_t ap_progress_ms;
    int home_set;                               /* SPAA life: parks at (home_x, home_z) near its spawn */
    float home_x, home_z;
} lane_seat_t;
/* AP_TO_COVER drives into cover before a stretch under fire, AP_OBSERVE looks
 * over it from there, AP_DASH crosses it without stopping. */
enum { AP_NONE, AP_TO_COVER, AP_OBSERVE, AP_DASH };
enum { M_MOVE, M_ENGAGE, M_PURSUE, M_RETREAT, M_INTERCEPT };
extern lane_seat_t lseat[MAX_SEATS];
extern uint64_t ground_scene_epoch;

/* s48: from ground_navigation.inc (the command unit uses nav_recent too). */
#define NAV_FEEDBACK_MS 500
static inline int nav_recent(uint64_t sampled, uint64_t now)
{
    return sampled && now >= sampled && now - sampled <= NAV_FEEDBACK_MS;
}

static inline int cmd_is_post(const lane_seat_t *s)
{
    return s->cmd_type == OB_OVERWATCH && s->cmd_id >= CMD_POST_ID_BASE;
}
static inline int cmd_post_in_arc(const lane_seat_t *s, float x, float z)
{
    float dx = x - s->x, dz = z - s->z, gx = s->post_gx - s->x, gz = s->post_gz - s->z;
    return dx * gx + dz * gz >= 0.8f * sqrtf((dx * dx + dz * dz) * (gx * gx + gz * gz));
}
/* Design rule: at least 75% of a team must attack / defend the points.
 * Staging holds before the point are short (were 45 / 20 / 45 / 15 / 30 s and
 * 25 s before moving on from a taken point). */
#define OVERWATCH_S1_S 30.0f
#define OVERWATCH_S2_S 10.0f
#define SCREEN_F1_S 20.0f
#define ADVANCE_HOLD_S 12.0f
static inline void lane_route(lane_seat_t *s, const lane_t *l)
{
    /* Nobody sits forever; no forward point past the capture zone (S4 was unreachable
     * on maps without a node graph). */
    route_step_t *r = s->route;
    int n = 0;
    r[n++] = (route_step_t){0, H_NONE, 0};
    if (l->kind == 0) {         /* target lane: S0 S1 S2, the point */
        if (s->role == LROLE_OVERWATCH) {
            r[n++] = (route_step_t){1, H_UNTIL_OWNED, OVERWATCH_S1_S};
            r[n++] = (route_step_t){2, H_FOR, OVERWATCH_S2_S};
        } else {
            r[n++] = (route_step_t){2, H_NONE, 0};
        }
    } else {                    /* flank lane: S0 F1 F2, then the outer point */
        if (s->role == LROLE_SCREEN)
            r[n++] = (route_step_t){1, H_FOR, SCREEN_F1_S};
        else
            r[n++] = (route_step_t){1, H_NONE, 0};
        r[n++] = (route_step_t){2, H_NONE, 0};     /* F2 is behind the outer point: turn in, no wait */
    }
    r[n++] = (route_step_t){SLOT_PT, H_UNTIL_OWNED, 60.0f};
    r[n++] = (route_step_t){SLOT_PT, H_ADVANCE, ADVANCE_HOLD_S};
    s->n = n;
}
static inline void unit_dir(float fx, float fz, float tx, float tz, float *ux, float *uz)
{
    float dx = tx - fx, dz = tz - fz, l = sqrtf(dx * dx + dz * dz);
    *ux = l > 1 ? dx / l : 0.0f;
    *uz = l > 1 ? dz / l : 0.0f;
}

#define FIGHT_IMMEDIATE_M 125.0f
#define FIGHT_CLOSE_MS 8000
#define FIGHT_STALE_MS 16000
#define REAR_HIT_MS 3000
static inline int fight_committed(const lane_seat_t *s, uint64_t ms)
{
    return s->fight_move && ms < s->fight_move_until;
}

static inline int fight_immediate(const lane_seat_t *s, uint64_t ms, float d, int seen_now)
{
    return seen_now && d < FIGHT_IMMEDIATE_M && s->fight_since
           && (ms - s->fight_since < FIGHT_CLOSE_MS
               || (s->last_hit_ms >= s->fight_since && ms - s->last_hit_ms < REAR_HIT_MS));
}

static inline int cmd_post_self_defense(const lane_seat_t *s, uint64_t ms, float d, int seen_now)
{
    return (seen_now && d < CMD_POST_SELF_DEFENSE_M) || (s->last_hit_ms && ms - s->last_hit_ms < REAR_HIT_MS);
}
#define INTERCEPT_STANDOFF_M 120.0f
#define INTERCEPT_MAX_M 1000.0f
enum { C_PUSH, C_AT_POINT, C_DEFEND, C_REINFORCE, C_HOLD, C_INTERCEPT, C_PURSUE, C_RETREAT, C_IDLE, C_N };
#define PASSIVE_DIV 4
static inline int seat_useful_fire(const lane_seat_t *s, uint64_t ms)
{
    return s->mode == M_ENGAGE && s->eng_own && !s->fight_yield && s->fight_effect_ms
           && ms - s->fight_effect_ms < FIGHT_STALE_MS;
}
#define ALERT_R_M 40.0f             /* the alert's area radius (ground_tactics.inc) */

typedef struct {
    float g32_x0, g32_z0, g32_cell;
    uint32_t g32_nx, g32_nz;
    float s1x, s1z, s2x, s2z, axx, axz, latx, latz, L;
    uint32_t nbands;
    float edge[9];
    uint32_t ncap;
    float cap[8][3];
    uint32_t nsrc, vis_w, vis_row;
    float vis_range, vis_eye, vis_tgt;
    float g10_x0, g10_z0, g10_cell;
    uint32_t g10_nx, g10_nz, ndir, nvband;
    float vband[4];
    uint32_t nhull_r;
    float hull_r[8];
    float speed_mps, margin;
} strat_meta_t;
typedef struct { float x, z, y, depth, lat; u8 flags, band, cover[2], veg, solid, pad[2]; } strat_cell_t;
#define STRAT_MAXB 8
enum { SR_FLANK, SR_BREACH, SR_OVERWATCH, SR_ESCORT, SR_N };
/* s47: the strategy layer state (was an unnamed static struct in ground_brain.c); file and map are the
 * brain's file and mapping handles, kept as void * here. */
typedef struct {
    int state;                          /* 0 not tried, 1 on, 2 off */
    void *file, *map;
    const u8 *base;
    const strat_meta_t *m;
    const strat_cell_t *cell;
    const u8 *visf, *viss;
    int n, nx, nz, W, S, nb, nbins;
    int SP, PX, PZ;                     /* window row padded to 4 floats; padded grid of own / en */
    float *wt[2], *wn[2];               /* S rows x SP: w(d) (900 m / SPAA 600 m), blind weight */
    uint64_t *wseen;                    /* per window row: columns within STRAT_SEEN_M */
    __m128 lut[16];                     /* 4 visibility bits -> lane mask */
    u8 *dbin[3];                        /* per army and cell: 64 m depth bin */
    float *prior[3], *capg[8];          /* hidden-enemy weights */
    uint32_t *seen[3];                  /* per army and cell: last second (+1) one of ours could spot it */
    float *own, *en;                    /* padded grid: cell (r, c) at (r + W) * PX + c + W */
    float *hid, *tmp;
    float front[3][STRAT_MAXB];
    int front_ok[3];
    uint64_t next_ms[3], log_ms[3], minute_ms;
    double us_sum, us_max, qpc_us;
    int updates;
    volatile long faults;
    FILE *snap;
    /* S2: optional sections (absent: no G10 overwatch spots / straight-line travel times) */
    const u8 *src, *g10, *g10hull, *g10view;
    const uint16_t *travel;
    int n10x, n10z, cap_src[8];
    float *enc[3];                      /* per army and cell: EN + hidden of that army's last update */
    float *ag, *acost;                  /* A* scratch, reused by the local flank search */
    int apath_cap;
    uint64_t flank_plan_ms;
    int *apar, *apath;
    uint32_t *astamp, agen;
    struct { float f; int i; } *aheap;
    int aheap_cap;
} strat_state_t;
extern strat_state_t strat;

/* S2: the latest strat_update results of each army, for the commander. */
typedef struct {
    int ok, nb;
    float front[STRAT_MAXB], ratio[STRAT_MAXB], fown[STRAT_MAXB], fen[STRAT_MAXB], fhid[STRAT_MAXB];
    float dmed;                         /* median depth of the army's live tanks */
    int ne, no, point_deficit;
    float point_pressure;
    uint64_t ms, point_deficit_since;
    struct { int slot, band, cell, spaa; float x, z, age, depth; } en[64];
    struct { int slot, cell; float x, z, p; } own[64];
    int role[64];                       /* default role per own slot (SR_*) */
} strat_view_t;
extern strat_view_t sview[3];

static inline int strat_cellof(float x, float z)
{
    int c = (int)floorf((x - strat.m->g32_x0) / strat.m->g32_cell), r = (int)floorf((z - strat.m->g32_z0) / strat.m->g32_cell);
    return r < 0 || r >= strat.nz || c < 0 || c >= strat.nx ? -1 : r * strat.nx + c;
}
static inline int strat_sees(const u8 *mat, int i, int j)
{
    int dr = j / strat.nx - i / strat.nx, dc = j % strat.nx - i % strat.nx;
    if (dr < -strat.W || dr > strat.W || dc < -strat.W || dc > strat.W)
        return 0;
    const uint64_t *row = (const uint64_t *)(mat + (size_t)i * strat.m->vis_row);
    return (int)((row[dr + strat.W] >> (dc + strat.W)) & 1);
}
static inline int strat_band(float lat)
{
    const strat_meta_t *m = strat.m;
    for (int b = 0; b < strat.nb - 1; ++b)
        if (lat < m->edge[b + 1])
            return b;
    return strat.nb - 1;
}

/* ---- command layer types and constants used by both units ---- */
#define TP_MAX_ZONES 8
#define TP_MAX_AREAS 64             /* s33: 32 -> 64 */
#define TP_MAX_SLOTS 6
#define TP_MAX_SUPPORT 4            /* s33: support areas per army and zone */
typedef struct {
    char id[8];
    int target, capacity, front[3];         /* s33: capacity 0 = no limit; front corridors per army (log only) */
    float x, z, r;
    int nsup[3], sup[3][TP_MAX_SUPPORT];    /* s33: per army the fire-support areas, preferred first */
} tp_zone_t;
typedef struct { int target; float x, z, face; } tp_slot_t;
typedef struct { int army, nslot; char id[16], cls; tp_slot_t slot[TP_MAX_SLOTS]; } tp_area_t;
/* s35 WTPLAN002 (tp_format=2): lanes. Lines (after tp_spawn / tp_zone):
 *   tp_format=2
 *   tp_axis=army|ux|uz|ox|oz|width_m|sectors
 *   tp_lane=army|ID|from|serves|end|sector|capacity|time_s|slots|bounds|neighbours[|opposes[|value]]
 *       opposes ID;ID;... lanes of the army met head-on in the same passage (s36; optional, resolved at the start)
 *       value   s37: seconds of enemy traffic a position (guard) lane's end overlooks (optional; missing / empty: 0)
 *       serves  zone id or "-" (a guard lane); end c / s / g (circle / support / guard)
 *       slots   target:x:z:face;...     bounds  x:z:face:at;...     neighbours  ID;ID;... (resolved at the start) */
#define TP_MAX_LANES 96
#define TP_LANE_SLOTS 4
#define TP_LANE_BOUNDS 16
#define TP_LANE_NB 4
#define TP_LANE_OP 12
enum { TLE_CIRCLE, TLE_SUPPORT, TLE_GUARD };
typedef struct { float x, z, face, at; } tp_bound_t;
typedef struct {
    int army, from, serves, end, sector, capacity, nslot, nbound, nnb;
    char id[16];
    float time_s, value;                /* s37 value: seconds of enemy traffic the end overlooks (0: none given) */
    tp_slot_t slot[TP_LANE_SLOTS];
    tp_bound_t bound[TP_LANE_BOUNDS];
    char nb_id[TP_LANE_NB][16];
    int nb[TP_LANE_NB];
    int nop;                            /* s36: lanes of the army met head-on in the same passage (resolved at the start) */
    char op_id[TP_LANE_OP][16];
    int op[TP_LANE_OP];
} tp_lane_t;
extern tp_lane_t tp_lanes[TP_MAX_LANES];
extern int ntp_lanes, tp_format;
extern tp_area_t tp_areas[TP_MAX_AREAS];

#define CMD_MS 2000
#define CMD_MAX_OBJ 64          /* 24 before v13: one line's posts (up to ~36) beside the points, bands and flanks */
#define CMD_MAX_SWITCH 2
/* Design rule: when the team has just taken every point, let the commander
 * move many tanks at once so the push toward the enemy spawn is decisive
 * (offline harness: only 6/14 past the point line after 60 s with 2 a tick). */
#define CMD_MAX_SWITCH_ALLMINE 8
#define CMD_ALLMINE_BURST_MS 30000
#define CMD_ENEMY_M 300.0f
#define CMD_ENEMY_AGE_S 30.0f
#define CMD_H 15.0f
#define CMD_ROLE_COST 10.0f
#define CMD_ROLE_FREE 0.7f
#define CMD_CONTACT_M 450.0f
#define CMD_ARRIVE_M 20.0f
#define CMD_ASTAR_MIN_M 200.0f
#define CMD_ASTAR_NODES 12000
#define CMD_ASTAR_HW 1.3f       /* heuristic weight (bounded-suboptimal A*, fewer expansions) */
#define CMD_LAMBDA 8.0f         /* A*: seconds per unit of EN + hidden in a cell */
#define CMD_MU 1.5f             /* A*: seconds per step on a cell without cover */
#define CMD_WP 12
#define CMD_CENSUS_MS 30000
#define CMD_LOGS_PER_MIN 40
#define CMD_FAR_M 700.0f
#define CMD_FAR_GAIN 40.0f      /* a reassignment over CMD_FAR_M needs this much gain */
#define CMD_GONE_GAIN 60.0f     /* gain bonus when the current objective is gone */
#define CMD_MOVE_M 80.0f        /* overwatch / push / press spot moved this far: the order follows */
#define CMD_MANEUVER_MS 8000
#define CMD_SWITCH_GAIN 10.0f
#define CMD_TURN_GAIN 25.0f
#define CMD_ROUTE_RISK 1.0f     /* exposure penalty for cached tactical routes */
/* Staying on the present order is worth CMD_H plus this times the share of it
 * done (1 = there): a reassignment has to beat what the tank is doing now. */
#define CMD_STAY_PROGRESS 20.0f
extern const char *const cmd_name[OB_N];
typedef struct { int type, id, need, band, lane, wing, cap; float x, z, value; int passive; } cmd_obj_t;
#define CMD_POST_MAX 96         /* v13: choke and overwatch posts of both lines follow the gates (real data: up to 46 an army);
                                 * s32: 64 -> 96, the plan's watch slots follow them */
#define CMD_POST_OPTIONS 3
#define CMD_POST_SAMPLES 3
#define CMD_POST_ARRIVE_M 10.0f
#define CMD_POST_RETRY_MS 16000
#define CMD_POST_BLOCK_MS 90000
/* fx, fz (v13): where a choke / overwatch spot aims (a gate post faces its gate, gx[1]). */
typedef struct { float x, z, quality; uint64_t blocked_until; float fx, fz; } cmd_post_pos_t;
/* kind (v13): PK_GATE today's gate post; PK_CHOKE an ambush of rchokes[ref] (pair: the second
 * post of a choke that needs two); PK_OVERWATCH the precomputed position rows[ref]. */
/* PK_WATCH (s32 teamplan): one slot of a plan watch area (ref: the area, line: the slot), offered only while the
 * army's plan has a watch entry for that area (ground_teamplan.inc), only to tanks of the entry's spawn area. */
/* PK_SUPPORT (s33): one slot of a zone's fire-support area (ref: the area, line: the slot, pair: 1 + the plan zone),
 * offered while the plan sends more tanks to that zone than its circle takes; open to any tank. */
/* PK_LANE (s35 WTPLAN002): one end slot of a support or guard lane (ref: the lane, line: the slot). */
enum { PK_GATE, PK_CHOKE, PK_OVERWATCH, PK_WATCH, PK_SUPPORT, PK_LANE, PK_N };
#define PK_PLAN(kind) ((kind) == PK_WATCH || (kind) == PK_SUPPORT || (kind) == PK_LANE)  /* s33 / s35: the plan's own posts */
typedef struct {
    int cap, lane, npos, route;
    float gx[CMD_POST_SAMPLES], gz[CMD_POST_SAMPLES], value;
    cmd_post_pos_t pos[CMD_POST_OPTIONS];
    int kind, ref, line, pair;
} cmd_post_t;
extern cmd_post_t cmd_posts[3][CMD_POST_MAX];
extern int cmd_nposts[3];
/* v13 commander state per army: battle start (the choke boost), active line and its candidate. */
#define ROADS_IDLE_MS 60000             /* a choke post with no enemy detected this long in place is left */
#define ROADS_SEEN_M 450.0f             /* team-known enemy this close to the choke counts as detected */
#define ROADS_HIT_KNOWN_S 10.0f         /* a hitter seen this recently is not "position unknown" */
#define ROADS_NEXT_OW_M 600.0f          /* a hit overwatch moves to another position this close */
enum { RS_CHOKE_POSTS, RS_CHOKE_NOLANE, RS_CHOKE_NOSPOT, RS_SPOT_KEPT, RS_SPOT_STAND, RS_SPOT_SPACING, RS_OW_POSTS,
       RS_OW_NOLANE, RS_OW_STAND, RS_OW_SPACING, RS_FULL, RS_N };
typedef struct {
    uint64_t epoch[3], start_ms[3], cand_ms[3];
    int line[3], posts[3], switched[3];
    int stat[3][2][RS_N];               /* roads_build_posts per army and line: posts built and why data was dropped */
    int band_ow[3][STRAT_MAXB];         /* band overwatch from rows[]: index + 1, 0 = cmd_ow_spot */
} roads_state_t;
extern roads_state_t roads;
typedef struct {
    uint64_t next_ms[3], census_ms[3], log_ms[3], allmine_ms[3];
    int allmine_logged[3], logs[3], dropped[3];
    int sup_on[3];
    uint64_t sup_since[3], sup_life[3];
    int push_on[3][STRAT_MAXB], ow_ok[3][STRAT_MAXB], ow_on[3][STRAT_MAXB];
    float ow_x[3][STRAT_MAXB], ow_z[3][STRAT_MAXB], ow_score[3][STRAT_MAXB], ow_front[3][STRAT_MAXB];
    uint64_t ow_ms[3][STRAT_MAXB];
    double us_sum, us_max, as_sum, as_max;
    int ticks, astars, astar_fail;
    uint32_t rng;
    float contact_s[3], backed_s[3];
    int switches[3], far_switches[3], cross_switches[3], firsts[3];
    volatile long faults;
    int off;
    /* Stage 2. SUPPORT of the human is no longer generated (design decision);
     * its code stays, set support_on to bring it back. */
    int support_on;
    int far_negative[3], beyond_reach[3], cap_open[3], immobile[3], immobile_in_point[3];
    uint64_t pt_ms[3];
    int pt_seen[3][8], pt_owner[3][8], pt_changes[3][8];
    float pt_owned_s[3][8], pt_manned_s[3][8];
} cmd_state_t;
extern cmd_state_t cmd;

/* Gate posts are worth 55 + 7 quality + 4 weight (about 70-95): the same scale. */
#define TP_WATCH_VALUE 80.0f    /* s32: a plan watch slot */
#define TP_SUPPORT_VALUE 85.0f  /* s33: a plan support slot (the zone's force beyond its circle) */
enum { RT_OVERWATCH, RT_CHOKE, RT_STREET };

#define CMD_SOFT_EXIT_MS 6000

typedef struct {
    ground_task_order_t active, pending, rejected;
    ground_task_report_t report;
    uint64_t life_id, scan_life, accepted_at_ms, first_accepted_at_ms;
    uint64_t retry_after_ms, rejected_until_ms, missing_since_ms, pending_missing_ms;
    uint64_t candidate_ms, urgency_since_ms, post_release_since_ms;
    uint32_t last_active_revision;
    int candidate_type, candidate_objective;
    int army, deferred_reason, invalid_reason;
    /* Stage 2: where this life started (the drive a fresh tank has, cmd_spawn_reach)
     * and where the active task was first accepted (its progress, cmd_progress). */
    int spawn_ok;
    float spawn_x, spawn_z, start_x, start_z;
    /* Mobility evidence of this life (cmd_observe_mobility): the newest hit
     * taken, where the hull was when it was seen, whether the hull is still
     * within CMD_MOBILE_M of there (hit_holds), and the stretch's ordered time
     * at that moment (the stall after the hit is what counts). expect_ms: the
     * last tick the hull was ordered to move. damage_flags: immobilised flags. */
    uint64_t hit_ms, hit_need_ms, expect_ms;
    float hit_x, hit_z;
    int hit_holds, damage_flags;
} cmd_task_state_t;
extern cmd_task_state_t cmd_tasks[MAX_SEATS];

typedef struct {
    int started, plan, roster, half_fired;
    int skip_open;                      /* s39: every main zone was ours once: skipped zones are objectives (latched) */
    int area_run[TP_MAX_ZONES];         /* s40: this commander tick's area counts with its picks so far (tp2_quota_have / take) */
    uint32_t rng, seed;
    uint64_t drawn_ms;
    int lane[TP_MAX_ZONES], owner[TP_MAX_ZONES];
    int need_total[TP_MAX_ZONES], need_cap[TP_MAX_ZONES], need_sup[TP_MAX_ZONES];    /* by zone index */
    int watch_first[TP_MAX_AREAS], watch_n[TP_MAX_AREAS], sup_first[TP_MAX_AREAS], sup_n[TP_MAX_AREAS];
    int post_zone[CMD_POST_MAX];        /* support post: the zone it was last offered for (+1) */
    int flank_prev[TP_MAX_ZONES], flank_seat[TP_MAX_ZONES];
    uint64_t flank_life[TP_MAX_ZONES];
    /* this tick's objectives: kind (TPK_*), zone or spawn area (+1) */
    int obj_kind[CMD_MAX_OBJ], obj_ref[CMD_MAX_OBJ];
} tp_army_t;
extern tp_army_t tp_state[3];
extern int tp_seat_target[MAX_SEATS];

/* ---- pure helpers that were further down in ground_brain.c ---- */
/* G10 cell of (x, z) and its centre, -1 outside the layer. */
static inline int cover_cell_of(float x, float z, float *px, float *pz)
{
    const strat_meta_t *m = strat.m;
    float cs = m->g10_cell;
    int c = (int)floorf((x - m->g10_x0) / cs), r = (int)floorf((z - m->g10_z0) / cs);
    if (c < 0 || c >= strat.n10x || r < 0 || r >= strat.n10z)
        return -1;
    if (px) {
        *px = m->g10_x0 + ((float)c + 0.5f) * cs;
        *pz = m->g10_z0 + ((float)r + 0.5f) * cs;
    }
    return r * strat.n10x + c;
}
static inline float position_leg_distance(float x, float z, float sx, float sz, float gx, float gz)
{
    float dx = gx - sx, dz = gz - sz;
    float d2 = dx * dx + dz * dz;
    float f = d2 > 1.0f ? fmaxf(0.0f, fminf(1.0f, ((x - sx) * dx + (z - sz) * dz) / d2)) : 0.0f;
    return dist2d(x, z, sx + f * dx, sz + f * dz);
}
static inline int position_direction(float x, float z, float tx, float tz)
{
    int k = (int)floorf(atan2f(tz - z, tx - x) / 0.39269908f + 0.5f);
    return (k + 16) & 15;
}
/* s48: from ground_vehicle_control.inc (the command unit uses it too). */
static inline float movement_turn_error(const lane_seat_t *s, float x, float z)
{
    const float dx = x - s->x, dz = z - s->z;
    return fabsf(atan2f(dx * s->fz - dz * s->fx, dx * s->fx + dz * s->fz)) * 57.29578f;
}

/* ---- brain functions the command unit calls (ground_brain.c and its .inc files; s49: they were the gc_host_*
 * services of s47 / s48, game-free now, so the command unit calls them directly) ---- */
void event(void *host, const char *fmt, ...);
int zone_owner(void *host, float x, float z);
const gr_lib_t *routes_library(void *host);
const vcls_t *active_cls(int seat, const u8 *unit);
int control_engagement_active(const lane_seat_t *s, uint64_t ms);
int motion_capture_occupied(const lane_seat_t *s, int seat, uint64_t ms);
void lane_begin_step(lane_seat_t *s, int seat, uint64_t ms);
float seat_ready_in(int seat, uint64_t ms);
void pick_troute(void *host, lane_seat_t *s, int seat);
int flank_objectives(void *host, int army, uint64_t ms, cmd_obj_t *ob, int n);
int contact_get(int army, int seat, int t, uint64_t ms, float *x, float *z, float *age_s);
extern contact_t contact[3][64];
void tac_alert_plan(void *host, int army, uint64_t ms, float osx, float osz, float ux, float uz, float front, int np,
                    int live, int passive, int task_max);

/* ---- command functions the adapter calls (defined in ground_command.c) ---- */
const ground_task_order_t *cmd_active_order(int seat);
void cmd_cancel_unit(void *host, lane_seat_t *s, int seat, uint64_t ms, int reason);
int cmd_cap_index(const lane_t *l);
int cmd_combat_busy(const lane_seat_t *s, uint64_t ms);
int cmd_control_task_valid(const lane_seat_t *s, int seat, uint64_t ms);
void cmd_issue(void *host, int army, int seat, const cmd_obj_t *o, float u, int first, uint64_t ms);
int cmd_known_near(const strat_view_t *v, float x, float z, float r, float max_age);
int cmd_lane_point(int lane);
int cmd_life_replaced(const lane_seat_t *s, int seat);
int cmd_live(const lane_seat_t *s, int seat, int army, uint64_t ms);
void cmd_log(void *host, int army, int force, const char *fmt, ...);
ground_engagement_policy_t cmd_motion_policy(const lane_seat_t *s, int seat, uint64_t ms);
void cmd_navigation_report(void *host, lane_seat_t *s, int seat, uint64_t ms, const ground_task_report_t *r);
void cmd_observe_lost_units(void *host, int army, uint64_t ms);
void cmd_observe_mobility(void *host, lane_seat_t *s, int seat, uint64_t ms);
int cmd_on(void);
const ground_task_order_t *cmd_pending_order(int seat, uint64_t ms);
int cmd_post_option(const cmd_post_t *p, uint64_t ms);
int cmd_post_standable(float x, float z, float gx, float gz);
void cmd_request_post_resume(void *host, lane_seat_t *s, int seat, uint64_t ms, int reason);
int cmd_seat_disabled(const lane_seat_t *s);
int cmd_seat_immobile(const lane_seat_t *s);
int cmd_snap(float x, float z, float *ox, float *oz);
void cmd_task_receipt(void *host, lane_seat_t *s, int seat, uint64_t ms,
                             const ground_task_order_t *order, int status, int reason, uint64_t retry_after_ms);
void cmd_tick(void *host, int army, uint64_t ms);
void cmd_unit_observe(void *host, lane_seat_t *s, int seat, uint64_t ms);
int cmd_validate_pending(void *host, const lane_seat_t *s, int seat, uint64_t ms,
                                const ground_task_order_t *order);
int fight_in_point(void *host, const lane_seat_t *s, int *contested);
int motion_combat_override(const lane_seat_t *s, uint64_t ms);
int motion_committed(const lane_seat_t *s, uint64_t ms);
float roads_fit(int army, int seat, int task);
float roads_fitw(int army, int seat, int kind, int ref);
int roads_post_check(void *host, lane_seat_t *s, int seat, cmd_post_t *p, uint64_t ms);
int roads_turretless(int seat);
const vfit_t *roads_vfit(int seat);
int seat_class(void *host, const lane_seat_t *o, uint64_t ms);
float strat_class(int s, float hp_med, int *role);
float strat_depth(int army, float x, float z);
float strat_lat(float x, float z);
int strat_spaa(int s);
void task_point(lane_seat_t *o, float cx, float cz);
void team_plan(void *host, int army, uint64_t ms);
const char *team_rear_class(void *host, int army, float x, float z, float osx, float osz, float ux, float uz,
                                   float front, int np);
int tp2_area_at(float x, float z);
void tp2_area_count(int army, uint64_t ms, int *area_have);
int tp2_area_open(int army, int seat, int a, const int *area_have);
int tp2_lane_of(int army, const lane_seat_t *o);
int tp2_slot_used(int army, int l, int s, uint64_t ms, int skip);
int tp_active(int army);
void tp_parse(const char *k, char *v);
int tp_post_held(int army, int id);
int tp_wait_first_order(const lane_seat_t *s, int seat, uint64_t ms);

#endif
