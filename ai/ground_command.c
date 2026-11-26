/* ground_command.c (s47, s48, s49): the command layer as its own translation unit. No game access: the brain
 * functions it calls are declared in ground_command.h (s49: game-free, called directly; the one game fact it reads
 * is a unit position, gh_unit_pos of ground_host.h). The code is the s46 command code, moved unchanged apart from
 * the adapter context parameter (now void *host) and those calls (tools_split_diff.py shows it). */
#define _CRT_SECURE_NO_WARNINGS
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ground_command.h"

/* Tables shared with the adapter, in their s46 order (they were static in ground_brain.c; the adapter fills
 * most of them from the config and the game). Declared in ground_command.h. s48: TASK_FORGET_MS and
 * learnt_logged came along with the team planner, the only code that uses them (not shared, still static). */
int authored_spawn_seat_army[MAX_SEATS];
troute_t troutes[MAX_TROUTES + MAX_SEATS];
int ntroutes;
tspot_t tspots[MAX_TSPOTS];
int ntspots;
rchoke_t rchokes[MAX_RCHOKES];
int nrchokes;
rchoke_spot_t rchoke_spots[MAX_RCHOKE_SPOTS];
int nrchoke_spots;
rows_t rows[MAX_ROWS];
int nrows;
vfit_t seat_vfit[MAX_SEATS], reserve_vfit[MAX_SEATS];
lane_t lanes[MAX_LANES];
int nlanes;
int seat_lane[MAX_SEATS], seat_has_lane[MAX_SEATS];
float seat_hp[MAX_SEATS]; /* power-to-weight, hp/t (yield: slower gives way) */
vcls_t seat_cls[MAX_SEATS];
vcls_t reserve_cls[MAX_SEATS];
int human_slot = -1;
#define TASK_FORGET_MS 30000
static uint64_t learnt_logged[3][64];
u8 *slot_unit[64];
int slot_alive[64], slot_army[64];
uint64_t slot_life[64];
config_t cfg;
gr_lib_t routes_lib;
int routes_lib_state; /* 0 not tried, 1 loaded, -1 off */
uint64_t hits_taken_ms[64][STRAT_RING], hits_dealt_ms[64][STRAT_RING];
int last_hitter[64];
uint64_t last_hitter_ms[64];
uint64_t last_unit_scan;
lane_seat_t lseat[MAX_SEATS];
uint64_t ground_scene_epoch;
strat_state_t strat;
strat_view_t sview[3];

#include "ground_teamplan_data.inc"
#include "ground_command_team.inc"
#include "ground_command_core.inc"
#include "ground_teamplan.inc"
#include "ground_command_loop.inc"
