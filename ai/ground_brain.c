/* WTGroundAi.dll: positional play for the private Ground RB host's bot tanks.
 *
 * Called once per bot tank per update of the game's own move order, before the
 * native node reads its destination (through the launcher's driver hook):
 *   wt_ground_plan(unit, node, host, dt)
 *
 * Each life gets its own short plan: one or two firing positions (from the
 * player kill heatmaps, else a left/right lane point) with a hold time, then the
 * native capture behaviour. Destinations are set the way the game's own capture
 * order sets them (the bot's destination setter + repath timer), and holding uses the
 * native arrival countdown, so the native driver, pathing and avoidance stay in
 * charge of the actual driving.
 *
 * Config: key=value file named by WT_GROUND_AI_CONFIG (written by ground_brain.py).
 * Game access only through ground_host.h (s49, s50); the game layout is private; see README.md and docs/.
 */
#define _CRT_SECURE_NO_WARNINGS

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <intrin.h>
#include <emmintrin.h>
#include <string.h>
#include <ctype.h>     /* s49: tolower (ground_geometry.inc); the Windows header brought it before */

#include "ground_command.h"

#define MAX_AUTHORED_SPAWN_SITES 512

#define MAX_WP 3

typedef struct { float x, y, z, hold; int kind; } waypoint_t; /* kind: 1 heat spot, 2 lane point */


#include "ground_navigation.h"
#include "ground_motion_control.h"
#include "ground_nav.h"
#include "ground_routes.h"
#include "ground_network.h"
#include "ground_task_order.h"

/* Authored opening spawn sites and roster seats from ground_brain.py (spawn_site / spawn_seat). */
typedef struct { int army; gm_point_t p; } authored_spawn_site_t;
static authored_spawn_site_t authored_spawn_sites[MAX_AUTHORED_SPAWN_SITES];
static int authored_spawn_site_count;
/* Per-seat commander or flank-leg routes sit outside the cached tactical set. */
#define CMD_TR(seat) (MAX_TROUTES + (seat))

enum { RK_JUNCTION, RK_BRIDGE, RK_STREET, RK_GRID };

static void parse_vfit(vfit_t *target, const char *v)
{
    vfit_t f = {0};
    float *d[] = {&f.dep, &f.elev, &f.rev, &f.turret, &f.tdps, &f.hdps, &f.reload, &f.len, &f.wid};
    char *e = (char *)v;
    for (int i = 0; i < 9; ++i) {
        if (i && *e != ',')
            return;
        *d[i] = (float)strtod(i ? e + 1 : e, &e);
        if (!isfinite(*d[i]))
            *d[i] = -1.0f;
    }
    f.ok = 1;
    *target = f;
}

static int seat_lrole[MAX_SEATS];

static char reserve_name[MAX_SEATS][96];

static void parse_cls(vcls_t *target, const char *v)
{
    vcls_t c = {0};
    char *e;
    c.role = (int)strtol(v, &e, 10);
    if (*e != ',')
        return;
    c.flags = (uint32_t)strtoul(e + 1, &e, 0);
    float *f[] = {&c.hp, &c.pen, &c.front, &c.speed, &c.w[0], &c.w[1], &c.w[2]};
    for (int i = 0; i < 7; ++i) {
        if (*e != ',')
            return;
        *f[i] = (float)strtod(e + 1, &e);
    }
    c.stab_h_kmh = c.stab_v_kmh = -1.0f;
    if (*e == ',') {
        c.stab_h_kmh = (float)strtod(e + 1, &e);
        if (*e == ',')
            c.stab_v_kmh = (float)strtod(e + 1, &e);
    }
    if (!(c.flags & VC_STAB_H))
        c.stab_h_kmh = -1.0f;
    if (!(c.flags & VC_STAB_V))
        c.stab_v_kmh = -1.0f;
    if (c.role < 0 || c.role > 3)
        return;
    for (int i = 0; i < 3; ++i)
        c.w[i] *= 0.01f;
    c.ok = 1;
    *target = c;
}

const vcls_t *active_cls(int seat, const u8 *unit)
{
    if (reserve_cls[seat].ok && unit) {
        const u8 *description = gh_unit_description(unit);
        const char *model = description ? (const char *)gh_description_model(description) : NULL;
        if (model) {
            const char *slash = strrchr(model, '/');
            const char *name = slash ? slash + 1 : model;
            size_t length = strlen(reserve_name[seat]);
            if (!strncmp(name, reserve_name[seat], length) && (!name[length] || !strcmp(name + length, ".blk")))
                return &reserve_cls[seat];
        }
    }
    return &seat_cls[seat];
}

/* Turret pre-aim inputs (2026-11-10). Written from the vision hook and the hit
 * hook, read by the lane executor. Plain stores; a torn read only costs one tick. */
typedef struct { float x, z; uint64_t ms; } seen_t;
static seen_t seen_by[64];    /* per bot observer slot: last enemy tank it saw */
static seen_t truth_pos[64];  /* per player slot: true position at the last vision query. LOGS AND BEARINGS ONLY */

/* ---- Team knowledge (2026-11-10, plan L0: no wallhack) --------
 * What a team may know about an enemy tank:
 *  - own eyes: the vision hook answered "visible" for this observer (< 1.5 s);
 *  - team report: a team-mate saw it; known REPORT_MS after the sighting run
 *    started, forgotten FORGET_MS after the last sighting;
 *  - being hit: only the bearing (hit_dir), not the shooter's position;
 *  - deaths are public: a dead or respawned slot's contact is dropped.
 * The planner and the per-tank logic read only this, never truth_pos. */
#define REPORT_MS 2500
#define FORGET_MS 40000
#define SIGHT_GAP_MS 3000
contact_t contact[3][64];    /* [observing army][enemy slot] */
static uint64_t sight_ms[64][64];   /* [observer slot][target slot]: last own line of sight */
typedef struct { float dx, dz; uint64_t ms; int from; } hitdir_t;
static hitdir_t hit_dir[64];        /* per victim slot: unit vector towards the shot's origin */
/* Units by player slot, refreshed once a second from the game's unit list (scan_units). */
static uint64_t next_scan;

typedef struct {
    u8 *unit;
    float sx, sz;              /* where this life started */
    waypoint_t objective;      /* the native tree's destination when the plan was made */
    int planned, n, idx, arrived;
    waypoint_t wp[MAX_WP];
    uint64_t hold_end, best_ms;
    float best;
    int claimed[MAX_WP];       /* spot index per waypoint, -1 for lane points */
    int role, army, team_index;
    float side;
    /* Unreachable/stuck (design rule): back off to the previous point,
     * then try a different point for the same step. */
    int retreating, retries;
    waypoint_t retreat;
} seat_t;

static seat_t seats[MAX_SEATS];
static int claims[3][MAX_SPOTS]; /* per army: how many live plans use a spot */
static int bad[3][MAX_SPOTS];    /* per army: spots a team-mate could not reach */
#define MAX_RETRIES 2
#define MAX_PER_SPOT 2   /* tanks of one team sharing a firing position */
static FILE *log_file;
static uint64_t t0;

static void trim(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == ' '))
        s[--n] = 0;
}


static int init(void)
{
    gh_lock_init(GH_LOCK_MAIN);
    gh_lock_init(GH_LOCK_KNOW);
    cfg.react = 1;
    cfg.team = 1;
    cfg.direct = 1;
    cfg.tree_cover = 0.4f;
    cfg.brain_mask = 0xffffffffu;
    cfg.direct_every = 4;
    cfg.vis = 1;
    cfg.vis_near = 35.0f;
    cfg.vis_max = 1500.0f;
    cfg.los_ms = 42;
    cfg.los_tier = 0;   /* opt-in (WT_GROUND_LOS_TIER=1): its pair hook was never reached in the Eastern Europe test */
    cfg.riex_owner = 1;
    cfg.preaim = 1;
    cfg.stab = 1;
    cfg.gate = 1;
    cfg.face = 1;
    cfg.shortstop = 1;
    cfg.headbias = 0;
    cfg.pursue = 1;
    cfg.close = 1;
    cfg.commander = 1;
    cfg.stab_native = 1;
    cfg.movefire = 1;
    cfg.stab_emu = 1;
    cfg.rear = 1;
    cfg.cover = 1;
    cfg.approach = 1;
    cfg.clutter_k = 4.0f;
    cfg.lidar = 1;
    cfg.lidar_us_per_tick = 1500;
    cfg.lidar_range_m = 12.0f;
    cfg.overhead = 1;
    cfg.overhead_us_per_tick = 1000;
    cfg.overhead_range_m = 30.0f;
    cfg.overhead_cell_m = 0.5f;
    cfg.local = 1;
    cfg.spawn_exit = 1;
    cfg.graft = 1;
    cfg.path = 0;       /* s26: s22c unless asked (WT_GROUND_PATH=1: s24) */
    cfg.routes = 1;
    cfg.near_first = 1;
    cfg.sense = cfg.alert = cfg.takeover = cfg.waitpeek = 1;
    cfg.sense_m = 25.0f;
    cfg.threatface = 1;
    cfg.cover_m = 30.0f;
    cfg.teamplan = 1;
    cfg.intel = 1;
    char path[MAX_PATH];
    unsigned long n = gh_env("WT_GROUND_AI_CONFIG", path, sizeof path);
    FILE *f = n && n < sizeof path ? fopen(path, "r") : NULL;
    static char line[8192];     /* troute lines carry up to MAX_TWP points */
    while (f && fgets(line, sizeof line, f)) {
        trim(line);
        char *eq = strchr(line, '=');
        if (!eq)
            continue;
        *eq = 0;
        const char *k = line, *v = eq + 1;
        if (!strcmp(k, "log_dir")) strncpy(cfg.log_dir, v, sizeof cfg.log_dir - 1);
        else if (!strcmp(k, "brain_mask")) cfg.brain_mask = (uint32_t)strtoul(v, NULL, 0);
        else if (!strcmp(k, "direct_every")) cfg.direct_every = atoi(v);
        else if (!strcmp(k, "plan")) cfg.plan = atoi(v);
        else if (!strcmp(k, "vis")) cfg.vis = atoi(v);
        else if (!strcmp(k, "vis_near")) cfg.vis_near = (float)atof(v);
        else if (!strcmp(k, "vis_max")) cfg.vis_max = (float)atof(v);
        else if (!strcmp(k, "los_ms")) cfg.los_ms = atoi(v);
        else if (!strcmp(k, "los_tier")) cfg.los_tier = atoi(v);
        else if (!strcmp(k, "riex_owner")) cfg.riex_owner = atoi(v);
        else if (!strcmp(k, "lanes")) cfg.lanes = atoi(v);
        else if (!strcmp(k, "native_control")) cfg.native_control = atoi(v);
        else if (!strcmp(k, "preaim")) cfg.preaim = atoi(v);
        else if (!strcmp(k, "react")) cfg.react = atoi(v);
        else if (!strcmp(k, "team")) cfg.team = atoi(v);
        else if (!strcmp(k, "stab")) cfg.stab = atoi(v);
        else if (!strcmp(k, "gate")) cfg.gate = atoi(v);
        else if (!strcmp(k, "face")) cfg.face = atoi(v);
        else if (!strcmp(k, "shortstop")) cfg.shortstop = atoi(v);
        else if (!strcmp(k, "headbias")) cfg.headbias = atoi(v);
        else if (!strcmp(k, "pursue")) cfg.pursue = atoi(v);
        else if (!strcmp(k, "close")) cfg.close = atoi(v);
        else if (!strcmp(k, "probe_seat")) cfg.probe_seat = atoi(v);
        else if (!strcmp(k, "commander")) cfg.commander = atoi(v);
        else if (!strcmp(k, "stab_native")) cfg.stab_native = atoi(v);
        else if (!strcmp(k, "movefire")) cfg.movefire = atoi(v);
        else if (!strcmp(k, "stab_emu")) cfg.stab_emu = atoi(v);
        else if (!strcmp(k, "rear")) cfg.rear = atoi(v);
        else if (!strcmp(k, "cover")) cfg.cover = atoi(v);
        else if (!strcmp(k, "approach")) cfg.approach = atoi(v);
        else if (!strcmp(k, "lidar")) cfg.lidar = atoi(v);
        else if (!strcmp(k, "lidar_us_per_tick")) cfg.lidar_us_per_tick = atoi(v);
        else if (!strcmp(k, "lidar_range_m")) cfg.lidar_range_m = (float)atof(v);
        else if (!strcmp(k, "overhead")) cfg.overhead = atoi(v);
        else if (!strcmp(k, "overhead_us_per_tick")) cfg.overhead_us_per_tick = atoi(v);
        else if (!strcmp(k, "overhead_range_m")) cfg.overhead_range_m = (float)atof(v);
        else if (!strcmp(k, "overhead_cell_m")) cfg.overhead_cell_m = (float)atof(v);
        else if (!strcmp(k, "local")) cfg.local = atoi(v);
        else if (!strcmp(k, "spawn_exit")) cfg.spawn_exit = atoi(v);
        else if (!strcmp(k, "bot_phys_hz")) cfg.bot_phys_hz = atoi(v);
        else if (!strcmp(k, "graft")) cfg.graft = atoi(v);
        else if (!strcmp(k, "path")) cfg.path = atoi(v);
        else if (!strcmp(k, "routes")) cfg.routes = atoi(v);
        else if (!strcmp(k, "routes_file")) strncpy(cfg.routes_file, v, sizeof cfg.routes_file - 1);
        else if (!strcmp(k, "routes_map_file")) strncpy(cfg.routes_map_file, v, sizeof cfg.routes_map_file - 1);
        else if (!strcmp(k, "routes_net_file")) strncpy(cfg.routes_net_file, v, sizeof cfg.routes_net_file - 1);
        else if (!strcmp(k, "near")) cfg.near_first = atoi(v);
        else if (!strcmp(k, "sense")) cfg.sense = atoi(v);
        else if (!strcmp(k, "sense_m")) cfg.sense_m = (float)atof(v);
        else if (!strcmp(k, "alert")) cfg.alert = atoi(v);
        else if (!strcmp(k, "takeover")) cfg.takeover = atoi(v);
        else if (!strcmp(k, "waitpeek")) cfg.waitpeek = atoi(v);
        else if (!strcmp(k, "threatface")) cfg.threatface = atoi(v);
        else if (!strcmp(k, "cover_m")) cfg.cover_m = (float)atof(v);
        else if (!strcmp(k, "teamplan")) cfg.teamplan = atoi(v);
        else if (!strcmp(k, "intel")) cfg.intel = atoi(v);
        else if (!strncmp(k, "tp_", 3)) tp_parse(k + 3, eq + 1);
        else if (!strncmp(k, "cls.", 4)) {
            int seat = atoi(k + 4);
            if (seat > 0 && seat < MAX_SEATS)
                parse_cls(&seat_cls[seat], v);
        } else if (!strncmp(k, "reserve.", 8)) {
            int seat = atoi(k + 8);
            const char *comma = strchr(v, ',');
            if (seat > 0 && seat < MAX_SEATS && comma && (size_t)(comma - v) < sizeof reserve_name[seat]) {
                memcpy(reserve_name[seat], v, (size_t)(comma - v));
                reserve_name[seat][comma - v] = '\0';
                parse_cls(&reserve_cls[seat], comma + 1);
            }
        }
        else if (!strcmp(k, "spawn_seat")) {
            int seat, army;
            if (sscanf(v, "%d,%d", &seat, &army) == 2 && seat > 0 && seat < MAX_SEATS && (army == 1 || army == 2))
                authored_spawn_seat_army[seat] = army;
        } else if (!strcmp(k, "spawn_site") && authored_spawn_site_count < MAX_AUTHORED_SPAWN_SITES) {
            authored_spawn_site_t site;
            if (sscanf(v, "%d,%f,%f", &site.army, &site.p.x, &site.p.z) == 3 && (site.army == 1 || site.army == 2)
                && isfinite(site.p.x) && isfinite(site.p.z))
                authored_spawn_sites[authored_spawn_site_count++] = site;
        }
        else if (!strcmp(k, "geometry_file")) strncpy(cfg.geometry_file, v, sizeof cfg.geometry_file - 1);
        else if (!strcmp(k, "scan_file")) strncpy(cfg.scan_file, v, sizeof cfg.scan_file - 1);
        else if (!strcmp(k, "clutter_file")) strncpy(cfg.clutter_file, v, sizeof cfg.clutter_file - 1);
        else if (!strcmp(k, "clutter_k")) cfg.clutter_k = (float)atof(v);
        else if (!strcmp(k, "scan_box"))
            sscanf(v, "%f,%f,%f,%f", &cfg.scan_box[0], &cfg.scan_box[1], &cfg.scan_box[2], &cfg.scan_box[3]);
        else if (!strcmp(k, "scan_step")) cfg.scan_step = (float)atof(v);
        else if (!strcmp(k, "scan_budget_us")) cfg.scan_budget_us = atoi(v);
        else if (!strcmp(k, "scan_flags")) cfg.scan_flags = (int)strtol(v, NULL, 0);
        else if (!strcmp(k, "direct")) cfg.direct = atoi(v);
        else if (!strcmp(k, "tree_cover")) cfg.tree_cover = (float)atof(v);
        else if (!strcmp(k, "mission")) strncpy(cfg.mission, v, sizeof cfg.mission - 1);
        else if (!strcmp(k, "objective_mode"))
            cfg.objective_mode = !strcmp(v, "capture") ? OBJECTIVE_CAPTURE
                                 : !strcmp(v, "kill") ? OBJECTIVE_KILL : OBJECTIVE_UNKNOWN;
        else if (!strcmp(k, "blocked_file")) strncpy(cfg.blocked_file, v, sizeof cfg.blocked_file - 1);
        else if (!strcmp(k, "strategy_file")) strncpy(cfg.strategy_file, v, sizeof cfg.strategy_file - 1);
        else if (!strcmp(k, "troute") && ntroutes < MAX_TROUTES) {
            /* army,variant,zx,zz,sx,sz,x,z;x,z;... (build_tactical.py via ground_brain.py) */
            troute_t *r = &troutes[ntroutes];
            const char *pts = v;
            for (int commas = 0; *pts && commas < 6; ++pts)
                if (*pts == ',')
                    ++commas;
            if (sscanf(v, "%d,%d,%f,%f,%f,%f", &r->army, &r->variant, &r->zx, &r->zz, &r->sx, &r->sz) == 6) {
                r->n = 0;
                while (*pts && r->n < MAX_TWP && sscanf(pts, "%f,%f", &r->x[r->n], &r->z[r->n]) == 2) {
                    ++r->n;
                    const char *semi = strchr(pts, ';');
                    if (!semi)
                        break;
                    pts = semi + 1;
                }
                if (r->n >= 2)
                    ++ntroutes;
            }
        } else if (!strcmp(k, "tspot") && ntspots < MAX_TSPOTS) {
            tspot_t *sp = &tspots[ntspots];
            if (sscanf(v, "%d,%f,%f,%f,%f,%d,%f", &sp->army, &sp->zx, &sp->zz, &sp->x, &sp->z, &sp->kind, &sp->score) == 7)
                ++ntspots;
        } else if (!strcmp(k, "tchoke") && nrchokes < MAX_RCHOKES) {
            rchoke_t *ch = &rchokes[nrchokes];
            if (sscanf(v, "%d,%d,%f,%f,%f,%f,%f,%d,%d", &ch->army, &ch->id, &ch->zx, &ch->zz, &ch->x, &ch->z, &ch->value,
                       &ch->line, &ch->kind) == 9 && (ch->army == 1 || ch->army == 2) && (ch->line == 1 || ch->line == 2))
                ++nrchokes;
        } else if (!strcmp(k, "tchokespot") && nrchoke_spots < MAX_RCHOKE_SPOTS) {
            rchoke_spot_t *cs = &rchoke_spots[nrchoke_spots];
            if (sscanf(v, "%d,%d,%f,%f,%f,%f,%f", &cs->army, &cs->choke, &cs->x, &cs->z, &cs->fx, &cs->fz, &cs->score) == 7
                && (cs->army == 1 || cs->army == 2))
                ++nrchoke_spots;
        } else if (!strcmp(k, "tow") && nrows < MAX_ROWS) {
            rows_t *w = &rows[nrows];
            if (sscanf(v, "%d,%d,%f,%f,%f,%f,%f,%d,%f,%f", &w->army, &w->id, &w->x, &w->z, &w->fx, &w->fz, &w->score,
                       &w->line, &w->arc, &w->cover) == 10 && (w->army == 1 || w->army == 2) && (w->line == 1 || w->line == 2))
                ++nrows;
        } else if (!strncmp(k, "vfit.", 5)) {
            int seat = atoi(k + 5);
            if (seat > 0 && seat < MAX_SEATS)
                parse_vfit(&seat_vfit[seat], v);
        } else if (!strncmp(k, "vfitr.", 6)) {
            int seat = atoi(k + 6);
            if (seat > 0 && seat < MAX_SEATS)
                parse_vfit(&reserve_vfit[seat], v);
        }
        else if (!strcmp(k, "lane") && nlanes < MAX_LANES) {
            /* army,id,kind,zx,zz,zr,x,z;x,z;... */
            lane_t *l = &lanes[nlanes];
            int id = -1;
            const char *pts = v;
            for (int commas = 0; *pts && commas < 6; ++pts)
                if (*pts == ',')
                    ++commas;
            if (sscanf(v, "%d,%d,%d,%f,%f,%f", &l->army, &id, &l->kind, &l->zx, &l->zz, &l->zr) == 6
                && id == nlanes) {
                l->n = 0;
                while (*pts && l->n < MAX_LANE_PTS) {
                    if (sscanf(pts, "%f,%f", &l->x[l->n], &l->z[l->n]) != 2)
                        break;
                    ++l->n;
                    const char *semi = strchr(pts, ';');
                    if (!semi)
                        break;
                    pts = semi + 1;
                }
                if (l->n > 0)
                    ++nlanes;
            }
        } else if (!strncmp(k, "hp.", 3)) {
            int seat = atoi(k + 3);
            if (seat > 0 && seat < MAX_SEATS)
                seat_hp[seat] = (float)atof(v);
        } else if (!strncmp(k, "seat.", 5)) {
            int seat = atoi(k + 5), lane = -1, role = 0;
            if (seat > 0 && seat < MAX_SEATS && sscanf(v, "%d,%d", &lane, &role) == 2 && lane >= 0) {
                seat_lane[seat] = lane;
                seat_lrole[seat] = role;
                seat_has_lane[seat] = 1;
            }
        }
        else if (!strncmp(k, "role.", 5)) {
            int seat = atoi(k + 5);
            if (seat > 0 && seat < MAX_SEATS)
                cfg.roles[seat] = !strcmp(v, "td") ? ROLE_TD : !strcmp(v, "heavy") ? ROLE_HEAVY
                                : !strcmp(v, "spaa") ? ROLE_SPAA : ROLE_TANK;
        } else if (!strcmp(k, "spot") && cfg.nspots < MAX_SPOTS) {
            spot_t *s = &cfg.spots[cfg.nspots];
            if (sscanf(v, "%f,%f,%f", &s->x, &s->z, &s->score) == 3)
                ++cfg.nspots;
        }
    }
    if (f) {
        fclose(f);
        cfg.loaded = 1;
    }
    {
        /* The launcher's environment reaches the game (the config path arrives
         * that way): a switch that needs no config line. */
        char off[8];
        unsigned long len = gh_env("WT_GROUND_APPROACH", off, sizeof off);
        if (len && len < sizeof off && off[0] == '0')
            cfg.approach = 0;
        len = gh_env("WT_GROUND_LIDAR", off, sizeof off);
        if (len && len < sizeof off && off[0] == '0')
            cfg.lidar = 0;
        len = gh_env("WT_GROUND_OVERHEAD", off, sizeof off);
        if (len && len < sizeof off && off[0] == '0')
            cfg.overhead = 0;
        len = gh_env("WT_GROUND_LOCAL", off, sizeof off);
        if (len && len < sizeof off)
            cfg.local = off[0] != '0';
        len = gh_env("WT_GROUND_SPAWN_EXIT", off, sizeof off);
        if (len && len < sizeof off)
            cfg.spawn_exit = off[0] != '0';
        len = gh_env("WT_GROUND_GRAFT", off, sizeof off);
        if (len && len < sizeof off)
            cfg.graft = off[0] != '0';
        len = gh_env("WT_GROUND_PATH", off, sizeof off);
        if (len && len < sizeof off)
            cfg.path = off[0] != '0';
        len = gh_env("WT_GROUND_ROUTES", off, sizeof off);
        if (len && len < sizeof off)
            cfg.routes = off[0] != '0';
        len = gh_env("WT_GROUND_NEAR", off, sizeof off);
        if (len && len < sizeof off)
            cfg.near_first = off[0] != '0';
        len = gh_env("WT_GROUND_SENSE", off, sizeof off);
        if (len && len < sizeof off)
            cfg.sense = off[0] != '0';
        len = gh_env("WT_GROUND_ALERT", off, sizeof off);
        if (len && len < sizeof off)
            cfg.alert = off[0] != '0';
        len = gh_env("WT_GROUND_TAKEOVER", off, sizeof off);
        if (len && len < sizeof off)
            cfg.takeover = off[0] != '0';
        len = gh_env("WT_GROUND_WAITPEEK", off, sizeof off);
        if (len && len < sizeof off)
            cfg.waitpeek = off[0] != '0';
        len = gh_env("WT_GROUND_THREATFACE", off, sizeof off);
        if (len && len < sizeof off)
            cfg.threatface = off[0] != '0';
        len = gh_env("WT_GROUND_TEAMPLAN", off, sizeof off);
        if (len && len < sizeof off)
            cfg.teamplan = off[0] != '0';
        len = gh_env("WT_GROUND_INTEL", off, sizeof off);
        if (len && len < sizeof off)
            cfg.intel = off[0] != '0';
    }
    if (!(cfg.sense_m > 1.0f && cfg.sense_m < 200.0f))
        cfg.sense_m = 25.0f;
    if (!(cfg.cover_m >= 2.0f && cfg.cover_m <= 150.0f))
        cfg.cover_m = 30.0f;
    if (cfg.objective_mode == OBJECTIVE_KILL)
        cfg.lanes = cfg.plan = 0;
    if (cfg.lanes || cfg.native_control)
        cfg.plan = 0;
    for (int i = 1; i < MAX_SEATS; ++i)
        if (seat_hp[i] <= 0.0f && seat_cls[i].ok)
            seat_hp[i] = seat_cls[i].hp;
    if (cfg.log_dir[0]) {
        char p2[MAX_PATH * 2];
        snprintf(p2, sizeof p2, "%s/ground_plan.log", cfg.log_dir);
        log_file = fopen(p2, "a");
    }
    t0 = gh_ms();
    return 1;
}

void event(void *host, const char *fmt, ...)
{
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    if (log_file) {
        fprintf(log_file, "%.2f %s\n", (double)(gh_ms() - t0) / 1000.0, msg);
        fflush(log_file);
    }
    gh_log(host, msg);
}

/* s26: the route library of this battle's map, read once (on the first use, the game thread). Library mode needs
 * graft, routes and both files; a file that is absent, of another magic, version or layout turns it off, loudly. */
static gn_net_t routes_net;  /* s38: loaded with the library (routes_library), when configured and accepted */
static int routes_net_state; /* 1 loaded */

/* s38: the road network, NULL unless the library and the network file are both loaded */
static const gn_net_t *routes_network(void)
{
    return routes_lib_state > 0 && routes_net_state > 0 ? &routes_net : NULL;
}

const gr_lib_t *routes_library(void *host)
{
    if (routes_lib_state == 0) {
        routes_lib_state = -1;
        if (!cfg.graft || !cfg.routes) {
            event(host, "routes: library mode off (%s): %s", cfg.graft ? "routes=0" : "graft=0",
                  cfg.path ? "s24 path mode" : "s22c");
        } else if (!cfg.routes_file[0] || !cfg.routes_map_file[0]) {
            event(host, "routes: layout %s has no route library configured: library mode off (%s)", cfg.mission,
                  cfg.path ? "s24 path mode" : "s22c");
        } else {
            char msg[400];
            if (gr_load(&routes_lib, cfg.routes_file, cfg.routes_map_file, msg, sizeof msg)) {
                char line[300];
                gr_describe(&routes_lib, line, sizeof line);
                event(host, "routes: library of layout %s: %s (%s, %s)", cfg.mission, line, cfg.routes_file,
                      cfg.routes_map_file);
                routes_lib_state = 1;
                /* s38: the road network next to the library; one line either way */
                if (!cfg.routes_net_file[0]) {
                    event(host, "routes: layout %s has no road network file: NET source off (s37 behaviour)", cfg.mission);
                } else if (gn_load(&routes_net, cfg.routes_net_file, &routes_lib, msg, sizeof msg)) {
                    gn_describe(&routes_net, line, sizeof line);
                    event(host, "routes: road network of layout %s: %s (%s)", cfg.mission, line, cfg.routes_net_file);
                    routes_net_state = 1;
                } else {
                    event(host, "routes: NETWORK REFUSED for layout %s: %s; NET source off (s37 behaviour)", cfg.mission,
                          msg);
                }
            } else {
                event(host, "routes: LIBRARY REFUSED for layout %s: %s; library mode off (%s)", cfg.mission, msg,
                      cfg.path ? "s24 path mode" : "s22c");
            }
        }
    }
    return routes_lib_state > 0 ? &routes_lib : NULL;
}

static int bot_seat(u8 *unit)
{
    u8 *player = unit ? gh_unit_player(unit) : NULL;
    if (!player || !gh_player_bot(player))
        return -1;
    int seat = gh_player_slot(player);
    return seat >= 1 && seat < MAX_SEATS ? seat : -1;
}

/* Terrain height the way the game's own capture order computes it (gh_height_query). */
static float ground_height(void *host, float x, float y, float z)
{
    u8 *game = host ? gh_game(host) : NULL;
    u8 *land = game ? gh_game_land(game) : NULL;
    if (!land)
        return y;
    float h = gh_height_query(host, land, x, y, z);
    return isfinite(h) ? h + gh_height_pad(host) : y;
}


static void release_claims(seat_t *s, int army)
{
    for (int i = 0; i < s->n; ++i)
        if (s->claimed[i] >= 0 && army >= 0 && army < 3 && claims[army][s->claimed[i]] > 0)
            --claims[army][s->claimed[i]];
}

/* Pick an unclaimed heat spot that overlooks the objective O from this team's
 * side: 120-450 m from O, nearer the start than O is, not behind the start.
 * Score favours kills, penalises spots already given to team-mates. */
static int pick_spot(int army, float sx, float sz, float ox, float oz, float min_d, float max_d, float side)
{
    float ax = sx - ox, az = sz - oz, len = sqrtf(ax * ax + az * az);
    if (len < 1)
        return -1;
    ax /= len;
    az /= len;
    int best = -1;
    float best_score = -1e9f;
    for (int i = 0; i < cfg.nspots; ++i) {
        const spot_t *p = &cfg.spots[i];
        float dx = p->x - ox, dz = p->z - oz, d = sqrtf(dx * dx + dz * dz);
        if (d < min_d || d > max_d || bad[army][i] || claims[army][i] >= MAX_PER_SPOT)
            continue;
        float along = (dx * ax + dz * az) / d;     /* cos to the start direction */
        if (along < 0.2f || d > len * 0.9f)
            continue;
        float lateral = dx * az - dz * ax;          /* + = one side, - = the other */
        float score = p->score - 0.6f * (float)claims[army][i];
        if (side != 0 && lateral * side < 0)
            score -= 0.4f;
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

/* A point `frac` of the way from the start S to the objective O, moved `lateral`
 * metres sideways (capped at 35% of the S-O distance). */
static void lane_point(float sx, float sz, float ox, float oz, float frac, float lateral, float *x, float *z)
{
    float ax = ox - sx, az = oz - sz, len = sqrtf(ax * ax + az * az);
    if (len < 1) {
        *x = ox;
        *z = oz;
        return;
    }
    float cap = 0.35f * len;
    lateral = lateral > cap ? cap : lateral < -cap ? -cap : lateral;
    *x = sx + ax * frac + (-az / len) * lateral;
    *z = sz + az * frac + (ax / len) * lateral;
}

/* Fill step `i` of the plan. `attempt` > 0 after a failed try: the lane swings
 * wider and alternates side, and a heat spot already marked bad is skipped. */
static void make_step(void *host, seat_t *s, int i, int attempt)
{
    const waypoint_t *o = &s->objective;
    waypoint_t *w = &s->wp[i];
    float side = (attempt % 2) ? -s->side : s->side;
    if (w->kind == 2 && w->hold == 0) {  /* departure lane: split the team half way */
        lane_point(s->sx, s->sz, o->x, o->z, 0.45f,
                   side * (120.0f + 60.0f * (s->team_index % 3) + 60.0f * attempt), &w->x, &w->z);
    } else {                            /* firing position */
        float min_d = s->role == ROLE_TD ? 220.0f : 120.0f, max_d = s->role == ROLE_TD ? 600.0f : 420.0f;
        int pick = pick_spot(s->army, s->sx, s->sz, o->x, o->z, min_d, max_d, side);
        if (s->claimed[i] >= 0 && s->army >= 0 && s->army < 3 && claims[s->army][s->claimed[i]] > 0)
            --claims[s->army][s->claimed[i]];
        s->claimed[i] = pick;
        if (pick >= 0) {
            w->x = cfg.spots[pick].x;
            w->z = cfg.spots[pick].z;
            w->kind = 1;
            ++claims[s->army][pick];
        } else {
            lane_point(s->sx, s->sz, o->x, o->z, s->role == ROLE_TD ? 0.72f : 0.82f,
                       side * (80.0f + 40.0f * (s->team_index % 3) + 60.0f * attempt), &w->x, &w->z);
            w->kind = 3;
        }
    }
    w->y = ground_height(host, w->x, o->y, w->z);
    event(host, "seat %d army %d role %d step %d try %d: %s (%.0f,%.1f,%.0f) hold %.0fs; objective (%.0f,%.0f)",
          (int)(s - seats), s->army, s->role, i, attempt,
          w->kind == 1 ? "heat spot" : w->kind == 2 ? "lane" : "fallback lane", w->x, w->y, w->z, w->hold,
          o->x, o->z);
}

static void build_plan(void *host, seat_t *s, int seat, int army, const float *dest)
{
    s->objective.x = dest[0];
    s->objective.y = dest[1];
    s->objective.z = dest[2];
    s->role = active_cls(seat, s->unit) == &reserve_cls[seat] ? ROLE_SPAA : cfg.roles[seat];
    s->army = army >= 0 && army < 3 ? army : 0;
    s->team_index = (seat - 1) % 15;                  /* 15 bot seats per team */
    s->side = (s->team_index % 2) ? 1.0f : -1.0f;     /* alternate left/right */
    s->n = 0;
    s->idx = 0;
    s->arrived = 0;
    s->planned = 1;
    for (int i = 0; i < MAX_WP; ++i)
        s->claimed[i] = -1;
    if (s->role == ROLE_SPAA || (cfg.direct_every > 0 && s->team_index % cfg.direct_every == 0)) {
        event(host, "seat %d army %d role %d: straight to the objective", seat, army, s->role);
        return;
    }
    if (s->role == ROLE_TANK) {  /* faster tanks leave on a side road first */
        s->wp[s->n].kind = 2;
        s->wp[s->n].hold = 0;
        make_step(host, s, s->n++, 0);
    }
    s->wp[s->n].kind = 1;
    s->wp[s->n].hold = s->role == ROLE_TD ? 80.0f : s->role == ROLE_HEAVY ? 45.0f : 35.0f;
    make_step(host, s, s->n++, 0);
}

typedef struct {
    const waypoint_t *target;
    float hold, repath;
    int write_hold, write_repath, clear_path, write_zone, zone;
} nav_ai_output_t;

/* Sole custom writer of native tank navigation fields. */
static void nav_write_ai(u8 *ai, const nav_ai_output_t *output)
{
    if (output->target) {
        const waypoint_t *w = output->target;
        gh_ai_set_dest(ai, w->x, w->y, w->z);
    }
    if (output->write_hold)
        gh_ai_set_hold(ai, output->hold);
    if (output->write_repath || output->target)
        gh_ai_set_repath(ai, output->repath);
    if (output->clear_path)
        gh_ai_clear_path(ai);
    if (output->write_zone)
        gh_ai_set_zone(ai, output->zone);
}

/* The legacy fixed plan is mutually exclusive with lane navigation. */
static void set_dest(u8 *ai, const waypoint_t *w)
{
    const nav_ai_output_t output = {.target = w};
    nav_write_ai(ai, &output);
}

static void finish_plan(void *host, seat_t *s, u8 *ai, const char *why)
{
    s->idx = s->n;
    s->retreating = 0;
    const nav_ai_output_t output = {.target = &s->objective, .write_hold = 1};
    nav_write_ai(ai, &output);
    event(host, "seat %d plan done (%s), native objective next", (int)(s - seats), why);
}

/* Go back to the previous point (or this life's start) and retry the step there. */
static void start_retreat(void *host, seat_t *s, u8 *ai, float d)
{
    int i = s->idx;
    if (s->claimed[i] >= 0)
        bad[s->army][s->claimed[i]] = 1;
    if (s->retries >= MAX_RETRIES) {
        event(host, "seat %d step %d unreachable %d times (%.0f m away)", (int)(s - seats), i, s->retries + 1, d);
        finish_plan(host, s, ai, "unreachable");
        return;
    }
    if (i > 0) {
        s->retreat = s->wp[i - 1];
    } else {
        s->retreat.x = s->sx;
        s->retreat.z = s->sz;
        s->retreat.y = ground_height(host, s->sx, s->objective.y, s->sz);
    }
    s->retreat.hold = 0;
    s->retreating = 1;
    s->best_ms = 0;
    set_dest(ai, &s->retreat);
    event(host, "seat %d no progress on step %d for 45 s (%.0f m away): back to %s (%.0f,%.0f)",
          (int)(s - seats), i, d, i > 0 ? "the previous point" : "the start", s->retreat.x, s->retreat.z);
}

/* ---- Probes for the planner's inputs (verification, 2026-11-09) ----------
 * All units of the game's list; a unit counts as active when the host says
 * so (gh_unit_not_active, the same test the game's own move order uses).
 * Capture zones: the game's zone list with owner army and flag word; the
 * centre of a zone comes from its shape (gh_shape_centre / gh_shape_extent,
 * the way the game's capture order reads it). */
static FILE *units_csv, *zones_csv, *hits_csv;
static uint64_t next_probe;

static FILE *open_csv(const char *name, const char *header)
{
    if (!cfg.log_dir[0])
        return NULL;
    char p[MAX_PATH * 2];
    snprintf(p, sizeof p, "%s/%s", cfg.log_dir, name);
    FILE *f = fopen(p, "a");
    if (f && ftell(f) == 0)
        fprintf(f, "%s\n", header);
    return f;
}

static void vis_log(double t);

static void probe(void *host, uint64_t ms)
{
    if (ms < next_probe)
        return;
    next_probe = ms + 5000;
    double t = (double)(ms - t0) / 1000.0;
    vis_log(t);
    u8 *game = gh_game(host);
    if (!game)
        return;
    if (!units_csv)
        units_csv = open_csv("units.csv", "t,id,type,army,slot,is_bot,active,x,y,z,name,target,last_hit,fire");
    if (!zones_csv)
        zones_csv = open_csv("zones.csv", "t,i,owner,flags,shape,x,y,z,size_x,size_z");
    u8 **units = gh_game_units(game);
    uint32_t n = gh_game_unit_count(game);
    for (uint32_t i = 0; units && units_csv && i < n && i < 8192; ++i) {
        u8 *u = gh_unit_at(units, i);
        if (!u)
            continue;
        u8 *player = gh_unit_player(u);
        if (!player)
            continue; /* mission objects and scenery: skip */
        int active = gh_unit_not_active(u) == 0;
        const float *p = gh_unit_pos(u);
        const char *name = gh_unit_name(u) ? (const char *)gh_unit_name(u) : "?";
        /* The tank's behaviour-tree bot: its current enemy target, the last time
         * it was hit (the game's own under-attack test: < 20 s ago); on fire. */
        u8 *bot = gh_unit_is_basic_tank(u) ? gh_unit_ai(u) : NULL;
        fprintf(units_csv, "%.1f,%u,%u,%u,%u,%d,%d,%.1f,%.1f,%.1f,%s,%d,%.1f,%u\n", t, i, gh_unit_kind(u),
                gh_unit_army(u), gh_player_slot(player), gh_player_bot(player) != 0, active,
                p[0], p[1], p[2], name, bot ? gh_bot_target(bot) != NULL : -1,
                bot ? gh_bot_hit_time(bot) : -1.0f, gh_unit_burning(u));
    }
    if (units_csv)
        fflush(units_csv);
    /* One-off: find the player record field holding the (negative) AI uid seen in hits.csv. */
    static int uid_scan_done;
    FILE *scan = (uid_scan_done || n < 20) ? NULL : open_csv("player_uid_scan.csv", "slot,is_bot,offset,value");
    for (uint32_t i = 0; scan && units && i < n && i < 8192; ++i) {
        u8 *u = gh_unit_at(units, i);
        u8 *player = u ? gh_unit_player(u) : NULL;
        if (!player)
            continue;
        gh_player_uid_scan(scan, player);
    }
    if (scan) {
        fclose(scan);
        uid_scan_done = 1;
    }
    u8 *zones_owner = gh_game_zones(game);
    if (zones_owner && zones_csv) {
        u8 **zones = gh_zone_list(zones_owner);
        uint32_t zn = gh_zone_count(zones_owner);
        for (uint32_t i = 0; zones && i < zn && i < 64; ++i) {
            u8 *z = gh_zone_at(zones, i);
            if (!z)
                continue;
            u8 *shape = gh_zone_shape(z);
            if (!shape)
                continue;
            uint32_t type = gh_shape_type(shape);
            const float *c = gh_shape_centre(shape, type);
            const float *e = gh_shape_extent(shape);
            float h = gh_zone_shape_is_area(type) ? 0.5f : 0.0f;
            fprintf(zones_csv, "%.1f,%u,%u,0x%x,%u,%.1f,%.1f,%.1f,%.1f,%.1f\n", t, i, gh_zone_owner(z),
                    gh_zone_flags(z), type, c[0] + h * e[0], c[1] + h * e[1], c[2] + h * e[2],
                    e[0], e[2]);
        }
        fflush(zones_csv);
    }
}

/* Strategy layer inputs (S1, 2026-11-10): recent hits taken / dealt and kills
 * per player slot, newest first. Written under `lock` (hit hook, scan_units). */
static uint64_t kills_ms[64][STRAT_RING];

static void ring_push(uint64_t *r, uint64_t ms)
{
    memmove(r + 1, r, (STRAT_RING - 1) * sizeof *r);
    r[0] = ms;
}


/* The game's hit flush: victim unit, offender player uid, hit record
 * (offender army, damage). Called for every effective hit.
 * review_5003: the record is a per offender/victim damage accumulator flushed
 * by the game (damage sum, hit count, critical parts, a kind (one kind is
 * always flushed), flush time); no shell or weapon id
 * in it (static read). So hits.csv gets those raw fields plus the offender's
 * last main-gun shot (age, shell id, from shots_poll): a row with
 * main_shot_age_s under flight time + 1.5 s is a main-gun hit, the rest are
 * machine guns / secondary weapons (5003: 26 of 89 bot hit rows, 23 of them
 * zero damage). */
static void hit_shot_info(int from, uint64_t now, float *age, int32_t *shell);
static void shot_effect(int from, u8 *victim, unsigned target, uint64_t now);

void ground_hit(u8 *victim, uint64_t offender_uid, u8 *record, void *host)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || !victim || !record)
        return;
    gh_lock(GH_LOCK_MAIN);
    if (!hits_csv)
        hits_csv = open_csv("hits.csv", "t,victim_slot,victim_bot,victim_army,victim,offender_uid,offender_slot,offender_army,damage,"
                                        "rec_kind,rec_hits,rec_crit,main_shot_age_s,main_shell");
    u8 *player = gh_unit_player(victim);
    const int32_t uid = (int32_t)offender_uid;
    const int from = gh_uid_ai_slot_valid(uid) ? gh_ai_slot_of_uid(uid) : gh_uid_is_human(uid) ? human_slot : -1;
    const int enemy = gh_hit_army(record) != gh_unit_army(victim);
    const float damage = gh_hit_damage(record);
    const int crit = gh_hit_parts(record);
    const uint64_t now = gh_ms();
    if (player && gh_uid_is_ai(uid) && enemy && isfinite(damage) && damage > 0.0f)
        shot_effect(from, victim, gh_player_slot(player) & 63, now);
    if (hits_csv) {
        const char *name = gh_unit_name(victim) ? (const char *)gh_unit_name(victim) : "?";
        /* The player record holds the uid; an AI seat's uid encodes its slot (gh_ai_slot_of_uid, verified 2026-11-09). */
        /* The crew knows where the round came from (bearing), not how far. */
        if (player && from >= 0 && from < 64 && from != (gh_player_slot(player) & 63)
            && gh_ms() - truth_pos[from].ms < 5000) {
            const float *vp = gh_unit_pos(victim);
            float dx = truth_pos[from].x - vp[0], dz = truth_pos[from].z - vp[2], l = sqrtf(dx * dx + dz * dz);
            if (l > 1.0f)
                hit_dir[gh_player_slot(player) & 63] = (hitdir_t){dx / l, dz / l, gh_ms(), from};
        }
        if (player) {
            unsigned vs = gh_player_slot(player) & 63;
            ring_push(hits_taken_ms[vs], now);
            if (from >= 0 && from < 64 && (unsigned)from != vs) {
                if (gh_hit_army(record) != gh_unit_army(victim))
                    ring_push(hits_dealt_ms[from], now);
                last_hitter[vs] = from;
                last_hitter_ms[vs] = now;
            }
        }
        float shot_age = -1.0f;
        int32_t shell = -1;
        if (gh_uid_is_ai(uid))
            hit_shot_info(from, now, &shot_age, &shell);
        fprintf(hits_csv, "%.2f,%d,%d,%u,%s,%d,%d,%d,%g,%d,%d,%d,%.2f,%d\n", (double)(gh_ms() - t0) / 1000.0,
                player ? gh_player_slot(player) : -1, player ? gh_player_bot(player) != 0 : -1,
                gh_unit_army(victim), name, uid, gh_uid_is_ai(uid) ? from : -1,
                gh_hit_army(record), damage, gh_hit_kind(record), gh_hit_count(record),
                crit, shot_age, shell);
        fflush(hits_csv);
    }
    (void)host;
    gh_unlock(GH_LOCK_MAIN);
}

/* ---- Bot tank vision (2026-11-10) ----------------------------------------
 * The game's visibility test (detector of T, observer V, flag) decides whether V sees T for
 * the gunner search, the target evaluator and the
 * behaviour nodes. It always computes the engine's pairwise line-of-sight
 * tables (terrain and clouds; gh_sight_lookup, 0 = clear,
 * refreshed every 2 s per pair), but for tank vs tank it then
 * uses the engine's anti-wallhack multipoint record, which for bot
 * observers is only ever set inside the crew hearing radius
 * (the game's tank-crew hearing distance, 40 m). Measured result: bots hit each
 * other at a median 12-14 m on open desert.
 * For a bot observer tank and an enemy tank target this returns:
 * within vis_near m -> seen (hearing, through walls; about 35 m),
 * beyond vis_max -> not seen, else the terrain line-of-sight table.
 * Everything else (human observers, non-tanks, same army) stays native.
 * Return: 0 = use native, 1 = not visible, 2 = visible. */
static volatile long vis_seen_near, vis_seen_los, vis_blocked, vis_far, vis_native;
static volatile long vis_near_blocked;  /* s28 sense=1: inside vis_near, the sight table said blocked */
static FILE *vis_csv;

static int is_tank(u8 *u)
{
    return gh_unit_is_tank(u);
}

/* Vision hook: observer slot o (army) has line of sight to enemy slot t at (x, z). */
static void note_sight(unsigned o, unsigned t, int army, float x, float z, uint64_t ms)
{
    if (army < 1 || army > 2)
        return;
    gh_lock(GH_LOCK_KNOW);
    sight_ms[o & 63][t & 63] = ms;
    contact_t *c = &contact[army][t & 63];
    if (!c->last_ms || ms - c->last_ms > SIGHT_GAP_MS) {
        c->first_ms = ms;
        c->vx = c->vz = 0.0f;
        c->x = x;
        c->z = z;
        c->last_ms = ms;
        c->by = (int)o;
    } else if (ms - c->last_ms >= 200) {
        float dt = (float)(ms - c->last_ms) / 1000.0f;
        c->vx = 0.5f * c->vx + 0.5f * (x - c->x) / dt;
        c->vz = 0.5f * c->vz + 0.5f * (z - c->z) / dt;
        c->x = x;
        c->z = z;
        c->last_ms = ms;
        c->by = (int)o;
    }
    gh_unlock(GH_LOCK_KNOW);
}

/* What seat `seat` of `army` knows about enemy slot t: 2 own eyes (< 1.5 s),
 * 1 team report / memory, 0 nothing. *age_s: seconds since the last sighting. */
int contact_get(int army, int seat, int t, uint64_t ms, float *x, float *z, float *age_s)
{
    if (army < 1 || army > 2 || t < 0 || t > 63 || !slot_alive[t] || slot_army[t] == army)
        return 0;
    gh_lock(GH_LOCK_KNOW);
    contact_t c = contact[army][t];
    uint64_t own = sight_ms[seat & 63][t];
    gh_unlock(GH_LOCK_KNOW);
    if (!c.last_ms || ms - c.last_ms > FORGET_MS)
        return 0;
    int level = own && ms - own < 1500 ? 2 : (ms >= c.first_ms + REPORT_MS || (own && ms - own < FORGET_MS)) ? 1 : 0;
    if (level) {
        *x = c.x;
        *z = c.z;
        *age_s = (float)(ms - c.last_ms) / 1000.0f;
    }
    return level;
}

/* Once a second: which unit drives each player slot, alive or not. A slot that
 * died or respawned loses its contacts (kills are public). */

static void scan_units(void *host, uint64_t ms)
{
    if (ms < next_scan)
        return;
    next_scan = ms + 1000;
    u8 *game = gh_game(host);
    if (!game)
        return;
    last_unit_scan = ms;
    u8 *now_unit[64] = {0};
    int now_alive[64] = {0}, now_army[64] = {0};
    u8 **units = gh_game_units(game);
    uint32_t n = gh_game_unit_count(game);
    for (uint32_t i = 0; units && i < n && i < 8192; ++i) {
        u8 *u = gh_unit_at(units, i);
        u8 *player = u ? gh_unit_player(u) : NULL;
        if (!player || !is_tank(u))
            continue;
        unsigned slot = gh_player_slot(player) & 63;
        int alive = gh_unit_not_active(u) == 0;
        if (alive || !now_alive[slot]) {
            now_unit[slot] = u;
            now_alive[slot] = alive;
            now_army[slot] = gh_unit_army(u);
        }
    }
    gh_lock(GH_LOCK_KNOW);
    gh_rw_write_lock(GH_RW_AIM);
    for (int s = 0; s < 64; ++s) {
        if (slot_alive[s] != now_alive[s] || slot_unit[s] != now_unit[s])
            ++slot_life[s];
        if (slot_alive[s] && (!now_alive[s] || now_unit[s] != slot_unit[s])) {
            /* a death: credit the last hitter (S1 momentum; read under `lock`) */
            if (last_hitter_ms[s] && ms - last_hitter_ms[s] < 15000 && last_hitter[s] >= 0 && last_hitter[s] < 64)
                ring_push(kills_ms[last_hitter[s]], ms);
            last_hitter_ms[s] = 0;
            memset(&contact[1][s], 0, sizeof contact[1][s]);
            memset(&contact[2][s], 0, sizeof contact[2][s]);
            for (int o = 0; o < 64; ++o)
                sight_ms[o][s] = 0;
        }
        slot_unit[s] = now_unit[s];
        slot_alive[s] = now_alive[s];
        slot_army[s] = now_army[s];
    }
    gh_rw_write_unlock(GH_RW_AIM);
    gh_unlock(GH_LOCK_KNOW);
}

/* The gunner's current target, read the way the game's own turret order does
 * (gh_gunner_target: the unit's weapons and their weak target pointer, checked
 * against the game's generation counter). A stale pointer counts as no target.
 * Only accepted when it is a live enemy tank in
 * slot_unit, so the result can be dereferenced. Returns the slot or -1. */
static int gunner_target(void *host, u8 *unit, int army)
{
    uint64_t p = gh_gunner_target(host, unit);
    if (!p)
        return -1;
    for (int s = 0; s < 64; ++s)
        if (slot_alive[s] && (uint64_t)slot_unit[s] == p && slot_army[s] != army)
            return s;
    return -1;
}

/* Diagnosis (2026-11-10): one line per bot observer per second when the
 * target is a human tank: did the bot see the player, and how far. */
static FILE *human_vis_csv;
static uint64_t human_vis_next[64];

static int human_vis_init(void)
{
    gh_lock_init(GH_LOCK_HUMAN_VIS);
    return 1;
}

static void human_vis(u8 *observer, u8 *target, float dist, const char *verdict)
{
    u8 *tp = gh_unit_player(target), *op = gh_unit_player(observer);
    if (!tp || gh_player_bot(tp) || !op)
        return; /* only human targets */
    unsigned seat = gh_player_slot(op) & 63;
    uint64_t ms = gh_ms();
    if (ms < human_vis_next[seat])
        return;
    human_vis_next[seat] = ms + 1000;
    gh_once(GH_ONCE_HUMAN_VIS, human_vis_init);
    gh_lock(GH_LOCK_HUMAN_VIS);
    if (!human_vis_csv)
        human_vis_csv = open_csv("human_vis.csv", "t,observer_slot,dist,verdict");
    if (human_vis_csv) {
        fprintf(human_vis_csv, "%.1f,%u,%.0f,%s\n", (double)(ms - t0) / 1000.0, seat, dist, verdict);
        fflush(human_vis_csv);
    }
    gh_unlock(GH_LOCK_HUMAN_VIS);
}

int ground_visible(u8 *detector, u8 *observer, uint64_t flag, void *host)
{
    (void)flag;
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.vis || !detector || !observer || !host)
        return 0;
    u8 *target = gh_detector_target(detector);
    if (!target || !is_tank(target) || !is_tank(observer))
        goto native;
    u8 *player = gh_unit_player(observer);
    if (!player || !gh_player_bot(player))
        goto native; /* human observer: keep the native anti-wallhack rules */
    if (gh_unit_army(target) == gh_unit_army(observer))
        goto native;
    /* Only units the host reports present (the game's own visibility test checks the
     * same; the probe's active test is stricter); anything else (dead, removed) stays native. */
    if (gh_unit_not_present(target) != 0 || gh_unit_not_present(observer) != 0)
        goto native;
    const float *a = gh_unit_pos(target), *b = gh_unit_pos(observer);
    float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    float d2 = dx * dx + dy * dy + dz * dz;
    uint64_t now_ms = gh_ms();
    u8 *tplayer = gh_unit_player(target);
    unsigned oslot = gh_player_slot(player) & 63;
    unsigned tslot = 0;
    if (tplayer) {
        tslot = gh_player_slot(tplayer) & 63;
        truth_pos[tslot] = (seen_t){a[0], a[2], now_ms};
        if (!gh_player_bot(tplayer))
            human_slot = (int)tslot;
    }
    /* s28 sense=1: inside vis_near the answer comes from the same sight table as beyond it (presence within
     * sense_m is a separate hint, ground_tactics.inc, never a sighting). sense=0: the s27 hearing branch. */
    const int in_near = d2 <= cfg.vis_near * cfg.vis_near;
    if (in_near && !cfg.sense) {
        gh_atomic_inc(&vis_seen_near);
        human_vis(observer, target, sqrtf(d2), "near");
        seen_by[oslot] = (seen_t){a[0], a[2], now_ms};
        if (tplayer)
            note_sight(oslot, tslot, gh_unit_army(observer), a[0], a[2], now_ms);
        return 2;
    }
    if (!in_near && d2 > cfg.vis_max * cfg.vis_max) {
        gh_atomic_inc(&vis_far);
        human_vis(observer, target, sqrtf(d2), "far");
        return 1;
    }
    u8 *game = gh_game(host);
    void **tables = game ? gh_game_sight_tables(game) : NULL;
    if (!tables || !gh_sight_table(tables))
        goto native;
    unsigned los = gh_sight_lookup(host, tables, target, observer);
    human_vis(observer, target, sqrtf(d2), in_near ? (los == 0 ? "near_los" : "near_blocked") : los == 0 ? "los" : "blocked");
    if (los == 0) {
        /* seen_near (sense=1): seen inside vis_near, by the table; seen_los: seen beyond it */
        gh_atomic_inc(in_near ? &vis_seen_near : &vis_seen_los);
        seen_by[oslot] = (seen_t){a[0], a[2], now_ms};
        if (tplayer)
            note_sight(oslot, tslot, gh_unit_army(observer), a[0], a[2], now_ms);
        return 2;
    }
    gh_atomic_inc(in_near ? &vis_near_blocked : &vis_blocked);
    return 1;
native:
    gh_atomic_inc(&vis_native);
    return 0;
}

static void vis_log(double t)
{
    if (!vis_csv)
        vis_csv = open_csv("vision.csv", "t,seen_near,seen_los,blocked,far,native,near_blocked");
    if (vis_csv) {
        fprintf(vis_csv, "%.1f,%ld,%ld,%ld,%ld,%ld,%ld\n", t, vis_seen_near, vis_seen_los, vis_blocked, vis_far,
                vis_native, vis_near_blocked);
        fflush(vis_csv);
    }
}

/* Behaviour-tree clock check: sum the move order's tick dt for one tank and log it
 * against wall time (wait/stuck timers looked ~12x slow on the host, 2026-11-10). */
static u8 *dt_unit;
static double dt_sum;
static uint64_t dt_t0, dt_calls, dt_next;
static FILE *dt_csv;

static void clock_check(u8 *unit, float dt, uint64_t ms)
{
    if (!dt_unit) {
        dt_unit = unit;
        dt_t0 = ms;
        dt_next = ms + 5000;
    }
    if (unit != dt_unit)
        return;
    dt_sum += dt;
    dt_calls++;
    if (ms < dt_next)
        return;
    dt_next = ms + 5000;
    if (!dt_csv)
        dt_csv = open_csv("tree_clock.csv", "wall_s,tree_dt_sum_s,ticks,last_dt");
    if (dt_csv) {
        fprintf(dt_csv, "%.2f,%.4f,%llu,%g\n", (double)(ms - dt_t0) / 1000.0, dt_sum,
                (unsigned long long)dt_calls, dt);
        fflush(dt_csv);
    }
}

/* ---- Approach-lane executor (step 1 of the lane design) ----
 * Runs once per bot tank per tick of the game's own move order, before the native node reads its
 * destination, so the command point we write wins over the tree's
 * own capture order. Engaged bots (stop-and-face branch) are not ticked;
 * that time does not count as "stuck". */
#define ROLL_M 70.0f            /* pass-through points switch this early (no native arrival) */
#define HOLD_ARRIVE_M 15.0f
/* v3 (2026-11-10, after a review of a battle log):
 * stuck handling only moves forward. "Not moving" = less than MOVE_M in STILL_MS. */
#define MOVE_M 12.0f
#define STILL_MS 12000
#define NEAR_SKIP_M 120.0f      /* still and this close to a pass point: count it as passed */
#define NATIVE_MS 20000         /* still on the way into the point: hand the native objective back */
#define L1_MS 8000
#define SPAWN_CALM_M 60.0f      /* no L1 this close to where the life started ... */
#define RETARGET_M 150.0f       /* further than this from its point when the team takes it: pick another */
#define LOAD_PENALTY_M 350.0f   /* target lane choice: distance + this per team-mate already on it */
/* Movement CSV owner values 4–6 are reserved for native, yield and combat. */
enum { RELIEF_REVERSE = 1, RELIEF_FORWARD, RECOVERY_NATIVE, RELIEF_SETTLE = 7, RELIEF_REJOIN };

static const char *const mode_names[] = {"move", "engage", "pursue", "retreat", "intercept"};

#include "ground_navigation.inc"

static void control_note_plan(const lane_seat_t *s, int seat, u8 *node, int tight_portal);
static void control_prepare_engagement(lane_seat_t *s, int seat, uint64_t ms, int route_hold);
int control_engagement_active(const lane_seat_t *s, uint64_t ms);
static int motion_native_repath_allowed(const lane_seat_t *s, int seat, uint64_t ms);
static int motion_capture_holding(const lane_seat_t *s, int seat, float x, float z, uint64_t ms);
int motion_capture_occupied(const lane_seat_t *s, int seat, uint64_t ms);
static void motion_apply_rejoin(void *host, lane_seat_t *s, int seat, uint64_t ms);
static void geometry_context(void *host, uint64_t ms);
static void geometry_focus(float x, float y, float z);
static int trace_with_flags(void *host, const float *p, const float *dir, float *t, int *kind, int flags);
static enum nav_geometry control_connector_prefix(const lane_seat_t *s, float tx, float tz);
static enum nav_geometry control_static_connector(const lane_seat_t *s, float x, float z, float tx, float tz);
static void nav_retire_route(lane_seat_t *s);
/* s28 tactics (ground_tactics.inc, included before lanes_tick). */
static int tac_gaze(const lane_seat_t *s, int seat, uint64_t ms, float *brg);
static int tac_pick_target(void *host, lane_seat_t *s, int seat, uint64_t ms, float *tx, float *tz, int *by_gunner, int *own);
static int tac_advance_ok(lane_seat_t *s, int seat, uint64_t ms, const char **why);
static const char *tac_advance_abort(lane_seat_t *s, int seat, uint64_t ms);
static int tac_propose(void *host, lane_seat_t *s, int seat, uint64_t ms, ground_engagement_policy_t policy);
static void tac_gunner_cols(const lane_seat_t *s, int seat, uint64_t ms, char *out, size_t n);
void tac_alert_plan(void *host, int army, uint64_t ms, float osx, float osz, float ux, float uz, float front, int np,
                           int live, int passive, int task_max);
static void tac_update(void *host, lane_seat_t *s, int seat, uint64_t ms);
static int tac_face_propose(lane_seat_t *s, int seat);
static int tac_face_hull(lane_seat_t *s, int seat, uint64_t ms);



static int stab_flags(u8 *unit);        /* native stabiliser state, after gun_read */
/* stab2 emulation (see "two-axis stabiliser emulation" below). */
#define EMU_PITCH_RATE_DEFAULT 6.0f   /* deg/s when no pitch drive speed is readable */
typedef struct {
    u8 *unit;
    uint64_t born_ms;
    int has_h, has_v;
    float om_h, om_v, lim_h, lim_v, spd_y, spd_p;
    int pv;                     /* elevation reference valid */
    float e_prev, p_prev;       /* barrel world elevation (rad) / A4 (deg) we left */
    uint64_t p_ms;
    float v_sum;                /* |vertical correction| this second (turret.csv) */
    long v_ticks;               /* ticks with a vertical correction written */
} emu_t;
static emu_t emu[MAX_SEATS];
static emu_t *emu_get(u8 *unit, int seat);
static int stab_emu_flags(u8 *unit);
static int turret_pitch_limits(u8 *turrets, float a0, float *lo, float *hi);
static float emu_pitch(u8 *unit, int seat, u8 *turrets, u8 *rec, float dt, float sp_pitch, int active);

/* Owner army (0 neutral) of the native capture zone nearest (x, z), or -1. */
int zone_owner(void *host, float x, float z)
{
    u8 *game = host ? gh_game(host) : NULL;
    u8 *owner_list = game ? gh_game_zones(game) : NULL;
    if (!owner_list)
        return -1;
    u8 **zones = gh_zone_list(owner_list);
    uint32_t zn = gh_zone_count(owner_list);
    int best = -1;
    float bd = 120.0f;
    for (uint32_t i = 0; zones && i < zn && i < 64; ++i) {
        u8 *zz = gh_zone_at(zones, i);
        u8 *shape = zz ? gh_zone_shape(zz) : NULL;
        if (!shape)
            continue;
        uint32_t type = gh_shape_type(shape);
        const float *c = gh_shape_centre(shape, type);
        const float *e = gh_shape_extent(shape);
        float h = gh_zone_shape_is_area(type) ? 0.5f : 0.0f;
        float d = dist2d(c[0] + h * e[0], c[2] + h * e[2], x, z);
        if (d < bd) {
            bd = d;
            best = gh_zone_owner(zz);
        }
    }
    return best;
}



/* Command point of route step i. Target lanes: S0/S1/S2 come from where this
 * life started (a team spawns in two groups; v2 used one group for everyone),
 * on the line from the start to the point. Flank lanes keep F1/F2 from Python. */
static void lane_point_xz(const lane_seat_t *s, int seat, int i, float *x, float *z)
{
    const lane_t *l = &lanes[s->lane];
    int pt = s->route[i].pt;
    if (pt == SLOT_PT) {
        /* Sunflower spread (2026-11-10): the old fixed 0.5 R ring put some
         * seat pairs 3-5 m apart. k = seat's index within its team. */
        int k = (seat - 1) % 15;
        float a = (float)k * 2.399963f, r = l->zr * 0.8f * sqrtf(((float)k + 0.5f) / 15.0f);
        *x = l->zx + cosf(a) * r;
        *z = l->zz + sinf(a) * r;
        return;
    }
    float ux, uz;
    if (pt == 1 && s->spot >= 0 && s->spot < ntspots) {     /* T2: overwatch holds a covered firing spot */
        *x = tspots[s->spot].x;
        *z = tspots[s->spot].z;
        return;
    }
    if (l->kind == 0) {
        float dist = dist2d(s->sx, s->sz, l->zx, l->zz);
        unit_dir(l->zx, l->zz, s->sx, s->sz, &ux, &uz);   /* point -> start */
        if (pt == 0) {
            float s2x = l->zx + ux * 150.0f, s2z = l->zz + uz * 150.0f, vx, vz;
            unit_dir(s->sx, s->sz, s2x, s2z, &vx, &vz);
            float step = dist > 300.0f ? 120.0f : 0.4f * dist;
            *x = s->sx + vx * step - vz * (float)((seat % 5) - 2) * 30.0f;
            *z = s->sz + vz * step + vx * (float)((seat % 5) - 2) * 30.0f;
        } else {
            float back = pt == 1 ? fminf(380.0f, 0.6f * dist) : fminf(150.0f, 0.5f * dist);
            *x = l->zx + ux * back;
            *z = l->zz + uz * back;
        }
        return;
    }
    if (pt >= l->n)
        pt = l->n - 1;
    if (pt == 0 && l->n > 1) {
        unit_dir(s->sx, s->sz, l->x[1], l->z[1], &ux, &uz);
        *x = s->sx + ux * 120.0f - uz * (float)((seat % 5) - 2) * 30.0f;
        *z = s->sz + uz * 120.0f + ux * (float)((seat % 5) - 2) * 30.0f;
        return;
    }
    *x = l->x[pt];
    *z = l->z[pt];
}

static int route_owns_flank(const lane_seat_t *s, int seat)
{
    return s->nav.route_active && s->tr == CMD_TR(seat) && s->flank_generation && s->flank_generation == s->route_generation;
}

void lane_begin_step(lane_seat_t *s, int seat, uint64_t ms)
{
    s->fight_post = 0;
    if (route_owns_flank(s, seat) && (s->flank_lane != s->lane || s->flank_step != s->step)) {
        nav_retire_route(s);
    }
    lane_point_xz(s, seat, s->step, &s->tx, &s->tz);
    s->holding = 0;
    s->native_until = 0;
    s->ax = s->x;
    s->az = s->z;
    s->anchor_ms = ms;
}

static void lane_goto_step(void *host, lane_seat_t *s, int seat, int step, uint64_t ms, const char *why)
{
    if (step >= s->n)
        step = s->n - 1;
    s->step = step;
    lane_begin_step(s, seat, ms);
    event(host, "lanes: seat %d lane %d step %d -> (%.0f,%.0f) [%s]", seat, s->lane, s->step, s->tx, s->tz, why);
}

static int lane_slot_step(const lane_seat_t *s)
{
    for (int i = 0; i < s->n; ++i)
        if (s->route[i].pt == SLOT_PT)
            return i;
    return s->n - 1;
}

/* Team-mates (alive, ticked in the last 2 s) currently on lane `lane`. */
static int lane_load(int army, int lane, const lane_seat_t *self, uint64_t ms)
{
    int n = 0;
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (o != self && o->unit && o->army == army && o->lane == lane && ms - o->last_ms < 2000)
            ++n;
    }
    return n;
}

/* A new life picks its lane from where it actually spawned: target lanes by
 * distance + LOAD_PENALTY_M per team-mate on it (+ OWNED_PENALTY_M when the
 * point is already ours: attack first), flank lanes by the nearer F1. */
#define OWNED_PENALTY_M 800.0f
static int lane_pick(void *host, lane_seat_t *s, int kind, uint64_t ms)
{
    int best = -1;
    float bd = 1e18f;
    for (int i = 0; i < nlanes; ++i) {
        const lane_t *l = &lanes[i];
        if (l->army != s->army || l->kind != kind)
            continue;
        float d = kind == 0 ? dist2d(s->sx, s->sz, l->zx, l->zz) + LOAD_PENALTY_M * (float)lane_load(s->army, i, s, ms)
                            : dist2d(s->sx, s->sz, l->x[l->n > 1 ? 1 : 0], l->z[l->n > 1 ? 1 : 0]);
        if (kind == 0 && zone_owner(host, l->zx, l->zz) == s->army)
            d += OWNED_PENALTY_M;
        if (d < bd) {
            bd = d;
            best = i;
        }
    }
    return best;
}

/* After taking a point: the nearest target lane of this team whose point is
 * not ours; the seat attacks it as an assault tank from its current position.
 * en_route: its point was taken while it was still driving there; a team-mate
 * nearer to that point on the same objective also counts as its anchor. */
static int lane_advance(void *host, lane_seat_t *s, int seat, uint64_t ms, int en_route)
{
    int best = -1;
    float bd = 1e9f;
    /* Anchor (plan L1): the last team-mate at a point we own stays. */
    if (cfg.team) {
        const lane_t *cur = &lanes[s->lane];
        float mine = dist2d(s->x, s->z, cur->zx, cur->zz);
        int others = 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            if (i == seat || !o->unit || !o->n || o->scene_epoch != ground_scene_epoch || o->army != s->army || ms - o->last_ms > 2000)
                continue;
            float od = dist2d(o->x, o->z, cur->zx, cur->zz);
            if (od < cur->zr + 40.0f
                || (en_route && !o->task && od < mine && dist2d(lanes[o->lane].zx, lanes[o->lane].zz, cur->zx, cur->zz) < 40.0f))
                ++others;
        }
        if (!others) {
            if (!en_route && ms - s->anchor_log_ms > 60000) {
                s->anchor_log_ms = ms;
                event(host, "lanes: seat %d is the last one at its point, stays as anchor", seat);
            }
            return 0;
        }
    }
    for (int i = 0; i < nlanes; ++i) {
        const lane_t *l = &lanes[i];
        if (l->army != s->army || l->kind != 0 || zone_owner(host, l->zx, l->zz) == s->army)
            continue;
        float d = dist2d(s->x, s->z, l->zx, l->zz);
        if (d < bd) {
            bd = d;
            best = i;
        }
    }
    if (best < 0)
        return 0;
    s->lane = best;
    s->role = LROLE_ASSAULT;
    s->sx = s->x;               /* approach from here */
    s->sz = s->z;
    nav_retire_route(s);
    s->spot = -1;
    lane_route(s, &lanes[best]);
    event(host, "lanes: seat %d %s to lane %d point (%.0f,%.0f), %.0f m", seat,
          en_route ? "switches (its point is ours already)" : "advances", best, lanes[best].zx, lanes[best].zz, bd);
    lane_goto_step(host, s, seat, 1, ms, "advance");
    return 1;
}

/* ---- Turret pre-aim (idle turrets all stared forward) ----
 * Native tank gunners issue no aim at all without a target (the game only
 * clears its state), so the turret keeps its last angle. Main turret record =
 * the unit's first turret record; its yaw / pitch (gh_turret_yaw / _pitch) are the
 * current angles relative to the hull in degrees (the engine builds the
 * barrel matrix from them); externally controlled turrets get the same angle
 * in the external, current and commanded yaw, so we write yaw to all three.
 * Only when this bot has seen no enemy tank for 2 s (the gunner can only
 * target visible tanks, so it has none) -> it never fights the gunner.
 * The yaw sign (left or right positive) is learnt in battle: while a bot sees
 * an enemy and its turret is steady, compare the turret yaw with the bearing under both
 * signs; 6 net votes fix it. Until then idle turrets only sweep symmetrically. */
static int yaw_sign;            /* 0 unknown, +1 / -1 */

/* ---- review_5003 A1 / B5 / A7 (2026-11-10) ---------------------------------
 * Run 20261110_224234_ground_5003: shot error grows linearly with the hull yaw
 * rate (0.44 s slope); the turret record and the gunner's commanded angle
 * both lag, and nothing keeps the gun on target while the hull turns.
 *  A1 (cfg.stab): while the gunner has a target, every tick the hull's world
 *     yaw change is taken out of the turret angles (hull frame, learnt yaw
 *     sign): the external and commanded yaw get the full correction (the gunner's
 *     intent stays fixed in the world); the current yaw gets it only
 *     inside the vehicle's own traverse speed: |native move + correction| <=
 *     speedYaw * dt over the tick (speedYaw of the weapon-AI data, never raised),
 *     clamped to a limited traverse sector. Pitch: not done (sign not learnt).
 *  B5: this tank sees an enemy (own eyes) but the gunner has no target yet:
 *     keep turning the turret towards it at speedYaw instead of stopping the
 *     pre-aim (5003: the gunner waited 2 s with the turret idle).
 *  A7 probe: turret.csv gets aim_calls (wt_ground_aim calls of this seat in
 *     the last second), gate_blocks (fire-gate blocks), stab_deg (turret
 *     correction applied in that second) and turret_mode; turret10.csv logs
 *     one seat at 10 Hz (cfg.probe_seat, else the first seat whose gunner
 *     gets a target; kept for that life, at most PROBE_MAX_ROWS rows). */
#define STAB_MAX_DT 0.25f
static int weapon_ai_read(u8 *unit, unsigned idx, float *yaw, float *pitch, float *cone);

/* GZ_ATTN (s28): the tactics attention (ground_tactics.inc tac_gaze): the one exit a reaction watches, else the
 * sector centre of the nearest presence hint; one key for both, so a change between them needs no hysteresis. */
enum { GZ_NONE, GZ_FWD, GZ_OBJ, GZ_ZONE, GZ_POST, GZ_CONTACT, GZ_TASK, GZ_HIT, GZ_ATTN };
static uint32_t gaze_rng = 0x9e3779b9u;

static float wrap180(float a)
{
    if (!isfinite(a))
        return 0.0f;
    a = fmodf(a, 360.0f);
    if (a > 180.0f)
        a -= 360.0f;
    else if (a < -180.0f)
        a += 360.0f;
    return a;
}

/* Uniform in [0, 1). */
static float gaze_rand(void)
{
    gaze_rng ^= gaze_rng << 13;
    gaze_rng ^= gaze_rng >> 17;
    gaze_rng ^= gaze_rng << 5;
    return (float)(gaze_rng >> 8) / 16777216.0f;
}

/* Yaw limits of turret record i: 1 when the traverse is limited (span < 359 deg).
 * The turret's limits record holds sectors (yaw lo, yaw hi, pitch lo, pitch hi, ...)
 * and their count (gh_turret_yaw_limits, read the way the game reads them). */
static int turret_yaw_limits(u8 *turrets, unsigned i, float *lo, float *hi)
{
    int limited = 0;
    limited = gh_turret_yaw_limits(turrets, i, lo, hi);
    return limited;
}

/* Where an idle crew would look: source key and world bearing (deg). */
static int gaze_source(void *host, const lane_seat_t *s, int seat, uint64_t ms, float *brg)
{
    unsigned slot = (unsigned)seat & 63;
    const lane_t *l = &lanes[s->lane];
    float kx, kz, kage;
    if (yaw_sign == 0)
        return GZ_FWD * 100;    /* sign unknown: only the centre is safe */
    /* All of it is team knowledge, no wallhack (same priority as before). */
    if (hit_dir[slot].ms && ms - hit_dir[slot].ms < 20000) {
        *brg = atan2f(hit_dir[slot].dz, hit_dir[slot].dx) * 57.29578f;
        return GZ_HIT * 100 + (hit_dir[slot].from & 63);
    }
    if (cfg.sense || cfg.waitpeek) {
        const int attn = tac_gaze(s, seat, ms, brg);   /* s28: exit watched, else presence-hint sector */
        if (attn)
            return attn;
    }
    if (s->task && contact_get(s->army, seat, s->task_tgt, ms, &kx, &kz, &kage)) {
        *brg = atan2f(kz - s->z, kx - s->x) * 57.29578f;
        return GZ_TASK * 100 + (s->task_tgt & 63);
    }
    if (cmd_is_post(s)) {
        *brg = atan2f(s->post_gz - s->z, s->post_gx - s->x) * 57.29578f;
        return GZ_POST * 100;
    }
    {
        float kbest = 600.0f;
        int kslot = -1;
        float bx = 0, bz = 0;
        for (int k = 0; cfg.team && k < 64; ++k)
            if (contact_get(s->army, seat, k, ms, &kx, &kz, &kage) && kage < 20.0f
                && dist2d(s->x, s->z, kx, kz) < kbest) {
                kbest = dist2d(s->x, s->z, kx, kz);
                kslot = k;
                bx = kx;
                bz = kz;
            }
        if (kslot >= 0 && kbest > 5.0f) {
            *brg = atan2f(bz - s->z, bx - s->x) * 57.29578f;
            return GZ_CONTACT * 100 + kslot;
        }
    }
    {
        float dz = dist2d(s->x, s->z, l->zx, l->zz);
        if (dz < 600.0f && dz > l->zr && zone_owner(host, l->zx, l->zz) != s->army) {
            *brg = atan2f(l->zz - s->z, l->zx - s->x) * 57.29578f;
            return GZ_ZONE * 100;
        }
    }
    if (dist2d(s->sx, s->sz, l->zx, l->zz) > 20.0f) {
        /* the approach direction: from where this life started towards the point */
        *brg = atan2f(l->zz - s->sz, l->zx - s->sx) * 57.29578f;
        return GZ_OBJ * 100;
    }
    return GZ_FWD * 100;
}

/* Clamp a hull-frame turret angle into the traverse sector (lim) or wrap it. */
static float turret_clamp(int lim, float lo, float hi, float a)
{
    return lim ? (a < lo ? lo : a > hi ? hi : a) : wrap180(a);
}

/* s49: fire control (ground_fire.inc, included further down), called from here on. */
static void preaim(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms);
static void stab_native_on(void *host, u8 *unit);
static void stab_diag(u8 *unit, char *out, size_t n);

/* ==== gun_shots.c : main-gun shot counter for ground_brain.c (shots.csv) ====
 *
 * INSERT this whole block in ground_brain.c directly AFTER the closing brace of
 *     static void preaim(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms)
 * (it uses yaw_sign / wrap180 from the pre-aim block and slot_unit / slot_alive,
 * open_csv, event, t0, mode_names, lane_seat_t from above it).
 *
 * THEN add one line in lanes_tick(), right after the existing call
 *     preaim(host, unit, s, seat, ms);
 * add:
 *     shots_poll(host, unit, s, seat, ms);
 *
 * The gun state comes from the host (gh_gun_state, gh_gun_shell; reverse-engineered
 * statically, notes in ground_host.c): the main turret record's weapon, whether it is
 * a gun, its current bullet set, the per-set shots fired and rounds left (the gun's
 * replicated ammo state, as the game's own shot code updates it), the time of the
 * last shot and the reload end; speedYaw, speedPitch and fireConeAngle of the
 * weapon-AI data come from gh_weapon_ai.
 * NOT verified in a running game (no launch in this task): the spawn dump below
 * prints rounds left per gun so the first run can be checked against the
 * "bot seat N ... loads <shell> xNN" lines of events.log. */

/* State of the gun in turret record idx. Returns 1 when the record exists. */
static int gun_read(void *host, u8 *unit, unsigned idx, gun_state_t *g)
{
    int ok = 0;
    memset(g, 0, sizeof *g);
    ok = gh_gun_state(host, unit, idx, g);
    return ok;
}

/* Shots the main gun (turret record 0) has fired in this life, or -1. */
static int gun_shots(void *host, u8 *unit)
{
    gun_state_t g;
    return gun_read(host, unit, 0, &g) && g.is_gun ? g.shots : -1;
}

/* speedYaw / speedPitch / fireConeAngle of weapon i as the game uses them. */
static int weapon_ai_read(u8 *unit, unsigned idx, float *yaw, float *pitch, float *cone)
{
    int ok = 0;
    ok = gh_weapon_ai(unit, idx, yaw, pitch, cone);
    return ok;
}

/* stab2 (2026-11-11): the inputs of the native stabiliser decision for one
 * bot, read only (the stabiliser-allowed byte of the controls, the unit's
 * exemption flag, the player's flag byte; the turret drive state flags; the
 * unit's simulation mode; the gun's stab object), see stab_diag. */

/* May this bot fire on the move (movefire rules)? 2: full stabiliser working
 * (horizontal + vertical), 1: vertical only, 0: no vertical stabiliser working
 * (none in the data, the engine not stabilising, or over its speed limit). */
static int seat_movefire(u8 *unit)
{
    int f = stab_flags(unit) | stab_emu_flags(unit);
    if (!(f & 2))
        return 0;
    return (f & 1) ? 2 : 1;
}

/* ---- stab2: two-axis stabiliser emulation (2026-11-11) -----------------
 * Where the engine does not stabilise a bot's main gun itself (its two
 * engine-stabiliser flags are 0), the DLL does what the stabiliser would: keep the barrel's
 * world direction while the hull turns / pitches / rolls, only for a gun whose
 * own stab object (gh_gun_stab_object, the engine's data, not a guess) has
 * that axis, under its speed limit (m/s, < 0 none), scaled by
 * its omega multiplier, with the turret drive intact
 * and, for the vertical, the vertical drive not blocked (the same tests the
 * engine's turret drive makes). The correction goes on top of the drive's own
 * move but |own move + correction| stays within the drive speed per tick
 * (weapon-AI speedYaw / speedPitch, or the stab object's speedYaw /
 * speedPitch when it gives a larger one), so it never turns faster than the
 * vehicle can. Horizontal: preaim_body A1. Vertical: emu_pitch below, exact
 * world elevation from the hull matrix (F = gh_unit_frame, U = gh_unit_up, P = F x U,
 * barrel = cos p (cos a F + sin a P) + sin p U, a = yaw_sign * A0, p = A4,
 * +A4 = up as in the fire gate). WT_GROUND_STAB_EMU=0: off. */

/* s49: the guarded block of emu_get, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *unit; emu_t *e; } emu_get_ctx_t;
static void emu_get_read(void *ctx)
{
    emu_get_ctx_t *const context = ctx;
    u8 *const unit = context->unit;
    emu_t *const e = context->e;
    {
        u8 *turrets = gh_unit_turrets(unit);
        u8 *rec = turrets && gh_turrets_count(turrets) ? gh_turrets_records(turrets) : NULL;
        u8 *gun = rec ? gh_turret_gun(rec) : NULL;
        u8 *props = gun ? gh_gun_props(gun) : NULL;
        u8 *so = props ? gh_gun_stab_object(props) : NULL;
        if (so) {
            e->has_h = gh_stab_axis_h(so) == 1;
            e->has_v = gh_stab_axis_v(so) == 1;
            e->om_h = gh_stab_omega_h(so);
            e->om_v = gh_stab_omega_v(so);
            e->lim_h = gh_stab_limit_h(so);
            e->lim_v = gh_stab_limit_v(so);
            e->spd_y = gh_stab_speed_yaw(so);
            e->spd_p = gh_stab_speed_pitch(so);
            if (!(e->om_h > 0.0f && e->om_h <= 4.0f))
                e->om_h = e->om_h > 4.0f ? 4.0f : 0.0f;
            if (!(e->om_v > 0.0f && e->om_v <= 4.0f))
                e->om_v = e->om_v > 4.0f ? 4.0f : 0.0f;
            if (!isfinite(e->lim_h))
                e->lim_h = 0.0f;
            if (!isfinite(e->lim_v))
                e->lim_v = 0.0f;
            if (!(e->spd_y > 0.0f && e->spd_y < 200.0f))
                e->spd_y = 0.0f;
            if (!(e->spd_p > 0.0f && e->spd_p < 200.0f))
                e->spd_p = 0.0f;
        }
    }
}

/* The stab object of this life's main gun (read once per life). */
static emu_t *emu_get(u8 *unit, int seat)
{
    if (seat < 0 || seat >= MAX_SEATS)
        return NULL;
    emu_t *e = &emu[seat];
    if (e->unit == unit && e->born_ms == lseat[seat].born_ms)
        return e;
    memset(e, 0, sizeof *e);
    e->unit = unit;
    e->born_ms = lseat[seat].born_ms;
    emu_get_ctx_t context = {unit, e};
    if (gh_guard(emu_get_read, &context)) {
        e->has_h = e->has_v = 0;
    }
    return e;
}

/* Pitch limits (deg) of turret record 0 at hull-frame yaw a0 (sector array of
 * the limits record, see turret_yaw_limits; per sector: yaw lo, yaw hi, pitch
 * lo, pitch hi). 0 when unreadable. */
static int turret_pitch_limits(u8 *turrets, float a0, float *lo, float *hi)
{
    int ok = 0;
    ok = gh_turret_pitch_limits(turrets, a0, lo, hi);
    return ok;
}

/* Barrel world elevation (rad) for hull-plane direction d, hull up U, pitch p (rad). */
static float emu_elev(const float d[3], const float U[3], float p)
{
    float y = cosf(p) * d[1] + sinf(p) * U[1];
    return asinf(y > 1.0f ? 1.0f : y < -1.0f ? -1.0f : y);
}

/* s49: the guarded block of emu_pitch, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *unit, *turrets, *rec; float dt, sp_pitch, shift; emu_t *e; } emu_pitch_ctx_t;
static void emu_pitch_run(void *ctx)
{
    emu_pitch_ctx_t *const context = ctx;
    u8 *const unit = context->unit;
    u8 *const turrets = context->turrets;
    u8 *const rec = context->rec;
    const float dt = context->dt;
    const float sp_pitch = context->sp_pitch;
    emu_t *const e = context->e;
    float shift = context->shift;
    {
        const float *F = gh_unit_frame(unit), *U = gh_unit_up(unit);
        float P[3] = {F[1] * U[2] - F[2] * U[1], F[2] * U[0] - F[0] * U[2], F[0] * U[1] - F[1] * U[0]};
        float a0 = wrap180(gh_turret_yaw(rec));
        float a = (float)yaw_sign * a0 * 0.01745329f;
        float d[3];
        for (int i = 0; i < 3; ++i)
            d[i] = cosf(a) * F[i] + sinf(a) * P[i];
        float p_now = wrap180(gh_turret_pitch(rec));
        if (!isfinite(p_now) || fabsf(p_now) > 60.0f) {
            e->pv = 0;
            goto leave;
        }
        float p_next = p_now;
        if (e->pv) {
            float native = p_now - e->p_prev;                       /* the drive's own move since last tick */
            float hold = e->e_prev + native * 0.01745329f;          /* keep the world elevation (+ that move) */
            float R = sqrtf(d[1] * d[1] + U[1] * U[1]);
            if (R > 0.3f) {
                float sv = sinf(hold) / R;
                float want = (asinf(sv > 1.0f ? 1.0f : sv < -1.0f ? -1.0f : sv) - atan2f(d[1], U[1])) * 57.29578f;
                float comp = (want - p_now) * e->om_v;
                float rate = sp_pitch > 1.0f && sp_pitch < 200.0f ? sp_pitch : EMU_PITCH_RATE_DEFAULT;
                if (e->spd_p > rate)
                    rate = e->spd_p;
                float lim = rate * dt;
                float lo_s = -lim - native, hi_s = lim - native;    /* |native + shift| <= rate * dt */
                if (lo_s > 0.0f)
                    lo_s = 0.0f;
                if (hi_s < 0.0f)
                    hi_s = 0.0f;
                shift = comp < lo_s ? lo_s : comp > hi_s ? hi_s : comp;
                float plo = -90.0f, phi = 90.0f;
                turret_pitch_limits(turrets, a0, &plo, &phi);
                p_next = p_now + shift;
                p_next = p_next < plo ? plo : p_next > phi ? phi : p_next;
                shift = p_next - p_now;
                if (fabsf(shift) > 1e-4f) {
                    gh_turret_set_pitch(rec, p_next);
                    float c84 = gh_turret_ext_pitch(rec) + shift, cdc = gh_turret_cmd_pitch(rec) + shift;
                    gh_turret_set_ext_pitch(rec, c84 < plo ? plo : c84 > phi ? phi : c84);
                    gh_turret_set_cmd_pitch(rec, cdc < plo ? plo : cdc > phi ? phi : cdc);
                    e->v_sum += fabsf(shift);
                    ++e->v_ticks;
                }
            }
        }
        e->p_prev = p_next;
        e->e_prev = emu_elev(d, U, p_next * 0.01745329f);
        e->p_ms = gh_ms();
        e->pv = isfinite(e->e_prev);
    }
leave:
    context->shift = shift;
}

/* Vertical emulation for one tick (gunner on a target). Returns the pitch
 * correction written (deg). active = 0 resets the reference. */
static float emu_pitch(u8 *unit, int seat, u8 *turrets, u8 *rec, float dt, float sp_pitch, int active)
{
    emu_t *e = emu_get(unit, seat);
    if (!e)
        return 0.0f;
    if (!active || !(dt > 0.0f) || dt > STAB_MAX_DT) {
        e->pv = 0;
        return 0.0f;
    }
    float shift = 0.0f;
    emu_pitch_ctx_t context = {unit, turrets, rec, dt, sp_pitch, shift, e};
    const int faulted = gh_guard(emu_pitch_run, &context);
    shift = context.shift;
    if (faulted) {
        e->pv = 0;
        shift = 0.0f;
    }
    return shift;
}

/* Shell id of round k of the main gun's current belt position: back = 0 the
 * next round, 1 the one just fired (belt reading as in aim_shell). -1 unknown. */
static int32_t gun_shell_id(void *host, u8 *unit, int back)
{
    int32_t id = -1;
    id = gh_gun_shell(host, unit, back);
    return id;
}

/* Scan identities and aim/gate state use aim_srw. The lane and hit hooks
 * already hold lock when they take aim_srw. Aim and fire never take lock. */
enum { SHOT_UNKNOWN, SHOT_KILL, SHOT_FIRE, SHOT_MOBILITY, SHOT_NONE };
typedef struct {
    int opportunity, precision_blocked, reposition;
    uint64_t aim_ms, blocked_since, effective_ms, fired_ms, flight_until;
} shot_opportunity_t;

static shot_opportunity_t shot_opportunity(int seat, const lane_seat_t *s, int target, uint64_t ms);

#define GATE_AIM_MS 600
#define GATE_EVAL_MS 700
typedef struct {
    u8 *shooter, *target;
    uint64_t shooter_life, target_life, ms;
    float point[3];              /* uncorrected impact point, before native ballistics */
    float origin[3];             /* target origin at the same sample */
    float half_w, half_h;        /* usable margins around the point, metres */
    int opportunity, native, node;
} aim_rec_t;
static aim_rec_t aim_rec[MAX_SEATS][64];
typedef struct {
    u8 *unit, *target;
    uint64_t shooter_life, target_life, hold_since, last_ms, eval_ms;
    int precision_blocked, reposition, opportunity, node, occluded, reload_held;
    uint64_t scene_ms;
    float rel_err, rel_tol, rel_perr, rel_ptol, rel_wait;
    uint64_t rel_ms, rel_flight;
    u8 *rel_target;
    uint64_t rel_shooter_life, rel_target_life;
    const char *rel_src;
    u8 *move_unit;              /* movefire hold in progress for this life */
    uint64_t move_since;
    float rel_move_wait;        /* s held by movefire before the last release */
    /* s45 (record only, shots.csv): at the release, the signed differences between the game's commanded direction and
     * the direction to the shot's aim point (the 14 / 18 deg tests of wt_ground_fire; rel_dir_ok 0: not computed),
     * and the gunner's target slot */
    int rel_dir_ok, rel_ts;
    float rel_dyaw, rel_dpitch;
} gate_seat_t;
static gate_seat_t gate_seat[MAX_SEATS];

typedef struct {
    u8 *shooter, *target;
    uint64_t shooter_life, target_life, fired_ms, flight_until, effective_ms;
} shot_result_t;
static shot_result_t shot_result[MAX_SEATS][64];

typedef struct {
    u8 *unit;                   /* the life the baseline belongs to */
    uint64_t life;
    int base;                   /* counter value already accounted for */
    int total;                  /* shots of this seat in the battle */
    int last_gt;                /* the gunner's last target and when it had it */
    uint64_t last_gt_ms;
    /* review_5003: reload window (A3 fire stop, B3 pivots) and hits.csv pairing. */
    uint64_t shot_ms, ready_ms; /* last detected shot; when the next round is in (wall clock) */
    int32_t shell;              /* shell id of the last shot */
    int left;                   /* rounds left */
} shot_seat_t;
static shot_seat_t shot_seat[MAX_SEATS];
static FILE *shots_csv;

/* Called every tick from lanes_tick (under `lock`). One shots.csv row per shot:
 * t, seat, n (battle total of the seat), life_n, ammo_left, target_slot, dist_m,
 * aim_err_deg (gun bearing minus bearing to the target, hull frame sign learnt
 * by preaim; empty when unknown), speed_mps, mode, gun_t.
 * Pair with hits.csv: offender_slot == seat, hit t within the flight time. */
static void shots_poll(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms)
{
    shot_seat_t *st = &shot_seat[seat];
    gun_state_t g;
    (void)gun_shots;            /* kept for other callers (W4: unreferenced static) */
    if (s->gt_slot >= 0) {
        st->last_gt = s->gt_slot;
        st->last_gt_ms = ms;
    }
    if (st->unit != unit || st->life != slot_life[seat]) {
        st->unit = unit;
        st->life = slot_life[seat];
        st->base = -1;
        st->last_gt = -1;
        st->last_gt_ms = 0;
        st->shot_ms = st->ready_ms = 0;     /* a fresh tank starts loaded */
        for (unsigned i = 0; i < 8 && gun_read(host, unit, i, &g); ++i) {
            float sy = 0, sp = 0, cone = 0;
            int ai = weapon_ai_read(unit, i, &sy, &sp, &cone);
            if (g.is_gun)
                event(host, "guns: seat %d rec %u sets %d cur %d left %d shots %d alt %d | speedYaw %.1f speedPitch %.1f cone %.1f%s",
                      seat, i, g.nsets, g.cur, g.left, g.shots, g.alt, sy, sp, cone, ai ? "" : " (no AI params)");
            else
                event(host, "guns: seat %d rec %u is not a gun | speedYaw %.1f speedPitch %.1f cone %.1f%s",
                      seat, i, sy, sp, cone, ai ? "" : " (no AI params)");
        }
    }
    if (!gun_read(host, unit, 0, &g) || !g.is_gun)
        return;
    st->left = g.left;
    if (st->base < 0 || g.shots < st->base) {   /* first sight, or the counter was rewritten (rearm) */
        st->base = g.shots;
        return;
    }
    int fired = g.shots - st->base;
    if (!fired)
        return;
    st->base = g.shots;
    st->total += fired;
    /* Reload window: reload end - last shot time is the reload the shot started (both game clock). */
    {
        float reload = g.next_t - g.last_t;
        st->shot_ms = ms;
        st->ready_ms = ms + (reload > 0.3f && reload < 60.0f ? (uint64_t)(reload * 1000.0f) : 6000u);
        st->shell = gun_shell_id(host, unit, 1);
    }

    int tgt = s->gt_slot >= 0 ? s->gt_slot : (st->last_gt >= 0 && ms - st->last_gt_ms < 1500 ? st->last_gt : -1);
    gh_rw_write_lock(GH_RW_AIM);
    const gate_seat_t release = gate_seat[seat];
    gate_seat[seat].rel_move_wait = 0.0f;
    gh_rw_write_unlock(GH_RW_AIM);
    const int gated = release.unit == unit && release.rel_shooter_life == st->life
                      && release.rel_ms && ms - release.rel_ms < 400;
    if (gated && release.rel_target) {
        tgt = -1;
        for (int i = 0; i < 64; ++i)
            if (slot_alive[i] && slot_unit[i] == release.rel_target && slot_life[i] == release.rel_target_life) {
                tgt = i;
                break;
            }
    }
    float dist = -1.0f, err = 0.0f;
    int have_err = 0;
    if (tgt >= 0 && slot_alive[tgt] && slot_unit[tgt]) {
        const float *p = gh_unit_pos(slot_unit[tgt]);
        dist = dist2d(s->x, s->z, p[0], p[2]);
        u8 *turrets = gh_unit_turrets(unit);
        u8 *rec = turrets ? gh_turrets_records(turrets) : NULL;
        if (rec && yaw_sign != 0 && dist > 1.0f) {
            float hull = atan2f(s->fz, s->fx) * 57.29578f;
            float brg = atan2f(p[2] - s->z, p[0] - s->x) * 57.29578f;
            err = wrap180(hull + (float)yaw_sign * gh_turret_yaw(rec) - brg);
            have_err = 1;
        }
    }
    if (tgt >= 0 && slot_alive[tgt] && slot_unit[tgt]) {
        shot_result_t *result = &shot_result[seat][tgt];
        if (result->shooter != unit || result->shooter_life != st->life || result->target != slot_unit[tgt]
            || result->target_life != slot_life[tgt])
            *result = (shot_result_t){unit, slot_unit[tgt], st->life, slot_life[tgt], 0, 0, 0};
        result->fired_ms = ms;
        result->flight_until = ms + (gated ? release.rel_flight : (uint64_t)(fmaxf(0.0f, dist) * 1000.0f / 600.0f)) + 750;
    }
    if (!shots_csv)
        shots_csv = open_csv("shots.csv", "t,seat,n,life_n,fired,ammo_left,target_slot,dist_m,aim_err_deg,speed_mps,mode,gun_t,"
                                          "reload_s,shell,hull_rate,stopping,close,gate_err_deg,gate_tol_deg,gate_pitch_deg,"
                                          "gate_wait_s,gate_forced,gate_src,stab_h_on,stab_v_on,move_wait_s,rear,emu_h,emu_v,gate_pitch_tol_deg,"
                                          "rel_cmd_dyaw_deg,rel_cmd_dpitch_deg,rel_target_slot,rel_src,rel_ms");
    if (shots_csv) {
        char e[24] = "", ge[96] = ",,,,,";
        if (have_err)
            snprintf(e, sizeof e, "%.1f", err);
        /* The gate's release (A2) for this shot: it happened at most a tick before. */
        char pt[24] = "";
        /* s45: commanded direction minus the direction to the aim point at the release (signed, deg; empty when the
         * gate did not compute it), the gunner's target slot, the aim point's source, the release time (ms since
         * the log start, the t column's clock); all empty without a gate release for this shot */
        char rl[96] = ",,,,";
        if (gated) {
            snprintf(ge, sizeof ge, "%.3f,%.3f,%.3f,%.2f,0,%s", release.rel_err, release.rel_tol, release.rel_perr,
                     release.rel_wait, release.rel_src ? release.rel_src : "");
            snprintf(pt, sizeof pt, "%.3f", release.rel_ptol);
            char dy[16] = "", dp[16] = "";
            if (release.rel_dir_ok) {
                snprintf(dy, sizeof dy, "%.3f", release.rel_dyaw);
                snprintf(dp, sizeof dp, "%.3f", release.rel_dpitch);
            }
            snprintf(rl, sizeof rl, "%s,%s,%d,%s,%llu", dy, dp, release.rel_ts, release.rel_src ? release.rel_src : "",
                     (unsigned long long)(release.rel_ms - t0));
        }
        int sf = stab_flags(unit), ef = stab_emu_flags(unit);
        fprintf(shots_csv, "%.2f,%d,%d,%d,%d,%d,%d,%.0f,%s,%.1f,%s,%.2f,%.2f,%d,%.1f,%d,%d,%s,%d,%d,%.2f,%d,%d,%d,%s,%s\n",
                (double)(ms - t0) / 1000.0, seat, st->total, g.shots, fired, g.left, tgt, dist, e,
                sqrtf(s->vx * s->vx + s->vz * s->vz), mode_names[s->mode], g.last_t, (double)(st->ready_ms - ms) / 1000.0,
                st->shell, s->hull_rate, s->stopping, s->close, ge, sf & 1, (sf >> 1) & 1,
                release.move_unit == unit ? release.rel_move_wait : 0.0f, s->rear, ef & 1, (ef >> 1) & 1, pt, rl);
        fflush(shots_csv);
    }
}

/* Called under lock. A same-target damage accumulator in the estimated
 * flight window is evidence of effective damage, with no part attribution. */
static void shot_effect(int from, u8 *victim, unsigned target, uint64_t now)
{
    if (from < 1 || from >= MAX_SEATS || !slot_alive[from] || !slot_alive[target])
        return;
    shot_result_t *r = &shot_result[from][target];
    if (r->shooter == slot_unit[from] && r->shooter_life == slot_life[from]
        && r->target == victim && victim == slot_unit[target] && r->target_life == slot_life[target]
        && r->fired_ms && now >= r->fired_ms && now <= r->flight_until + 1000)
        r->effective_ms = now;
}

static shot_opportunity_t shot_opportunity(int seat, const lane_seat_t *s, int target, uint64_t ms)
{
    shot_opportunity_t out = {0};
    if (seat <= 0 || seat >= MAX_SEATS || target < 0 || target >= 64 || s->gt_slot != target
        || !slot_alive[seat] || slot_unit[seat] != s->unit || !slot_alive[target])
        return out;
    u8 *victim = slot_unit[target];
    uint64_t sl = slot_life[seat], tl = slot_life[target];
    const shot_result_t *result = &shot_result[seat][target];
    if (result->shooter == s->unit && result->shooter_life == sl && result->target == victim && result->target_life == tl
        && result->fired_ms >= s->born_ms) {
        out.fired_ms = result->fired_ms;
        out.flight_until = result->flight_until;
        if (result->effective_ms >= s->born_ms && ms - result->effective_ms <= 30000)
            out.effective_ms = result->effective_ms;
    }
    gh_rw_read_lock(GH_RW_AIM);
    const aim_rec_t ar = aim_rec[seat][target];
    const gate_seat_t gs = gate_seat[seat];
    gh_rw_read_unlock(GH_RW_AIM);
    if (ar.shooter == s->unit && ar.shooter_life == sl && ar.target == victim && ar.target_life == tl
        && ar.ms >= s->born_ms && ms - ar.ms <= GATE_AIM_MS) {
        out.opportunity = ar.opportunity;
        out.aim_ms = ar.ms;
        out.reposition = ar.opportunity == SHOT_NONE;
    }
    if (gs.unit == s->unit && gs.shooter_life == sl && gs.target == victim && gs.target_life == tl
        && gs.eval_ms >= s->born_ms && ms - gs.eval_ms <= GATE_EVAL_MS && gs.hold_since
        && (!out.aim_ms || (ar.opportunity == gs.opportunity && ar.node == gs.node))) {
        out.precision_blocked = gs.precision_blocked;
        out.blocked_since = gs.hold_since;
        out.reposition |= gs.reposition;
    }
    return out;
}

/* hits.csv: the offender's last main-gun shot (age s, -1 none) and its shell id. */
static void hit_shot_info(int from, uint64_t now, float *age, int32_t *shell)
{
    if (from < 1 || from >= MAX_SEATS || !shot_seat[from].shot_ms)
        return;
    *age = (float)(now - shot_seat[from].shot_ms) / 1000.0f;
    *shell = shot_seat[from].shell;
}

/* Seconds until the main gun is loaded again (0 = loaded; from shots_poll),
 * or a large value without ammunition. */
float seat_ready_in(int seat, uint64_t ms)
{
    const shot_seat_t *st = &shot_seat[seat];
    if (st->unit && st->base >= 0 && st->left == 0)  /* -1 (seen in 5003 for some guns) is not "empty" */
        return 99.0f;
    return st->ready_ms > ms ? (float)(st->ready_ms - ms) / 1000.0f : 0.0f;
}

/* Cover seeking (2026-11-11, defined after the strategy layer: cover_find). */
enum { COV_NONE, COV_FIRE, COV_RETREAT, COV_HOLD, COV_SUPPORT, COV_PUSH, COV_CLOSE, COV_BOUND, COV_NUSE };
static const char *const cov_name[COV_NUSE] = {"none", "fire", "retreat", "hold", "support", "push", "close", "bound"};
#define CVF_VIEW 1              /* the turret must see the threat's distance band that way (G10VIEW) */
#define CVF_AWAY 2              /* no cell further towards the threat than the origin (retreat) */
#define CVF_AHEAD 4             /* only cells the tank can drive into with its front still to the threat (COV_AHEAD_*) */
#define CVF_CALM 8              /* no cell under a fire threat of AP_TH_HI or more (a stop before a stretch, not in it) */
#define COV_RET_MIN_M 20.0f     /* retreat from an unseen shooter: cover 20-80 m away */
#define COV_RET_MAX_M 80.0f
#define COV_RET_STAY_MS 6000    /* ... and wait in it this long */
#define COV_ARRIVE_M 7.0f       /* in (or at the edge of) the 10 m cell */
#define COV_HOP_SPEED 3.0f      /* a cover move may take 3 s + distance / this (m/s) */
typedef struct { float x, z, dist, score; int q, qraw, type, conceal, k; } cover_t;
static int cover_find(int seat, int army, int use, float x, float z, float tx, float tz, float r_min, float r_max, int flags,
                      cover_t *out);
static int cover_here(float x, float z, float tx, float tz);
static int cover_engage(void *host, lane_seat_t *s, int seat, uint64_t ms, float d, int seen_now);
static void cover_used(void *host, int seat, int use, const cover_t *c, int here, float tx, float tz, float ox, float oz);
static void cover_end(void *host, lane_seat_t *s, int seat, uint64_t ms, const char *why);

/* ---- Per-tank reactions (2026-11-10, plan L2) ----------------------
 * Each tick, before the lane logic:
 *  - ENGAGE: the gunner has a target (read like the game's own turret order), else an enemy
 *    in this tank's own sight. Beyond ENG_MIN_M the tank makes a short stop
 *    and, while stopped or holding, can request that the hull turn
 *    its hull to the target (wt_ground_after writes throttle/steering). The
 *    destination stays the lane's / task's.
 *  - lost the target ENG_LOST_MS: PURSUE its last seen position (< PURSUE_MAX_M,
 *    PURSUE_MS at most; not while holding a point);
 *  - UNDER FIRE: hit by an unseen shooter -> hull to the bearing when stopped;
 *    a second hit within HIT_AGAIN_MS -> RETREAT RETREAT_M away from it;
 *  - INTERCEPT: the team planner's task (team_plan).
 * Destination priority: retreat > pursue > intercept > lane. Returns 1 when
 * it set the destination (lanes_tick then returns). */
#define ENG_MIN_M 60.0f
#define ENG_MAX_M 1500.0f
#define ENG_STOP_MAX_M 800.0f   /* no short stop for targets further than this */
/* Short-stop policy (2026-11-10 21:00; was: any target 60-800 m, stop 10 s / move
 * 6 s): stop for targets under ENG_STOP_NEAR_M; between that and ENG_STOP_MAX_M
 * only when hit in the last ENG_HIT_MS or the gunner's turret is laid on the
 * target (TURRET_ON_DEG); otherwise shoot on the move. A point pusher only
 * stops for targets under PUSH_STOP_M. */
#define ENG_STOP_NEAR_M 450.0f
#define ENG_HIT_MS 10000
#define TURRET_ON_DEG 3.0f
#define PUSH_STOP_M 150.0f
#define ENG_LOST_MS 4000
#define PURSUE_MS 15000
#define PURSUE_MAX_M 350.0f
#define HIT_FACE_MS 4000
#define HIT_AGAIN_MS 20000
#define RETREAT_M 40.0f
#define RETREAT_MS 8000
#define RETREAT_COOL_MS 20000
#define INTERCEPT_HOLD_M 25.0f
/* review_5003 B1 / A3 / B6. */
#define CLOSE_M 200.0f
#define FIGHT_ROUTE_MS 10000
#define FIGHT_RETRY_MS 5000
#define FIGHT_LOCAL_M 45.0f
enum { FIGHT_ROUTE = 1, FIGHT_LOCAL };
#define CLOSE_LOST_MS 4000
#define FSTOP_MIN_M 100.0f
#define FSTOP_MAX_M 800.0f
#define FSTOP_SPEED 1.0f
#define FSTOP_COOL_MS 2000
#define FSTOP_LEAD_S 1.5f       /* no vertical stabiliser: brake this long before the round is in */
#define FSTOP_VONLY_M 250.0f    /* vertical stabiliser only: on the move below this */
#define FSTOP_LONG_M 500.0f     /* full stabiliser: on the move below this, short stops beyond */
/* B7 rear guard (2026-11-11: a bot showed its rear to the enemy with the
 * gun pointing backwards): engaged, the enemy in own sight (or a hit in the
 * last REAR_HIT_MS) under REAR_M and more than REAR_START_DEG off the nose (or
 * the gun more than that off the nose) -> stop, no lane, pivot until within
 * REAR_DONE_DEG, within the vehicle controller's engagement budget. */
#define REAR_M 1000.0f
#define REAR_START_DEG 120.0f
#define REAR_DONE_DEG 25.0f
#define REAR_COOL_MS 8000
static int rear_logs;
#define STICKY_NEAR 0.7f        /* a new target must be 30 % nearer ... */
#define STICKY_LOST_MS 2000     /* ... or the old one out of sight this long */
static FILE *gunner_csv;


static int fight_progress(void *host, lane_seat_t *s, int seat, uint64_t ms, const char *reason);
static void threat_hit(int army, float x, float z);     /* ground_threat.inc: an own tank was hit there */

/* The tank nearest a point, and (stage 3) one crossing a stretch under fire:
 * no fire stop beyond close range, no cover hop, no pursuit, no backing off. */
static int seat_rush(const lane_seat_t *s)
{
    return s->pusher || (s->ap_state == AP_DASH && s->ap_rush);
}

/* Is the main gun laid on (tx, tz)? Turret record yaw (gh_turret_yaw, see preaim)
 * against the bearing; unknown until the yaw sign has been learnt. */
static int turret_on(u8 *unit, const lane_seat_t *s, float tx, float tz)
{
    u8 *turrets = gh_unit_turrets(unit);
    u8 *rec = turrets ? gh_turrets_records(turrets) : NULL;
    if (!rec || !yaw_sign || !isfinite(gh_turret_yaw(rec)))
        return 0;
    float rel = wrap180((atan2f(tz - s->z, tx - s->x) - atan2f(s->fz, s->fx)) * 57.29578f);
    return fabsf(wrap180(gh_turret_yaw(rec) - (float)yaw_sign * rel)) < TURRET_ON_DEG;
}

static void react_update(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms)
{
    unsigned slot = (unsigned)seat & 63;
    s->face = 0;
    if (!cfg.react) {
        s->mode = M_MOVE;
        s->stopping = 0;
        return;
    }
    /* 1. What it is fighting. */
    if (ms >= s->gt_next) {
        s->gt_next = ms + 200;
        s->gt_slot = gunner_target(host, unit, s->army);
    }
    int tgt = -1, by_gunner = 0, own = 0, kept = 0;
    float tx = 0, tz = 0, td = 0, kx, kz, kage;
    if (cfg.takeover) {
        /* s28: keep the present target; re-evaluate once on a new own sighting, a hit or the target lost
         * (ground_tactics.inc). The B6 stickiness below is that inertia's old form: skipped. */
        tgt = tac_pick_target(host, s, seat, ms, &tx, &tz, &by_gunner, &own);
        if (tgt >= 0)
            td = dist2d(s->x, s->z, tx, tz);
    } else if (s->gt_slot >= 0 && slot_alive[s->gt_slot] && slot_unit[s->gt_slot]) {
        const float *p = gh_unit_pos(slot_unit[s->gt_slot]);
        tgt = s->gt_slot;
        tx = p[0];
        tz = p[2];
        by_gunner = 1;
        own = 1;
    } else {
        float best = ENG_MAX_M;
        for (int k = 0; k < 64; ++k)
            if (contact_get(s->army, seat, k, ms, &kx, &kz, &kage) == 2 && dist2d(s->x, s->z, kx, kz) < best) {
                best = dist2d(s->x, s->z, kx, kz);
                tgt = k;
                tx = kx;
                tz = kz;
                own = 1;
            }
    }
    if (tgt >= 0) {
        td = dist2d(s->x, s->z, tx, tz);
        if (td > ENG_MAX_M)
            tgt = -1;
    }
    /* B6 (review_5003: 2.5 target switches a minute): keep the engaged target
     * unless the new one is STICKY_NEAR nearer or the old one has been out of
     * sight STICKY_LOST_MS. */
    if (!cfg.takeover && tgt >= 0 && s->mode == M_ENGAGE && s->eng >= 0 && tgt != s->eng && slot_alive[s->eng]
        && ms - s->eng_seen_ms < STICKY_LOST_MS) {
        int lvl = contact_get(s->army, seat, s->eng, ms, &kx, &kz, &kage);
        float ox = lvl ? kx : s->ex, oz = lvl ? kz : s->ez, od = dist2d(s->x, s->z, ox, oz);
        if (td >= STICKY_NEAR * od) {
            kept = 1;
            by_gunner = s->gt_slot == s->eng;
            own = by_gunner || lvl == 2;
            tgt = s->eng;
            tx = ox;
            tz = oz;
            td = od;
        }
    }

    /* 2. Hits. */
    if (hit_dir[slot].ms > s->last_hit_ms) {
        uint64_t prev = s->last_hit_ms;
        s->last_hit_ms = hit_dir[slot].ms;
        threat_hit(s->army, s->x, s->z);        /* the team remembers where it is being shot at */
        if (tgt < 0 && prev && s->last_hit_ms - prev < HIT_AGAIN_MS && ms >= s->retreat_cool_until && !seat_rush(s)) {
            float side = (seat & 1) ? 1.0f : -1.0f, dx = hit_dir[slot].dx, dz = hit_dir[slot].dz;
            s->rx = s->x - dx * RETREAT_M - dz * 15.0f * side;
            s->rz = s->z - dz * RETREAT_M + dx * 15.0f * side;
            s->retreat_until = ms + RETREAT_MS;
            s->retreat_cool_until = ms + RETREAT_COOL_MS;
            s->mode = M_RETREAT;
            s->stopping = 0;
            /* Cover (2026-11-11): the best cover cell COV_RET_MIN_M..COV_RET_MAX_M away,
             * hidden towards the hit bearing, none further towards it. */
            cover_t c;
            if (cover_find(seat, s->army, COV_RETREAT, s->x, s->z, s->x + dx * 500.0f, s->z + dz * 500.0f, COV_RET_MIN_M,
                           COV_RET_MAX_M, CVF_AWAY, &c)) {
                if (s->cov_use)
                    cover_end(host, s, seat, ms, "retreat");
                s->rx = c.x;
                s->rz = c.z;
                s->retreat_until = ms + 3000 + (uint64_t)(c.dist / COV_HOP_SPEED * 1000.0f);
                s->cov_use = COV_RETREAT;
                s->cov_state = 1;
                s->cov_x = c.x;
                s->cov_z = c.z;
                s->cov_q = c.q;
                s->cov_type = c.type;
                s->cov_dist = c.dist;
                s->cov_since = ms;
                cover_used(host, seat, COV_RETREAT, &c, cover_here(s->x, s->z, s->x + dx * 500.0f, s->z + dz * 500.0f),
                           s->x + dx * 500.0f, s->z + dz * 500.0f, s->x, s->z);
            }
            event(host, "react: seat %d hit twice by an unseen shooter (bearing %.0f), backs off to (%.0f,%.0f)%s",
                  seat, atan2f(dz, dx) * 57.29578f, s->rx, s->rz, s->cov_use == COV_RETREAT ? " (cover)" : "");
        } else if (tgt < 0) {
            event(host, "react: seat %d hit by an unseen shooter, bearing %.0f", seat,
                  atan2f(hit_dir[slot].dz, hit_dir[slot].dx) * 57.29578f);
        }
    }
    if (tgt < 0 && ms - hit_dir[slot].ms < HIT_FACE_MS && hit_dir[slot].ms) {
        s->face = 1;
        s->face_x = s->x + hit_dir[slot].dx * 100.0f;
        s->face_z = s->z + hit_dir[slot].dz * 100.0f;
    }

    /* 3. Engagement: short stop and hull facing; the destination is kept. */
    if (tgt >= 0) {
        if (s->mode != M_ENGAGE || s->eng != tgt || s->fight_life != slot_life[tgt]) {
            event(host, "react: seat %d engages slot %d at %.0f m (%s)", seat, tgt, td,
                  by_gunner ? "gunner's target" : "own sight");
            s->eng_switch_ms = ms;
        }
        if (s->eng != tgt || s->fight_life != slot_life[tgt]) {
            s->fight_life = slot_life[tgt];
            s->fight_since = s->fight_window_ms = ms;
            s->fight_effect_ms = s->fight_exhausted_ms = 0;
            s->fight_move = s->fight_last = s->fight_yield = 0;
            s->fight_retry_ms = 0;
            s->close = s->rear = s->stopping = 0;
        }
        if (s->mode == M_RETREAT)
            s->retreat_until = 0;
        s->mode = M_ENGAGE;
        s->eng = tgt;
        s->ex = tx;
        s->ez = tz;
        if (own)
            s->eng_seen_ms = ms;
        s->eng_own = own;
        (void)kept;
    } else if (s->mode == M_ENGAGE && ms - s->eng_seen_ms > ENG_LOST_MS) {
        s->stopping = 0;
        if (cfg.takeover && cfg.pursue && s->eng >= 0)
            /* s28: sight lost ends the fight; never a drive to the old coordinates (wait / peek: ground_tactics.inc) */
            event(host, "react: seat %d lost slot %d: no pursuit (takeover), back to its order", seat, s->eng);
        if (!cfg.takeover && cfg.pursue && !cmd_is_post(s) && !s->holding && !seat_rush(s) && !s->fight_yield && !fight_committed(s, ms)
            && s->eng >= 0 && slot_alive[s->eng]
            && dist2d(s->x, s->z, s->ex, s->ez) < PURSUE_MAX_M) {
            s->mode = M_PURSUE;
            s->pursue_until = ms + PURSUE_MS;
            event(host, "react: seat %d lost slot %d, pursues to (%.0f,%.0f)", seat, s->eng, s->ex, s->ez);
        } else {
            s->mode = M_MOVE;
        }
    }
    if (s->eng >= 0 && (!slot_alive[s->eng] || s->fight_life != slot_life[s->eng])) {
        s->fight_life = 0;
        s->fight_since = s->fight_effect_ms = s->fight_exhausted_ms = s->fight_window_ms = 0;
        s->fight_move = s->fight_last = s->fight_yield = 0;
    }
    if (s->mode == M_ENGAGE) {
        float d = dist2d(s->x, s->z, s->ex, s->ez);
        int seen_now = s->eng_own && ms - s->eng_seen_ms < 1000;
        shot_opportunity_t shot = shot_opportunity(seat, s, s->eng, ms);
        float ready = seat_ready_in(seat, ms);
        if (shot.effective_ms >= s->fight_since && shot.effective_ms > s->fight_effect_ms) {
            s->fight_effect_ms = shot.effective_ms;
            s->fight_exhausted_ms = 0;
        }
        uint64_t useful_since = s->fight_effect_ms > s->fight_window_ms ? s->fight_effect_ms : s->fight_window_ms;
        if (shot.aim_ms && shot.aim_ms >= s->fight_since && seen_now && useful_since
            && ms - useful_since >= FIGHT_STALE_MS && ms >= shot.flight_until
            && (!shot.fired_ms || ms - shot.fired_ms >= 1500) && (ready < 0.3f || ready > 90.0f)) {
            s->fight_exhausted_ms = ms;
            s->fight_window_ms = ms;
        }
        s->post_defending = cmd_is_post(s) && cmd_post_self_defense(s, ms, d, seen_now);
        if (cmd_is_post(s) && !s->post_defending) {
            s->close = s->stopping = s->rear = s->fight_move = s->fight_yield = 0;
            if (s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE)
                cover_end(host, s, seat, ms, "return to gate post");
            if (s->cmd_parked && seen_now && cmd_post_in_arc(s, s->ex, s->ez)) {
                s->face = 1;
                s->face_x = s->ex;
                s->face_z = s->ez;
            }
            goto engagement_done;
        }
        int immediate = fight_immediate(s, ms, d, seen_now);
        int low_return = seen_now && ((shot.reposition && ms >= shot.flight_until) || ready > 90.0f
                                     || (s->fight_exhausted_ms && ms - s->fight_exhausted_ms < FIGHT_ROUTE_MS));
        const lane_t *zl = &lanes[s->lane];
        int in_zone = dist2d(s->x, s->z, zl->zx, zl->zz) < zl->zr;
        int part_advance = seen_now && (shot.opportunity == SHOT_FIRE || shot.opportunity == SHOT_MOBILITY)
                           && d > PUSH_STOP_M && !in_zone && (s->pusher || s->cmd_release);
        if (immediate && s->fight_move) {
            event(host, "react: seat %d interrupts movement for immediate threat", seat);
            s->fight_move = 0;
            s->fight_retry_ms = ms + FIGHT_RETRY_MS;
        }
        if (!immediate && low_return && (s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE) && s->cov_state == 2)
            cover_end(host, s, seat, ms, "low-return engagement");
        if (!cmd_is_post(s) && !immediate && (low_return || part_advance))
            fight_progress(host, s, seat, ms, shot.precision_blocked ? "precision gate"
                           : shot.opportunity == SHOT_NONE ? "no shot opportunity"
                           : part_advance && !low_return ? "advancing with a subsystem shot" : "no effective result");
        s->fight_yield = !immediate && (low_return || fight_committed(s, ms));
        if (s->fight_yield) {
            if (s->close)
                event(host, "react: seat %d releases close fight with slot %d to move", seat, s->close_slot);
            s->close = s->stopping = s->rear = 0;
        }
        /* Face a close threat early. Continued engagement needs a productive
         * firing window, or fresh incoming fire at immediate range. */
        if (!s->fight_yield && cfg.close && seen_now && d < CLOSE_M && !(s->pusher && in_zone)
            && (ms - s->fight_since < FIGHT_CLOSE_MS || immediate)) {
            if ((!s->close || s->close_slot != s->eng) && ms - s->close_log_ms > 5000) {
                s->close_log_ms = ms;
                event(host, "react: seat %d close fight with slot %d at %.0f m: stops, hull to it", seat, s->eng, d);
            }
            s->close = 1;
            s->close_slot = s->eng;
            s->close_seen_ms = ms;
        }
        /* B7 rear guard (see REAR_*). */
        if (!s->fight_yield && cfg.rear && s->fx * s->fx + s->fz * s->fz > 0.25f && d > 1.0f) {
            float c = ((s->ex - s->x) * s->fx + (s->ez - s->z) * s->fz) / d;
            float err = acosf(c > 1.0f ? 1.0f : c < -1.0f ? -1.0f : c) * 57.29578f;
            u8 *turrets = gh_unit_turrets(unit);
            u8 *rec = turrets ? gh_turrets_records(turrets) : NULL;
            float gun_off = rec && isfinite(gh_turret_yaw(rec)) ? fabsf(wrap180(gh_turret_yaw(rec))) : 0.0f;
            int threat = seen_now || (hit_dir[slot].ms && ms - hit_dir[slot].ms < REAR_HIT_MS);
            if (!s->rear && threat && d < REAR_M && ms >= s->rear_cool_until && !(s->pusher && in_zone)
                && (err > REAR_START_DEG || (gun_off > REAR_START_DEG && err > 60.0f))) {
                s->rear = 1;
                s->rear_since = ms;
                if (rear_logs++ < 80)
                    event(host, "react: seat %d fight with slot %d at %.0f m is %.0f deg off the nose (gun %.0f deg): "
                          "stops and turns the hull", seat, s->eng, d, err, gun_off);
            } else if (s->rear && (err < REAR_DONE_DEG || ms - s->eng_seen_ms > CLOSE_LOST_MS)) {
                if (rear_logs++ < 80)
                    event(host, "react: seat %d rear turn over after %.1f s: %.0f deg off the nose%s", seat,
                          (double)(ms - s->rear_since) / 1000.0, err, err < REAR_DONE_DEG ? "" : " (gave up)");
                s->rear = 0;
                s->rear_cool_until = ms + REAR_COOL_MS;
            }
        }
        int cover_active = (s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE) && s->cov_state == 1;
        if (s->cmd_release && !immediate && (s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE) && s->cov_state == 2)
            cover_end(host, s, seat, ms, "needed for point pressure");
        if ((cover_active || (!s->fight_yield && (!s->cmd_release || immediate)))
            && cover_engage(host, s, seat, ms, d, seen_now)) {
            /* cover (2026-11-11): drive into the cell first, then stop in it to fire */
            s->stopping = s->cov_state == 2;
        } else if (s->fight_yield || (s->cmd_release && !immediate)) {
            s->stopping = 0;
        } else if (cfg.shortstop == 2) {
            /* Eligibility for the range-based fire-stop request. */
            int in_range = d >= ENG_MIN_M && d <= ENG_STOP_MAX_M;
            int worth = d < ENG_STOP_NEAR_M || (hit_dir[slot].ms && ms - hit_dir[slot].ms < ENG_HIT_MS)
                        || (tgt >= 0 && by_gunner && turret_on(unit, s, s->ex, s->ez));
            if (seat_rush(s) && d >= PUSH_STOP_M)
                worth = 0;
            if (!s->stopping && in_range && worth && ms >= s->stop_cool_until)
                s->stopping = 1;
            else if (s->stopping && !in_range)
                s->stopping = 0;
        } else if (cfg.shortstop == 1) {
            /* A3 fire stop: loaded, target in sight, FSTOP_MIN_M..FSTOP_MAX_M, moving
             * -> request a stop until the shot, stale sight, or a control decision.
             * Design rule: no vertical stabiliser -> do not fire on the move:
             * with movefire on, the band depends on what the engine stabilises
             * now (seat_movefire): none -> every shot from a stop (0..ENG_MAX_M,
             * requesting a stop from FSTOP_LEAD_S before the round is in); vertical only -> on the move under FSTOP_VONLY_M; full
             * -> on the move under FSTOP_LONG_M, short stops beyond it. */
            float speed = sqrtf(s->vx * s->vx + s->vz * s->vz);
            int mf = cfg.movefire ? seat_movefire(unit) : -1;
            float lo = FSTOP_MIN_M, hi = FSTOP_MAX_M, lead = 0.0f;
            if (mf == 0) {
                lo = 0.0f;
                hi = ENG_MAX_M;
                lead = FSTOP_LEAD_S;
            } else if (mf > 0) {
                lo = mf == 2 ? FSTOP_LONG_M : FSTOP_VONLY_M;
                hi = ENG_MAX_M;
            }
            float push_m = mf == 0 ? ENG_STOP_NEAR_M : PUSH_STOP_M;
            if (!s->stopping) {
                if (seen_now && seat_ready_in(seat, ms) <= lead && d >= lo && d <= hi
                    && speed > FSTOP_SPEED && ms >= s->stop_cool_until && (!seat_rush(s) || d < push_m)) {
                    s->stopping = 1;
                    s->stop_shots = shot_seat[seat].total;
                }
            } else if (shot_seat[seat].total != s->stop_shots || d > hi
                       || ms - s->eng_seen_ms > 1500) {
                s->stopping = 0;
                s->stop_cool_until = ms + FSTOP_COOL_MS;
            }
        } else {
            s->stopping = 0;
        }
        s->face = 1;
        s->face_x = s->ex;
        s->face_z = s->ez;
    } else {
        s->post_defending = 0;
        s->fight_yield = 0;
        s->stopping = 0;
        if (s->rear) {
            s->rear = 0;
            s->rear_cool_until = ms + REAR_COOL_MS;
        }
    }
engagement_done:
    if (s->close && (s->mode != M_ENGAGE || !slot_alive[s->close_slot & 63] || ms - s->close_seen_ms > CLOSE_LOST_MS
        || (s->fight_since && ms - s->fight_since >= FIGHT_CLOSE_MS
            && !fight_immediate(s, ms, dist2d(s->x, s->z, s->ex, s->ez), s->eng_own)))) {
        s->close = 0;
        s->cov_close_tried = 0;
        event(host, "react: seat %d close fight with slot %d over (%s)", seat, s->close_slot,
              !slot_alive[s->close_slot & 63] ? "dead" : ms - s->close_seen_ms > CLOSE_LOST_MS ? "out of sight" : "window over");
    }
    if (s->cov_use && (s->cov_use == COV_RETREAT ? s->mode != M_RETREAT : s->mode != M_ENGAGE))
        cover_end(host, s, seat, ms, s->mode == M_ENGAGE ? "engaged" : "fight over");

    /* gunner.csv: 1 Hz per seat. */
    if (ms >= s->gunner_log_next && cfg.log_dir[0]) {
        s->gunner_log_next = ms + 1000;
        if (!gunner_csv)
            gunner_csv = open_csv("gunner.csv", "t,seat,mode,gunner_slot,eng_slot,eng_dist,stopping,holding,task_slot,hit_age_s,x,z,"
                                                "close,eng_own,ready_s,pivoting,hull_rate,rear,creep,hull_deg,"
                                                "cover,cov_state,cov_dist,cov_q,cov_type,dcov,dcov_q,dcov_type,"
                                                "ready_state,takeover,tac,tac_clue,hint_band,hint_sector,face_state,face_off_deg,intel_grade,intel_slot");
        if (gunner_csv) {
            int dcov = s->cov_dest_ok && ms - s->cov_dest_seen < 2000;
            char tac_cols[160];
            tac_gunner_cols(s, seat, ms, tac_cols, sizeof tac_cols);
            fprintf(gunner_csv, "%.1f,%d,%s,%d,%d,%.0f,%d,%d,%d,%.1f,%.0f,%.0f,%d,%d,%.1f,%d,%.1f,%d,%d,%.1f,%s,%d,%.0f,%d,%d,%d,%d,%d,%s\n",
                    (double)(ms - t0) / 1000.0,
                    seat, mode_names[s->mode], s->gt_slot, s->mode == M_ENGAGE ? s->eng : -1,
                    s->mode == M_ENGAGE ? dist2d(s->x, s->z, s->ex, s->ez) : 0.0f, s->stopping, s->holding,
                    s->task ? s->task_tgt : -1,
                    hit_dir[slot].ms ? (double)(ms - hit_dir[slot].ms) / 1000.0 : -1.0, s->x, s->z, s->close, s->eng_own,
                    seat_ready_in(seat, ms), s->pivoting, s->hull_rate, s->rear, s->pivoting && s->pivot_creep,
                    atan2f(s->fz, s->fx) * 57.29578f, cov_name[s->cov_use < COV_NUSE ? s->cov_use : 0], s->cov_state,
                    s->cov_use ? s->cov_dist : 0.0f, s->cov_use ? s->cov_q : 0, s->cov_use ? s->cov_type : 0, dcov,
                    dcov ? s->cov_dest_q : 0, dcov ? s->cov_dest_type : 0, tac_cols);
            fflush(gunner_csv);
        }
    }

}

/* Fire-stop and facing proposals are consumed by the vehicle controller.
 * These goals describe the separate moving reactions and tactical positions. */
static int react_propose(void *host, lane_seat_t *s, int seat, uint64_t ms, ground_engagement_policy_t policy)
{
    if (!cfg.react || ms < s->l1_until)
        return 0;
    if (cfg.threatface && tac_face_propose(s, seat))
        return 1;                       /* s29: the threat facing's move to cover holds the hull */
    if ((s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE) && s->mode == M_ENGAGE
        && s->cov_state == 1 && (policy.allowed & GFM_RETURN_COVER)) {
        nav_propose(&s->nav, NAV_REACTION, s->cov_x, s->cov_z, NAV_NATIVE, 1, 0, 0);
        return 1;
    }
    if (s->mode == M_RETREAT) {
        int cov = s->cov_use == COV_RETREAT;
        if (ms < s->retreat_until && dist2d(s->x, s->z, s->rx, s->rz) > (cov ? COV_ARRIVE_M : 8.0f)) {
            nav_propose(&s->nav, NAV_REACTION, s->rx, s->rz, NAV_NATIVE, 1, 0, 0);
            return 1;
        }
        if (cov && ms < s->retreat_until) {
            if (s->cov_state == 1) {
                s->cov_state = 2;
                s->cov_since = ms;
                s->retreat_until = ms + COV_RET_STAY_MS;
            }
            nav_propose(&s->nav, NAV_REACTION, s->rx, s->rz, NAV_PARK, 0, 1, 0);
            return 1;
        }
        s->mode = M_MOVE;
        if (cov)
            cover_end(host, s, seat, ms, "retreat over");
    }
    if (s->mode == M_PURSUE && (policy.allowed & GFM_ADVANCE)) {
        if (ms < s->pursue_until && dist2d(s->x, s->z, s->ex, s->ez) > 30.0f && s->eng >= 0 && slot_alive[s->eng]) {
            nav_propose(&s->nav, NAV_REACTION, s->ex, s->ez, NAV_NATIVE, 1, 0, 0);
            return 1;
        }
        s->mode = M_MOVE;
        event(host, "react: seat %d ends the pursuit", seat);
    }
    if (cfg.waitpeek && tac_propose(host, s, seat, ms, policy))
        return 1;                       /* s28: the reaction record's wait / peek holds the hull */
    if (s->task && !cmd_is_post(s) && !s->cmd_release && !fight_committed(s, ms)) {
        if (s->mode == M_MOVE)
            s->mode = M_INTERCEPT;
        const int moving = dist2d(s->x, s->z, s->task_x, s->task_z) > INTERCEPT_HOLD_M;
        nav_propose(&s->nav, NAV_REACTION, s->task_x, s->task_z, moving ? NAV_NATIVE : NAV_PARK,
                    moving, !moving, 0);
        float kx, kz, kage;
        if (!moving && !s->face && contact_get(s->army, seat, s->task_tgt, ms, &kx, &kz, &kage)) {
            s->face = 1;
            s->face_x = kx;
            s->face_z = kz;
        }
        return 1;
    }
    if (s->mode == M_INTERCEPT)
        s->mode = M_MOVE;
    return 0;
}

/* s48: the team planner that was here (offensive share census, team_census, task_point, team_rear_class,
 * team_plan) is in ground_command.c (ground_command_team.inc). */

/* ---- Strategy layer S1 (2026-11-10; the strategy design) ----
 * Summarise public point ownership and team knowledge for the commander.
 * build_strategy.py writes bot_trees/strategy/<mission>.bin (layout in its
 * docstring), ground_brain.py passes it as strategy_file; it is mapped
 * read-only. Every STRAT_MS per army (army 2 half a period later), from team
 * knowledge only (own tanks, contact_get, hit bearings; public: alive counts,
 * point owners), on the 32 m grid:
 *   OWN = sum over own tanks P * kappa * w(d) * V
 *   ENK = known enemies: conf exp(-age / 30) * P, uncertainty r = min(20 + 6 age,
 *         250): the centre 0.34 + 6 ring points (0.7 r) 0.11 each when r >= 40.
 *         A hit bearing with no known enemy in its +-12 deg, 100-900 m wedge:
 *         3 points on it, mass 0.6 in all, fading out over 15 s.
 *   ENH = (enemies alive - known) * STRAT_HID spread over the cells our
 *         spotting lines (<= 600 m) missed for 10 s, weighted exp(-d / 500)
 *         from their spawn + 0.8 * a 160 m gaussian on every point not ours,
 *         blurred 3 times.
 *   w(d) = 1 to 200 m, linear to 0.25 at 900 m (SPAA: 600 m), 0 beyond (cell
 *   centre distances); V = 1 with a fire line, 0.5 under 70 m without, else 0.
 * Bands: lateral strips between the points (edges in the file). Front per band:
 * 1-D max cut of OWN - EN over 64 m depth bins (+0.15 towards half way),
 * capped 200 m ahead of the band's deepest tank with kappa >= 1 (L/2 when it
 * has none), EMA 0.5. Band forces: f_own = P * kappa of own tanks deeper than
 * front - 300; f_en = known enemies shallower than front + 450, hid = hidden
 * mass in (front, front + 450) / STRAT_HID * 0.6; ratio (f_own + 0.5) /
 * (f_en + hid + 0.5); gaps (no own force, enemy >= 0.8) and salients (front
 * 250 m past / short of both neighbours).
 * Per own tank: P = class (MBT / heavy 1, fast 0.8, TD 0.9, SPAA 0.5) x hits
 * taken in 60 s (-0.15 each, >= 0.5) x empty gun 0.3 x human 1.3; isolation
 * class (group: >= 2 team-mates within 250 m; rear: not 150 m deeper than the
 * team median; else lone / exploit / endangered by value vs threat, see the
 * design) with 4 s hysteresis and kappa 1 / 1 / 0.5 / min(1 + 0.15 hits +
 * 0.2 F, 1.8) / 0.2; exploit without a hit or kill for 45 s drops to lone.
 * Default role (logged, not used yet): fast (>= 1.2 x team median hp/t) flank,
 * MBT / heavy breach, TD overwatch, SPAA escort. An enemy tank engaged by two
 * of our bots from bearings > 60 deg apart counts P x 0.6.
 * Logs: `strat:` per army every STRAT_LOG_MS (bands, isolation classes,
 * roles), one timing line a minute, strat_snap.csv every STRAT_LOG_MS (the
 * inputs and band results, for strategy_replay.py). fault-guarded: 3 faults
 * switch it off. */

#define STRAT_MS 1000
#define STRAT_LOG_MS 10000
#define STRAT_DBIN_M 64.0f
#define STRAT_MAXBINS 160
#define STRAT_HID 40.0f
#define STRAT_SEEN_M 600.0f
#define STRAT_SEEN_S 10u
#define STRAT_SUPPORT_M 250.0f
#define STRAT_FWD_M 150.0f
#define STRAT_FLANK_M 700.0f
#define STRAT_HYST_MS 4000
#define STRAT_EXPLOIT_MS 45000
#define STRAT_WEDGE_MS 15000
#define FLANK_GRID_CELLS 24576
enum { SC_GROUP, SC_REAR, SC_LONE, SC_EXPLOIT, SC_ENDANGERED, SC_N };
static const char *const strat_cls_name[SC_N] = {"grp", "rear", "lone", "expl", "endg"};
static const char *const strat_role_name[SR_N] = {"flank", "breach", "ow", "escort"};


static struct { u8 *unit; int cls, cand; uint64_t cand_ms; } strat_unit[64];

static double strat_now_us(void)
{
    long long c;
    if (strat.qpc_us == 0.0) {
        long long f;
        gh_qpc_frequency(&f);
        strat.qpc_us = 1e6 / (double)f;
    }
    gh_qpc(&c);
    return (double)c * strat.qpc_us;
}

static const u8 *strat_section(const char *tag, uint32_t want, size_t file_size)
{
    uint32_t n = (*(uint32_t *)((u8 *)(strat.base) + (12)));
    for (uint32_t i = 0; i < n && i < 64; ++i) {
        const u8 *d = strat.base + 80 + 16 * i;
        if (strncmp((const char *)d, tag, 8))
            continue;
        uint32_t off = (*(uint32_t *)((u8 *)(d) + (8))), size = (*(uint32_t *)((u8 *)(d) + (12)));
        if ((uint64_t)off + size > file_size || (want && size != want))
            return NULL;
        return strat.base + off;
    }
    return NULL;
}

static void strat_off(void *host, const char *why)
{
    strat.state = 2;
    event(host, "strat: off (%s)", why);
}

/* Maps the file and builds the lookup tables. Returns 1 when on. */
static int strat_load(void *host)
{
    if (strat.state)
        return strat.state == 1;
    strat.state = 2;
    if (!cfg.strategy_file[0])
        return 0;
    strat.file = gh_file_open(cfg.strategy_file);
    if (gh_file_failed(strat.file)) {
        strat_off(host, "cannot open the strategy file");
        return 0;
    }
    long long fs;
    gh_file_size(strat.file, &fs);
    strat.map = gh_file_map(strat.file);
    strat.base = strat.map ? (const u8 *)gh_file_view(strat.map) : NULL;
    size_t size = (size_t)fs;
    if (!strat.base || size < 80 || memcmp(strat.base, "WTSTR001", 8) || (*(uint32_t *)((u8 *)(strat.base) + (8))) != 1) {
        strat_off(host, "not a version 1 strategy file");
        return 0;
    }
    const strat_meta_t *m = (const strat_meta_t *)strat_section("META", sizeof(strat_meta_t), size);
    if (!m || m->g32_nx < 4 || m->g32_nz < 4 || m->g32_nx * m->g32_nz > 40000 || m->nbands < 1
        || m->nbands > STRAT_MAXB || m->ncap > 8 || m->vis_w < 4 || m->vis_w > 31
        || m->vis_row != (2 * m->vis_w + 1) * 8 || m->L < 100.0f
        || m->L / STRAT_DBIN_M + 2 > STRAT_MAXBINS) {
        strat_off(host, "bad META");
        return 0;
    }
    strat.m = m;
    strat.nx = (int)m->g32_nx;
    strat.nz = (int)m->g32_nz;
    strat.n = strat.nx * strat.nz;
    strat.W = (int)m->vis_w;
    strat.S = 2 * strat.W + 1;
    strat.nb = (int)m->nbands;
    strat.nbins = (int)(m->L / STRAT_DBIN_M) + 2;
    strat.cell = (const strat_cell_t *)strat_section("G32", (uint32_t)(strat.n * sizeof(strat_cell_t)), size);
    strat.visf = strat_section("VISF", (uint32_t)strat.n * m->vis_row, size);
    strat.viss = strat_section("VISS", (uint32_t)strat.n * m->vis_row, size);
    if (!strat.cell || !strat.visf || !strat.viss || sizeof(strat_cell_t) != 28) {
        strat_off(host, "missing G32 / VISF / VISS");
        return 0;
    }
    int n = strat.n, W = strat.W;
    strat.SP = (strat.S + 3) & ~3;
    strat.PX = strat.nx + 2 * W + strat.SP;
    strat.PZ = strat.nz + 2 * W;
    int nw = strat.S * strat.SP, np_ = strat.PX * strat.PZ;
    strat.wt[0] = (float *)calloc((size_t)nw * 4, sizeof(float));
    strat.wseen = (uint64_t *)calloc((size_t)strat.S, sizeof(uint64_t));
    float *buf = (float *)calloc((size_t)n * 10 + (size_t)np_ * 2, sizeof(float));
    u8 *bins = (u8 *)calloc((size_t)n * 2, 1);
    uint32_t *seen = (uint32_t *)calloc((size_t)n * 2, sizeof(uint32_t));
    if (!strat.wt[0] || !strat.wseen || !buf || !bins || !seen) {
        strat_off(host, "out of memory");
        return 0;
    }
    strat.wt[1] = strat.wt[0] + nw;
    strat.wn[0] = strat.wt[0] + 2 * nw;
    strat.wn[1] = strat.wt[0] + 3 * nw;
    for (int dr = -W; dr <= W; ++dr)
        for (int dc = -W; dc <= W; ++dc) {
            int i = (dr + W) * strat.SP + dc + W;
            float d = m->g32_cell * sqrtf((float)(dr * dr + dc * dc));
            for (int t = 0; t < 2; ++t) {
                float reach = t ? 600.0f : 900.0f;
                float w = d <= 200.0f ? 1.0f : d <= reach ? 1.0f - 0.75f * (d - 200.0f) / (reach - 200.0f) : 0.0f;
                strat.wt[t][i] = w;
                strat.wn[t][i] = d < 70.0f ? 0.5f * w : 0.0f;
            }
            if (d <= STRAT_SEEN_M)
                strat.wseen[dr + W] |= 1ull << (dc + W);
        }
    for (int i = 0; i < 16; ++i)
        strat.lut[i] = _mm_castsi128_ps(_mm_set_epi32(i & 8 ? -1 : 0, i & 4 ? -1 : 0, i & 2 ? -1 : 0, i & 1 ? -1 : 0));
    strat.hid = buf;
    strat.tmp = buf + n;
    strat.own = buf + 10 * n;
    strat.en = buf + 10 * n + np_;
    strat.prior[1] = buf + 2 * n;
    strat.prior[2] = buf + 3 * n;
    for (int k = 0; k < 6; ++k)
        strat.capg[k] = buf + (4 + k) * n;
    strat.dbin[1] = bins;
    strat.dbin[2] = bins + n;
    strat.seen[1] = seen;
    strat.seen[2] = seen + n;
    for (int i = 0; i < n; ++i) {
        const strat_cell_t *c = &strat.cell[i];
        for (int a = 1; a <= 2; ++a) {
            float d = a == 1 ? c->depth : m->L - c->depth;
            int b = (int)(d / STRAT_DBIN_M);
            strat.dbin[a][i] = (u8)(b < 0 ? 0 : b >= strat.nbins ? strat.nbins - 1 : b);
            float ex = a == 1 ? m->s2x : m->s1x, ez = a == 1 ? m->s2z : m->s1z;
            strat.prior[a][i] = expf(-dist2d(c->x, c->z, ex, ez) / 500.0f);
        }
        for (uint32_t k = 0; k < m->ncap && k < 6; ++k) {
            float d = dist2d(c->x, c->z, m->cap[k][0], m->cap[k][1]) / 160.0f;
            strat.capg[k][i] = expf(-d * d);
        }
    }
    /* S2 inputs (optional) and buffers */
    if (m->nsrc && m->nsrc < 64) {
        strat.src = strat_section("SRC", m->nsrc * 16, size);
        strat.travel = (const uint16_t *)strat_section("TRAVEL", m->nsrc * (uint32_t)n * 2, size);
    }
    for (int k = 0; k < 8; ++k) {
        strat.cap_src[k] = -1;
        for (uint32_t j = 0; strat.src && strat.travel && k < (int)m->ncap && j < m->nsrc; ++j)
            if ((*(uint32_t *)((u8 *)(strat.src) + (16 * j))) == 1
                && dist2d((*(float *)((u8 *)(strat.src) + (16 * j + 8))), (*(float *)((u8 *)(strat.src) + (16 * j + 12))), m->cap[k][0], m->cap[k][1]) < 40.0f)
                strat.cap_src[k] = (int)j;
    }
    if (m->ndir == 16 && m->nvband == 3 && m->g10_nx > 0 && m->g10_nz > 0 && m->g10_nx * m->g10_nz <= 1000000u) {
        uint32_t n10 = m->g10_nx * m->g10_nz;
        strat.g10 = strat_section("G10", n10 * 8, size);
        strat.g10hull = strat_section("G10HULL", n10 * 16, size);
        strat.g10view = strat_section("G10VIEW", n10 * 96, size);
        if (strat.g10 && strat.g10hull && strat.g10view) {
            strat.n10x = (int)m->g10_nx;
            strat.n10z = (int)m->g10_nz;
        } else {
            strat.g10 = NULL;
        }
    }
    int path_n = strat.g10 && n < FLANK_GRID_CELLS ? FLANK_GRID_CELLS : n;
    strat.apath_cap = path_n;
    strat.enc[1] = (float *)calloc((size_t)n * 2 + (size_t)path_n * 2, sizeof(float));
    strat.apar = (int *)calloc((size_t)path_n * 2, sizeof(int));
    strat.astamp = (uint32_t *)calloc((size_t)path_n, sizeof(uint32_t));
    strat.aheap_cap = n * 8 > 32768 ? n * 8 : 32768;
    strat.aheap = calloc((size_t)strat.aheap_cap, sizeof *strat.aheap);
    if (!strat.enc[1] || !strat.apar || !strat.astamp || !strat.aheap) {
        strat_off(host, "out of memory (S2)");
        return 0;
    }
    strat.enc[2] = strat.enc[1] + n;
    strat.ag = strat.enc[1] + 2 * n;
    strat.acost = strat.ag + path_n;
    strat.apath = strat.apar + path_n;
    strat.state = 1;
    event(host, "strat: loaded %s: G32 %dx%d, %d bands, %u points, L %.0f m, window %d; travel %s, G10 %dx%d", cfg.strategy_file,
          strat.nx, strat.nz, strat.nb, m->ncap, m->L, strat.W, strat.travel ? "yes" : "no", strat.n10x, strat.n10z);
    return 1;
}



/* Index of cell i in the padded own / en grids. */
static int strat_pad(int i)
{
    return (i / strat.nx + strat.W) * strat.PX + i % strat.nx + strat.W;
}

/* acc (padded grid) += mass * w(d) * V around cell src; 4 cells per step,
 * no bounds checks (the padding takes the window's overhang). */
static void strat_stamp(float *acc, int src, float mass, int spaa)
{
    if (src < 0 || mass <= 0.0f)
        return;
    int r0 = src / strat.nx, c0 = src % strat.nx;
    const uint64_t *row = (const uint64_t *)(strat.visf + (size_t)src * strat.m->vis_row);
    const __m128 m4 = _mm_set1_ps(mass);
    for (int k = 0; k < strat.S; ++k) {         /* window row k = dr + W is padded row r0 + k */
        float *a = acc + (size_t)(r0 + k) * strat.PX + c0;     /* padded column of c0 - W */
        const float *w = strat.wt[spaa] + k * strat.SP, *wn = strat.wn[spaa] + k * strat.SP;
        uint64_t bits = row[k];
        for (int j = 0; j < strat.SP; j += 4, bits >>= 4) {
            __m128 mk = strat.lut[bits & 15];
            __m128 v = _mm_or_ps(_mm_and_ps(mk, _mm_loadu_ps(w + j)), _mm_andnot_ps(mk, _mm_loadu_ps(wn + j)));
            _mm_storeu_ps(a + j, _mm_add_ps(_mm_loadu_ps(a + j), _mm_mul_ps(m4, v)));
        }
    }
}

typedef struct {
    int slot, cell, band, cls, role, human;
    float x, z, depth, lat, p, kappa, flank, threat;
    int hits, kills;
} strat_own_t;
typedef struct { int slot, cell, band, spaa; float x, z, age, conf, p, depth; } strat_en_t;


#include "ground_threat.inc"


static void strat_snap_row(double t, int army, char kind, int id, float x, float z, float v1, float v2, float v3, float v4)
{
    if (strat.snap)
        fprintf(strat.snap, "%.1f,%d,%c,%d,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f\n", t, army, kind, id, x, z, v1, v2, v3, v4);
}

static void strat_update(void *host, int army, uint64_t ms)
{
    const strat_meta_t *m = strat.m;
    const int n = strat.n, enemy = 3 - army;
    double t_us = strat_now_us();
    uint32_t now_s = (uint32_t)((ms - t0) / 1000) + 1;
    strat_own_t own[64];
    strat_en_t en[64];
    int no = 0, ne = 0, en_alive = 0;
    float cx = 0, cz = 0, hp[64];
    int nhp = 0;
    /* own tanks (positions are the team's own knowledge) */
    for (int s = 0; s < 64; ++s) {
        if (slot_alive[s] && slot_army[s] == enemy)
            ++en_alive;
        if (!slot_alive[s] || slot_army[s] != army || !slot_unit[s])
            continue;
        u8 *u = slot_unit[s];
        const float *p = gh_unit_pos(u);
        u8 *pl = gh_unit_player(u);
        strat_own_t *o = &own[no++];
        memset(o, 0, sizeof *o);
        o->slot = s;
        o->x = p[0];
        o->z = p[2];
        o->human = pl && !gh_player_bot(pl);
        o->cell = strat_cellof(o->x, o->z);
        float d1 = (o->x - m->s1x) * m->axx + (o->z - m->s1z) * m->axz;
        o->depth = army == 1 ? d1 : m->L - d1;
        o->lat = (o->x - m->s1x) * m->latx + (o->z - m->s1z) * m->latz;
        o->band = strat_band(o->lat);
        cx += o->x;
        cz += o->z;
        if (s < MAX_SEATS && seat_hp[s] > 0.0f)
            hp[nhp++] = seat_hp[s];
    }
    if (!no)
        return;
    cx /= (float)no;
    cz /= (float)no;
    for (int i = 1; i < nhp; ++i)       /* insertion sorts, <= 32 entries */
        for (int j = i; j > 0 && hp[j - 1] > hp[j]; --j) {
            float tmp = hp[j]; hp[j] = hp[j - 1]; hp[j - 1] = tmp;
        }
    float hp_med = nhp ? hp[nhp / 2] : 0.0f, dmed;
    {
        float dd[64];
        for (int i = 0; i < no; ++i)
            dd[i] = own[i].depth;
        for (int i = 1; i < no; ++i)
            for (int j = i; j > 0 && dd[j - 1] > dd[j]; --j) {
                float tmp = dd[j]; dd[j] = dd[j - 1]; dd[j - 1] = tmp;
            }
        dmed = dd[no / 2];
    }
    /* P, role */
    for (int i = 0; i < no; ++i) {
        strat_own_t *o = &own[i];
        int s = o->slot;
        float p = strat_class(s, hp_med, &o->role);
        float st = 1.0f - 0.15f * (float)ring_count(hits_taken_ms[s], ms, 60000);
        p *= st < 0.5f ? 0.5f : st;
        gun_state_t gs;
        if (gun_read(host, slot_unit[s], 0, &gs) && gs.is_gun && gs.left <= 0)
            p *= 0.3f;
        if (o->human)
            p *= 1.3f;
        o->p = p;
        o->hits = ring_count(hits_dealt_ms[s], ms, 60000);
        o->kills = ring_count(kills_ms[s], ms, 120000);
    }
    /* known enemies */
    memset(strat.en, 0, (size_t)strat.PX * strat.PZ * sizeof(float));
    for (int t = 0; t < 64; ++t) {
        float x, z, age;
        if (!contact_get(army, 0, t, ms, &x, &z, &age))
            continue;
        strat_en_t *e = &en[ne++];
        e->slot = t;
        e->x = x;
        e->z = z;
        e->age = age;
        e->conf = expf(-age / 30.0f);
        e->cell = strat_cellof(x, z);
        float lat = (x - m->s1x) * m->latx + (z - m->s1z) * m->latz, d1 = (x - m->s1x) * m->axx + (z - m->s1z) * m->axz;
        e->band = strat_band(lat);
        e->depth = army == 1 ? d1 : m->L - d1;
        int erole;
        e->spaa = strat_spaa(t);
        e->p = strat_class(t, 0.0f, &erole);
        if (t == human_slot)
            e->p *= 1.3f;
        /* engaged by two of our bots from bearings > 60 deg apart: x0.6 */
        float bx[MAX_SEATS], bz[MAX_SEATS];
        int nbr = 0, two = 0;
        for (int i = 1; i < MAX_SEATS; ++i) {
            const lane_seat_t *o = &lseat[i];
            if (!o->unit || o->army != army || ms - o->last_ms > 2000 || (o->eng != t && o->gt_slot != t))
                continue;
            float l = dist2d(o->x, o->z, x, z);
            if (l < 1.0f)
                continue;
            bx[nbr] = (o->x - x) / l;
            bz[nbr] = (o->z - z) / l;
            for (int j = 0; j < nbr && !two; ++j)
                two = bx[j] * bx[nbr] + bz[j] * bz[nbr] < 0.5f;
            ++nbr;
        }
        if (two)
            e->p *= 0.6f;
        float mass = e->conf * e->p, rr = 20.0f + 6.0f * age;
        rr = rr > 250.0f ? 250.0f : rr;
        if (rr < 40.0f) {
            strat_stamp(strat.en, e->cell, mass, e->spaa);
        } else {
            strat_stamp(strat.en, e->cell, 0.34f * mass, e->spaa);
            for (int k = 0; k < 6; ++k) {
                float a = (float)k * 1.0471976f;
                strat_stamp(strat.en, strat_cellof(x + 0.7f * rr * cosf(a), z + 0.7f * rr * sinf(a)), 0.11f * mass, e->spaa);
            }
        }
    }
    /* hit bearings of our tanks with no known enemy in the wedge */
    int wedges = 0;
    for (int i = 0; i < no; ++i) {
        const hitdir_t *h = &hit_dir[own[i].slot];
        if (!h->ms || ms - h->ms > STRAT_WEDGE_MS)
            continue;
        int covered = 0;
        for (int k = 0; k < ne && !covered; ++k) {
            float ex = en[k].x - own[i].x, ez = en[k].z - own[i].z, d = sqrtf(ex * ex + ez * ez);
            covered = d >= 100.0f && d <= 900.0f && (ex * h->dx + ez * h->dz) > 0.978f * d;   /* cos 12 deg */
        }
        if (covered)
            continue;
        float mass = 0.6f / 3.0f * (1.0f - (float)(ms - h->ms) / (float)STRAT_WEDGE_MS);
        for (int k = 0; k < 3; ++k) {
            float d = 250.0f + 250.0f * (float)k;
            strat_stamp(strat.en, strat_cellof(own[i].x + h->dx * d, own[i].z + h->dz * d), mass, 0);
        }
        ++wedges;
    }
    /* spotting cover of the last 10 s */
    uint32_t *seen = strat.seen[army];
    for (int i = 0; i < no; ++i) {
        int src = own[i].cell;
        if (src < 0)
            continue;
        const uint64_t *row = (const uint64_t *)(strat.viss + (size_t)src * m->vis_row);
        int r0 = src / strat.nx, c0 = src % strat.nx, W = strat.W;
        uint64_t cols = ~0ull;                          /* window columns inside the grid */
        if (c0 < W)
            cols &= ~0ull << (W - c0);
        if (strat.nx - c0 + W < 64)
            cols &= (1ull << (strat.nx - c0 + W)) - 1;
        for (int k = 0; k < strat.S; ++k) {
            int r = r0 + k - W;
            if (r < 0 || r >= strat.nz)
                continue;
            uint64_t bits = row[k] & strat.wseen[k] & cols;
            int base = r * strat.nx + c0 - W;
            while (bits) {
                unsigned long j;
                _BitScanForward64(&j, bits);
                bits &= bits - 1;
                seen[base + (int)j] = now_s;
            }
        }
    }
    /* hidden enemies */
    int owner[8], our_points = 0, enemy_points = 0;
    for (uint32_t k = 0; k < m->ncap && k < 8; ++k) {
        owner[k] = zone_owner(host, m->cap[k][0], m->cap[k][1]);
        our_points += owner[k] == army;
        enemy_points += owner[k] == enemy;
    }
    int nh = en_alive - ne;
    nh = nh < 0 ? 0 : nh;
    float qsum = 0.0f;
    int unseen = 0;
    for (int i = 0; i < n; ++i) {
        float q = 0.0f;
        if ((strat.cell[i].flags & 1) && (!seen[i] || now_s - seen[i] > STRAT_SEEN_S)) {   /* 0: never seen */
            q = strat.prior[army][i];
            for (uint32_t k = 0; k < m->ncap && k < 6; ++k)
                if (owner[k] != army)
                    q += 0.8f * strat.capg[k][i];
            ++unseen;
        }
        strat.hid[i] = q;
        qsum += q;
    }
    float scale = qsum > 1e-9f ? (float)nh * STRAT_HID / qsum : 0.0f;
    for (int i = 0; i < n; ++i)
        strat.hid[i] *= scale;
    for (int pass = 0; pass < 3; ++pass) {
        float *h = strat.hid, *o2 = strat.tmp;
        for (int r = 0; r < strat.nz; ++r)
            for (int c = 0; c < strat.nx; ++c) {
                int i = r * strat.nx + c;
                float up = r > 0 ? h[i - strat.nx] : h[i], dn = r < strat.nz - 1 ? h[i + strat.nx] : h[i];
                float lf = c > 0 ? h[i - 1] : h[i], rt = c < strat.nx - 1 ? h[i + 1] : h[i];
                o2[i] = (up + dn + lf + rt + 4.0f * h[i]) * 0.125f;
            }
        strat.tmp = h;
        strat.hid = o2;
    }
    /* isolation class and kappa */
    for (int i = 0; i < no; ++i) {
        strat_own_t *o = &own[i];
        int mates = 0;
        for (int j = 0; j < no; ++j)
            mates += j != i && dist2d(o->x, o->z, own[j].x, own[j].z) < STRAT_SUPPORT_M;
        float threat = o->cell >= 0 ? strat.en[strat_pad(o->cell)] + strat.hid[o->cell] : 0.0f, fl = 0.0f;
        for (int k = 0; k < ne; ++k) {
            const strat_en_t *e = &en[k];
            float d = dist2d(e->x, e->z, o->x, o->z);
            if (d > STRAT_FLANK_M || o->cell < 0 || e->cell < 0 || !strat_sees(strat.visf, o->cell, e->cell))
                continue;
            float fx = cx - e->x, fz = cz - e->z, fl2 = sqrtf(fx * fx + fz * fz);
            float cosang = fl2 > 1.0f && d > 1.0f ? (fx * (o->x - e->x) + fz * (o->z - e->z)) / (fl2 * d) : 1.0f;
            fl += e->conf * (cosang > 0.7071f ? 0.0f : cosang > -0.342f ? 1.0f : 1.5f);  /* 45 / 110 deg */
        }
        o->flank = fl;
        o->threat = threat;
        int raw;
        if (mates >= 2)
            raw = SC_GROUP;
        else if (o->depth <= dmed + STRAT_FWD_M)
            raw = SC_REAR;
        else {
            float cover = o->cell >= 0 ? (float)strat.cell[o->cell].cover[army - 1] / 255.0f : 0.0f;
            float value = o->p * (0.4f + fl) * (1.0f + 0.25f * (float)o->hits + 0.5f * (float)o->kills) * (0.5f + 0.5f * cover);
            if ((o->hits || o->kills || fl >= 1.0f) && value >= 0.8f * threat)
                raw = SC_EXPLOIT;
            else if (threat > 1.2f && value < 0.8f * threat)
                raw = SC_ENDANGERED;
            else
                raw = SC_LONE;
        }
        uint64_t result = hits_dealt_ms[o->slot][0] > kills_ms[o->slot][0] ? hits_dealt_ms[o->slot][0] : kills_ms[o->slot][0];
        if (raw == SC_EXPLOIT && (!result || ms - result > STRAT_EXPLOIT_MS))
            raw = SC_LONE;
        if (strat_unit[o->slot].unit != slot_unit[o->slot]) {
            strat_unit[o->slot].unit = slot_unit[o->slot];
            strat_unit[o->slot].cls = strat_unit[o->slot].cand = raw;
            strat_unit[o->slot].cand_ms = ms;
        } else if (raw != strat_unit[o->slot].cand) {
            strat_unit[o->slot].cand = raw;
            strat_unit[o->slot].cand_ms = ms;
        } else if (raw != strat_unit[o->slot].cls && ms - strat_unit[o->slot].cand_ms >= STRAT_HYST_MS) {
            strat_unit[o->slot].cls = raw;
        }
        if (strat_unit[o->slot].cls == SC_EXPLOIT && raw == SC_LONE && (!result || ms - result > STRAT_EXPLOIT_MS))
            strat_unit[o->slot].cls = SC_LONE;     /* 45 s without a hit or kill: no waiting */
        o->cls = strat_unit[o->slot].cls;
        float k = o->cls == SC_EXPLOIT ? 1.0f + 0.15f * (float)o->hits + 0.2f * fl
                  : o->cls == SC_ENDANGERED ? 0.2f : o->cls == SC_LONE ? 0.5f : 1.0f;
        o->kappa = k > 1.8f ? 1.8f : k;
    }
    /* own influence */
    memset(strat.own, 0, (size_t)strat.PX * strat.PZ * sizeof(float));
    for (int i = 0; i < no; ++i)
        strat_stamp(strat.own, own[i].cell, own[i].p * own[i].kappa, strat_spaa(own[i].slot));
    /* fronts */
    static float prof[STRAT_MAXB][STRAT_MAXBINS];
    float hidb[STRAT_MAXB][STRAT_MAXBINS];
    memset(prof, 0, sizeof prof);
    memset(hidb, 0, sizeof hidb);
    for (int i = 0; i < n; ++i) {
        const strat_cell_t *c = &strat.cell[i];
        if (!(c->flags & 1) || c->band >= strat.nb)
            continue;
        int j = strat.dbin[army][i], pi = strat_pad(i);
        prof[c->band][j] += strat.own[pi] - strat.en[pi] - strat.hid[i];
        hidb[c->band][j] += strat.hid[i];
    }
    float braw[STRAT_MAXB], bfront[STRAT_MAXB], fown[STRAT_MAXB], fen[STRAT_MAXB], fhid[STRAT_MAXB], ratio[STRAT_MAXB];
    for (int b = 0; b < strat.nb; ++b) {
        float cum = 0.0f, best = -1e30f;
        int bj = 0;
        for (int j = 0; j < strat.nbins; ++j) {
            cum += prof[b][j] + ((float)j * STRAT_DBIN_M + 32.0f < 0.5f * m->L ? 0.15f : -0.15f);
            if (cum > best) {
                best = cum;
                bj = j;
            }
        }
        float fr = (float)(bj + 1) * STRAT_DBIN_M, held = -1e30f;
        for (int i = 0; i < no; ++i)
            if (own[i].band == b && own[i].kappa >= 1.0f && own[i].depth > held)
                held = own[i].depth;
        float cap = held > -1e29f ? held + 200.0f : 0.5f * m->L;
        fr = fr < cap ? fr : cap;
        braw[b] = fr;
        if (strat.front_ok[army])
            fr = 0.5f * strat.front[army][b] + 0.5f * fr;
        strat.front[army][b] = bfront[b] = fr;
        fown[b] = fen[b] = fhid[b] = 0.0f;
        for (int i = 0; i < no; ++i)
            if (own[i].band == b && own[i].depth > fr - 300.0f)
                fown[b] += own[i].p * own[i].kappa;
        for (int k = 0; k < ne; ++k)
            if (en[k].band == b && en[k].depth < fr + 450.0f)
                fen[b] += en[k].conf * en[k].p;
        for (int j = 0; j < strat.nbins; ++j) {
            float lo = (float)j * STRAT_DBIN_M;
            if (lo + STRAT_DBIN_M > fr && lo < fr + 450.0f)
                fhid[b] += hidb[b][j];
        }
        fhid[b] *= 0.6f / STRAT_HID;
        ratio[b] = (fown[b] + 0.5f) / (fen[b] + fhid[b] + 0.5f);
    }
    strat.front_ok[army] = 1;
    {   /* S2: keep this army's view for the commander */
        strat_view_t *v = &sview[army];
        v->point_deficit = enemy_points - our_points;
        if (v->point_deficit > 0) {
            if (!v->point_deficit_since)
                v->point_deficit_since = ms;
            float age = fminf(1.0f, (float)(ms - v->point_deficit_since) / 45000.0f);
            v->point_pressure = fminf(1.0f, 0.28f + 0.12f * v->point_deficit + 0.55f * age);
        } else {
            v->point_deficit_since = 0;
            v->point_pressure = 0.0f;
        }
        v->nb = strat.nb;
        for (int b = 0; b < strat.nb; ++b) {
            v->front[b] = bfront[b];
            v->ratio[b] = ratio[b];
            v->fown[b] = fown[b];
            v->fen[b] = fen[b];
            v->fhid[b] = fhid[b];
        }
        v->dmed = dmed;
        v->ms = ms;
        v->ne = ne;
        v->no = no;
        for (int k = 0; k < ne; ++k) {
            v->en[k].slot = en[k].slot;
            v->en[k].band = en[k].band;
            v->en[k].cell = en[k].cell;
            v->en[k].spaa = en[k].spaa;
            v->en[k].x = en[k].x;
            v->en[k].z = en[k].z;
            v->en[k].age = en[k].age;
            v->en[k].depth = en[k].depth;
        }
        for (int i = 0; i < no; ++i) {
            v->role[own[i].slot] = own[i].role;
            v->own[i].slot = own[i].slot;
            v->own[i].cell = own[i].cell;
            v->own[i].x = own[i].x;
            v->own[i].z = own[i].z;
            v->own[i].p = own[i].p;
        }
        float *enc = strat.enc[army];
        for (int i = 0; i < n; ++i)
            enc[i] = strat.en[strat_pad(i)] + strat.hid[i];
        threat_keep_known(army);        /* stage 3: EN is a buffer both armies share */
        v->ok = 1;
    }
    double us = strat_now_us() - t_us;
    strat.us_sum += us;
    strat.us_max = us > strat.us_max ? us : strat.us_max;
    ++strat.updates;
    if (ms < strat.log_ms[army])
        return;
    strat.log_ms[army] = ms + STRAT_LOG_MS;
    /* `strat:` line and the snapshot */
    char line[480];
    int len = snprintf(line, sizeof line, "strat: army %d known %d/%d hid %d wedge %d |", army, ne, en_alive, nh, wedges);
    char gaps[24] = "", sal[24] = "";
    for (int b = 0; b < strat.nb && len < (int)sizeof line - 40; ++b) {
        len += snprintf(line + len, sizeof line - (size_t)len, " b%d %.0f %.1f:%.1f+%.1f r%.2f", b, bfront[b], fown[b], fen[b],
                        fhid[b], ratio[b]);
        if (fown[b] < 0.5f && fen[b] + fhid[b] >= 0.8f)
            snprintf(gaps + strlen(gaps), sizeof gaps - strlen(gaps), "%d", b);
        float lo = b > 0 ? bfront[b - 1] : bfront[b + (strat.nb > 1)], hi = b < strat.nb - 1 ? bfront[b + 1] : bfront[b - (strat.nb > 1)];
        if (strat.nb > 1 && bfront[b] - (lo > hi ? lo : hi) > 250.0f)
            snprintf(sal + strlen(sal), sizeof sal - strlen(sal), "+%d", b);
        else if (strat.nb > 1 && (lo < hi ? lo : hi) - bfront[b] > 250.0f)
            snprintf(sal + strlen(sal), sizeof sal - strlen(sal), "-%d", b);
    }
    int cls[SC_N] = {0}, roles[SR_N] = {0};
    for (int i = 0; i < no; ++i) {
        ++cls[own[i].cls];
        ++roles[own[i].role];
    }
    snprintf(line + len, sizeof line - (size_t)len, " | gap %s sal %s | %s %d %s %d %s %d %s %d %s %d | %s %d %s %d %s %d %s %d",
             gaps[0] ? gaps : "-", sal[0] ? sal : "-", strat_cls_name[0], cls[0], strat_cls_name[1], cls[1],
             strat_cls_name[2], cls[2], strat_cls_name[3], cls[3], strat_cls_name[4], cls[4], strat_role_name[0], roles[0],
             strat_role_name[1], roles[1], strat_role_name[2], roles[2], strat_role_name[3], roles[3]);
    event(host, "%s", line);
    if (!strat.snap)
        strat.snap = open_csv("strat_snap.csv", "t,army,kind,id,x,z,v1,v2,v3,v4");
    double t = (double)(ms - t0) / 1000.0;
    int zc = 0;
    for (uint32_t k = 0; k < m->ncap && k < 8; ++k)
        zc = zc * 10 + (owner[k] < 0 ? 9 : owner[k]);
    strat_snap_row(t, army, 'n', nh, (float)en_alive, (float)ne, (float)unseen, (float)zc, (float)us, (float)wedges);
    for (int i = 0; i < no; ++i)
        strat_snap_row(t, army, own[i].human ? 'h' : 'u', own[i].slot, own[i].x, own[i].z, own[i].p, own[i].kappa,
                       (float)own[i].cls, (float)own[i].role);
    for (int k = 0; k < ne; ++k)
        strat_snap_row(t, army, 'c', en[k].slot, en[k].x, en[k].z, en[k].age, en[k].p, (float)en[k].spaa, en[k].conf);
    for (int i = 0; i < no; ++i) {
        const hitdir_t *h = &hit_dir[own[i].slot];
        if (h->ms && ms - h->ms <= STRAT_WEDGE_MS)
            strat_snap_row(t, army, 'w', own[i].slot, own[i].x, own[i].z, h->dx, h->dz, (float)(ms - h->ms) / 1000.0f, 0.0f);
    }
    for (int b = 0; b < strat.nb; ++b)
        strat_snap_row(t, army, 'b', b, bfront[b], braw[b], fown[b], fen[b], fhid[b], ratio[b]);
    /* S2 orders: o, seat, destination, type, id, age s, pending (next type, or 0) */
    for (int i = 1; i < MAX_SEATS; ++i) {
        const lane_seat_t *o = &lseat[i];
        if (!o->unit || o->army != army || (!o->pose_observed_ms || ms - o->pose_observed_ms > 2000) || (!o->cmd_type && !o->cmd_apply_ms))
            continue;
        strat_snap_row(t, army, 'o', i, o->cmd_type ? o->cmd_x : o->cmd_nx, o->cmd_type ? o->cmd_z : o->cmd_nz,
                       (float)o->cmd_type, (float)o->cmd_id, o->cmd_type ? (float)(ms - o->cmd_since) / 1000.0f : 0.0f,
                       o->cmd_apply_ms ? (float)o->cmd_ntype : 0.0f);
    }
    if (strat.snap)
        fflush(strat.snap);
}

/* s47 / s48: the command code that was here (S2 team commander and its tick, roads posts, the task layer
 * and the team plan) is in ground_command.c; this brain function was declared inside it. */
void pick_troute(void *host, lane_seat_t *s, int seat);


/* s49: the guarded block of strat_tick, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; int on; } strat_load_ctx_t;
static void strat_load_run(void *ctx)
{
    strat_load_ctx_t *const context = ctx;
    void *const host = context->host;
    int on = context->on;
    {
        on = strat_load(host);
    }
    context->on = on;
}

/* s49: the guarded block of strat_tick, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; int army; uint64_t ms; } strat_call_ctx_t;
static void strat_update_run(void *ctx)
{
    strat_call_ctx_t *const context = ctx;
    void *const host = context->host;
    const int army = context->army;
    const uint64_t ms = context->ms;
    {
        strat_update(host, army, ms);
    }
}

/* s49: the guarded block of strat_tick, run under the host's fault guard (gh_guard, ground_host.h). */
static void cmd_tick_run(void *ctx)
{
    strat_call_ctx_t *const context = ctx;
    void *const host = context->host;
    const int army = context->army;
    const uint64_t ms = context->ms;
    {
        cmd_tick(host, army, ms);
    }
}

/* Called from lanes_tick under `lock`: each army once per STRAT_MS, army 2 half a period later. */
static void strat_tick(void *host, int army, uint64_t ms)
{
    if (army < 1 || army > 2)
        return;
    cmd_observe_lost_units(host, army, ms);
    if (strat.state == 2 || !cfg.strategy_file[0])
        return;
    if (!strat.state) {
        int on = 0;
        strat_load_ctx_t load = {host, on};
        const int faulted = gh_guard(strat_load_run, &load);
        on = load.on;
        if (faulted) {
            strat.state = 2;
        }
        if (!on)
            return;
        strat.next_ms[1] = ms;
        strat.next_ms[2] = ms + STRAT_MS / 2;
        strat.minute_ms = ms + 60000;
    }
    if (ms < strat.next_ms[army])
        return;
    strat.next_ms[army] += STRAT_MS;
    if (strat.next_ms[army] <= ms)
        strat.next_ms[army] = ms + STRAT_MS;
    strat_call_ctx_t update = {host, army, ms};
    if (gh_guard(strat_update_run, &update)) {
        if (gh_atomic_inc(&strat.faults) >= 3)
            strat.state = 2;
    }
    if (cmd_on() && ms >= cmd.next_ms[army]) {      /* S2 commander, every CMD_MS per army */
        cmd.next_ms[army] = ms + CMD_MS;
        double c_us = strat_now_us();
        strat_call_ctx_t tick = {host, army, ms};
        if (gh_guard(cmd_tick_run, &tick)) {
            if (gh_atomic_inc(&cmd.faults) >= 3) {
                cmd.off = 1;
                event(host, "cmd: off after 3 faults (lanes as before)");
            }
        }
        c_us = strat_now_us() - c_us;
        cmd.us_sum += c_us;
        cmd.us_max = c_us > cmd.us_max ? c_us : cmd.us_max;
        ++cmd.ticks;
    }
    if (ms >= strat.minute_ms) {
        strat.minute_ms = ms + 60000;
        event(host, "strat: last 60 s %d updates, %.0f us avg, %.0f us max, faults %ld%s", strat.updates,
              strat.updates ? strat.us_sum / strat.updates : 0.0, strat.us_max, strat.faults,
              strat.state == 2 ? " (off)" : "");
        strat.updates = 0;
        strat.us_sum = strat.us_max = 0.0;
        if (cfg.commander && cfg.lanes)
            event(host, "cmd: last 60 s %d ticks, %.0f us avg, %.0f us max; A* %d (%d failed), %.0f us avg, %.0f us max; faults %ld%s",
                  cmd.ticks, cmd.ticks ? cmd.us_sum / cmd.ticks : 0.0, cmd.us_max, cmd.astars, cmd.astar_fail,
                  cmd.astars ? cmd.as_sum / cmd.astars : 0.0, cmd.as_max, cmd.faults, cmd_on() ? "" : " (off)");
        cmd.ticks = cmd.astars = cmd.astar_fail = 0;
        cmd.us_sum = cmd.us_max = cmd.as_sum = cmd.as_max = 0.0;
    }
}

/* ---- Cover seeking (2026-11-11) --------------------------------------------
 * cover_find ranks passable G10 cells r_min..r_max from (x, z). It combines
 * directional hull cover with the current strategy view's incoming and
 * outgoing fire lines, friendly support and a short escape connection.
 * Firing and holding cells need a clear turret and CVF_VIEW requires a useful
 * G10VIEW fire share. A retreat cell can block the turret toward the hit
 * bearing. Vegetation contributes concealment. Claims, distance, movement
 * toward the threat and the existing passability limits still apply.
 * Results are cached per seat and use COV_CACHE_MS. Uses:
 *  - COV_FIRE (A3 / engage): target > COV_FIRE_MIN_M, a cell within COV_FIRE_R
 *    COV_FIRE_GAIN better than here -> hop there, then stop in it to fire
 *    (COV_STAY_MS / COV_STAY_SHOTS); the no-stabiliser fire-from-a-stop rule
 *    is kept (the gate holds the shot during the hop).
 *  - COV_CLOSE (B1 < 200 m): not under COV_CLOSE_MIN_M; one hop of at most
 *    COV_CLOSE_R per close fight when COV_CLOSE_GAIN better.
 *  - COV_RETREAT (hit twice by an unseen shooter): COV_RET_MIN_M..COV_RET_MAX_M,
 *    none further towards the hit bearing, wait COV_RET_STAY_MS in it.
 *  - COV_HOLD (threatened HOLD order at the point): inside 0.9 x the point
 *    radius, towards the known enemy nearest the point.
 *  - COV_SUPPORT / COV_PUSH (also PRESS): within COV_DEST_R / COV_PUSH_R of
 *    the order's destination, towards the known enemy nearest it (else the
 *    enemy side of the spawn axis).
 * Logs: `cover:` lines (COV_LOGS_PER_MIN) and a minute summary. fault-guarded:
 * 3 faults switch it off. WT_GROUND_COVER=0 (cover=0) disables it. */
#define COV_Q_MIN 4             /* effective hull quality (0-15) a cell needs at all */
#define COV_VEG_W 0.35f         /* vegetation-only cover */
#define COV_CONCEAL_W 0.08f
#define COV_HARD_MAX 4
#define COV_VIEW_MIN 3          /* CVF_VIEW: fire share 3/15 */
#define COV_CLAIM_M 15.0f
#define COV_MATE_M 10.0f
#define COV_W_DIST 0.25f        /* score per r_max of distance */
#define COV_W_FWD 0.5f          /* score per max(r_max, 40 m) of travel towards the threat */
#define COV_TOPK 6
#define COV_CACHE_MS 1500
#define COV_FIRE_MIN_M 150.0f
#define COV_FIRE_R 40.0f
#define COV_FIRE_R0 8.0f
#define COV_FIRE_GAIN 3
#define COV_FIRE_QMIN 6
#define COV_HERE_OK 9           /* the cell it is in is this good: just stop */
#define COV_CLOSE_MIN_M 80.0f
#define COV_CLOSE_R 25.0f
#define COV_CLOSE_GAIN 5
#define COV_CLOSE_QMIN 7
#define COV_STAY_MS 15000
#define COV_STAY_SHOTS 3
#define COV_LOST_MS 4000
#define COV_COOL_MS 8000
#define COV_TRY_MS 1000
#define COV_HOP_STALL_MS 5000
#define COV_HOP_PROGRESS_M 1.5f
#define COV_FAILED_CELL_MS 90000
#define COV_FAILED_CELL_M 8.0f
#define COV_FAILED_ORIGIN_M 12.0f
#define COV_FAILED_CONTEXT_M 80.0f
#define COV_DEST_R 40.0f
#define COV_PUSH_R 60.0f
#define COV_DEST_NEAR_M 250.0f  /* order stop spots: only this close to the destination */
#define COV_DEST_MS 5000
#define COV_DEST_KEEP 0.13f     /* a new stop spot must improve the position score */
#define COV_THREAT_AGE_S 30.0f
#define COV_LOGS_PER_MIN 30
typedef struct { uint64_t ms; float x, z, tx, tz, r0, r1; int flags, ok; cover_t c; } cov_cache_t;
static struct {
    cov_cache_t cache[MAX_SEATS][COV_NUSE];
    int queries[COV_NUSE], found[COV_NUSE], used[COV_NUSE];
    float dsum[COV_NUSE], qsum[COV_NUSE];
    int computes, hits, logs, dropped;
    double us_sum, us_max;
    uint64_t minute_ms;
    volatile long faults;
    int off;
} cov;

static int cover_ok(void)
{
    return cfg.cover && !cov.off && strat.state == 1 && strat.g10 && strat.g10hull && strat.g10view;
}


/* Position evaluation uses the latest team snapshot. Keep only contacts near
 * the candidate area or route leg, and credit allied fire at those contacts
 * separately from the chance of being fired on. */
#define POSITION_CONTACTS 8
#define POSITION_REACH_M 1800.0f
#define POSITION_SUPPORT_M 550.0f
typedef struct {
    int army, n;
    struct { int cell, spaa; float x, z, confidence, support, priority; } en[POSITION_CONTACTS];
} position_context_t;
typedef struct { float fire, support, exposure, hard, conceal; } position_value_t;


static int position_los_sampled(int a, int b)
{
    if (a < 0 || b < 0 || abs(a / strat.nx - b / strat.nx) > strat.W || abs(a % strat.nx - b % strat.nx) > strat.W)
        return 0;
    const strat_cell_t *p = &strat.cell[a], *q = &strat.cell[b];
    return dist2d(p->x, p->z, q->x, q->z) <= strat.m->vis_range;
}

static void position_context(position_context_t *ctx, int army, int seat, float sx, float sz, float gx, float gz,
                             uint64_t ms, int firepower)
{
    ctx->army = army;
    ctx->n = 0;
    const strat_view_t *v = &sview[army];
    if (!v->ok || ms < v->ms || ms - v->ms >= FORGET_MS)
        return;
    float elapsed = (float)(ms - v->ms) / 1000.0f;
    for (int k = 0; k < v->ne; ++k) {
        const float d = position_leg_distance(v->en[k].x, v->en[k].z, sx, sz, gx, gz);
        const float age = v->en[k].age + elapsed;
        if (age * 1000.0f >= FORGET_MS || d > POSITION_REACH_M + 240.0f)
            continue;
        float conf = expf(-age / 30.0f);
        float priority = conf / (1.0f + d / 400.0f);
        int at = ctx->n;
        for (int j = 0; j < ctx->n; ++j)
            if (priority > ctx->en[j].priority) {
                at = j;
                break;
            }
        if (at == POSITION_CONTACTS)
            continue;
        if (ctx->n < POSITION_CONTACTS)
            ++ctx->n;
        for (int j = ctx->n - 1; j > at; --j)
            ctx->en[j] = ctx->en[j - 1];
        ctx->en[at].cell = v->en[k].cell;
        ctx->en[at].spaa = v->en[k].spaa;
        ctx->en[at].x = v->en[k].x;
        ctx->en[at].z = v->en[k].z;
        ctx->en[at].confidence = conf;
        ctx->en[at].priority = priority;
        ctx->en[at].support = 0.0f;
    }
    if (!firepower || !ctx->n || ms - v->ms > 2500)
        return;
    for (int i = 0; i < v->no; ++i) {
        if (v->own[i].slot == seat || v->own[i].p <= 0.0f
            || position_leg_distance(v->own[i].x, v->own[i].z, sx, sz, gx, gz) > POSITION_SUPPORT_M)
            continue;
        int src = v->own[i].cell;
        for (int k = 0; k < ctx->n; ++k) {
            int dst = ctx->en[k].cell;
            if (!position_los_sampled(src, dst) || !strat_sees(strat.visf, src, dst))
                continue;
            float d = dist2d(v->own[i].x, v->own[i].z, ctx->en[k].x, ctx->en[k].z);
            float range = strat.m->vis_range;
            if (d <= range)
                ctx->en[k].support = fmaxf(ctx->en[k].support, fminf(1.0f, v->own[i].p) * (1.0f - 0.7f * d / range));
        }
    }
}


/* G10 directional statistics describe a sector at sampled distances. Partial
 * hull-down helps even when the G32 representative has a fire line. The
 * turret-blocked bit is stronger evidence, especially when VISF agrees. */
static float position_hard_cover(u8 h)
{
    int type = (h >> 4) & 3;
    if (type != 1 && type != 2)
        return 0.0f;
    return h & 0x80 ? 0.9f : (float)(h & 15) / 15.0f;
}

static position_value_t position_value(const position_context_t *ctx, float x, float z, int c32, int c10, int firepower)
{
    position_value_t v = {0};
    const strat_meta_t *m = strat.m;
    const u8 *hull = c10 >= 0 ? strat.g10hull + (size_t)c10 * 16 : NULL;
    float sum = 0.0f, strongest = 0.0f, cover_sum = 0.0f, hide_sum = 0.0f, mass = 0.0f;
    for (int k = 0; k < ctx->n; ++k) {
        float tx = ctx->en[k].x, tz = ctx->en[k].z;
        float d = dist2d(x, z, tx, tz);
        if (d > POSITION_REACH_M)
            continue;
        int bearing = hull ? position_direction(x, z, tx, tz) : 0;
        u8 h = hull ? hull[bearing] : 0;
        float hard = position_hard_cover(h);
        float conceal = (h & 0x40) ? 1.0f : 0.0f;
        int enemy = ctx->en[k].cell;
        int sampled = d <= m->vis_range && position_los_sampled(enemy, c32);
        /* A missing sample is uncertain exposure. Directional cover beyond
         * the cached range can help locally, but cannot prove a blocked ray. */
        float incoming = 0.48f * (1.0f - 0.35f * hard);
        if (sampled) {
            int fire_in = strat_sees(strat.visf, enemy, c32);
            int spot_in = strat_sees(strat.viss, enemy, c32);
            incoming = fire_in ? 1.0f - 0.36f * hard : 0.30f - 0.25f * hard;
            if (fire_in && !spot_in)
                conceal = 1.0f;
            if (firepower && strat_sees(strat.visf, c32, enemy)) {
                float fire = h & 0x80 ? 0.35f : 1.0f;
                if (hull && strat.g10view) {
                    int band = d < m->vband[1] ? 0 : d < m->vband[2] ? 1 : 2;
                    int share = strat.g10view[((size_t)c10 * 16 + (size_t)bearing) * 6 + (size_t)band * 2] >> 4;
                    fire *= 0.5f + 0.5f * (float)share / 15.0f;
                }
                v.fire = fmaxf(v.fire, fire * ctx->en[k].confidence * (1.0f - 0.65f * d / m->vis_range));
            }
        }
        float reach = ctx->en[k].spaa ? 600.0f : m->vis_range;
        float end = ctx->en[k].spaa ? 1150.0f : POSITION_REACH_M;
        float falloff = d <= reach ? 1.0f - 0.30f * d / reach
                        : 0.70f * sqrtf(fmaxf(0.0f, (end - d) / (end - reach)));
        float w = ctx->en[k].confidence * fmaxf(0.0f, falloff);
        float threat = w * incoming;
        strongest = fmaxf(strongest, threat);
        sum += threat;
        mass += w;
        cover_sum += w * hard;
        hide_sum += w * conceal;
        if (firepower)
            v.support = fmaxf(v.support, w * ctx->en[k].support);
    }
    v.exposure = strongest + fminf(0.65f, 0.30f * (sum - strongest));
    if (mass > 0.1f) {
        v.hard = cover_sum / mass;
        v.conceal = hide_sum / mass;
    } else if (c32 >= 0) {
        v.hard = (float)strat.cell[c32].cover[ctx->army - 1] / 255.0f;
        v.conceal = (float)strat.cell[c32].veg / 255.0f;
    }
    return v;
}

static int movement_local_clear(float x, float z, float tx, float tz)
{
    return control_static_connector(NULL, x, z, tx, tz) == NAV_CLEAR;
}

static float position_exit(float x, float z, float ux, float uz)
{
    float clear = 0.0f;
    for (int k = -1; k <= 1; ++k) {
        float dx = -ux * 20.0f - uz * 15.0f * (float)k;
        float dz = -uz * 20.0f + ux * 15.0f * (float)k;
        clear += (float)movement_local_clear(x, z, x + dx, z + dz);
    }
    return clear / 3.0f;
}

/* Hull-down quality (0-15, bins k-1..k+1) of cell i towards (tx, tz) and the
 * centre bin's cover type / concealment; -1 when the turret cannot fire that way. */
static float cover_cell_q(int i, float px, float pz, float tx, float tz, int *k_out, int *type, int *conceal)
{
    int k = position_direction(px, pz, tx, tz);
    const u8 *h = strat.g10hull + (size_t)i * 16;
    if (h[k] & 0x80)
        return -1.0f;
    *k_out = k;
    *type = (h[k] >> 4) & 3;
    *conceal = (h[k] >> 6) & 1;
    return 0.5f * (float)(h[k] & 15) + 0.25f * (float)((h[(k + 15) & 15] & 15) + (h[(k + 1) & 15] & 15));
}

static float cover_type_w(int type)
{
    return type == 3 ? COV_VEG_W : 1.0f;
}

/* Effective hull quality (0-15) where it stands, towards (tx, tz). */
static int cover_here(float x, float z, float tx, float tz)
{
    if (!cover_ok())
        return 0;
    float px, pz;
    int i = cover_cell_of(x, z, &px, &pz), k, type, conceal;
    if (i < 0)
        return 0;
    float q = cover_cell_q(i, px, pz, tx, tz, &k, &type, &conceal);
    return q < 0.0f ? 0 : (int)(q * cover_type_w(type) + 0.5f);
}

/* A straight passable line from (x0, z0) to (x1, z1), 5 m samples (the first
 * 8 m are not checked: the tank stands there). */
static int cover_reach(float x0, float z0, float x1, float z1)
{
    float d = dist2d(x0, z0, x1, z1);
    for (float t = 8.0f; t < d; t += 5.0f) {
        float f = t / d;
        int i = cover_cell_of(x0 + (x1 - x0) * f, z0 + (z1 - z0) * f, NULL, NULL);
        if (i < 0)
            return 0;
        const u8 *g = strat.g10 + (size_t)i * 8;
        if (!(g[4] & 2) || g[5] > 6)
            return 0;
    }
    return 1;
}

static int cover_candidate(const position_context_t *ctx, int i, float px, float pz, float x, float z,
                           float tx, float tz, float ux, float uz, float r_max, int flags, cover_t *out)
{
    float dd = dist2d(px, pz, x, z);
    float toward = (px - x) * ux + (pz - z) * uz;
    if (dd > r_max || ((flags & CVF_AWAY) && toward > 5.0f))
        return 0;
    const u8 *g = strat.g10 + (size_t)i * 8;
    if ((g[4] & 3) != 3 || g[5] > COV_HARD_MAX)
        return 0;
    int k = position_direction(px, pz, tx, tz);
    u8 h = strat.g10hull[(size_t)i * 16 + k];
    int type = (h >> 4) & 3, conceal = (h >> 6) & 1;
    float q = cover_cell_q(i, px, pz, tx, tz, &k, &type, &conceal);
    float hard = position_hard_cover(h);
    if (q < 0.0f)
        q = (flags & CVF_AWAY) ? 15.0f * hard : 0.0f;
    float qe = q * cover_type_w(type);
    if (qe < (float)COV_Q_MIN)
        return 0;

    float sector = 0.0f;
    if (!(flags & CVF_AWAY)) {
        float td = dist2d(px, pz, tx, tz);
        const strat_meta_t *m = strat.m;
        int band = td < m->vband[1] ? 0 : td < m->vband[2] ? 1 : 2;
        int fire = strat.g10view[((size_t)i * 16 + (size_t)k) * 6 + (size_t)band * 2] >> 4;
        if ((flags & CVF_VIEW) && fire < COV_VIEW_MIN)
            return 0;
        if (td <= m->vband[3])
            sector = 0.65f * (float)fire / 15.0f;
    }
    position_value_t v = position_value(ctx, px, pz, strat_cellof(px, pz), i, !(flags & CVF_AWAY));
    float exit = position_exit(px, pz, ux, uz);
    float sc = qe / 15.0f + COV_CONCEAL_W * fmaxf((float)conceal, v.conceal);
    if (flags & CVF_AWAY) {
        /* The hit bearing is a direction, even when the shooter is unknown. */
        sc += 0.65f * hard + 0.35f * exit - 1.60f * v.exposure;
    } else {
        sc += 0.25f * v.hard + 0.22f * fmaxf(v.fire, sector)
              + 0.18f * exit - 0.85f * v.exposure;
        sc += 0.12f * v.support / (1.0f + v.exposure);
    }
    sc -= COV_W_DIST * dd / r_max + COV_W_FWD * fmaxf(0.0f, toward) / fmaxf(r_max, 40.0f);
    *out = (cover_t){px, pz, dd, sc, (int)(qe + 0.5f), (int)(q + 0.5f), type, conceal, k};
    return 1;
}

/* A physical cell failure applies across cover uses while the tank is nearby. */
static int cover_hop_failed(const lane_seat_t *s, float x, float z, uint64_t ms)
{
    if (!s->cov_fail_count || dist2d(s->x, s->z, s->cov_fail_origin_x, s->cov_fail_origin_z) >= COV_FAILED_ORIGIN_M)
        return 0;
    for (int i = 0; i < s->cov_fail_count; ++i)
        if (ms - s->cov_fail_ms[i] < COV_FAILED_CELL_MS
            && dist2d(x, z, s->cov_fail_x[i], s->cov_fail_z[i]) < COV_FAILED_CELL_M)
            return 1;
    return 0;
}

/* CVF_AHEAD (stage 3, "only when there is time for it"). Of the 144 hops into
 * fire cover in the three battles of 2026-11-14, 100 were given up on the way:
 * 46 because the turn towards the cell showed the enemy the rear (the rear guard
 * stopped them), 54 for no progress. A hop is worth starting only when the cell
 * lies within COV_AHEAD_TURN_DEG of the nose and its direction within
 * COV_AHEAD_THREAT_DEG of the threat's (the hull stays under REAR_START_DEG off
 * the enemy), and when the turn (0.35 rad/s as measured) plus the drive fit in
 * COV_AHEAD_S, about one reload of the gun it is hiding from. (x, z): the tank. */
#define COV_AHEAD_TURN_DEG 75.0f
#define COV_AHEAD_THREAT_DEG 100.0f
#define COV_AHEAD_S 8.0f
static int cover_ahead(const lane_seat_t *s, float x, float z, float px, float pz, float ux, float uz)
{
    const float dx = px - x, dz = pz - z, d = sqrtf(dx * dx + dz * dz);
    if (d < 4.0f || s->fx * s->fx + s->fz * s->fz < 0.25f)
        return 1;                       /* next to it, or the heading is not known */
    const float nose = (dx * s->fx + dz * s->fz) / d, threat = (dx * ux + dz * uz) / d;
    if (nose < cosf(COV_AHEAD_TURN_DEG / 57.29578f) || threat < cosf(COV_AHEAD_THREAT_DEG / 57.29578f))
        return 0;
    return 1.0f + acosf(nose > 1.0f ? 1.0f : nose) / 0.35f + d / 4.0f <= COV_AHEAD_S;
}

static int cover_scan(int seat, int army, float x, float z, float tx, float tz, float r_min, float r_max, int flags,
                      uint64_t ms, cover_t *out)
{
    const strat_meta_t *m = strat.m;
    float claim[3 * MAX_SEATS][2], mate[MAX_SEATS][2];
    int ncl = 0, nmt = 0;
    for (int j = 1; j < MAX_SEATS; ++j) {
        const lane_seat_t *o = &lseat[j];
        if (j == seat || !o->unit || o->army != army || ms - o->last_ms > 1500)
            continue;
        mate[nmt][0] = o->x;
        mate[nmt++][1] = o->z;
        if (o->cov_use) {
            claim[ncl][0] = o->cov_x;
            claim[ncl++][1] = o->cov_z;
        }
        if (o->cov_dest_ok && ms - o->cov_dest_seen < 2000) {
            claim[ncl][0] = o->cov_dx;
            claim[ncl++][1] = o->cov_dz;
        }
        if (o->ap_state == AP_TO_COVER || o->ap_state == AP_OBSERVE) {     /* stage 3: its stop before a stretch under fire */
            claim[ncl][0] = o->ap_x;
            claim[ncl++][1] = o->ap_z;
        }
    }
    float ux, uz;
    unit_dir(x, z, tx, tz, &ux, &uz);
    position_context_t ctx;
    position_context(&ctx, army, seat, x, z, x, z, ms, !(flags & CVF_AWAY));
    float cs = m->g10_cell;
    int c0 = (int)floorf((x - r_max - m->g10_x0) / cs), c1 = (int)floorf((x + r_max - m->g10_x0) / cs);
    int r0 = (int)floorf((z - r_max - m->g10_z0) / cs), r1 = (int)floorf((z + r_max - m->g10_z0) / cs);
    c0 = c0 < 0 ? 0 : c0;
    r0 = r0 < 0 ? 0 : r0;
    c1 = c1 >= strat.n10x ? strat.n10x - 1 : c1;
    r1 = r1 >= strat.n10z ? strat.n10z - 1 : r1;
    cover_t top[COV_TOPK];
    int ntop = 0;
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c) {
            float px = m->g10_x0 + ((float)c + 0.5f) * cs, pz = m->g10_z0 + ((float)r + 0.5f) * cs;
            float dd = dist2d(px, pz, x, z);
            if (dd < r_min || dd > r_max || cover_hop_failed(&lseat[seat], px, pz, ms)
                || ((flags & CVF_AHEAD) && !cover_ahead(&lseat[seat], x, z, px, pz, ux, uz))
                || ((flags & CVF_CALM) && threat_at(army, px, pz) >= AP_TH_HI))
                continue;
            int i = r * strat.n10x + c;
            int taken = 0;
            for (int j = 0; j < ncl && !taken; ++j)
                taken = dist2d(px, pz, claim[j][0], claim[j][1]) < COV_CLAIM_M;
            if (taken)
                continue;
            cover_t candidate;
            if (!cover_candidate(&ctx, i, px, pz, x, z, tx, tz, ux, uz, r_max, flags, &candidate))
                continue;
            float sc = candidate.score;
            for (int j = 0; j < nmt; ++j)
                if (dist2d(px, pz, mate[j][0], mate[j][1]) < COV_MATE_M) {
                    sc -= 0.15f;
                    break;
                }
            if (ntop == COV_TOPK && sc <= top[COV_TOPK - 1].score)
                continue;
            int at = ntop < COV_TOPK ? ntop++ : COV_TOPK - 1;
            while (at > 0 && top[at - 1].score < sc) {
                top[at] = top[at - 1];
                --at;
            }
            candidate.score = sc;
            top[at] = candidate;
        }
    for (int j = 0; j < ntop; ++j)
        if (cover_reach(x, z, top[j].x, top[j].z)) {
            *out = top[j];
            return 1;
        }
    return 0;
}

/* s49: the guarded block of cover_find, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { int seat, army; float x, z, tx, tz, r_min, r_max; int flags; uint64_t ms; cover_t *c; int ok; } cover_scan_ctx_t;
static void cover_scan_run(void *ctx)
{
    cover_scan_ctx_t *const context = ctx;
    const int seat = context->seat, army = context->army;
    const float x = context->x, z = context->z, tx = context->tx, tz = context->tz;
    const float r_min = context->r_min, r_max = context->r_max;
    const int flags = context->flags;
    const uint64_t ms = context->ms;
    cover_t *const c = context->c;
    int ok = context->ok;
    {
        ok = cover_scan(seat, army, x, z, tx, tz, r_min, r_max, flags, ms, c);
    }
    context->ok = ok;
}

static int cover_find(int seat, int army, int use, float x, float z, float tx, float tz, float r_min, float r_max, int flags,
                      cover_t *out)
{
    if (!cover_ok() || seat <= 0 || seat >= MAX_SEATS || use <= COV_NONE || use >= COV_NUSE || r_max < 5.0f)
        return 0;
    uint64_t ms = gh_ms();
    ++cov.queries[use];
    if (!cov.minute_ms)
        cov.minute_ms = ms + 60000;
    cov_cache_t *e = &cov.cache[seat][use];
    if (e->ms && ms - e->ms < COV_CACHE_MS && dist2d(x, z, e->x, e->z) < 4.0f && dist2d(tx, tz, e->tx, e->tz) < 25.0f
        && e->r0 == r_min && e->r1 == r_max && e->flags == flags
        && (!e->ok || !cover_hop_failed(&lseat[seat], e->c.x, e->c.z, ms))) {
        ++cov.hits;
        if (e->ok) {
            *out = e->c;
            out->dist = dist2d(x, z, out->x, out->z);
            ++cov.found[use];
        }
        return e->ok;
    }
    int ok = 0;
    cover_t c = {0};
    double t = strat_now_us();
    cover_scan_ctx_t scan = {seat, army, x, z, tx, tz, r_min, r_max, flags, ms, &c, ok};
    const int faulted = gh_guard(cover_scan_run, &scan);
    ok = scan.ok;
    if (faulted) {
        ok = 0;
        if (gh_atomic_inc(&cov.faults) >= 3)
            cov.off = 1;
    }
    t = strat_now_us() - t;
    cov.us_sum += t;
    cov.us_max = t > cov.us_max ? t : cov.us_max;
    ++cov.computes;
    e->ms = ms;
    e->x = x;
    e->z = z;
    e->tx = tx;
    e->tz = tz;
    e->r0 = r_min;
    e->r1 = r_max;
    e->flags = flags;
    e->ok = ok;
    e->c = c;
    if (ok) {
        *out = c;
        ++cov.found[use];
    }
    return ok;
}

/* A cover move or stop spot was taken: stats and a rate-limited `cover:` line.
 * (ox, oz): the query origin (the tank, or the order's destination). */
static void cover_used(void *host, int seat, int use, const cover_t *c, int here, float tx, float tz, float ox, float oz)
{
    static const char *const tname[4] = {"none", "terrain", "solid", "vegetation"};
    ++cov.used[use];
    cov.dsum[use] += c->dist;
    cov.qsum[use] += (float)c->q;
    if (cov.logs >= COV_LOGS_PER_MIN) {
        ++cov.dropped;
        return;
    }
    ++cov.logs;
    event(host, "cover: seat %d %s -> (%.0f,%.0f) %.0f m from (%.0f,%.0f), %s, hull %d/15 (raw %d, here %d)%s, "
          "threat bearing %.0f at %.0f m", seat, cov_name[use], c->x, c->z, c->dist, ox, oz, tname[c->type & 3], c->q,
          c->qraw, here, c->conceal ? ", concealed" : "", atan2f(tz - c->z, tx - c->x) * 57.29578f,
          dist2d(c->x, c->z, tx, tz));
}

static void cover_end(void *host, lane_seat_t *s, int seat, uint64_t ms, const char *why)
{
    if (!s->cov_use)
        return;
    if (cov.logs < COV_LOGS_PER_MIN && s->cov_use != COV_RETREAT) {
        ++cov.logs;
        event(host, "cover: seat %d %s over after %.1f s (%s, %s)", seat, cov_name[s->cov_use],
              (double)(ms - s->cov_since) / 1000.0, s->cov_state == 2 ? "was in the cell" : "had not got there", why);
    }
    if (s->cov_state == 2 && s->cov_use != COV_RETREAT)
        s->stopping = 0;
    s->cov_use = s->cov_state = 0;
    s->cov_cool_until = ms + COV_COOL_MS;
}

static void cover_hop_context(lane_seat_t *s, int seat)
{
    const float route_x = s->cmd_type ? s->cmd_x : s->tx, route_z = s->cmd_type ? s->cmd_z : s->tz;
    if (s->cov_fail_count
        && dist2d(s->x, s->z, s->cov_fail_origin_x, s->cov_fail_origin_z) < COV_FAILED_ORIGIN_M
        && dist2d(s->ex, s->ez, s->cov_fail_threat_x, s->cov_fail_threat_z) < COV_FAILED_CONTEXT_M
        && dist2d(route_x, route_z, s->cov_fail_route_x, s->cov_fail_route_z) < COV_FAILED_CONTEXT_M)
        return;
    if (s->cov_fail_count)
        memset(cov.cache[seat], 0, sizeof cov.cache[seat]);
    s->cov_fail_count = 0;
    s->cov_fail_origin_x = s->x;
    s->cov_fail_origin_z = s->z;
    s->cov_fail_threat_x = s->ex;
    s->cov_fail_threat_z = s->ez;
    s->cov_fail_route_x = route_x;
    s->cov_fail_route_z = route_z;
}

/* COV_FIRE / COV_CLOSE (react_update, engaged; d = distance to the target).
 * Returns 1 while a cover move is in progress (react drives it). */
static int cover_engage(void *host, lane_seat_t *s, int seat, uint64_t ms, float d, int seen_now)
{
    if (s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE) {
        const char *why = NULL;
        const float left = dist2d(s->x, s->z, s->cov_x, s->cov_z);
        int failed = 0;
        if (s->cov_state == 1 && (left <= s->cov_hop_best - COV_HOP_PROGRESS_M
                                  || (cfg.approach && s->motion_state == GM_TURNING))) {
            /* Stage 3: turning towards the cell is progress too (the hop's time limit still holds). */
            s->cov_hop_best = fminf(left, s->cov_hop_best);
            s->cov_hop_progress_ms = ms;
        }
        const char *gate = cfg.takeover && s->cov_state == 1 ? tac_advance_abort(s, seat, ms) : NULL;
        if (!cover_ok())
            why = "off";
        else if (gate)
            why = gate;                 /* s28 takeover: the move out to fire ends at once */
        else if (s->rear)
            why = "rear turn";
        else if (ms - s->eng_seen_ms > COV_LOST_MS)
            why = "target lost";
        else if (s->cov_state == 1 && d < COV_CLOSE_MIN_M)
            why = "enemy too close";
        else if (s->cov_state == 1 && s->close && s->cov_use == COV_FIRE && left > COV_CLOSE_R)
            why = "close fight";
        else if (s->cov_state == 1 && ms > s->cov_until) {
            why = "hop timed out";
            failed = 1;
        } else if (s->cov_state == 1 && left >= COV_ARRIVE_M && ms - s->cov_hop_progress_ms >= COV_HOP_STALL_MS) {
            why = "hop made no progress";
            failed = 1;
        } else if (s->cov_state == 2 && (ms > s->cov_until || shot_seat[seat].total - s->cov_shots >= COV_STAY_SHOTS))
            why = "done";
        if (why) {
            if (failed) {
                const int i = s->cov_fail_count++;
                s->cov_fail_x[i] = s->cov_x;
                s->cov_fail_z[i] = s->cov_z;
                s->cov_fail_ms[i] = ms;
            }
            cover_end(host, s, seat, ms, why);
            return 0;
        }
        if (s->cov_state == 1 && left < COV_ARRIVE_M) {
            s->cov_state = 2;
            s->cov_since = ms;
            s->cov_until = ms + COV_STAY_MS;
            s->cov_shots = shot_seat[seat].total;
        }
        return 1;
    }
    if (!cover_ok() || ms < s->cov_cool_until || ms < s->cov_try_ms || !seen_now || seat_rush(s) || s->holding || s->rear
        || s->cov_use || d > ENG_MAX_M)
        return 0;
    if (cfg.takeover && !tac_advance_ok(s, seat, ms, NULL))
        return 0;                       /* s28: no move out to fire without sight, ability, the gunner on it */
    cover_hop_context(s, seat);
    if (s->cov_fail_count >= (int)(sizeof s->cov_fail_ms / sizeof s->cov_fail_ms[0]))
        return 0;
    int use, gain, qmin;
    float r0, r1;
    if (s->close) {
        if (d < COV_CLOSE_MIN_M || s->cov_close_tried == s->close_slot + 1)
            return 0;
        s->cov_close_tried = s->close_slot + 1;     /* one try per close fight */
        use = COV_CLOSE;
        r0 = 5.0f;
        r1 = COV_CLOSE_R;
        gain = COV_CLOSE_GAIN;
        qmin = COV_CLOSE_QMIN;
    } else if (d > COV_FIRE_MIN_M) {
        use = COV_FIRE;
        r0 = COV_FIRE_R0;
        r1 = COV_FIRE_R;
        gain = COV_FIRE_GAIN;
        qmin = COV_FIRE_QMIN;
    } else {
        return 0;
    }
    s->cov_try_ms = ms + COV_TRY_MS;
    int here = cover_here(s->x, s->z, s->ex, s->ez);
    if (use == COV_FIRE && here >= COV_HERE_OK)
        return 0;
    cover_t c;
    if (!cover_find(seat, s->army, use, s->x, s->z, s->ex, s->ez, r0, r1, CVF_VIEW | (cfg.approach ? CVF_AHEAD : 0), &c)
        || c.q < qmin || c.q < here + gain)
        return 0;
    s->cov_use = use;
    s->cov_state = 1;
    s->cov_x = c.x;
    s->cov_z = c.z;
    s->cov_q = c.q;
    s->cov_type = c.type;
    s->cov_dist = c.dist;
    s->cov_since = ms;
    s->cov_until = ms + 3000 + (uint64_t)(c.dist / COV_HOP_SPEED * 1000.0f);
    s->cov_hop_best = c.dist;
    s->cov_hop_progress_ms = ms;
    cover_used(host, seat, use, &c, here, s->ex, s->ez, s->x, s->z);
    return 1;
}

/* The known enemy (sview, <= COV_THREAT_AGE_S old) nearest (x, z) within r. */
static int cover_threat(int army, float x, float z, float r, float *ex, float *ez)
{
    const strat_view_t *v = &sview[army];
    int found = 0;
    for (int k = 0; v->ok && k < v->ne && k < 64; ++k) {
        float d = dist2d(v->en[k].x, v->en[k].z, x, z);
        if (v->en[k].age <= COV_THREAT_AGE_S && d < r) {
            r = d;
            *ex = v->en[k].x;
            *ez = v->en[k].z;
            found = 1;
        }
    }
    return found;
}

/* Order stop spot near (ox, oz) (cmd_drive / the hold branch): recomputed every
 * COV_DEST_MS or when the destination moved 10 m; a viable old spot is kept
 * unless the new position score improves by COV_DEST_KEEP. */
static int cover_spot(void *host, lane_seat_t *s, int seat, int use, float ox, float oz, float tx, float tz, float r, uint64_t ms)
{
    int moved = dist2d(ox, oz, s->cov_ox, s->cov_oz) > 10.0f;
    const int failed = s->cov_dest_ok && cover_hop_failed(s, s->cov_dx, s->cov_dz, ms);
    if (failed)
        s->cov_dest_ok = 0;
    if (ms >= s->cov_dest_ms || moved || failed) {
        s->cov_dest_ms = ms + COV_DEST_MS;
        s->cov_ox = ox;
        s->cov_oz = oz;
        cover_t c;
        if (cover_find(seat, s->army, use, ox, oz, tx, tz, 0.0f, r, CVF_VIEW, &c)) {
            cover_t old;
            int keep = 0;
            if (s->cov_dest_ok && !moved) {
                int cell = cover_cell_of(s->cov_dx, s->cov_dz, NULL, NULL);
                if (cell >= 0) {
                    position_context_t ctx;
                    float ux, uz;
                    position_context(&ctx, s->army, seat, ox, oz, ox, oz, ms, 1);
                    unit_dir(ox, oz, tx, tz, &ux, &uz);
                    keep = cover_candidate(&ctx, cell, s->cov_dx, s->cov_dz, ox, oz, tx, tz, ux, uz, r, CVF_VIEW, &old)
                           && c.score < old.score + COV_DEST_KEEP;
                }
            }
            if (!keep || dist2d(c.x, c.z, s->cov_dx, s->cov_dz) < 5.0f) {
                int fresh = !s->cov_dest_ok || dist2d(c.x, c.z, s->cov_dx, s->cov_dz) > 10.0f;
                s->cov_dx = c.x;
                s->cov_dz = c.z;
                s->cov_dest_q = c.q;
                s->cov_dest_type = c.type;
                s->cov_dest_ok = 1;
                if (fresh)
                    cover_used(host, seat, use, &c, cover_here(ox, oz, tx, tz), tx, tz, ox, oz);
            }
        } else if (moved || !s->cov_dest_ok || cover_here(s->cov_dx, s->cov_dz, tx, tz) < COV_Q_MIN) {
            s->cov_dest_ok = 0;
        }
    }
    if (!s->cov_dest_ok)
        return 0;
    s->cov_dest_seen = ms;
    return 1;
}

/* SUPPORT / PUSH / PRESS stop spot (cmd_drive): replaces (gx, gz). */
static int cover_dest(void *host, lane_seat_t *s, int seat, uint64_t ms, float *gx, float *gz)
{
    if (!cover_ok() || (s->cmd_type != OB_SUPPORT && s->cmd_type != OB_PUSH && s->cmd_type != OB_PRESS)
        || dist2d(s->x, s->z, s->cmd_x, s->cmd_z) > COV_DEST_NEAR_M) {
        s->cov_dest_ok = 0;
        return 0;
    }
    float ex, ez;
    if (!cover_threat(s->army, s->cmd_x, s->cmd_z, POSITION_REACH_M, &ex, &ez)) {
        const strat_meta_t *m = strat.m;
        float sg = s->army == 1 ? 1.0f : -1.0f;  /* +axis: towards the army-2 spawn */
        ex = s->cmd_x + sg * m->axx * 400.0f;
        ez = s->cmd_z + sg * m->axz * 400.0f;
    }
    int use = s->cmd_type == OB_SUPPORT ? COV_SUPPORT : COV_PUSH;
    if (!cover_spot(host, s, seat, use, s->cmd_x, s->cmd_z, ex, ez, use == COV_SUPPORT ? COV_DEST_R : COV_PUSH_R, ms))
        return 0;
    *gx = s->cov_dx;
    *gz = s->cov_dz;
    return 1;
}

/* Threatened HOLD at a point of ours (the lanes hold branch): replaces (hx, hz). */
static int cover_hold(void *host, lane_seat_t *s, int seat, const lane_t *l, uint64_t ms, float *hx, float *hz)
{
    float ex, ez;
    if (!cover_ok() || !cmd_on() || s->cmd_type != OB_HOLD || s->route[s->step].pt != SLOT_PT
        || zone_owner(host, l->zx, l->zz) != s->army || !cover_threat(s->army, l->zx, l->zz, 400.0f, &ex, &ez)) {
        s->cov_dest_ok = 0;
        return 0;
    }
    float r = 0.9f * l->zr;
    if (!cover_spot(host, s, seat, COV_HOLD, l->zx, l->zz, ex, ez, r < 80.0f ? r : 80.0f, ms))
        return 0;
    *hx = s->cov_dx;
    *hz = s->cov_dz;
    return 1;
}

static void cover_minute(void *host, uint64_t ms)
{
    if (!cov.minute_ms || ms < cov.minute_ms)
        return;
    cov.minute_ms = ms + 60000;
    char line[512];
    int len = snprintf(line, sizeof line, "cover: last 60 s %d computed + %d cached, %.0f us avg, %.0f us max |",
                       cov.computes, cov.hits, cov.computes ? cov.us_sum / cov.computes : 0.0, cov.us_max);
    float ds = 0.0f, qs = 0.0f;
    int nu = 0;
    for (int u = 1; u < COV_NUSE && len < (int)sizeof line - 60; ++u) {
        if (!cov.queries[u])
            continue;
        len += snprintf(line + len, sizeof line - (size_t)len, " %s %d/%d/%d", cov_name[u], cov.queries[u], cov.found[u],
                        cov.used[u]);
        if (cov.used[u])
            len += snprintf(line + len, sizeof line - (size_t)len, " %.0fm q%.1f", cov.dsum[u] / cov.used[u],
                            cov.qsum[u] / cov.used[u]);
        ds += cov.dsum[u];
        qs += cov.qsum[u];
        nu += cov.used[u];
    }
    if (len < (int)sizeof line - 80)
        snprintf(line + len, sizeof line - (size_t)len, " | used %d, avg %.0f m, avg hull %.1f/15; %d lines dropped; faults %ld%s",
                 nu, nu ? ds / nu : 0.0f, nu ? qs / nu : 0.0f, cov.dropped, cov.faults, cov.off ? " (off)" : "");
    event(host, "%s", line);
    memset(cov.queries, 0, sizeof cov.queries);
    memset(cov.found, 0, sizeof cov.found);
    memset(cov.used, 0, sizeof cov.used);
    memset(cov.dsum, 0, sizeof cov.dsum);
    memset(cov.qsum, 0, sizeof cov.qsum);
    cov.computes = cov.hits = cov.logs = cov.dropped = 0;
    cov.us_sum = cov.us_max = 0.0;
}

/* ---- T2: tactical routes (2026-11-10) -------------------------------------
 * build_tactical.py plans, per army / spawn group / capture point, up to four
 * variants over terrain + map objects + the collision scan: fast, cover (low
 * exposure to likely enemy viewpoints), flank, trees (tree-line concealment).
 * A new life on a target lane picks one by weight (fast 1, cover 1.3, flank
 * 0.6, trees cfg.tree_cover: tree-line creeping is to be rarer, not gone),
 * then follows its waypoints before the lane's own steps. Hops are driven
 * straight (cfg.direct): the bot's path point count = 0 and repath
 * timer = 1 each tick, so the game's own move order steers at the destination (its
 * branch for fewer than 2 path points). Hops with a blocked mapped chord use the
 * game's own path search. Stalled straight hops get a bounded native trial.
 * Overwatch: route cut at the waypoint nearest its firing spot, whose
 * position replaces lane point S1 (lane_point_xz). */
#define ROUTE_TURN_START_DEG 80.0f
#define ROUTE_TURN_END_DEG 65.0f
#define ROUTE_BRAKE_SPEED 2.0f
#define ROUTE_BRAKE_MS 2500
#define ROUTE_DIRECT_STALL_MS 4000
#define ROUTE_NATIVE_TRIAL_MS 8000

static int route_is_flank(const lane_seat_t *s, int seat)
{
    return s->cmd_type < OB_SUPPORT && route_owns_flank(s, seat)
        && s->flank_lane == s->lane && s->flank_step == s->step;
}

/* Drive to (x, z) on the lane / route. While it fights a target it sees inside
 * BIAS_MAX_M and the course points more than BIAS_NOSE_DEG away from it, the
 * course is turned towards the target until the nose is BIAS_NOSE_DEG off it,
 * when that takes no more than BIAS_TURN_DEG: the command point becomes a
 * point BIAS_AHEAD_M ahead on that course, renewed every BIAS_MS. Threats
 * further round (side, rear) are left to the short stop and the hull pivot.
 * Not for pushers (straight for the point) nor right after an unstuck. */
#define BIAS_MIN_M 400.0f       /* review_5003 B4 (was: 60 m up to BIAS_MAX_M 400 m) */
#define BIAS_NOSE_DEG 30.0f
#define BIAS_TURN_DEG 45.0f
#define BIAS_AHEAD_M 60.0f
#define BIAS_MS 2000
static void lane_drive(lane_seat_t *s, float x, float z, uint64_t ms)
{
    if (ms >= s->bias_until) {
        s->bias_until = ms + BIAS_MS;
        s->bias_on = 0;
        float td = dist2d(s->x, s->z, s->ex, s->ez);
        /* review_5003 B4: the bias moved the destination back and forth in close
         * fights; off by default, and only for targets beyond BIAS_MIN_M. */
        if (cfg.headbias && !cmd_is_post(s) && s->mode == M_ENGAGE && !s->pusher && !s->unstuck_n && ms - s->eng_seen_ms < 1500
            && td > BIAS_MIN_M && td < ENG_STOP_MAX_M && dist2d(s->x, s->z, x, z) > BIAS_AHEAD_M) {
            float course = atan2f(z - s->z, x - s->x) * 57.29578f;
            float off = wrap180(atan2f(s->ez - s->z, s->ex - s->x) * 57.29578f - course);
            float turn = fabsf(off) - BIAS_NOSE_DEG;
            if (turn > 0.0f && turn <= BIAS_TURN_DEG) {
                float a = (course + (off > 0.0f ? turn : -turn)) / 57.29578f;
                s->bias_x = s->x + cosf(a) * BIAS_AHEAD_M;
                s->bias_z = s->z + sinf(a) * BIAS_AHEAD_M;
                s->bias_on = 1;
            }
        }
    }
    nav_propose(&s->nav, NAV_LANE, s->bias_on ? s->bias_x : x, s->bias_on ? s->bias_z : z,
                NAV_NATIVE, 1, 0, 0);
}

static int pick_spot_for(lane_seat_t *s, const lane_t *l, int seat)
{
    int best = -1;
    float bs = -1;
    position_context_t ctx;
    int evaluated = strat.state == 1 && sview[s->army].ok;
    if (evaluated)
        position_context(&ctx, s->army, seat, s->x, s->z, l->zx, l->zz, gh_ms(), 1);
    for (int i = 0; i < ntspots; ++i) {
        const tspot_t *p = &tspots[i];
        if (p->army != s->army || dist2d(p->zx, p->zz, l->zx, l->zz) > 40.0f)
            continue;
        int taken = 0;
        for (int k = 1; k < MAX_SEATS; ++k)
            taken |= k != seat && lseat[k].unit && lseat[k].spot == i;
        float sc = p->score * (p->kind == 2 ? cfg.tree_cover : 1.0f) * (taken ? 0.2f : 1.0f);
        if (evaluated) {
            int c32 = strat_cellof(p->x, p->z);
            int c10 = strat.g10 ? cover_cell_of(p->x, p->z, NULL, NULL) : -1;
            position_value_t v = position_value(&ctx, p->x, p->z, c32, c10, 1);
            sc *= (1.0f + 0.30f * v.fire + 0.22f * v.hard + 0.08f * v.conceal
                         + 0.12f * v.support / (1.0f + v.exposure)) * expf(-v.exposure);
        }
        if (sc > bs) {
            bs = sc;
            best = i;
        }
    }
    return best;
}

static void nav_set_state(lane_seat_t *s, int state, int reason, uint64_t ms, uint64_t retry)
{
    nav_state_t *nav = &s->nav;
    if (state == GNR_BLOCKED || state == GNR_REPLAN_PENDING || state == GNR_UNREACHABLE) {
        if (!nav->blocked_since_ms)
            nav->blocked_since_ms = ms;
    } else if ((!nav_recent(nav->feedback_ms, ms) || !nav->blocked)
               && (state == GNR_FOLLOWING || state == GNR_READY || state == GNR_ARRIVED || state == GNR_CANCELLED)) {
        nav->blocked_since_ms = 0;
    }
    nav->state = state;
    nav->reason = reason;
    nav->retry_after_ms = retry;
}

static void nav_retire_route(lane_seat_t *s)
{
    s->nav.route_active = 0;
    s->nav.needs_rejoin = 0;
    s->direct_off_until = 0;
    s->route_drive = 0;
}

static int nav_route_active(const lane_seat_t *s)
{
    return s->nav.route_active && s->tr >= 0 && s->wpi < s->wpn;
}

/* A fresh tactical route can only join the initial, contiguous part of the
 * route. An in-flight route never uses this entry selection to resynchronize. */
static int route_entry(const lane_seat_t *s, const troute_t *r, int count)
{
    for (int i = 0; i < count && i < 2; ++i) {
        const float d = dist2d(s->x, s->z, r->x[i], r->z[i]);
        if (d > NAV_PORTAL_M || i + 1 >= count)
            return i;
        if (movement_turn_error(s, r->x[i + 1], r->z[i + 1]) > 55.0f
            || control_static_connector(s, s->x, s->z, r->x[i + 1], r->z[i + 1]) != NAV_CLEAR)
            return i;
    }
    return count > 2 ? 2 : count - 1;
}

/* Publishing a route is the only place that changes its generation. The old
 * slot and progress remain readable until the entire new candidate is ready. */
static int nav_install_route(lane_seat_t *s, int index, const troute_t *r, int first, int count, uint64_t ms)
{
    if (first < 0 || first >= count || count > MAX_TWP)
        return 0;
    if (hypotf(s->vx, s->vz) > ROUTE_BRAKE_SPEED
        && dist2d(s->x, s->z, r->x[first], r->z[first]) > NAV_PORTAL_M
        && movement_turn_error(s, r->x[first], r->z[first]) > ROUTE_TURN_START_DEG)
        return 0;
    /* Commit a validated entrance and candidate topology. Remote UNKNOWN
     * stays advisory and will be checked again from the actual pose. */
    if (control_connector_prefix(s, r->x[first], r->z[first]) == NAV_BLOCKED)
        return 0;
    float lengths[MAX_TWP] = {0};
    float x = s->x, z = s->z, total = 0.0f;
    for (int i = first; i < count; ++i) {
        const float tx = r->x[i], tz = r->z[i];
        if (!isfinite(tx) || !isfinite(tz))
            return 0;
        if (control_static_connector(s, x, z, tx, tz) == NAV_BLOCKED)
            return 0;
        total += dist2d(x, z, tx, tz);
        lengths[i] = total;
        x = tx;
        z = tz;
    }
    if (total < NAV_PORTAL_M)
        return 0;

    if (index >= MAX_TROUTES)
        troutes[index] = *r;
    nav_state_t *nav = &s->nav;
    s->tr = index;
    s->wpi = first;
    s->wpn = count;
    nav->generation = ++s->route_generation;
    nav->route_id = index + 1;
    nav->route_active = 1;
    nav->route_tactical = index < MAX_TROUTES;
    nav->route_task_id = nav->task_id;
    nav->route_revision = nav->revision;
    nav->needs_rejoin = 0;
    nav->entry = nav->previous_segment = first;
    nav->origin_x = s->x;
    nav->origin_z = s->z;
    nav->actual_m = nav->completed_m = 0.0f;
    nav->valid_projection = 0;
    nav->pose_ms = 0;
    nav->recovery_result = 0;
    memcpy(nav->arc, lengths, sizeof lengths);
    s->route_interrupted = 1;
    s->native_tried = s->route_native_only = 0;
    s->direct_off_until = 0;
    nav_set_state(s, GNR_READY, GTR_NONE, ms, 0);
    return 1;
}

static void nav_route_observe(lane_seat_t *s, uint64_t ms, int advance)
{
    if (!nav_route_active(s))
        return;
    nav_state_t *nav = &s->nav;
    const troute_t *r = &troutes[s->tr];
    const int wp = s->wpi;
    const float ax = wp == nav->entry ? nav->origin_x : r->x[wp - 1];
    const float az = wp == nav->entry ? nav->origin_z : r->z[wp - 1];
    const float start_m = wp == nav->entry ? 0.0f : nav->arc[wp - 1];
    const nav_projection_t current = nav_project(s->x, s->z, s->fx, s->fz, ax, az,
                                                 r->x[wp], r->z[wp], start_m);
    nav_projection_t previous = {0};
    if (wp > nav->entry) {
        const float px = wp == nav->entry + 1 ? nav->origin_x : r->x[wp - 2];
        const float pz = wp == nav->entry + 1 ? nav->origin_z : r->z[wp - 2];
        previous = nav_project(s->x, s->z, s->fx, s->fz, px, pz, ax, az,
                               wp == nav->entry + 1 ? 0.0f : nav->arc[wp - 2]);
    }
    const float movement = nav->pose_ms ? dist2d(nav->prev_x, nav->prev_z, s->x, s->z) : 0.0f;
    const float window = NAV_PORTAL_M + fminf(10.0f, 2.0f * movement);
    const int continuous = nav_recent(nav->pose_ms, ms) && movement <= 12.0f;
    nav->prev_x = s->x;
    nav->prev_z = s->z;
    nav->pose_ms = ms;
    nav_projection_t projection = current;
    if (previous.valid && (!projection.valid || (nav->previous_segment < wp
        && previous.lateral + 1.0f < projection.lateral))) {
        projection = previous;
        nav->previous_segment = wp - 1;
    } else if (current.valid) {
        nav->previous_segment = wp;
    }
    nav->valid_projection = projection.valid && fabsf(projection.arc - nav->actual_m) <= window;
    if (nav->valid_projection)
        nav->actual_m = fmaxf(nav->actual_m, projection.arc);
    if (!advance || !continuous || !nav->valid_projection)
        return;

    const float d = dist2d(s->x, s->z, r->x[wp], r->z[wp]);
    const float length = nav->arc[wp] - start_m;
    const int portal = d <= NAV_PORTAL_M || (current.valid && current.forward >= length && current.lateral <= 2.5f);
    const int same_side = (current.valid && fabsf(current.arc - nav->actual_m) <= NAV_PORTAL_M) || length < 0.1f;
    if (portal && same_side && control_connector_prefix(s, r->x[wp], r->z[wp]) != NAV_BLOCKED) {
        nav->actual_m = nav->completed_m = nav->arc[wp];
        ++s->wpi;
        s->route_interrupted = 1;
        s->native_tried = s->route_native_only = 0;
        s->direct_off_until = 0;
        s->ax = s->x;
        s->az = s->z;
        s->anchor_ms = ms;
        return;
    }
    if (current.valid)
        nav->completed_m = fmaxf(nav->completed_m, fminf(nav->actual_m, fmaxf(start_m, nav->arc[wp] - 0.1f)));
}

static void nav_control_feedback(lane_seat_t *s, uint64_t ms, int waiting, int blocked, int clearing)
{
    if (blocked && !s->nav.blocked_since_ms)
        s->nav.blocked_since_ms = ms;
    else if (!blocked)
        s->nav.blocked_since_ms = 0;
    s->nav.feedback_ms = ms;
    s->nav.waiting = waiting;
    s->nav.blocked = blocked;
    s->nav.clearing = clearing;
}

/* v13 (spec 7): where this spawn group (gx, gz) has street_flank routes to lane l's point (urban
 * maps; *street = 1), they replace the grid flank route for the flank roles (lane flanker /
 * screen, or the vehicle's default flank role), weighted 0.6 x 3 x (0.5 + street-flank fit),
 * i.e. the grid flank's flanker weight 1.8 at a mid fit; nobody else takes them (0). */
static float roads_street_weight(const lane_seat_t *s, int seat, const lane_t *l, float gx, float gz, int *street)
{
    *street = 0;
    for (int i = 0; i < ntroutes && !*street; ++i)
        *street = troutes[i].variant == TV_STREET && troutes[i].army == s->army
                  && dist2d(troutes[i].zx, troutes[i].zz, l->zx, l->zz) < 40.0f && dist2d(troutes[i].sx, troutes[i].sz, gx, gz) <= 1.0f;
    const int flanker = *street && !strat_spaa(seat)
                        && (s->role == LROLE_FLANKER || s->role == LROLE_SCREEN
                            || (s->army >= 1 && s->army <= 2 && sview[s->army].ok && (sview[s->army].role[seat & 63] & 3) == SR_FLANK));
    return flanker ? 0.6f * 3.0f * (0.5f + roads_fit(s->army, seat, RT_STREET)) : 0.0f;
}

/* Pick weight of a tactical route (fast 1, cover 1.3, flank 0.6, x3 for the lane flank roles;
 * trees cfg.tree_cover; v13: street_flank street_w, and the x3 moves to it where it exists). */
static float troute_pick_weight(const lane_seat_t *s, const troute_t *r, int street, float street_w)
{
    static const float base_w[4] = {1.0f, 1.3f, 0.6f, 0.0f};
    float w = r->variant == TV_TREES ? cfg.tree_cover : r->variant == TV_STREET ? street_w : base_w[r->variant & 3];
    if (r->variant == TV_FLANK && (s->role == LROLE_FLANKER || s->role == LROLE_SCREEN) && !street)
        w *= 3.0f;
    return w;
}

void pick_troute(void *host, lane_seat_t *s, int seat)
{
    const lane_t *l = &lanes[s->lane];
    if (!ntroutes || l->kind != 0)
        return;
    /* the spawn group nearest to where this life started */
    float gbest = 1e18f, gx = 0, gz = 0;
    for (int i = 0; i < ntroutes; ++i) {
        const troute_t *r = &troutes[i];
        if (r->army == s->army && dist2d(r->zx, r->zz, l->zx, l->zz) < 40.0f && dist2d(r->sx, r->sz, s->sx, s->sz) < gbest) {
            gbest = dist2d(r->sx, r->sz, s->sx, s->sz);
            gx = r->sx;
            gz = r->sz;
        }
    }
    if (gbest > 1e17f)
        return;
    int street = 0;
    const float street_w = roads_street_weight(s, seat, l, gx, gz, &street);
    if (street)
        event(host, "roads: seat %d %s street_flank routes to (%.0f,%.0f), weight %.2f", seat,
              street_w > 0.0f ? "weighs" : "skips (not a flank role)", l->zx, l->zz, street_w);
    int pick = -1;
    if (approach_on()) {
        /* Stage 3: the route of lowest cost under the fire-threat field (driving
         * time + AP_ALPHA x time under fire, ground_threat.inc), not a weighted
         * roll that knew only the enemies already seen. The old preferences stay
         * as -AP_PICK_TAU_S ln w seconds (cover a little cheaper, tree lines
         * dearer, flank routes cheaper for the flank roles); a team-mate on the
         * first half of a route makes it AP_LOAD_S dearer, which spreads a spawn
         * group over its routes; a few seconds by seat break ties. */
        const uint64_t now = gh_ms();
        float best = 1e30f, fire = 0.0f, length = 0.0f;
        for (int i = 0; i < ntroutes; ++i) {
            const troute_t *r = &troutes[i];
            if (r->army != s->army || dist2d(r->zx, r->zz, l->zx, l->zz) >= 40.0f || dist2d(r->sx, r->sz, gx, gz) > 1.0f)
                continue;
            const float w = troute_pick_weight(s, r, street, street_w);
            if (w <= 0.0f)
                continue;
            const threat_measure_t m = threat_measure_route(s->army, s->x, s->z, r, 0, r->n);
            int ahead = 0;
            for (int j = 1; j < MAX_SEATS; ++j) {
                const lane_seat_t *o = &lseat[j];
                ahead += j != seat && o->unit && o->army == s->army && o->scene_epoch == ground_scene_epoch
                         && now - o->last_ms < 2000 && o->nav.route_active && o->tr == i && 2 * o->wpi < o->wpn;
            }
            const float cost = threat_cost(&m) - AP_PICK_TAU_S * logf(w) + AP_LOAD_S * (float)ahead
                               + (float)((seat * 7 + i * 13) % 6);
            if (cost < best) {
                best = cost;
                fire = m.exposed;
                length = m.length;
                pick = i;
            }
        }
        if (pick >= 0)
            event(host, "route: seat %d judges its routes under the threat field%s: the best costs %.0f s, %.0f m long, "
                  "%.0f m of it under fire", seat, threat_ready(s->army) ? "" : " (hidden enemies not projected yet)", best,
                  length, fire);
        s->ap_pick_blind = !threat_ready(s->army);
    } else {
        float w[MAX_TROUTES], total = 0;
        position_context_t ctx;
        int evaluated = cmd_on();
        if (evaluated)
            position_context(&ctx, s->army, seat, s->x, s->z, l->zx, l->zz, gh_ms(), 0);
        for (int i = 0; i < ntroutes; ++i) {
            const troute_t *r = &troutes[i];
            w[i] = 0;
            if (r->army != s->army || dist2d(r->zx, r->zz, l->zx, l->zz) >= 40.0f || dist2d(r->sx, r->sz, gx, gz) > 1.0f)
                continue;
            w[i] = troute_pick_weight(s, r, street, street_w);
            if (evaluated && w[i] > 0.0f) {
                float x0 = s->x, z0 = s->z, total_m = 0.0f, risk_m = 0.0f, peak = 0.0f;
                for (int k = 0; k < r->n; ++k) {
                    float dx = r->x[k] - x0, dz = r->z[k] - z0, d = sqrtf(dx * dx + dz * dz);
                    int samples = (int)fmaxf(1.0f, fminf(4.0f, ceilf(d / 36.0f)));
                    for (int j = 0; j < samples; ++j) {
                        float f = ((float)j + 0.5f) / (float)samples;
                        float x = x0 + dx * f, z = z0 + dz * f;
                        int c32 = strat_cellof(x, z);
                        int c10 = strat.g10 ? cover_cell_of(x, z, NULL, NULL) : -1;
                        position_value_t v = position_value(&ctx, x, z, c32, c10, 0);
                        float prior = c32 >= 0 ? fminf(1.0f, strat.enc[s->army][c32]) : 0.25f;
                        float risk = v.exposure + 0.24f * prior + 0.18f * (1.0f - v.hard) + 0.08f * (1.0f - v.conceal);
                        risk_m += d / (float)samples * risk;
                        peak = fmaxf(peak, risk);
                    }
                    total_m += d;
                    x0 = r->x[k];
                    z0 = r->z[k];
                }
                w[i] *= expf(-CMD_ROUTE_RISK * (risk_m / fmaxf(1.0f, total_m) + 0.35f * peak));
            }
            total += w[i];
        }
        if (total <= 0)
            return;
        float roll = (float)((gh_ms() * 2654435761ull + (uint64_t)seat * 40503ull) % 10000ull) / 10000.0f * total;
        for (int i = 0; i < ntroutes && pick < 0; ++i) {
            if (w[i] <= 0)
                continue;
            if (roll < w[i])
                pick = i;
            roll -= w[i];
        }
        /* The first second of a battle: the strategy view and the threat field do
         * not exist yet. The choice is made again once they do (approach_review). */
        s->ap_pick_blind = cfg.approach;
    }
    /* Judged again and the route it is on is still the choice (approach_review):
     * nothing to install, and no going back to that route's first waypoint. */
    if (pick < 0 || (nav_route_active(s) && s->nav.route_tactical && s->tr == pick))
        return;
    const troute_t *r = &troutes[pick];
    int spot = -1, count = r->n;
    if (s->role == LROLE_OVERWATCH) {
        spot = pick_spot_for(s, l, seat);
        if (spot >= 0) {
            const tspot_t *p = &tspots[spot];
            float nearest = 1e18f;
            for (int i = 0; i < r->n; ++i) {
                const float d = dist2d(r->x[i], r->z[i], p->x, p->z);
                if (d < nearest) {
                    nearest = d;
                    count = i + 1;
                }
            }
        }
    }
    const int first = route_entry(s, r, count);
    if (!nav_install_route(s, pick, r, first, count, s->nav.sampled_ms))
        return;
    s->spot = spot;
    static const char *const vn[] = {"fast", "cover", "flank", "trees", "street_flank"};
    event(host, "route: seat %d takes the %s route to (%.0f,%.0f): %d waypoints from #%d%s", seat,
          vn[r->variant == TV_STREET ? TV_STREET : r->variant & 3],
          r->zx, r->zz, s->wpn, s->wpi, s->spot >= 0 ? ", then a covered firing spot" : "");
}

static int follow_troute(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (!s->nav.route_active || s->tr < 0)
        return 0;
    if (s->wpi >= s->wpn) {
        const int tactical = s->nav.route_tactical;
        nav_retire_route(s);
        if (tactical) {
            const int step = s->role == LROLE_OVERWATCH && s->spot >= 0 ? 1 : lane_slot_step(s);
            lane_goto_step(host, s, seat, step, ms, "tactical route done");
        }
        return 0;
    }
    const troute_t *r = &troutes[s->tr];
    if (s->route_interrupted && s->wpi + 1 < s->wpn) {
        /* Taken up again after an interruption: on from the first waypoint
         * ahead of the tank, never one it has passed. The nearest of the next
         * legs (within 60 m; a route that winds back past here further on is
         * not looked at) tells where it is; forwards only. */
        float best = 60.0f;
        int ahead = s->wpi;
        for (int k = s->wpi; k + 1 < s->wpn && k < s->wpi + 4; ++k) {
            const float ax = r->x[k], az = r->z[k], dx = r->x[k + 1] - ax, dz = r->z[k + 1] - az;
            const float l2 = dx * dx + dz * dz;
            const float t = l2 > 1e-4f ? fmaxf(0.0f, fminf(1.0f, ((s->x - ax) * dx + (s->z - az) * dz) / l2)) : 0.0f;
            const float d = dist2d(s->x, s->z, ax + t * dx, az + t * dz);
            if (d < best) {
                best = d;
                ahead = t > 0.0f ? k + 1 : k;
            }
        }
        if (ahead > s->wpi) {
            s->nav.actual_m = fmaxf(s->nav.actual_m, s->nav.arc[ahead - 1]);
            s->nav.completed_m = fmaxf(s->nav.completed_m, s->nav.arc[ahead - 1]);
            s->nav.previous_segment = ahead - 1;
            s->nav.pose_ms = 0;
            s->wpi = ahead;
        }
    }
    const float x = r->x[s->wpi], z = r->z[s->wpi];
    const enum nav_path_mode path = s->route_interrupted ? NAV_REPATH_NOW : NAV_NATIVE;
    nav_propose(&s->nav, NAV_ROUTE, x, z, path, 1, 0, 0);
    s->route_interrupted = 0;
    nav_set_state(s, GNR_FOLLOWING, GTR_NONE, ms, 0);
    return 1;
}

/* ---- S2 commander: execution ------------------------------------------------
 * cmd_astar: 8-neighbour A* on the G32 cells (inside the battle polygon and
 * passable; a diagonal needs both side cells), step cost = travel time at
 * 8 m/s (x 1 + 0.5 vegetation share) + CMD_LAMBDA x EN + hidden of the cell +
 * CMD_MU x (1 - cover towards the enemy); at most CMD_ASTAR_NODES expansions.
 * The path is thinned towards CMD_WP points, keeping intervening turns up to
 * the route array's capacity when mapped clearance requires extra points.
 * An individual A* step with no clear straight connection can use native
 * pathing. If the route cannot fit, cmd_drive uses native pathing to its goal. */
static void cmd_heap_push(int *nh, float f, int i)
{
    int k = (*nh)++;
    while (k > 0) {
        int p = (k - 1) / 2;
        if (strat.aheap[p].f <= f)
            break;
        strat.aheap[k] = strat.aheap[p];
        k = p;
    }
    strat.aheap[k].f = f;
    strat.aheap[k].i = i;
}

static int cmd_heap_pop(int *nh)
{
    int top = strat.aheap[0].i, n = --*nh, k = 0;
    if (n <= 0)
        return top;
    float f = strat.aheap[n].f;
    int last = strat.aheap[n].i;
    for (;;) {
        int c = 2 * k + 1;
        if (c >= n)
            break;
        if (c + 1 < n && strat.aheap[c + 1].f < strat.aheap[c].f)
            ++c;
        if (strat.aheap[c].f >= f)
            break;
        strat.aheap[k] = strat.aheap[c];
        k = c;
    }
    strat.aheap[k].f = f;
    strat.aheap[k].i = last;
    return top;
}

static int cmd_astar(int army, float sx, float sz, float gx, float gz, float *wx, float *wz)
{
    double t_us = strat_now_us();
    float tx, tz;
    int s0 = cmd_snap(sx, sz, &tx, &tz), g0 = cmd_snap(gx, gz, &tx, &tz), found = 0, nh = 0, expanded = 0;
    if (s0 < 0 || g0 < 0 || s0 == g0)
        return 0;
    if (++strat.agen >= 0x7fffffffu) {
        memset(strat.astamp, 0, (size_t)strat.apath_cap * sizeof(uint32_t));
        strat.agen = 1;
    }
    const uint32_t open = 2 * strat.agen, closed = 2 * strat.agen + 1;
    const int nx = strat.nx, nz = strat.nz, fire = approach_on();
    const float step = strat.m->g32_cell / 8.0f, *enc = strat.enc[army];
    float *g = strat.ag;
    int *par = strat.apar, gr = g0 / nx, gc = g0 % nx;
    uint32_t *st = strat.astamp;
    g[s0] = 0.0f;
    par[s0] = -1;
    st[s0] = open;
    cmd_heap_push(&nh, 0.0f, s0);
    /* Under the threat field the costs differ far more from the heuristic, so
     * the search may open every cell before it gives up (each at most once: a
     * few ms on the largest map) rather than fall back to a straight drive. */
    const int budget = fire ? strat.n : CMD_ASTAR_NODES;
    while (nh > 0 && expanded < budget) {
        int i = cmd_heap_pop(&nh);
        if (st[i] == closed)
            continue;
        st[i] = closed;
        ++expanded;
        if (i == g0) {
            found = 1;
            break;
        }
        int r = i / nx, c = i % nx;
        for (int dr = -1; dr <= 1; ++dr)
            for (int dc = -1; dc <= 1; ++dc) {
                int rr = r + dr, cc = c + dc;
                if ((!dr && !dc) || rr < 0 || rr >= nz || cc < 0 || cc >= nx)
                    continue;
                int j = rr * nx + cc;
                if (st[j] == closed || (strat.cell[j].flags & 3) != 3)
                    continue;
                if (dr && dc && ((strat.cell[r * nx + cc].flags & 3) != 3 || (strat.cell[rr * nx + c].flags & 3) != 3))
                    continue;
                const strat_cell_t *cl = &strat.cell[j];
                const float len = dr && dc ? 1.4142f : 1.0f;
                float w = len * step * (1.0f + 0.5f * (float)cl->veg / 255.0f)
                          + CMD_LAMBDA * enc[j] + CMD_MU * (1.0f - (float)cl->cover[army - 1] / 255.0f);
                if (fire) {
                    /* Stage 3: time under fire costs AP_ALPHA times its length, for
                     * known and hidden enemies and remembered hits alike (the threat
                     * field). EN moves from the line above to here; what stays there
                     * is the chance of meeting a hidden enemy in the cell itself. An
                     * approach search also pays for cells on team-mates' routes. */
                    w += len * step * AP_ALPHA * threat_cell(army, j) - CMD_LAMBDA * th.k[army][j];
                    if (th.load_on)
                        w += AP_LOAD_CELL_S * th.load[j];
                }
                float ng = g[i] + w;
                if (st[j] != open || ng < g[j]) {
                    if (nh >= strat.aheap_cap)
                        continue;
                    g[j] = ng;
                    par[j] = i;
                    st[j] = open;
                    float hr = (float)(rr - gr), hc = (float)(cc - gc);
                    cmd_heap_push(&nh, ng + CMD_ASTAR_HW * step * sqrtf(hr * hr + hc * hc), j);
                }
            }
    }
    int n = 0;
    if (found) {
        int len = 0;
        for (int i = g0; i >= 0 && i != s0 && len < strat.n; i = par[i])
            strat.apath[len++] = i;             /* goal first */
        const int stride = (len + CMD_WP - 1) / CMD_WP;
        float x = sx, z = sz;
        int pos = 0;
        while (pos < len && n < MAX_TWP) {
            int next = pos + 1;
            for (int k = next; k <= len && k <= pos + stride; ++k) {
                const int i = strat.apath[len - k];
                const float px = k == len ? gx : strat.cell[i].x;
                const float pz = k == len ? gz : strat.cell[i].z;
                if (!movement_local_clear(x, z, px, pz))
                    break;
                next = k;
            }
            pos = next;
            const int i = strat.apath[len - pos];
            wx[n] = x = pos == len ? gx : strat.cell[i].x;
            wz[n] = z = pos == len ? gz : strat.cell[i].z;
            ++n;
        }
        if (pos < len)
            n = 0;
    }
    double us = strat_now_us() - t_us;
    cmd.as_sum += us;
    cmd.as_max = us > cmd.as_max ? us : cmd.as_max;
    ++cmd.astars;
    cmd.astar_fail += !n;
    return n;
}

static int cmd_human_pos(int army, int h, float *x, float *z)
{
    if (h < 0 || h >= 64 || !slot_alive[h] || slot_army[h] != army || !slot_unit[h])
        return 0;
    const float *p = gh_unit_pos(slot_unit[h]);
    *x = p[0];
    *z = p[2];
    return 1;
}

/* Build and verify off the active slot. A failed candidate cannot end the
 * previous route, advance a cursor, or start a new generation. */
static void cmd_route(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    s->cmd_replan_ms = ms + 20000;
    const float d = dist2d(s->x, s->z, s->cmd_x, s->cmd_z);
    if (d < CMD_ASTAR_MIN_M)
        return;
    nav_set_state(s, GNR_PLANNING, GTR_NONE, ms, 0);
    troute_t candidate = {0};
    candidate.n = cmd_astar(s->army, s->x, s->z, s->cmd_x, s->cmd_z, candidate.x, candidate.z);
    candidate.army = s->army;
    candidate.variant = TV_FAST;
    candidate.sx = s->x;
    candidate.sz = s->z;
    candidate.zx = s->cmd_x;
    candidate.zz = s->cmd_z;
    if (!nav_install_route(s, CMD_TR(seat), &candidate, 0, candidate.n, ms)) {
        nav_set_state(s, nav_route_active(s) ? GNR_REPLAN_PENDING : GNR_BLOCKED,
                      GTR_GEOMETRY_UNKNOWN, ms, s->cmd_replan_ms);
        return;
    }
    if (cmd.logs[s->army] < CMD_LOGS_PER_MIN)
        event(host, "route: seat %d takes a commander route to (%.0f,%.0f): %d waypoints",
              seat, s->cmd_x, s->cmd_z, candidate.n);
}

#include "ground_approach.inc"

static void cmd_post_start(void *host, lane_seat_t *s, int seat, cmd_post_t *p, int option, uint64_t ms)
{
    s->post_option = option;
    s->post_retry = 0;
    s->post_traffic_failures = 0;
    s->post_defending = 0;
    s->post_gx = p->kind ? p->pos[option].fx : p->gx[1];   /* v13: a choke / overwatch spot aims at its own point */
    s->post_gz = p->kind ? p->pos[option].fz : p->gz[1];
    s->cmd_parked = s->parked = s->bias_on = s->pusher = 0;
    s->post_wp = -1;
    s->post_progress_ms = ms;
    s->post_best = 1e18f;
    s->native_until = 0;
    s->post_in_place_ms = s->post_enemy_ms = s->post_hit_ms = 0;
    cmd_route(host, s, seat, ms);
    event(host, "cmd: army %d seat %d post %d cap %d position %d (%.0f,%.0f), faces (%.0f,%.0f)",
          s->army, seat, s->cmd_id, p->cap, option + 1, s->cmd_x, s->cmd_z, s->post_gx, s->post_gz);
    if (PK_PLAN(p->kind))
        event(host, "teamplan: army %d seat %d starts %s post %d (area ref %d slot %d) at (%.0f,%.0f), %.0f m away, faces "
              "(%.0f,%.0f)", s->army, seat, p->kind == PK_WATCH ? "watch" : "support", s->cmd_id, p->ref, p->line, s->cmd_x, s->cmd_z,
              dist2d(s->x, s->z, s->cmd_x, s->cmd_z), s->post_gx, s->post_gz);
    else if (p->kind)
        event(host, "roads: army %d seat %d starts %s post %d (%s %d, line %d) spot %d of %d at (%.0f,%.0f), %.0f m away, "
              "faces (%.0f,%.0f), fit %.2f%s", s->army, seat, p->kind == PK_CHOKE ? "choke" : "overwatch", s->cmd_id,
              p->kind == PK_CHOKE ? "choke" : "position", p->kind == PK_CHOKE ? rchokes[p->ref].id : rows[p->ref].id,
              p->line, option + 1, p->npos, s->cmd_x, s->cmd_z, dist2d(s->x, s->z, s->cmd_x, s->cmd_z), s->post_gx,
              s->post_gz, roads_fitw(s->army, seat, p->kind, p->ref), roads_turretless(seat) ? ", turretless" : "");
}

/* Each flank leg keeps its strategic destination. Search a bounded corridor
 * around that leg, using the 10 m layer for bends and the G32 grid for terrain
 * and fire coverage. The original lane step handles arrival and native fallback. */
#define FLANK_CORRIDOR_M 175.0f
#define FLANK_SEARCH_NODES 8000
#define FLANK_PLAN_GAP_MS 125
#define FLANK_RISK_CHECK_MS 1800
#define FLANK_RISK_RISE 0.16f
#define FLANK_SEGMENT_M 60.0f
typedef struct { int c0, r0, nx, nz, fine; float x0, z0, cs; } flank_grid_t;
typedef struct { float length, cost, peak; } flank_measure_t;

static int flank_grid_init(flank_grid_t *grid, float sx, float sz, float gx, float gz)
{
    const strat_meta_t *m = strat.m;
    int fine = strat.g10 && m->g10_cell > 0.0f;
    float cs = fine ? m->g10_cell : m->g32_cell;
    float x0 = fine ? m->g10_x0 : m->g32_x0, z0 = fine ? m->g10_z0 : m->g32_z0;
    int nx = fine ? strat.n10x : strat.nx, nz = fine ? strat.n10z : strat.nz;
    int c0 = (int)floorf((fminf(sx, gx) - FLANK_CORRIDOR_M - x0) / cs);
    int c1 = (int)floorf((fmaxf(sx, gx) + FLANK_CORRIDOR_M - x0) / cs);
    int r0 = (int)floorf((fminf(sz, gz) - FLANK_CORRIDOR_M - z0) / cs);
    int r1 = (int)floorf((fmaxf(sz, gz) + FLANK_CORRIDOR_M - z0) / cs);
    c0 = c0 < 0 ? 0 : c0;
    r0 = r0 < 0 ? 0 : r0;
    c1 = c1 >= nx ? nx - 1 : c1;
    r1 = r1 >= nz ? nz - 1 : r1;
    int w = c1 - c0 + 1, h = r1 - r0 + 1;
    if (w <= 0 || h <= 0 || (size_t)w * h > (size_t)strat.apath_cap || (fine && (size_t)w * h > FLANK_GRID_CELLS))
        return 0;
    *grid = (flank_grid_t){c0, r0, w, h, fine, x0, z0, cs};
    return 1;
}

static int flank_grid_cell(const flank_grid_t *grid, float x, float z)
{
    int c = (int)floorf((x - grid->x0) / grid->cs) - grid->c0;
    int r = (int)floorf((z - grid->z0) / grid->cs) - grid->r0;
    return r < 0 || r >= grid->nz || c < 0 || c >= grid->nx ? -1 : r * grid->nx + c;
}

static int flank_grid_point(const flank_grid_t *grid, int i, float *x, float *z)
{
    int r = i / grid->nx + grid->r0, c = i % grid->nx + grid->c0;
    if (grid->fine) {
        const u8 *g = strat.g10 + (size_t)(r * strat.n10x + c) * 8;
        *x = grid->x0 + ((float)c + 0.5f) * grid->cs;
        *z = grid->z0 + ((float)r + 0.5f) * grid->cs;
        return (g[4] & 3) == 3 && g[5] <= 6;
    }
    const strat_cell_t *cl = &strat.cell[r * strat.nx + c];
    *x = cl->x;
    *z = cl->z;
    return (cl->flags & 3) == 3;
}

static float flank_cost(const position_context_t *ctx, float x, float z, float *exposure)
{
    int c32 = strat_cellof(x, z);
    int c10 = strat.g10 ? cover_cell_of(x, z, NULL, NULL) : -1;
    position_value_t v = position_value(ctx, x, z, c32, c10, 1);
    float veg = c32 >= 0 ? (float)strat.cell[c32].veg / 255.0f : 0.0f;
    float uncertain = c32 >= 0 ? fminf(1.0f, strat.enc[ctx->army][c32]) : 0.25f;
    *exposure = v.exposure + 0.24f * uncertain;
    /* Charge exposure independently. Firing opportunity and allied support
     * have bounded weights alongside the terrain and concealment costs. */
    return 1.0f + 0.5f * veg + 2.5f * *exposure
           + 0.42f * (1.0f - v.hard) + 0.10f * (1.0f - v.conceal)
           + 0.18f * (1.0f - v.fire) + 0.08f * (1.0f - v.support);
}

static void flank_measure_segment(const position_context_t *ctx, flank_measure_t *sum,
                                  float sx, float sz, float gx, float gz)
{
    float d = dist2d(sx, sz, gx, gz);
    int steps = (int)ceilf(d / 15.0f);
    for (int k = 0; k < steps; ++k) {
        float f = ((float)k + 0.5f) / (float)steps, exposure;
        float cost = flank_cost(ctx, sx + (gx - sx) * f, sz + (gz - sz) * f, &exposure);
        sum->cost += d / (float)steps * cost;
        sum->peak = fmaxf(sum->peak, exposure);
    }
    sum->length += d;
}

static flank_measure_t flank_measure_route(const position_context_t *ctx, float x, float z, const troute_t *r, int wp)
{
    flank_measure_t sum = {0};
    for (int k = wp; k < r->n; ++k) {
        flank_measure_segment(ctx, &sum, x, z, r->x[k], r->z[k]);
        x = r->x[k];
        z = r->z[k];
    }
    return sum;
}

static int flank_goal(const flank_grid_t *grid, const lane_seat_t *s, const position_context_t *ctx)
{
    int anchor = flank_grid_cell(grid, s->tx, s->tz);
    if (anchor < 0)
        return -1;
    /* Pass points can be approached off centre. A held point still has to be
     * reached through its usual, smaller lane arrival radius. */
    float radius = s->route[s->step].hold == H_NONE ? 35.0f : HOLD_ARRIVE_M;
    int span = (int)ceilf(radius / grid->cs), r0 = anchor / grid->nx, c0 = anchor % grid->nx;
    int best = -1;
    float best_cost = 1e18f;
    for (int dr = -span; dr <= span; ++dr)
        for (int dc = -span; dc <= span; ++dc) {
            int r = r0 + dr, c = c0 + dc, i = r * grid->nx + c;
            float x, z;
            if (r < 0 || r >= grid->nz || c < 0 || c >= grid->nx || !flank_grid_point(grid, i, &x, &z))
                continue;
            float d = dist2d(x, z, s->tx, s->tz);
            int c32 = strat_cellof(x, z);
            if (d > radius || c32 < 0 || (strat.cell[c32].flags & 3) != 3)
                continue;
            float exposure;
            float cost = d + 11.0f * flank_cost(ctx, x, z, &exposure);
            if (cost < best_cost) {
                best_cost = cost;
                best = i;
            }
        }
    return best;
}

static int flank_thin(const flank_grid_t *grid, const position_context_t *ctx,
                      const lane_seat_t *s, int start, int goal, troute_t *route)
{
    int len = 0;
    for (int i = goal; i >= 0 && i != start && len < strat.apath_cap; i = strat.apar[i])
        strat.apath[len++] = i;
    if (!len)
        return 0;
    int n = 0, pos = 0;
    float x = s->x, z = s->z;
    while (pos < len && n < MAX_TWP) {
        int next = pos + 1;
        float px, pz;
        flank_grid_point(grid, strat.apath[len - next], &px, &pz);
        flank_measure_t bends = {0};
        flank_measure_segment(ctx, &bends, x, z, px, pz);
        float last_x = px, last_z = pz;
        for (int k = next + 1; k <= len; ++k) {
            float tx, tz;
            flank_grid_point(grid, strat.apath[len - k], &tx, &tz);
            flank_measure_segment(ctx, &bends, last_x, last_z, tx, tz);
            if (bends.length > FLANK_SEGMENT_M || !movement_local_clear(x, z, tx, tz))
                break;
            flank_measure_t shortcut = {0};
            flank_measure_segment(ctx, &shortcut, x, z, tx, tz);
            if (shortcut.cost > 1.06f * bends.cost || shortcut.peak > bends.peak + 0.12f)
                break;
            next = k;
            px = last_x = tx;
            pz = last_z = tz;
        }
        pos = next;
        route->x[n] = x = px;
        route->z[n] = z = pz;
        ++n;
    }
    return pos == len ? n : 0;
}

static int flank_astar(const lane_seat_t *s, const position_context_t *ctx, troute_t *route)
{
    flank_grid_t grid;
    float sx = s->x, sz = s->z;
    if (!flank_grid_init(&grid, sx, sz, s->tx, s->tz))
        return 0;
    int start = flank_grid_cell(&grid, sx, sz), goal = flank_goal(&grid, s, ctx);
    if (start < 0 || goal < 0 || start == goal)
        return 0;
    float gx, gz;
    flank_grid_point(&grid, goal, &gx, &gz);
    if (++strat.agen >= 0x7fffffffu) {
        memset(strat.astamp, 0, (size_t)strat.apath_cap * sizeof(uint32_t));
        strat.agen = 1;
    }
    uint32_t open = 2 * strat.agen, closed = open + 1;
    uint32_t *stamp = strat.astamp;
    int nh = 0, expanded = 0;
    strat.ag[start] = 0.0f;
    strat.apar[start] = -1;
    stamp[start] = open;
    cmd_heap_push(&nh, 0.0f, start);
    while (nh > 0 && expanded < FLANK_SEARCH_NODES) {
        int i = cmd_heap_pop(&nh);
        if (stamp[i] == closed)
            continue;
        stamp[i] = closed;
        ++expanded;
        if (i == goal)
            return flank_thin(&grid, ctx, s, start, goal, route);
        int r = i / grid.nx, c = i % grid.nx;
        float x = sx, z = sz;
        if (i != start)
            flank_grid_point(&grid, i, &x, &z);
        for (int dr = -1; dr <= 1; ++dr)
            for (int dc = -1; dc <= 1; ++dc) {
                int rr = r + dr, cc = c + dc;
                if ((!dr && !dc) || rr < 0 || rr >= grid.nz || cc < 0 || cc >= grid.nx)
                    continue;
                int j = rr * grid.nx + cc;
                float tx, tz;
                if (stamp[j] == closed || !flank_grid_point(&grid, j, &tx, &tz))
                    continue;
                float d = dist2d(x, z, tx, tz);
                if (stamp[j] == open && strat.ag[i] + d * strat.acost[j] >= strat.ag[j])
                    continue;
                if (nh >= strat.aheap_cap || position_leg_distance(tx, tz, sx, sz, s->tx, s->tz) > FLANK_CORRIDOR_M
                    || !movement_local_clear(x, z, tx, tz))
                    continue;
                if (stamp[j] != open) {
                    float exposure;
                    strat.acost[j] = flank_cost(ctx, tx, tz, &exposure);
                }
                float cost = strat.ag[i] + d * strat.acost[j];
                if (i == start)
                    cost += 0.15f * movement_turn_error(s, tx, tz);
                strat.ag[j] = cost;
                strat.apar[j] = i;
                stamp[j] = open;
                cmd_heap_push(&nh, cost + CMD_ASTAR_HW * dist2d(tx, tz, gx, gz), j);
            }
    }
    return 0;
}

static void flank_route_update(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (strat.state != 1 || lanes[s->lane].kind == 0 || s->cmd_type >= OB_SUPPORT || s->holding || s->task
        || motion_combat_override(s, ms) || ms < s->native_until || ms < s->direct_off_until || ms < s->route_turn_until)
        return;
    if (s->flank_lane != s->lane || s->flank_step != s->step) {
        s->flank_lane = s->lane;
        s->flank_step = s->step;
        s->flank_attempted = s->flank_replanned = 0;
        s->flank_generation = 0;
        s->flank_check_ms = 0;
    }
    int active = route_is_flank(s, seat) && s->wpi < s->wpn;
    if (active && s->motion_cmd_ms && ms < s->motion_cmd_ms + CMD_MANEUVER_MS)
        return;
    if (ms < s->flank_check_ms || ms < strat.flank_plan_ms || s->flank_replanned
        || dist2d(s->x, s->z, s->tx, s->tz) < (s->route[s->step].hold == H_NONE ? ROLL_M : 35.0f))
        return;
    /* If the spawn position had no mapped connection, native movement can
     * reach a usable start cell. Retry once after meaningful displacement. */
    if (!active && s->flank_attempted && (s->flank_generation || s->flank_attempted >= 2
        || dist2d(s->x, s->z, s->flank_attempt_x, s->flank_attempt_z) < 45.0f))
        return;

    position_context_t ctx;
    position_context(&ctx, s->army, seat, s->x, s->z, s->tx, s->tz, ms, 1);
    flank_measure_t old = {0};
    if (active) {
        s->flank_check_ms = ms + FLANK_RISK_CHECK_MS;
        old = flank_measure_route(&ctx, s->x, s->z, &troutes[s->tr], s->wpi);
        uint64_t hit = hit_dir[seat].ms;
        int urgent = hit && ms >= hit && ms - hit < 4000;
        float rise = urgent ? 0.09f : FLANK_RISK_RISE;
        if (old.peak < fmaxf(urgent ? 0.22f : 0.26f, s->flank_exposure + rise))
            return;
        s->flank_replanned = 1;
    } else {
        ++s->flank_attempted;
        s->flank_attempt_x = s->x;
        s->flank_attempt_z = s->z;
        s->flank_check_ms = ms + FLANK_RISK_CHECK_MS;
    }
    strat.flank_plan_ms = ms + FLANK_PLAN_GAP_MS;
    troute_t next = {0};
    double start_us = strat_now_us();
    next.n = flank_astar(s, &ctx, &next);
    double elapsed_us = strat_now_us() - start_us;
    cmd.as_sum += elapsed_us;
    cmd.as_max = fmax(cmd.as_max, elapsed_us);
    ++cmd.astars;
    cmd.astar_fail += !next.n;
    if (!next.n)
        return;
    flank_measure_t trial = flank_measure_route(&ctx, s->x, s->z, &next, 0);
    if (active && (trial.length > 1.25f * old.length + 70.0f
        || (trial.cost > 0.94f * old.cost && (trial.peak > old.peak - 0.18f || trial.cost > 1.05f * old.cost))
        || (s->vx * s->vx + s->vz * s->vz > 4.0f && movement_turn_error(s, next.x[0], next.z[0]) > 110.0f)))
        return;

    next.army = s->army;
    next.variant = TV_FLANK;
    next.sx = s->x;
    next.sz = s->z;
    next.zx = s->tx;
    next.zz = s->tz;
    if (!nav_install_route(s, CMD_TR(seat), &next, 0, next.n, ms))
        return;
    s->flank_generation = s->route_generation;
    s->flank_exposure = trial.peak;
    s->flank_check_ms = ms + FLANK_RISK_CHECK_MS;
    s->native_tried = s->route_native_only = 0;
    s->route_interrupted = 1;
    if (cmd.logs[s->army] < CMD_LOGS_PER_MIN) {
        cmd_log(host, s->army, 0, "route: seat %d flank lane %d step %d %s: %d waypoints, %.0f m, peak exposure %.2f",
                seat, s->lane, s->step, active ? "replanned" : "planned", s->wpn, trial.length, trial.peak);
    }
}

/* Navigation accepts or defers a due order. The commander commits its
 * mirrors after receiving the receipt. */
static void cmd_apply(void *host, lane_seat_t *s, int seat, uint64_t ms, int clearing)
{
    const ground_task_order_t *pending = cmd_pending_order(seat, ms);
    if (!pending)
        return;
    const ground_task_order_t order = *pending;
    const int invalid = cmd_validate_pending(host, s, seat, ms, pending);
    if (invalid != GTR_NONE) {
        cmd_task_receipt(host, s, seat, ms, pending, GTR_REJECTED, invalid, 0);
        return;
    }
    const ground_task_order_t *old = cmd_active_order(seat);
    /* s37 G9: a plan lane change within the same zone is a revision of the same capture task (same type, objective,
     * lane, position); its new lane slot target makes it another goal, else s->goal_target1 below kept the old slot */
    const int same_goal = old && old->task_id == order.task_id && old->type == order.type
                          && old->objective == order.objective && old->lane == order.lane
                          && old->post_option == order.post_option && old->goal_target1 == order.goal_target1
                          && dist2d(old->x, old->z, order.x, order.z) <= 0.5f;
    if (!same_goal && (ms < s->l1_until || clearing)) {
        const uint64_t retry = ms + NAV_FEEDBACK_MS;
        cmd_task_receipt(host, s, seat, ms, pending, GTR_DEFERRED, GTR_CLEARANCE, retry);
        return;
    }
    int contested = 0;
    const int point = fight_in_point(host, s, &contested);
    if (!same_goal && (order.reason == GTR_NONE || order.reason == GTR_SOFT_EXIT)
        && (cmd_combat_busy(s, ms) || fight_committed(s, ms)
            || (contested && (order.type != OB_CAPTURE || order.lane != point)))) {
        cmd_task_receipt(host, s, seat, ms, pending, GTR_DEFERRED, GTR_COMBAT, ms + NAV_FEEDBACK_MS);
        return;
    }
    cmd_post_t *post = NULL;
    if (order.type == OB_OVERWATCH && order.objective >= CMD_POST_ID_BASE) {
        const int index = order.objective - CMD_POST_ID_BASE;
        if (index >= cmd_nposts[s->army] || order.post_option < 0
            || order.post_option >= cmd_posts[s->army][index].npos) {
            cmd_task_receipt(host, s, seat, ms, pending, GTR_REJECTED, GTR_INVALID, 0);
            return;
        }
        post = &cmd_posts[s->army][index];
    }
    if (((order.type == OB_CAPTURE || order.type == OB_HOLD) && (order.lane < 0 || order.lane >= nlanes))
        || (order.type != OB_NONE && (!isfinite(order.x) || !isfinite(order.z) || order.radius <= 0.0f))) {
        cmd_task_receipt(host, s, seat, ms, pending, GTR_REJECTED, GTR_INVALID, 0);
        return;
    }

    const int same_point = (order.type == OB_CAPTURE || order.type == OB_HOLD) && s->cmd_type < OB_SUPPORT
                           && cmd_lane_point(s->lane) == order.lane;
    cmd_task_receipt(host, s, seat, ms, pending, GTR_ACCEPTED, GTR_NONE, 0);
    const ground_task_order_t *active = cmd_active_order(seat);
    if (order.type != OB_NONE && (!active || active->task_id != order.task_id || active->revision != order.revision))
        return;
    s->nav.task_id = order.type != OB_NONE ? order.task_id : 0;
    s->nav.revision = order.type != OB_NONE ? order.revision : 0;
    if (!same_goal) {
        s->route_variant = order.route_variant;     /* s32: decided with the order, kept while it lives */
        /* s35: the lane slot's library target; s37: none for a release (it copies the active order's fields, and the
         * lane default it then drives is no lane slot) */
        s->goal_target1 = order.type != OB_NONE ? order.goal_target1 : 0;
    }
    if (same_point || same_goal) {
        s->nav.route_task_id = s->nav.task_id;
        s->nav.route_revision = s->nav.revision;
        return;
    }
    nav_retire_route(s);
    s->nav.blocked_since_ms = 0;
    nav_set_state(s, order.type == OB_NONE ? GNR_CANCELLED : GNR_NEED_ROUTE,
                  order.type == OB_NONE ? order.reason : GTR_NONE, ms, 0);
    s->task = s->stopping = s->close = s->rear = s->fight_post = s->fight_move = 0;
    if (s->cov_use == COV_FIRE || s->cov_use == COV_CLOSE)
        cover_end(host, s, seat, ms, "new order");
    s->holding = s->cmd_parked = s->parked = 0;
    s->cov_dest_ok = 0;
    s->cov_dest_ms = 0;
    s->motion_cmd_ms = ms;
    s->ax = s->x;
    s->az = s->z;
    s->anchor_ms = ms;
    s->spot = -1;
    if (order.type == OB_NONE) {
        lane_goto_step(host, s, seat, lane_slot_step(s), ms, "order cancelled");
    } else if (order.type == OB_CAPTURE || order.type == OB_HOLD) {
        s->lane = order.lane;
        s->role = LROLE_ASSAULT;
        lane_route(s, &lanes[s->lane]);
        if (ms - s->born_ms < 45000 && dist2d(s->x, s->z, s->sx, s->sz) < 250.0f) {
            s->step = 0;
            pick_troute(host, s, seat);
            lane_begin_step(s, seat, ms);
        } else {
            s->sx = s->x;
            s->sz = s->z;
            lane_goto_step(host, s, seat, lane_slot_step(s), ms, "commander");
        }
    } else if (post) {
        cmd_post_start(host, s, seat, post, order.post_option, ms);
    } else {
        cmd_route(host, s, seat, ms);
    }
}

static void cmd_park(lane_seat_t *s, float x, float z, uint64_t ms)
{
    nav_propose(&s->nav, NAV_LANE, x, z, NAV_PARK, 0, 1, 0);
    s->cmd_parked = 1;
    s->ax = s->x;
    s->az = s->z;
    s->anchor_ms = ms;
}

/* s48: the post leave rules that were here (roads_post_leave, roads_post_check) are in ground_command.c
 * (ground_command_loop.inc). */

static void cmd_post_fail(void *host, lane_seat_t *s, int seat, uint64_t ms, const char *reason)
{
    if (!s->cmd_apply_ms)
        cmd_request_post_resume(host, s, seat, ms, GTR_POST_INVALID);
    nav_set_state(s, GNR_BLOCKED, GTR_POST_INVALID, ms, s->cmd_apply_ms);
    nav_propose(&s->nav, NAV_LANE, s->x, s->z, NAV_PARK, 0, 0, 0);
    if (s->nav.reported_reason != GTR_POST_INVALID)
        event(host, "cmd: army %d seat %d post %d position %d invalid (%s)", s->army, seat, s->cmd_id,
              s->post_option + 1, reason);
}

static int cmd_post_drive(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    cmd_post_t *p = &cmd_posts[s->army][s->cmd_id - CMD_POST_ID_BASE];
    const float d = dist2d(s->x, s->z, s->cmd_x, s->cmd_z);
    const int gone = cmd_tasks[seat].invalid_reason;
    if (p->kind && s->cmd_apply_ms && (gone == GTR_POST_IDLE || gone == GTR_POST_HIT)) {
        nav_propose(&s->nav, NAV_LANE, s->x, s->z, NAV_PARK, 0, 0, 0);     /* v13: left by roads_post_leave, next order pending */
        return 1;
    }
    if (ms < p->pos[s->post_option].blocked_until
        || (!PK_PLAN(p->kind) && !cmd_post_standable(s->cmd_x, s->cmd_z, s->post_gx, s->post_gz))) {
        /* s32: a watch slot is a library target from the plan pack: not re-checked against the cover layer */
        cmd_post_fail(host, s, seat, ms, "position obstructed");
        return 1;
    }
    if (d <= CMD_POST_ARRIVE_M + (s->cmd_parked ? 3.0f : 0.0f)
        && control_static_connector(s, s->x, s->z, s->cmd_x, s->cmd_z) == NAV_CLEAR) {
        if (!s->cmd_parked)
            cmd_log(host, s->army, 1, "cmd: army %d seat %d in position for post %d cap %d", s->army, seat, s->cmd_id, p->cap);
        cmd_park(s, s->cmd_x, s->cmd_z, ms);
        s->post_progress_ms = ms;
        s->post_best = 1e18f;
        if (!s->face) {
            s->face = 1;
            s->face_x = s->post_gx;
            s->face_z = s->post_gz;
        }
        if (p->kind)
            roads_post_check(host, s, seat, p, ms);    /* v13: the leave rules of choke / overwatch posts */
        return 1;
    }
    s->cmd_parked = 0;
    const int wp = nav_route_active(s) ? s->wpi : -1;
    const float left = wp < 0 ? d : dist2d(s->x, s->z, troutes[s->tr].x[wp], troutes[s->tr].z[wp]);
    if (wp != s->post_wp || left + 2.0f < s->post_best || (nav_recent(s->nav.feedback_ms, ms) && s->nav.waiting)) {
        s->post_wp = wp;
        s->post_best = left;
        s->post_progress_ms = ms;
    }
    if (ms - s->post_progress_ms > CMD_POST_RETRY_MS) {
        if (s->post_retry) {
            cmd_post_fail(host, s, seat, ms, "no route progress after native retry");
            return 1;
        }
        s->post_retry = 1;
        s->post_progress_ms = ms;
        s->route_interrupted = 1;
        if (!nav_route_active(s))
            cmd_route(host, s, seat, ms);
    }
    if (follow_troute(host, s, seat, ms))
        return 1;
    nav_propose(&s->nav, NAV_LANE, s->cmd_x, s->cmd_z, NAV_NATIVE, 1, 0, 0);
    return 1;
}

/* CAPTURE and HOLD use the approach lane. Other orders use their region. */
static int cmd_drive(void *host, lane_seat_t *s, int seat, uint64_t ms, uint64_t still)
{
    if (!cmd_on() || s->cmd_type < OB_SUPPORT)
        return 0;
    if (cmd_is_post(s))
        return cmd_post_drive(host, s, seat, ms);
    if (s->cmd_type == OB_STAY) {       /* stage 2: immobilised, fights where it stands (no cover hop) */
        cmd_park(s, s->cmd_x, s->cmd_z, ms);
        return 1;
    }
    if (s->cmd_type == OB_SUPPORT) {
        float hx, hz;
        if (cmd_human_pos(s->army, s->cmd_id, &hx, &hz)
            && strat_depth(s->army, s->x, s->z) > strat_depth(s->army, hx, hz) - 20.0f
            && dist2d(s->x, s->z, hx, hz) < 250.0f) {
            cmd_park(s, s->x, s->z, ms);
            return 1;
        }
    }
    float gx = s->cmd_x, gz = s->cmd_z;
    if (s->fight_post && dist2d(gx, gz, s->fight_post_ox, s->fight_post_oz) > CMD_ARRIVE_M)
        s->fight_post = 0;
    int gc = 0;
    if (s->fight_post) {
        gx = s->fight_post_x;
        gz = s->fight_post_z;
    } else {
        gc = cover_dest(host, s, seat, ms, &gx, &gz);
    }
    const ground_task_order_t *order = cmd_active_order(seat);
    if (order && dist2d(gx, gz, order->x, order->z) > order->radius) {
        gx = order->x;
        gz = order->z;
        gc = 0;
    }
    const float d = dist2d(s->x, s->z, gx, gz);
    const float arrival = gc ? COV_ARRIVE_M : CMD_ARRIVE_M;
    if (d < arrival && control_static_connector(s, s->x, s->z, gx, gz) == NAV_CLEAR) {
        if (!s->cmd_parked)
            cmd_log(host, s->army, 0, "cmd: army %d seat %d in position for %s %d%s", s->army, seat, cmd_name[s->cmd_type],
                    s->cmd_id, gc ? " (cover)" : "");
        const int row = s->cmd_type == OB_OVERWATCH && s->cmd_id >= 0 && s->cmd_id < STRAT_MAXB && s->army >= 1 && s->army <= 2
                        ? roads.band_ow[s->army][s->cmd_id] - 1 : -1;
        if (row >= 0 && !s->face) {         /* v13: a precomputed band overwatch faces its field of fire */
            s->face = 1;
            s->face_x = rows[row].fx;
            s->face_z = rows[row].fz;
        }
        cmd_park(s, gx, gz, ms);
        return 1;
    }
    s->cmd_parked = 0;
    if (motion_native_repath_allowed(s, seat, ms) && !nav_route_active(s)
        && still > STILL_MS && ms >= s->cmd_replan_ms && d > CMD_ASTAR_MIN_M)
        cmd_route(host, s, seat, ms);
    if (approach_propose(host, s, seat, ms))      /* stage 3: a stop in cover before a stretch under fire */
        return 1;
    if (follow_troute(host, s, seat, ms))
        return 1;
    lane_drive(s, gx, gz, ms);
    return 1;
}


/* Reuse the unpassed waypoint or the lane's current leg. Parked objectives
 * have no remaining leg even if a completed A* connection is still cached. */
static int fight_route_goal(const lane_seat_t *s, float *x, float *z)
{
    if (s->holding || s->cmd_parked)
        return 0;
    if (nav_route_active(s)) {
        const troute_t *r = &troutes[s->tr];
        *x = r->x[s->wpi];
        *z = r->z[s->wpi];
        return 1;
    }
    *x = s->cmd_type >= OB_SUPPORT ? s->cmd_x : s->tx;
    *z = s->cmd_type >= OB_SUPPORT ? s->cmd_z : s->tz;
    return dist2d(s->x, s->z, *x, *z) > CMD_ARRIVE_M;
}

static int fight_progress(void *host, lane_seat_t *s, int seat, uint64_t ms, const char *reason)
{
    if (s->fight_move || ms < s->fight_retry_ms || (s->cmd_apply_ms && ms >= s->cmd_apply_ms)
        || motion_committed(s, ms) || ms < s->native_until || (s->cov_use && s->cov_state == 1))
        return 0;
    float gx, gz;
    const lane_t *l = &lanes[s->lane];
    int cap = s->cmd_type < OB_SUPPORT && zone_owner(host, l->zx, l->zz) != s->army;
    float d = dist2d(s->x, s->z, s->ex, s->ez);
    int route = fight_route_goal(s, &gx, &gz);
    /* s28 takeover: a checked reposition towards the enemy leaves cover to fight: only when the tank sees it,
     * this life can engage, the gunner holds the same target and the task allows it (route continuation stays). */
    const char *gate = NULL;
    const int local_ok = !cfg.takeover || tac_advance_ok(s, seat, ms, &gate);
    if (route && s->fight_last != FIGHT_ROUTE
        && (cap || dist2d(gx, gz, s->ex, s->ez) < d + 15.0f)) {
        s->fight_move = FIGHT_ROUTE;
    } else if (!local_ok) {
        event(host, "takeover: seat %d no checked reposition for slot %d: %s", seat, s->eng, gate ? gate : "?");
    } else if (d > 1.0f && strat.state == 1) {
        const float ux = (s->ex - s->x) / d, uz = (s->ez - s->z) / d;
        const float r = fminf(FIGHT_LOCAL_M, fmaxf(18.0f, 0.35f * d));
        float best = -1e9f;
        int contested = 0, point = fight_in_point(host, s, &contested);
        const int positional = s->holding || s->cmd_parked;
        const float ox = s->cmd_type >= OB_SUPPORT ? s->cmd_x : s->tx;
        const float oz = s->cmd_type >= OB_SUPPORT ? s->cmd_z : s->tz;
        const float post_radius = fmaxf(80.0f, dist2d(s->x, s->z, ox, oz));
        float hx = 0.0f, hz = 0.0f;
        const int support = s->cmd_type == OB_SUPPORT && cmd_human_pos(s->army, s->cmd_id, &hx, &hz);
        position_context_t ctx;
        position_context(&ctx, s->army, seat, s->x, s->z, s->ex, s->ez, ms, 1);
        for (int i = -3; i <= 3; ++i) {
            /* Test a forward fan, including lateral moves that change the line of fire. */
            const float a = (float)i * 0.48f, dx = cosf(a) * ux - sinf(a) * uz, dz = sinf(a) * ux + cosf(a) * uz;
            for (int j = 0; j < 2; ++j) {
                const float reach = j ? r * 0.65f : r, x = s->x + reach * dx, z = s->z + reach * dz;
                const float gain = d - dist2d(x, z, s->ex, s->ez);
                if ((point >= 0 && (contested || positional)
                     && dist2d(x, z, lanes[point].zx, lanes[point].zz) >= lanes[point].zr)
                    || (positional && point < 0 && dist2d(x, z, ox, oz) > post_radius)
                    || (support && strat_depth(s->army, x, z) > strat_depth(s->army, hx, hz) - 20.0f)
                    || (gain < 5.0f && abs(i) < 2) || cover_hop_failed(s, x, z, ms)
                    || (s->fight_last == FIGHT_LOCAL && dist2d(x, z, s->fight_last_x, s->fight_last_z) < 14.0f))
                    continue;
                float progress = route ? dist2d(s->x, s->z, gx, gz) - dist2d(x, z, gx, gz) : 0.0f;
                const int c10 = strat.g10hull ? cover_cell_of(x, z, NULL, NULL) : -1;
                position_value_t v = position_value(&ctx, x, z, strat_cellof(x, z), c10, 1);
                float task_gain = cap ? 0.030f + 0.025f * sview[s->army].point_pressure : 0.012f;
                float score = 0.035f * gain + 0.024f * fabsf(sinf(a)) * reach + task_gain * progress
                              + 0.6f * v.hard + 0.4f * v.fire - 0.9f * v.exposure
                              - 0.004f * movement_turn_error(s, x, z);
                if (score > best && control_static_connector(s, s->x, s->z, x, z) != NAV_BLOCKED) {
                    best = score;
                    s->fight_x = x;
                    s->fight_z = z;
                    s->fight_move = FIGHT_LOCAL;
                }
            }
        }
    }
    if (!s->fight_move && route && s->fight_last != FIGHT_ROUTE) {
        s->fight_move = FIGHT_ROUTE;
    }
    if (!s->fight_move) {
        s->fight_retry_ms = ms + FIGHT_RETRY_MS;
        s->fight_last = 0;
        event(host, "react: seat %d has no checked reposition for slot %d (%s)", seat, s->eng, reason);
        return 0;
    }
    s->fight_move_until = ms + (s->fight_move == FIGHT_ROUTE ? FIGHT_ROUTE_MS : 14000);
    s->fight_best = s->fight_move == FIGHT_LOCAL ? dist2d(s->x, s->z, s->fight_x, s->fight_z) : 0.0f;
    s->ax = s->x;
    s->az = s->z;
    s->anchor_ms = ms;
    event(host, "react: seat %d %s for slot %d (%s)", seat,
          s->fight_move == FIGHT_ROUTE ? "continues its route" : "starts a checked reposition", s->eng, reason);
    return 1;
}

static int fight_drive(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    if (!s->fight_move || ms < s->l1_until || ms < s->direct_off_until)
        return 0;
    if (cfg.takeover && s->fight_move == FIGHT_LOCAL && ms < s->fight_move_until) {
        /* s28: sight lost, the gunner on someone else, or this life cannot fire now: the advance ends at once */
        const char *abort = tac_advance_abort(s, seat, ms);
        if (abort) {
            event(host, "takeover: seat %d advance on slot %d ends: %s", seat, s->eng, abort);
            s->fight_move_until = ms;
        }
    }
    const float left = dist2d(s->x, s->z, s->fight_x, s->fight_z);
    if (s->fight_move == FIGHT_LOCAL && left + 3.0f < s->fight_best) {
        s->fight_best = left;
        s->anchor_ms = ms;
    }
    const int arrived = s->fight_move == FIGHT_LOCAL && left <= COV_ARRIVE_M;
    const int stalled = s->fight_move == FIGHT_LOCAL && ms >= s->anchor_ms && ms - s->anchor_ms > 6500;
    if (ms >= s->fight_move_until || arrived || stalled) {
        s->fight_last = s->fight_move;
        s->fight_last_x = s->fight_x;
        s->fight_last_z = s->fight_z;
        s->fight_move = 0;
        s->fight_retry_ms = ms + FIGHT_RETRY_MS;
        if (arrived && (s->cmd_parked || s->holding)) {
            s->fight_post = 1;
            s->fight_post_x = s->x;
            s->fight_post_z = s->z;
            s->fight_post_ox = s->cmd_type >= OB_SUPPORT ? s->cmd_x : s->tx;
            s->fight_post_oz = s->cmd_type >= OB_SUPPORT ? s->cmd_z : s->tz;
        }
        if (s->fight_last == FIGHT_LOCAL) {
            s->route_interrupted = 1;
            s->anchor_ms = ms;
            s->ax = s->x;
            s->az = s->z;
        }
        event(host, "react: seat %d combat movement %s, resumes its order", seat,
              arrived ? "reached position" : stalled ? "stalled" : "window over");
        return 0;
    }
    if (s->fight_move != FIGHT_LOCAL)
        return 0;
    nav_propose(&s->nav, NAV_FIGHT, s->fight_x, s->fight_z, NAV_REPATH_NOW, 1, 0, 0);
    return 1;
}

/* The legacy lane legs remain available when no tactical route has been
 * installed. Only a checked transition can bypass a rolling point. */
static int nav_lane_portal(const lane_seat_t *s, int seat, float distance)
{
    if (distance > NAV_PORTAL_M || s->step + 1 >= s->n)
        return 0;
    float x, z;
    lane_point_xz(s, seat, s->step + 1, &x, &z);
    return movement_turn_error(s, x, z) <= ROUTE_TURN_START_DEG
           && control_connector_prefix(s, x, z) != NAV_BLOCKED;
}

static void lanes_propose(void *host, lane_seat_t *s, int seat, uint64_t ms, int clearing)
{
    nav_state_t *nav = &s->nav;
    const int feedback = nav_recent(nav->feedback_ms, ms);
    const int waiting = feedback && nav->waiting;
    const lane_t *lane = &lanes[s->lane];
    if (tp_wait_first_order(s, seat, ms)) {
        /* s33: a plan army's tank holds on its spawn until the plan's first order (no lane default) */
        nav_propose(nav, NAV_LANE, s->x, s->z, NAV_PARK, 0, 0, 0);
        return;
    }
    if (!cmd_on() && !waiting && !clearing && ms >= s->retarget_next) {
        s->retarget_next = ms + 1000;
        if (!s->holding && !s->task && !motion_committed(s, ms) && ms - s->born_ms > 3000
            && dist2d(s->x, s->z, lane->zx, lane->zz) > lane->zr + RETARGET_M
            && zone_owner(host, lane->zx, lane->zz) == s->army
            && lane_advance(host, s, seat, ms, 1))
            lane = &lanes[s->lane];
    }
    const uint64_t still = !waiting && !clearing && !motion_combat_override(s, ms)
                           && ms > s->anchor_ms ? ms - s->anchor_ms : 0;
    if (cmd_drive(host, s, seat, ms, still))
        return;
    if (!waiting && !clearing && !motion_committed(s, ms))
        flank_route_update(host, s, seat, ms);
    /* Stage 3: a covered route for a tank that would drive straight at its
     * point, and the stops in cover before stretches under fire. */
    if (!clearing && approach_propose(host, s, seat, ms))
        return;
    if (follow_troute(host, s, seat, ms))
        return;

    route_step_t *step = &s->route[s->step];
    if (s->holding) {
        int release = 0;
        const float held = (float)(ms - s->hold_start) / 1000.0f;
        if (step->hold == H_FOR && (held > step->secs || s->cmd_release))
            release = 1;
        if (step->hold == H_UNTIL_OWNED && (zone_owner(host, lane->zx, lane->zz) == s->army || held > step->secs))
            release = 1;
        if (step->hold == H_ADVANCE && held > 10.0f) {
            const int owner = zone_owner(host, lane->zx, lane->zz);
            if (owner >= 0 && owner != s->army) {
                s->hold_start = ms;
            } else if (!waiting && !clearing && held > step->secs && !cmd_on() && lane_advance(host, s, seat, ms, 0)) {
                lane_drive(s, s->tx, s->tz, ms);
                return;
            } else if (held > step->secs) {
                s->hold_start = ms;
            }
        }
        if (!release) {
            if (motion_capture_holding(s, seat, s->tx, s->tz, ms)) {
                nav_propose(nav, NAV_LANE, s->tx, s->tz, NAV_PARK, 0, 0, 0);
                return;
            }
            float x = s->fight_post ? s->fight_post_x : s->tx;
            float z = s->fight_post ? s->fight_post_z : s->tz;
            int reposition = !s->fight_post && cover_hold(host, s, seat, lane, ms, &x, &z)
                             && dist2d(s->x, s->z, x, z) > COV_ARRIVE_M;
            nav_propose(nav, NAV_LANE, x, z, reposition ? NAV_NATIVE : NAV_PARK, reposition, 0, 0);
            return;
        }
        lane_goto_step(host, s, seat, s->step + 1, ms,
                       step->hold == H_UNTIL_OWNED ? "point taken or timeout" : "hold over");
        step = &s->route[s->step];
    }

    const float d = dist2d(s->x, s->z, s->tx, s->tz);
    if (step->hold == H_NONE && nav_lane_portal(s, seat, d)) {
        lane_goto_step(host, s, seat, s->step + 1, ms, "passed");
    } else if (step->hold != H_NONE && (motion_capture_holding(s, seat, s->tx, s->tz, ms)
               ? motion_capture_occupied(s, seat, ms)
               : d < HOLD_ARRIVE_M && control_static_connector(s, s->x, s->z, s->tx, s->tz) == NAV_CLEAR)) {
        s->holding = 1;
        s->hold_start = ms;
        event(host, "lanes: seat %d lane %d step %d holding", seat, s->lane, s->step);
        nav_propose(nav, NAV_LANE, s->tx, s->tz, NAV_PARK, 0, 0, 0);
        return;
    }
    lane_drive(s, s->tx, s->tz, ms);
}

static int nav_zone(void *host, int army, float x, float z)
{
    u8 *game = gh_game(host);
    u8 *owner_list = game ? gh_game_zones(game) : NULL;
    if (!owner_list)
        return -1;
    u8 **zones = gh_zone_list(owner_list);
    const uint32_t zn = gh_zone_count(owner_list);
    int pick = -1;
    float best = 1e18f;
    for (uint32_t i = 0; zones && i < zn && i < 64; ++i) {
        u8 *zone = gh_zone_at(zones, i);
        u8 *shape = zone ? gh_zone_shape(zone) : NULL;
        if (!shape || gh_zone_owner(zone) == army)
            continue;
        const uint32_t type = gh_shape_type(shape);
        const float *c = gh_shape_centre(shape, type);
        const float *e = gh_shape_extent(shape);
        const float h = gh_zone_shape_is_area(type) ? 0.5f : 0.0f;
        const float d = dist2d(c[0] + h * e[0], c[2] + h * e[2], x, z);
        if (d < best) {
            best = d;
            pick = (int)i;
        }
    }
    return pick;
}

/* Single host commit after all proposals. Native driving and the control
 * overlay have their own callback stamp and their own commit point. */
static void nav_commit(void *host, u8 *ai, lane_seat_t *s, int seat, u8 *node, uint64_t ms, float y, int clearing)
{
    enum nav_source source = NAV_SOURCE_COUNT;
    const nav_goal_t *goal = nav_choose(&s->nav, ms, clearing, &source);
    const int tight_portal = goal && goal->move && (source == NAV_ROUTE
        || (source == NAV_LANE && s->route[s->step].hold == H_NONE
            && dist2d(goal->x, goal->z, s->tx, s->tz) < 0.1f));
    control_prepare_engagement(s, seat, ms, goal && goal->path == NAV_PARK);
    s->nav.applied_source = source;
    const int repath = motion_native_repath_allowed(s, seat, ms) && s->nav.blocked_since_ms
                       && nav_recent(s->nav.feedback_ms, ms) && s->nav.blocked
                       && ms - s->nav.blocked_since_ms >= STILL_MS
                       && ms >= s->nav.next_replan_ms;
    if (repath)
        s->nav.next_replan_ms = ms + 4000;
    s->route_drive = source == NAV_ROUTE && goal->move;
    if (goal) {
        const int direct = goal->path == NAV_DIRECT;
        const int park = goal->path == NAV_PARK;
        const float d = dist2d(goal->x, goal->z, gh_ai_dest_x(ai), gh_ai_dest_z(ai));
        const int changed = !isfinite(d) || d > (tight_portal ? 0.01f : source >= NAV_ROUTE ? 0.5f : 3.0f);
        waypoint_t target = {0};
        if (changed) {
            target.x = goal->x;
            target.z = goal->z;
            target.y = ground_height(host, goal->x, y, goal->z);
        }
        const int zone = goal->move && !tight_portal && source != NAV_RECOVERY && !cmd_is_post(s)
                         ? nav_zone(host, s->army, goal->x, goal->z) : -1;
        const nav_ai_output_t output = {
            .target = changed ? &target : NULL,
            .hold = park ? 5.0f : zone < 0 ? 0.5f : 0.0f,
            .repath = park ? 5.0f : direct ? 1.0f : 0.0f,
            .write_hold = 1,
            .write_repath = repath || changed || park || direct || s->direct_on || goal->path == NAV_REPATH_NOW,
            .clear_path = repath || changed || direct,
            .write_zone = 1,
            .zone = zone
        };
        nav_write_ai(ai, &output);
        s->wants_move = goal->move;
        s->parked = goal->parked;
        s->direct_on = direct && source == NAV_ROUTE;
    } else {
        if (repath) {
            const nav_ai_output_t output = {.write_repath = 1, .clear_path = 1};
            nav_write_ai(ai, &output);
        }
        if (s->nav.epoch_id == 1)
            s->wants_move = 1;
    }
    s->move_target_x = goal ? goal->x : gh_ai_dest_x(ai);
    s->move_target_z = goal ? goal->z : gh_ai_dest_z(ai);
    ++s->move_seq;
    control_note_plan(s, seat, node, tight_portal);
}

static void nav_report(void *host, lane_seat_t *s, int seat, uint64_t ms)
{
    cmd_observe_mobility(host, s, seat, ms);   /* stage 2: before the report that carries it */
    nav_state_t *nav = &s->nav;
    const ground_task_order_t *order = cmd_active_order(seat);
    const int feedback = nav_recent(nav->feedback_ms, ms);
    const int valid_order = order && order->life_id == s->born_ms && order->task_id == nav->task_id
                            && order->revision == nav->revision && (!order->deadline_ms || ms < order->deadline_ms);
    const int route = nav->route_id && nav->route_task_id == nav->task_id && nav->route_revision == nav->revision;
    int reachability = route && nav->route_active ? GNR_REACH_LOCAL : GNR_REACH_UNKNOWN;
    float remaining = route && s->wpn > 0 ? fmaxf(0.0f, nav->arc[s->wpn - 1] - nav->completed_m) : 0.0f;
    if (valid_order && order->type == OB_CAPTURE && motion_capture_occupied(s, seat, ms)) {
        nav_set_state(s, GNR_ARRIVED, GTR_NONE, ms, 0);
        reachability = GNR_REACH_LOCAL;
        remaining = 0.0f;
    } else if (feedback && nav->blocked) {
        nav_set_state(s, GNR_BLOCKED, GTR_NO_ROUTE, ms, nav->retry_after_ms);
    } else if (feedback && nav->waiting) {
        if (s->motion_state == GM_WAIT_GEOMETRY || s->motion_state == GM_OBSERVED)
            nav_set_state(s, GNR_PLANNING, GTR_GEOMETRY_UNKNOWN, ms, 0);
        else
            nav_set_state(s, GNR_WAIT_TRAFFIC, GTR_CLEARANCE, ms, 0);
    } else if (valid_order) {
        const float d = dist2d(s->x, s->z, order->x, order->z);
        if (!route)
            remaining = d;
        if (order->type != OB_CAPTURE && d <= order->radius && (order->type == OB_HOLD
            || control_static_connector(s, s->x, s->z, order->x, order->z) == NAV_CLEAR)) {
            nav_set_state(s, GNR_ARRIVED, GTR_NONE, ms, 0);
            reachability = GNR_REACH_LOCAL;
            remaining = 0.0f;
        } else if (nav->state == GNR_ARRIVED || nav->state == GNR_NEED_ROUTE || nav->state == GNR_READY
                   || nav->state == GNR_WAIT_TRAFFIC || nav->state == GNR_PLANNING
                   || nav->state == GNR_BLOCKED) {
            nav_set_state(s, s->wants_move ? GNR_FOLLOWING : GNR_NEED_ROUTE, GTR_NONE, ms, 0);
        }
    }
    if (!valid_order)
        return;
    /* Stage 2: a stalled or immobilised hull is reported blocked whatever nav
     * thinks; nav's own state is left alone (movement owns it). Immobilised
     * (damage) only for the damaged case; a healthy stall is traffic_blocked:
     * blocked in its motion, recoverable, and no consumer treats it as final
     * (no_route would claim there is no route, clearance is a queue's wait). */
    const int state = cmd_seat_immobile(s) ? GNR_BLOCKED : nav->state;
    const int reason = cmd_seat_disabled(s) ? GTR_IMMOBILIZED : cmd_seat_immobile(s) ? GTR_TRAFFIC_BLOCKED : nav->reason;
    if (nav->reported_ms && ms - nav->reported_ms < 1500 && nav->reported_state == state
        && nav->reported_reason == reason && nav->reported_generation == nav->generation
        && nav->reported_revision == nav->revision)
        return;
    const ground_task_report_t report = {
        .life_id = s->born_ms, .task_id = nav->task_id, .revision = nav->revision,
        .sampled_ms = ms, .retry_after_ms = nav->retry_after_ms,
        .blocked_since_ms = cmd_seat_immobile(s) ? s->imm_since : nav->blocked_since_ms,
        .route_id = route ? (uint32_t)nav->route_id : 0, .generation = nav->generation,
        .seat = seat, .state = state, .reason = reason, .reachability = reachability,
        .completed_m = route ? nav->completed_m : 0.0f, .actual_m = route ? nav->actual_m : 0.0f,
        .remaining_m = remaining
    };
    cmd_navigation_report(host, s, seat, ms, &report);
    nav->reported_ms = ms;
    nav->reported_state = state;
    nav->reported_reason = reason;
    nav->reported_generation = nav->generation;
    nav->reported_revision = nav->revision;
}

/* A world observation can refresh position without creating a native drive stamp. */
static void ground_observe_pose(lane_seat_t *s, const u8 *unit, uint64_t ms)
{
    const float *pos = gh_unit_pos(unit);
    const float fx = gh_unit_fwd_x(unit), fz = gh_unit_fwd_z(unit), heading = hypotf(fx, fz);
    if (!isfinite(pos[0]) || !isfinite(pos[2]) || !isfinite(heading))
        return;
    const uint64_t elapsed = s->pose_observed_ms && ms >= s->pose_observed_ms ? ms - s->pose_observed_ms : 0;
    if (elapsed > 10 && elapsed < 1000) {
        const float dt = (float)elapsed / 1000.0f;
        s->vx = 0.6f * s->vx + 0.4f * (pos[0] - s->x) / dt;
        s->vz = 0.6f * s->vz + 0.4f * (pos[2] - s->z) / dt;
    } else if (!s->pose_observed_ms || elapsed >= 1000) {
        s->vx = s->vz = 0.0f;
    }
    if (!s->pose_observed_ms || elapsed > 10) {
        s->x = pos[0];
        s->z = pos[2];
        s->pose_observed_ms = ms;
    }
    s->fx = heading > 0.01f ? fx / heading : 0.0f;
    s->fz = heading > 0.01f ? fz / heading : 0.0f;
}

/* 20261115: a bot on its self-contained SPAA (the second-life reserve, see ground_rb_lineup) stays
 * at home instead of joining the push. Its home is SPAA_HOME_M from its spawn toward its first objective
 * (the native objective, else its lane's first step), so it keeps the spawn exit clear; it drives there and
 * parks. The proposal has the highest arbitration rank (NAV_RECOVERY, otherwise unused), so no lane,
 * commander, fight or reaction goal moves it; turret and fire control are untouched. */
#define SPAA_HOME_M 60.0f

static void spaa_hold_home(void *host, lane_seat_t *s, int seat, const u8 *unit, const u8 *ai, int has_lane)
{
    const vcls_t *v = active_cls(seat, unit);
    if (v != &reserve_cls[seat] || !v->ok || !(v->flags & VC_SPAA))
        return;
    if (!s->home_set) {
        float tx = gh_ai_dest_x(ai), ty = gh_ai_dest_y(ai), tz = gh_ai_dest_z(ai);
        float d = dist2d(tx, tz, s->sx, s->sz);
        if (!isfinite(d) || d < 2.0f * SPAA_HOME_M || gh_ai_dest_height_none(ty)) {
            tx = s->tx;
            tz = s->tz;
            d = has_lane ? dist2d(tx, tz, s->sx, s->sz) : 0.0f;
        }
        const float k = isfinite(d) && d > 2.0f * SPAA_HOME_M ? SPAA_HOME_M / d : 0.0f;
        s->home_x = s->sx + (tx - s->sx) * k;
        s->home_z = s->sz + (tz - s->sz) * k;
        s->home_set = 1;
        event(host, "spaa: seat %d army %d holds home (%.0f,%.0f), spawned (%.0f,%.0f)",
              seat, s->army, s->home_x, s->home_z, s->sx, s->sz);
    }
    const int away = dist2d(s->x, s->z, s->home_x, s->home_z) > 12.0f;
    nav_propose(&s->nav, NAV_RECOVERY, s->home_x, s->home_z, away ? NAV_NATIVE : NAV_PARK, away, !away, 0);
}

#include "ground_tactics.inc"

static void lanes_tick(u8 *unit, u8 *node, void *host)
{
    const int seat = bot_seat(unit);
    if (seat < 1 || !((cfg.brain_mask >> (seat - 1)) & 1))
        return;
    u8 *ai = gh_unit_ai(unit);
    if (!ai || !node)
        return;
    gh_lock(GH_LOCK_MAIN);
    lane_seat_t *s = &lseat[seat];
    const float *pos = gh_unit_pos(unit);
    uint64_t ms = gh_ms();
    geometry_context(host, ms);
    const int has_lane = cfg.lanes && seat_has_lane[seat] && seat_lane[seat] >= 0 && seat_lane[seat] < nlanes;
    if (!has_lane && !cfg.native_control) {
        gh_unlock(GH_LOCK_MAIN);
        return;
    }
    geometry_focus(pos[0], pos[1], pos[2]);
    scan_units(host, ms);
    const int spawned = s->unit != unit || s->scene_epoch != ground_scene_epoch || cmd_life_replaced(s, seat);
    if (spawned) {
        const uint64_t last_life = s->born_ms;
        memset(s, 0, sizeof *s);
        s->unit = unit;
        s->scene_epoch = ground_scene_epoch;
        s->lane = -1;
        s->army = gh_unit_army(unit);
        s->role = seat_lrole[seat];
        if (s->role == LROLE_OVERWATCH && (cmd_on() || (strat.state == 0 && cfg.commander && cfg.strategy_file[0])))
            s->role = LROLE_ASSAULT;
        s->sx = s->x = pos[0];
        s->sz = s->z = pos[2];
        s->born_ms = ms > last_life ? ms : last_life + 1;
        if (ms < s->born_ms)
            ms = s->born_ms;
        s->gt_slot = s->eng = s->yield_to = s->relief_peer = -1;
        s->tr = s->spot = -1;
        s->last_hit_ms = hit_dir[seat & 63].ms;
        s->last_ms = ms;
    }
    s->node = node;
    const uint64_t elapsed = ms > s->last_ms ? ms - s->last_ms : 0;
    if (elapsed > 1500 || control_engagement_active(s, ms)
        || (nav_recent(s->nav.feedback_ms, ms) && s->nav.waiting)) {
        s->anchor_ms += elapsed;
        if (cmd_is_post(s))
            s->post_progress_ms += elapsed;
    }
    ground_observe_pose(s, unit, ms);
    s->last_ms = ms;
    nav_tick_begin(&s->nav, s->born_ms, ms);
    if (!has_lane) {
        /* Adopt only this invocation's stock target for traffic prediction.
         * With no proposed nav goal, nav_commit leaves the engine path intact. */
        if (spawned)
            event(host, "navigation: seat %d follows the native objective, layout %s", seat, cfg.mission);
        cmd_unit_observe(host, s, seat, ms);
        s->wants_move = 1;
        spaa_hold_home(host, s, seat, unit, ai, 0);
        nav_commit(host, ai, s, seat, node, ms, pos[1], 0);
        gh_unlock(GH_LOCK_MAIN);
        return;
    }
    if (spawned) {
        const int kind = lanes[seat_lane[seat]].kind;
        const int pick = lane_pick(host, s, kind, ms);
        s->lane = pick >= 0 ? pick : seat_lane[seat];
        lane_route(s, &lanes[s->lane]);
        pick_troute(host, s, seat);
        lane_begin_step(s, seat, ms);
        event(host, "lanes: seat %d army %d spawned (%.0f,%.0f) lane %d (%s) role %d, %d steps, first (%.0f,%.0f)",
              seat, s->army, s->sx, s->sz, s->lane, kind ? "flank" : "target", s->role, s->n, s->tx, s->tz);
    }
    s->d = dist2d(s->x, s->z, s->tx, s->tz);
    if (dist2d(s->x, s->z, s->ax, s->az) > MOVE_M) {
        s->ax = s->x;
        s->az = s->z;
        s->anchor_ms = ms;
    }
    motion_apply_rejoin(host, s, seat, ms);
    nav_route_observe(s, ms, s->nav.applied_source == NAV_ROUTE && s->wants_move && !motion_combat_override(s, ms));
    cmd_unit_observe(host, s, seat, ms);
    team_plan(host, s->army, ms);
    strat_tick(host, s->army, ms);
    threat_tick(host, s->army, ms);    /* stage 3: after the strategy view it reads */
    if (!cmd_on() && s->nav.task_id) {
        const int post = cmd_is_post(s);
        cmd_cancel_unit(host, s, seat, ms, GTR_DISABLED);
        s->nav.task_id = 0;
        s->nav.revision = 0;
        nav_retire_route(s);
        if (post) {
            s->cmd_parked = s->parked = 0;
            lane_goto_step(host, s, seat, lane_slot_step(s), ms, "post commander unavailable");
        }
        nav_set_state(s, GNR_CANCELLED, GTR_DISABLED, ms, 0);
    }
    cover_minute(host, ms);
    approach_minute(host, ms);
    stab_native_on(host, unit);
    if (!s->stab_logged && ms - s->born_ms > 4000) {
        static int stab_logs;
        const vcls_t *vc = active_cls(seat, unit);
        int sf = stab_flags(unit), ef = stab_emu_flags(unit);
        s->stab_logged = 1;
        if (stab_logs < 60) {
            char diag[320];
            ++stab_logs;
            stab_diag(unit, diag, sizeof diag);
            event(host, "stab: seat %d diag %s", seat, diag);
            event(host, "stab: seat %d %s engine stabiliser h %d v %d | data h %s v %s (%.0f / %.0f km/h) | fire on the move: %s",
                  seat, gh_unit_name(unit) ? (const char *)gh_unit_name(unit) : "?", sf & 1, (sf >> 1) & 1,
                  vc->ok ? (vc->stab_h_kmh >= 0 ? "yes" : "no") : "?", vc->ok ? (vc->stab_v_kmh >= 0 ? "yes" : "no") : "?",
                  vc->ok ? vc->stab_h_kmh : -2.0f, vc->ok ? vc->stab_v_kmh : -2.0f,
                  !cfg.movefire ? "old rules" : ((sf | ef) & 2) ? (((sf | ef) & 1) ? "yes (full)" : "near only (vertical)") : "no (stops)");
            if (ef)
                event(host, "stab: seat %d DLL emulation h %d v %d (engine h %d v %d)", seat, ef & 1, (ef >> 1) & 1, sf & 1, (sf >> 1) & 1);
        }
    }
    preaim(host, unit, s, seat, ms);
    shots_poll(host, unit, s, seat, ms);
    const int clearing = 0;
    if (cmd_on())
        cmd_apply(host, s, seat, ms, clearing);
    approach_expire(s, ms);     /* stage 3: before the reactions read the approach state */
    react_update(host, unit, s, seat, ms);
    tac_update(host, s, seat, ms);     /* s28: presence hints and the reaction record (ground_tactics.inc) */
    lanes_propose(host, s, seat, ms, clearing);
    const ground_engagement_policy_t policy = cmd_motion_policy(s, seat, ms);
    if (!react_propose(host, s, seat, ms, policy) && (policy.allowed & GFM_ADVANCE))
        fight_drive(host, s, seat, ms);
    spaa_hold_home(host, s, seat, unit, ai, 1);
    nav_commit(host, ai, s, seat, node, ms, pos[1], clearing);
    nav_report(host, s, seat, ms);
    gh_unlock(GH_LOCK_MAIN);
}

#include "ground_drive.inc"
#include "ground_vehicle_control.inc"
#include "ground_lidar_adapter.inc"
#include "ground_overhead_adapter.inc"
#include "ground_phys_rate.inc"
#include "ground_control_graft.inc"


static float aim_skill = 0.7f, aim_scatter = 0.5f;
static uint32_t aim_rng = 0x9e3779b9u;

#define AIM_MAX_CANDS 24


/* The gun's current horizontal error to the target (turret record + hull), deg; -1 unknown. */
static float shooter_gun_err(u8 *shooter, const float *tpos)
{
    u8 *turrets = gh_unit_turrets(shooter);
    u8 *rec = turrets ? gh_turrets_records(turrets) : NULL;
    if (!rec || !yaw_sign || !isfinite(gh_turret_yaw(rec)))
        return -1.0f;
    const float *sp = gh_unit_pos(shooter);
    float hull = atan2f(gh_unit_fwd_z(shooter), gh_unit_fwd_x(shooter)) * 57.29578f;
    return fabsf(wrap180(hull + (float)yaw_sign * gh_turret_yaw(rec)
                         - atan2f(tpos[2] - sp[2], tpos[0] - sp[0]) * 57.29578f));
}

#include "ground_fire.inc"

/* Path diagnosis (2026-11-10): the launcher's path hook calls this after
 * the game's own path search. gh_path_points is the point count (the hook
 * treats >= 2 as a usable path; a review found the first
 * version logged the wrong side); gh_path_from / gh_path_to are the request's ends.
 * One line per seat per 5 s when the path search gave fewer than 2 points. */
static uint64_t path_log_next[MAX_SEATS];

void ground_path(u8 *unit, u8 *path, u8 *request, void *host)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || !unit || !path || !request)
        return;
    int seat = bot_seat(unit);
    uint32_t status = gh_path_points(path);
    if (seat < 0 || status >= 2)
        return;
    uint64_t ms = gh_ms();
    if (ms < path_log_next[seat])
        return;
    path_log_next[seat] = ms + 5000;
    const float *a = gh_path_from(request), *b = gh_path_to(request);
    event(host, "path: seat %d no path (%u points) from (%.0f,%.0f) to (%.0f,%.0f), %.0f m straight", seat, status, a[0],
          a[2], b[0], b[2], dist2d(a[0], a[2], b[0], b[2]));
}

/* ---- One-off obstacle scan (design rule: obstacles via rays, computed once
 * per map, never every battle) -------------------------------------------
 * The engine's collision ray (gh_ray; not rendering): start, direction, the
 * distance (in: maximum, out: hit), the flags, the kind of what it hit.
 * Flags (ground_host.h): GH_RAY_GROUND land mesh / heightmap, GH_RAY_STATIC_AND_OBJECTS
 * static collision and placed objects (buildings, walls, trees...). out_kind: 1 land, 2
 * static, 4 placed object. We trace GH_RAY_STATIC_AND_OBJECTS only, so terrain never answers.
 * Per cell (scan_step m) of scan_box: one ray down from 40 m above the ground
 * (obstacle height), and two horizontal rays at 1.2 m (+X and +Z edges, thin
 * walls/fences). Spread over frames: SCAN_SLICE_US every SCAN_GAP_MS.
 * Output (scan_file, written once via .tmp + rename): "WTOBS001", u32 w, h,
 * float x0, z0, step, then w*h u16 [row z][col x]:
 *   bits 0-5 obstacle height above ground, 0.5 m units (63 = 31.5 m or more)
 *   bit 6 +X edge blocked at 1.2 m, bit 7 +Z edge blocked at 1.2 m
 *   bits 8-10 vertical ray hit kind (2 static, 4 placed object), bit 15 scanned */
#define SCAN_SLICE_US 2000
#define SCAN_GAP_MS 20
#define SCAN_TOP_M 40.0f
#define SCAN_EYE_M 1.2f
static uint16_t *scan_grid;
static uint32_t scan_w, scan_h, scan_i;
static int scan_state;          /* 0 not started, 1 running, 2 done, 3 failed */
static uint64_t scan_next_ms, scan_rays, scan_t0;
static double scan_us;

static int trace_with_flags(void *host, const float *p, const float *dir, float *t, int *kind, int flags)
{
    u8 hit = 0;
    *kind = 0;
    if (gh_ray(host, p, dir, t, flags, kind, &hit)) {
        return -1;
    }
    return hit != 0;
}

static int trace(void *host, const float *p, const float *dir, float *t, int *kind)
{
    return trace_with_flags(host, p, dir, t, kind, cfg.scan_flags ? cfg.scan_flags : GH_RAY_STATIC_AND_OBJECTS);
}

static void obstacle_scan(void *host, uint64_t ms)
{
    if (!cfg.scan_file[0] || scan_state >= 2 || ms < scan_next_ms)
        return;
    scan_next_ms = ms + SCAN_GAP_MS;
    float step = cfg.scan_step > 0.5f ? cfg.scan_step : 4.0f;
    if (scan_state == 0) {
        float w = cfg.scan_box[2] - cfg.scan_box[0], h = cfg.scan_box[3] - cfg.scan_box[1];
        if (w <= step || h <= step || w * h / (step * step) > 2.0e7f) {
            scan_state = 3;
            event(host, "scan: bad box, no obstacle scan");
            return;
        }
        scan_w = (uint32_t)(w / step);
        scan_h = (uint32_t)(h / step);
        scan_grid = (uint16_t *)calloc((size_t)scan_w * scan_h, sizeof *scan_grid);
        if (!scan_grid) {
            scan_state = 3;
            return;
        }
        scan_state = 1;
        scan_t0 = ms;
        event(host, "scan: obstacle scan %ux%u cells of %.0f m -> %s", scan_w, scan_h, step, cfg.scan_file);
    }
    long long f, a, b;
    gh_qpc_frequency(&f);
    gh_qpc(&a);
    uint32_t total = scan_w * scan_h, quarter = total / 4 ? total / 4 : 1;
    static const float down[3] = {0, -1, 0}, px[3] = {1, 0, 0}, pz[3] = {0, 0, 1};
    for (;;) {
        gh_qpc(&b);
        double us = (double)(b - a) * 1e6 / (double)f;
        if (us > (cfg.scan_budget_us > 0 ? cfg.scan_budget_us : SCAN_SLICE_US) || scan_i >= total) {
            scan_us += us;
            break;
        }
        uint32_t col = scan_i % scan_w, row = scan_i / scan_w;
        float x = cfg.scan_box[0] + ((float)col + 0.5f) * step, z = cfg.scan_box[1] + ((float)row + 0.5f) * step;
        float g = ground_height(host, x, 300.0f, z);
        uint16_t cell = 0x8000;
        int kind, r;
        float p[3] = {x, g + SCAN_TOP_M, z}, t = SCAN_TOP_M - 0.2f;
        r = trace(host, p, down, &t, &kind);
        if (r < 0) {
            scan_state = 3;
            event(host, "scan: the collision ray raised an exception at (%.0f,%.0f), scan abandoned", x, z);
            free(scan_grid);
            scan_grid = NULL;
            return;
        }
        if (r) {
            float above = SCAN_TOP_M - t;
            int code = (int)(above * 2.0f);
            cell |= (uint16_t)(code > 63 ? 63 : code < 0 ? 0 : code);
            cell |= (uint16_t)((kind & 7) << 8);
        }
        float q[3] = {x - 0.5f * step, g + SCAN_EYE_M, z};
        t = step;
        if (trace(host, q, px, &t, &kind) == 1)
            cell |= 0x40;
        float q2[3] = {x, g + SCAN_EYE_M, z - 0.5f * step};
        t = step;
        if (trace(host, q2, pz, &t, &kind) == 1)
            cell |= 0x80;
        scan_rays += 3;
        scan_grid[scan_i++] = cell;
        if (scan_i % quarter == 0 && scan_i < total)
            event(host, "scan: %u%% (%.1f us per ray)", scan_i * 100 / total, scan_us / (double)(scan_rays ? scan_rays : 1));
    }
    if (scan_i < total)
        return;
    char tmp[MAX_PATH + 8];
    snprintf(tmp, sizeof tmp, "%s.tmp", cfg.scan_file);
    FILE *out = fopen(tmp, "wb");
    int ok = out != NULL;
    if (out) {
        float hdr[3] = {cfg.scan_box[0], cfg.scan_box[1], step};
        ok &= fwrite("WTOBS001", 1, 8, out) == 8;
        ok &= fwrite(&scan_w, 4, 1, out) == 1 && fwrite(&scan_h, 4, 1, out) == 1;
        ok &= fwrite(hdr, 4, 3, out) == 3;
        ok &= fwrite(scan_grid, sizeof *scan_grid, total, out) == total;
        ok &= fclose(out) == 0;
    }
    ok = ok && gh_file_replace(tmp, cfg.scan_file);
    uint32_t blocked = 0;
    for (uint32_t i = 0; i < total; ++i)
        blocked += (scan_grid[i] & 0x3f) >= 3 || (scan_grid[i] & 0xc0);
    event(host, "scan: %s, %u cells, %u with obstacles, %llu rays, %.1f us per ray, %.0f s", ok ? "saved" : "SAVE FAILED",
          total, blocked, (unsigned long long)scan_rays, scan_us / (double)(scan_rays ? scan_rays : 1),
          (double)(ms - scan_t0) / 1000.0);
    free(scan_grid);
    scan_grid = NULL;
    scan_state = ok ? 2 : 3;
    if (ok)
        geometry_scan_completed(host);
}

/* ---- Tiered line of sight (20261115) ----------------------------------------
 * The engine re-traces a pair of units' sight line once the shorter of
 * their two intervals (gh_unit_los_interval, ms) has
 * passed since the pair's last trace. Lowering every bot tank's own interval
 * to 42 ms (24 Hz, 2026-11-10) made EVERY pair with a bot in it trace at
 * 24 Hz: on Eastern Europe (forests, villages) the host's main thread sat at
 * 98-99% and fell to 3-7 frames a second from the first minute, which every
 * client saw as stutter at 0% loss. Now the hook asks here per pair:
 *   close (< LOS_CLOSE_M) or in contact (either bot saw the other within 3 s): cfg.los_ms (24 Hz);
 *   the offline G32 spotting matrix (terrain, solids, vegetation; 32 m cells,
 *   within its window) says neither cell sees the other: LOS_HIDDEN_MS;
 *   otherwise by distance: LOS_MID_MS up to LOS_FAR_M, LOS_FAR_MS beyond.
 * 0 = no bot tank in the pair: the engine's own interval. Read-only use of
 * strat (loaded once) and seen_by (plain stores); no lock on this hot path. */
#define LOS_CLOSE_M 300.0f
#define LOS_FAR_M 1000.0f
#define LOS_MID_MS 100
#define LOS_FAR_MS 300
#define LOS_HIDDEN_MS 1000
#define LOS_CONTACT_MS 3000u

static volatile long los_count[4];
static volatile long long los_next_ms;

static int los_contact(u8 *unit, const float *other, uint64_t ms)
{
    const int seat = bot_seat(unit);
    if (seat < 1 || seat >= 64)
        return 0;
    const seen_t seen = seen_by[seat];
    return seen.ms && ms - seen.ms < LOS_CONTACT_MS && dist2d(seen.x, seen.z, other[0], other[2]) < 40.0f;
}

int ground_los_interval(u8 *a, u8 *b, int native_ms, void *host)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || !cfg.los_tier || cfg.los_ms <= 0 || !a || !b)
        return 0;
    const int bot_a = bot_seat(a) >= 1 && is_tank(a), bot_b = bot_seat(b) >= 1 && is_tank(b);
    if (!bot_a && !bot_b)
        return 0;
    const float *pa = gh_unit_pos(a), *pb = gh_unit_pos(b);
    const float d = dist2d(pa[0], pa[2], pb[0], pb[2]);
    const uint64_t ms = gh_ms();
    int tier, interval;
    if (!isfinite(d) || d < LOS_CLOSE_M || (bot_a && los_contact(a, pb, ms)) || (bot_b && los_contact(b, pa, ms))) {
        tier = 3;
        interval = cfg.los_ms;
    } else {
        int hidden = 0;
        if (strat.state == 1 && strat.viss) {
            const int ca = strat_cellof(pa[0], pa[2]), cb = strat_cellof(pb[0], pb[2]);
            if (ca >= 0 && cb >= 0) {
                const int dr = cb / strat.nx - ca / strat.nx, dc = cb % strat.nx - ca % strat.nx;
                const int inside = dr >= -strat.W && dr <= strat.W && dc >= -strat.W && dc <= strat.W;
                hidden = inside && !strat_sees(strat.viss, ca, cb) && !strat_sees(strat.viss, cb, ca);
            }
        }
        tier = hidden ? 0 : d < LOS_FAR_M ? 2 : 1;
        interval = hidden ? LOS_HIDDEN_MS : d < LOS_FAR_M ? LOS_MID_MS : LOS_FAR_MS;
    }
    gh_atomic_inc(&los_count[tier]);
    if ((long long)ms >= los_next_ms) {
        const long long due = los_next_ms;
        if (gh_atomic_cas64(&los_next_ms, (long long)ms + 60000, due) == due && due) {
            const long c0 = gh_atomic_swap(&los_count[0], 0), c1 = gh_atomic_swap(&los_count[1], 0);
            const long c2 = gh_atomic_swap(&los_count[2], 0), c3 = gh_atomic_swap(&los_count[3], 0);
            event(host, "los: last 60 s pair checks %ld: close/contact %ld (%d ms), mid %ld (%d ms), far %ld (%d ms), "
                  "hidden by the G32 matrix %ld (%d ms); engine interval was %d ms",
                  (long)(c0 + c1 + c2 + c3), (long)c3, cfg.los_ms, (long)c2, LOS_MID_MS, (long)c1, LOS_FAR_MS,
                  (long)c0, LOS_HIDDEN_MS, native_ms);
        }
    }
    return interval;
}

/* ---- Scenery-object pending queue on the host (20261115) ---------------------
 * Moved / destroyed scenery (trees, fences knocked down) updates its transforms
 * through a family of engine functions. When a
 * layer has a render owner thread (not -1) and the caller is another
 * thread, they append to that layer's pending queue,
 * which the owner drains in its render preparation, and
 * the engine first searches the whole queue for the same id. The host
 * draws no scene (the launcher turns the scene draw off), so nobody ever drains it:
 * on Eastern Europe the main thread sat at 91-99% in that search and the host fell
 * to 3-7 frames a second (clients: stutter at 0% loss); with drawing on, 0 slow
 * frames. Owner -1 is the engine's own "no asynchronous renderer" mode: every
 * producer then applies the change at once and nothing is queued.
 * There are four layers (gh_riex_release). Checked every 500 ms; logged when set. */

/* First run (Eastern Europe): the owner was the host's main thread itself, which drains
 * the queue only while preparing a frame to draw, and layer 1 already held 176070
 * entries from level load. A non-empty queue keeps every producer on the queue path even
 * with no owner, so the backlog goes too: it was never applied in any battle this host
 * ran since 20261113, so dropping it leaves the game where it was, minus the search. */
static void riex_owner_tick(void *host, uint64_t ms)
{
    static uint64_t next_ms;
    if (!host || ms < next_ms)
        return;
    next_ms = ms + 500;
    for (unsigned layer = 0; layer < 4; ++layer) {
        int64_t was;
        int32_t pending;
        if (!gh_riex_release(host, layer, &was, &pending))
            continue;
        event(host, "riex: layer %u render owner %lld -> none, %d pending entries dropped", layer, (long long)was,
              (int)pending);
    }
}

void ground_plan(u8 *unit, u8 *node, void *host, float dt)
{
    gh_once(GH_ONCE_INIT, init);
    if (cfg.loaded && cfg.riex_owner)
        riex_owner_tick(host, gh_ms());
    if (cfg.loaded && node) {
        gh_lock(GH_LOCK_MAIN);
        uint64_t ms = gh_ms();
        probe(host, ms);
        clock_check(unit, dt, ms);
        phys_rate_tick(host, unit);
        obstacle_scan(host, ms);
        gh_unlock(GH_LOCK_MAIN);
    }
    /* Line-of-sight refresh (design rule: bot-bot and bot-player at 24 Hz).
     * The engine's sight-line updater re-traces a pair once the shorter of the
     * two units' intervals (gh_unit_los_interval, ms of wall time) has passed;
     * tanks get 2000 ms at spawn. Lowering a
     * bot tank's own interval is enough for all of its pairs. */
    if (cfg.loaded && cfg.los_ms > 0 && !cfg.los_tier && unit && is_tank(unit) && gh_unit_los_interval(unit) > cfg.los_ms)
        gh_unit_set_los_interval(unit, cfg.los_ms);
    if (cfg.loaded && (cfg.lanes || cfg.native_control)) {
        lanes_tick(unit, node, host);
        return;
    }
    if (!cfg.plan)
        return;
    if (!cfg.loaded || !node)
        return;
    int seat = bot_seat(unit);
    if (seat < 0 || !((cfg.brain_mask >> (seat - 1)) & 1))
        return;
    u8 *ai = gh_unit_ai(unit);
    if (!ai)
        return;
    gh_lock(GH_LOCK_MAIN);
    seat_t *s = &seats[seat];
    int army = gh_unit_army(unit);
    const float *pos = gh_unit_pos(unit);
    uint64_t ms = gh_ms();
    if (s->unit != unit) {
        release_claims(s, s->army);
        memset(s, 0, sizeof *s);
        s->unit = unit;
        s->sx = pos[0];
        s->sz = pos[2];
    }
    const float *dest = gh_ai_dest(ai);
    if (!s->planned) {
        /* Wait for the native tree's first objective, far enough from the start. */
        if (dist2d(dest[0], dest[2], s->sx, s->sz) > 200.0f && gh_ai_dest_height_valid(dest[1]))
            build_plan(host, s, seat, army, dest);
        gh_unlock(GH_LOCK_MAIN);
        return;
    }
    if (s->idx >= s->n) {
        gh_unlock(GH_LOCK_MAIN);
        return;
    }
    waypoint_t *w = s->retreating ? &s->retreat : &s->wp[s->idx];
    float d = dist2d(pos[0], pos[2], w->x, w->z);
    float arrive = gh_node_arrival(node);
    arrive = arrive > 2.0f ? arrive - 1.0f : 9.0f;

    if (s->retreating) {
        if (d < 25.0f) {
            /* Back at the previous point: retry this step with a different point. */
            s->retreating = 0;
            ++s->retries;
            s->best_ms = 0;
            make_step(host, s, s->idx, s->retries);
            set_dest(ai, &s->wp[s->idx]);
        } else {
            if (dist2d(dest[0], dest[2], w->x, w->z) > 1.0f)
                set_dest(ai, w);
            if (!s->best_ms || d < s->best - 15.0f) {
                s->best = d;
                s->best_ms = ms;
            } else if (ms - s->best_ms > 45000) {
                finish_plan(host, s, ai, "cannot get back either");
            }
        }
        gh_unlock(GH_LOCK_MAIN);
        return;
    }

    int next = 0;
    if (d < (w->hold > 0 ? arrive : 25.0f)) {
        if (!s->arrived) {
            s->arrived = 1;
            s->hold_end = ms + (uint64_t)(w->hold * 1000.0f);
            event(host, "seat %d reached step %d (%s) after %.0fs", seat, s->idx,
                  w->kind == 1 ? "heat spot" : "lane", (double)(ms - t0) / 1000.0);
        }
        if (ms < s->hold_end) {
            const nav_ai_output_t output = {.target = w, .repath = 5.0f, .write_hold = 1,
                                            .hold = (float)(s->hold_end - ms) / 1000.0f};
            nav_write_ai(ai, &output);
        } else {
            next = 1;
        }
    } else {
        if (dist2d(dest[0], dest[2], w->x, w->z) > 1.0f)
            set_dest(ai, w);
        if (!s->best_ms || d < s->best - 15.0f) {
            s->best = d;
            s->best_ms = ms;
        } else if (ms - s->best_ms > 45000) {
            start_retreat(host, s, ai, d);
        }
    }
    if (next) {
        ++s->idx;
        s->arrived = 0;
        s->retries = 0;
        s->best_ms = 0;
        if (s->idx >= s->n) {
            finish_plan(host, s, ai, "all points held");
        } else {
            const nav_ai_output_t output = {.target = &s->wp[s->idx], .write_hold = 1};
            nav_write_ai(ai, &output);
        }
    }
    gh_unlock(GH_LOCK_MAIN);
}

