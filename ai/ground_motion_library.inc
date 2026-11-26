/* ---- s26 library mode (snapshot path_mode with routes_lib): pre-generated routes (ground_routes.c). ----
 * Included by ground_motion.c (plain C, no Windows header). One authority per decision:
 *  - command layer: WHERE (the vehicle's goal, as s24 hands it; unchanged);
 *  - route layer (gl_events / gl_serve): WHICH WAY, decided on events only (goal changed by more than 1 m or to
 *    another library target, first plan of a life, the avoidance layer asks, more than 10 m off the curve, a failed
 *    search's retry after 4 s). On an event, in this order: DESIGNATED (standing on a library spawn pose: the
 *    designated route (pose, target), no search), ATTACH (bounded Dijkstra on map.bin's 0.5 m clearance grid out to
 *    80 m of path, joined to the route sample leading to the target with the least connector cost + remaining
 *    length), GRID (A* to a goal that maps to no target, within 150 m), NAV (s24's run-time search, counted). At
 *    most one grid search per snapshot over all tanks; the others wait their turn.
 *  - following (gl_follow, one rule for every source): projection (monotone), aim L ahead, stop distance = least of
 *    vehicles marched along the curve, a turn of more than 30 deg, the end of the curve. No static term: the curves
 *    are verified offline; the cross-track error and the stored clearance are published for the probe.
 *  - avoidance (gl_decide): vehicles only, idle unless the stop distance from a standing hull (or the stuck rule)
 *    stays under 2 m for 2 s; then one decision kept until done: a detour on the grid with the blocking hulls
 *    stamped (rejoining the curve 15 m beyond the blocker), else wait 4 s, or back 3 m when the search cannot start.
 *  - driver: only W / S / A / D from aim + go + stop distance (ground_drive.inc, unchanged). */

#define GL_MAX_SAMPLES 8192
#define GL_CONNECT_CAP 4000
#define GL_POST_RADIUS 12.0f       /* m: a goal this close to a post's point maps to that post */
#define GL_POSE_DIST 2.0f          /* standing on a library pose: within 2 m ... */
#define GL_POSE_DEG 15.0f          /* ... heading within 15 deg ... */
#define GL_POSE_SPEED 1.0f         /* ... and slower than 1 m/s */
#define GL_ORDER_WAIT_MS 8000u     /* on its pose, a life this young waits for a goal that maps to a target */
#define GL_GOAL_MOVED 1.0f         /* m: the goal changed identity */
#define GL_OFF_CURVE 10.0f         /* m from the curve: a new pick */
#define GL_ATTACH_LEN 80.0f        /* m of path the attach search reaches */
#define GL_RELOCATE 5.0f           /* m: start from the nearest free cell this close */
#define GL_GOAL_RELOCATE 10.0f     /* m: a GRID goal moves to the nearest free cell this close */
#define GL_GRID_MAX 150.0f         /* m: GRID only for goals this close */
#define GL_RETRY_MS 4000u          /* a failed search: decide again after this */
#define GL_TURN_DEG 8.0f           /* start turn: until the nose is this close to the point GL_TURN_AHEAD along */
#define GL_TURN_AHEAD 5.0f
#define GL_AIM_K 0.8f              /* aim L = clamp(0.8 * speed, 4, 12), <= 0.6 / max |kappa| over 12 m, >= 3 */
#define GL_AIM_MIN 4.0f
#define GL_AIM_MAX 12.0f
#define GL_AIM_FLOOR 3.0f
#define GL_AIM_LAT 0.6f
#define GL_AIM_LOOK 12.0f
#define GL_STOP_MAX 60.0f          /* m the own hull is marched along the curve */
#define GL_HULL_GROW 0.3f
#define GL_PREDICT_S 1.2f          /* a moving hull also counts at its position this far ahead */
#define GL_TURN_STOP 0.5236f       /* rad: a sample whose direction differs from the heading by more: stop there */
#define GL_TURN_NEAR 1.5f          /* m: (b) looks at the samples from this far beyond the projection on */
#define GL_BLOCKED_M 2.0f
#define GL_BLOCKED_MS 2000u
#define GL_STUCK_MPS 0.3f
#define GL_STUCK_MS 2000u
#define GL_DETOUR_RANGE 30.0f      /* m: standing hulls stamped into the detour search */
#define GL_DETOUR_BEYOND 15.0f     /* m of arc beyond the blocker where the detour rejoins */
#define GL_BACK_M 3.0f
#define GL_BACK_MS 6000u
#define GL_BACK_MIN 1.0f
#define GL_BACK_AGAIN_MS 30000u
#define GL_BACK_SAME 6.0f
#define GL_FINISH_M 30.0f          /* m before the end of a route to a zone: the slot (or the goal itself) is taken */
#define GL_PARK_M 1.5f             /* parked on the slot within this */
#define GL_SLOT_TAKEN 1.0f         /* m around another hull's rectangle: the slot is occupied */
#define GL_MAX_EXCLUDED 8

#define GL_SRC_NONE GM_LIB_SRC_NONE
#define GL_SRC_DESIGNATED GM_LIB_SRC_DESIGNATED
#define GL_SRC_ATTACH GM_LIB_SRC_ATTACH
#define GL_SRC_GRID GM_LIB_SRC_GRID
#define GL_SRC_NAV GM_LIB_SRC_NAV
#define GL_SRC_DETOUR GM_LIB_SRC_DETOUR
#define GL_SRC_SLOT GM_LIB_SRC_SLOT
#define GL_SRC_FINISH GM_LIB_SRC_FINISH
#define GL_SRC_NET GM_LIB_SRC_NET
#define GL_SRC_COUNT GM_LIB_SRC_COUNT
static const char *const gl_src_name[GL_SRC_COUNT] = {"none", "designated", "attach", "grid", "nav", "detour", "slot",
                                                      "finish", "net"};
enum { GL_NEED_NONE, GL_NEED_PLAN, GL_NEED_GRID, GL_NEED_DETOUR, GL_NEED_SLOT, GL_NEED_FINISH, GL_NEED_STREET,
       GL_NEED_NET };
/* s38 NET (the road network, snapshot routes_net; NULL: none of this runs). A library target: ATTACH, then NET, then
 * GRID, then NAV. A free point: GRID within GL_GRID_MAX, then NET to the network samples within GL_ATTACH_LEN of path
 * of the goal (round 3; the goal side in a snapshot of its own), then NAV; at the end of such a NET curve the GRID search takes the tank to the goal. */
#define GL_NET_SPEED 5.0f          /* m/s: connector metres (reach cost) and exit metres to seconds for the network search */
#define GL_NET_END_SLACK 10.0f     /* m: the GRID search from the end of a NET curve may start this much beyond GL_GRID_MAX */
/* s34 "street closed" (the avoidance layer reports, the route layer picks again) */
#define GL_STREET_MS 6000u         /* blocked by the same standing hull this long in all: the street is closed */
#define GL_STREET_BEYOND 12.0f     /* m of the curve beyond the blocker inside the closed box */
#define GL_STREET_KEEP_MS 60000u   /* a closed box lives this long */
#define GL_STREET_LEN 250.0f       /* m of path the new attach may run (the grid window caps it near 160 m) */
#define GL_STREET_RETRY_MS 20000u  /* no other street: stand, try again after this */
#define GL_STREET_BACK_M 15.0f     /* ... at least this far short of the blocker when it can back that far */
#define GL_STREET_NEAR 25.0f       /* m: a blocker this close, on the curve ahead, counts */
enum { GL_AV_IDLE, GL_AV_PENDING, GL_AV_WAIT, GL_AV_BACK };
/* s27 fix 1: the turn on the spot: no decision, forward-gear arc, reverse-gear arc, backing for room, hold */
enum { GL_PV_NONE, GL_PV_FWD, GL_PV_REV, GL_PV_BACK, GL_PV_HOLD };
/* why it backs: for a detour (s26), for room to turn (fix 1: in steps until an arc is free), off a marked spot (fix 2) */
enum { GL_BACK_DETOUR, GL_BACK_TURN, GL_BACK_MARK, GL_BACK_YIELD };
/* s36 rings: blockers walked at most, a repeated ring within this long gets no second give-way, the lower seats of a
 * repeated ring wait this long at most for the highest one to pick again */
#define GL_RING_STEPS 6
#define GL_RING_AGAIN_MS 30000u
#define GL_RING_REPEAT_WAIT_MS 6000u
enum { GL_RING_NONE, GL_RING_WAIT, GL_RING_GIVE, GL_RING_ON };
#define GL_MAX_MARKS 8
#define GL_WALL_GAP 0.2f           /* m: a turn on the spot keeps this from a hard cell (unless it starts closer) */
#define GL_PV_MOVED 1.0f           /* m: a turn decision is kept until the pose moved this far */
#define GL_PV_HOLD_MS 4000u        /* no arc, no room behind: decide again after this */
#define GL_BACK_TURN_MAX 30.0f     /* m backed at most for room to turn */
#define GL_PV_STILL_MS 3000u       /* a turn on the spot that does not turn for this long: that arc counts as blocked */

typedef struct {
    int used;
    uint64_t id, life, seen_ms;
    uint64_t first_seen_ms;     /* this life first seen (GL_ORDER_WAIT_MS) */
    int order_wait_logged;
    int army;
    /* goal identity */
    int has_goal, target;
    gm_point_t goal;
    /* the curve: samples 0..n-1 (0.5 m), arc from sample 0 */
    int n, proj, source, route, route_k0, route_rev, conn_n;
    unsigned rev;
    float x[GL_MAX_SAMPLES], z[GL_MAX_SAMPLES], kappa[GL_MAX_SAMPLES], arc[GL_MAX_SAMPLES];
    uint8_t clr[GL_MAX_SAMPLES];
    int finishing;              /* the curve ends on a slot or the goal itself (no further finishing step) */
    int turn_pending, turn_gear;
    /* a search owed (gl_serve) */
    int need, failed;
    uint64_t need_since, retry_ms;
    const char *need_why;
    /* NAV fallback */
    int nav_wait;
    unsigned nav_rev;
    /* avoidance */
    int av;
    uint64_t low_since, stuck_since, av_until, backed_ms;
    gm_point_t back_from, backed_at;
    uint64_t blocker;
    float block_arc;
    uint64_t waits_for;
    /* zone slot */
    int slot;                   /* library slot index, -1 none */
    int arrived_logged;
    int excluded[GL_MAX_EXCLUDED], excluded_n;
    /* s27 fix 1: the turn-on-the-spot decision (GL_PV_*), made once per pose: where, until when a hold lasts,
     * whether the reverse-gear arc is free of walls; backing for room to turn (back_kind GL_BACK_*), metres so far;
     * since when a turn on the spot does not turn */
    int pv, pv_rev_free, pv_stalled, pv_published; /* pv_stalled: bit 0 forward, bit 1 reverse arc did not turn */
    gm_point_t pv_at;
    uint64_t pv_until, pv_still_since;
    int back_kind;
    float back_total;
    /* s27 fix 2: where it stuck against something the map does not have (this life), stamped into its searches */
    gr_box_t marks[GL_MAX_MARKS];
    int marks_n;
    /* s29: an align (holding, hull to a heading: the reaction layer's facing) under the same turn-room rule, its own
     * state so that the curve's turn decision (pv) and avoidance (av) are left as they were: al_pv GL_PV_*, where it
     * was decided, hold end, stalled arcs, published, backing for room (from, total, until), last snapshot */
    int al_pv, al_stalled, al_published;
    gm_point_t al_at, al_back_from;
    uint64_t al_until, al_still_since, al_back_until, al_ms;
    float al_back_total;
    /* s34 "street closed": blocked-time accounting against one standing hull (sc_blocker, since sc_from), the
     * decision (sc_state 1 a pick owed, 2 no other street until sc_until), the closed boxes (60 s each) */
    uint64_t sc_blocker, sc_ms, sc_last_ms, sc_until;
    gm_point_t sc_from;
    int sc_state, cl_n;
    gr_box_t cl_box[4];
    uint64_t cl_until[4];
    /* s36 rings of tanks waiting for each other: the standing hull it was last blocked by (veh_blocker, when), a
     * give-way (for whom, when it began, metres to back, hold end; 0 not holding), a repeated ring's wait (since, for
     * which highest seat, that seat's curve revision then), the last ring line logged (members' signature, when) */
    uint64_t veh_blocker, veh_blocker_ms;
    uint64_t yield_for, yield_ms, yield_hold_until;
    float yield_back_m;
    uint64_t ring_since, ring_high, ring_log_sig, ring_log_ms;
    unsigned ring_rev0;
    /* s38 NET: net_free: the curve is a NET curve to a free point (kept through a detour; GRID takes it from the end);
     * net_end: the GRID request made at that end; net_tried: NET failed for this pick before GRID (a library target);
     * net_from_grid: the NET search owed was asked by a failed GRID search; the last NET pick's figures for its log line */
    int net_free, net_end, net_tried, net_from_grid;
    int net_edges;
    float net_time, net_len, net_conn, net_exit;
    /* round 2 legs: net_leg the curve ends at the path's first counted turn (more follows; kept through a detour),
     * net_leg_next the last gl_net's answer, net_leg_node the node of that turn, net_leg_req the NET request was made at a
     * leg end (loop guard); the whole path's end (moved-goal rule); log figures: counted turns, the first one's angle,
     * the exit's reach metres, the search times (goal side, hull side, network + connector) */
    int net_leg, net_leg_next, net_leg_node, net_leg_req, net_turns;
    float net_turn_deg, net_end_x, net_end_z, net_exit_reach;
    double net_us_goal, net_us_hull, net_us_net;
} gl_tank_t;

static gl_tank_t gl_tanks[GM_MAX_VEHICLES];
static unsigned gl_session_counter;
static gr_pt_t gl_buf[GL_CONNECT_CAP];
static float gl_tmp_x[GL_MAX_SAMPLES], gl_tmp_z[GL_MAX_SAMPLES], gl_tmp_k[GL_MAX_SAMPLES];
static uint8_t gl_tmp_c[GL_MAX_SAMPLES];
static gm_lib_stats_t gl_stats;
/* s38 round 3: whose exits the network's exit buffer holds (tank, life, goal; a NET goal-side search of a free point) */
static int gl_net_exits_valid, gl_net_exits_n;
static uint64_t gl_net_exits_id, gl_net_exits_life;
static gm_point_t gl_net_exits_goal;

void gm_lib_stats(gm_lib_stats_t *out)
{
    *out = gl_stats;
}

void gm_lib_stats_reset(void)
{
    memset(&gl_stats, 0, sizeof gl_stats);
}

static gl_tank_t *gl_find(uint64_t id, uint64_t life)
{
    for (int i = 0; i < GM_MAX_VEHICLES; ++i)
        if (gl_tanks[i].used && gl_tanks[i].id == id && gl_tanks[i].life == life)
            return &gl_tanks[i];
    return NULL;
}

static gl_tank_t *gl_get(uint64_t id, uint64_t life, uint64_t ms)
{
    gl_tank_t *t = gl_find(id, life);
    if (t)
        return t;
    int pick = -1;
    for (int i = 0; i < GM_MAX_VEHICLES && pick < 0; ++i)
        if (!gl_tanks[i].used)
            pick = i;
    if (pick < 0) {
        pick = 0;
        for (int i = 1; i < GM_MAX_VEHICLES; ++i)
            if (gl_tanks[i].seen_ms < gl_tanks[pick].seen_ms)
                pick = i;
    }
    t = &gl_tanks[pick];
    t->used = 1;
    t->id = id;
    t->life = life;
    t->seen_ms = ms;
    t->first_seen_ms = ms;
    t->order_wait_logged = 0;
    t->has_goal = t->n = t->proj = t->need = t->failed = t->nav_wait = t->av = 0;
    t->source = GL_SRC_NONE;
    t->route = t->target = t->slot = -1;
    t->turn_pending = t->finishing = t->excluded_n = t->arrived_logged = 0;
    t->pv = GL_PV_NONE; /* s27 */
    t->pv_rev_free = t->back_kind = t->marks_n = t->pv_stalled = t->pv_published = 0;
    t->back_total = 0.0f;
    t->pv_until = t->pv_still_since = 0;
    t->low_since = t->stuck_since = t->backed_ms = t->blocker = t->waits_for = 0;
    t->rev = 0;
    t->al_pv = GL_PV_NONE; /* s29 */
    t->al_stalled = t->al_published = 0;
    t->al_until = t->al_still_since = t->al_back_until = t->al_ms = 0;
    t->al_back_total = 0.0f;
    t->sc_blocker = t->sc_ms = t->sc_last_ms = t->sc_until = 0; /* s34 */
    t->sc_state = t->cl_n = 0;
    t->veh_blocker = t->veh_blocker_ms = t->yield_for = t->yield_ms = t->yield_hold_until = 0; /* s36 */
    t->yield_back_m = 0.0f;
    t->ring_since = t->ring_high = t->ring_log_sig = t->ring_log_ms = 0;
    t->ring_rev0 = 0;
    t->net_free = t->net_end = t->net_tried = t->net_from_grid = t->net_edges = 0; /* s38 */
    t->net_time = t->net_len = t->net_conn = t->net_exit = 0.0f;
    t->net_leg = t->net_leg_next = t->net_leg_req = t->net_turns = 0;
    t->net_leg_node = -1;
    t->net_turn_deg = t->net_end_x = t->net_end_z = t->net_exit_reach = 0.0f;
    t->net_us_goal = t->net_us_hull = t->net_us_net = 0.0;
    return t;
}

/* A new session (gm_state_t zeroed: map or match change) forgets every tank. */
static void gl_session(gm_state_t *state)
{
    if (state->lib_session)
        return;
    state->lib_session = ++gl_session_counter ? gl_session_counter : ++gl_session_counter;
    memset(gl_tanks, 0, sizeof gl_tanks);
    gl_net_exits_valid = 0;     /* s38 round 3 */
}

/* ---- curve building ---- */

static void gl_finish_curve(gl_tank_t *t, int source, uint64_t ms)
{
    t->arc[0] = 0.0f;
    for (int k = 1; k < t->n; ++k)
        t->arc[k] = t->arc[k - 1] + hypotf(t->x[k] - t->x[k - 1], t->z[k] - t->z[k - 1]);
    t->proj = 0;
    t->source = source;
    t->rev += 1;
    t->need = 0;
    t->failed = 0;
    t->nav_wait = source == GL_SRC_NAV;
    t->low_since = t->stuck_since = 0;
    t->net_end = 0;             /* s38: a new curve answers the GRID request made at a NET curve's end */
    t->net_leg_req = 0;         /* s38 round 2: ... and the NET request made at a leg end */
    if (source != GL_SRC_DETOUR)
        t->net_free = t->net_leg = 0; /* s38: set again by the NET pick itself; a detour keeps its curve's tail */
    (void)ms;
}

static float gl_kappa_at(const float *x, const float *z, int n, int k)
{
    if (k <= 0 || k >= n - 1)
        return 0.0f;
    const float h0 = atan2f(z[k] - z[k - 1], x[k] - x[k - 1]), h1 = atan2f(z[k + 1] - z[k], x[k + 1] - x[k]);
    const float ds = 0.5f * (hypotf(x[k] - x[k - 1], z[k] - z[k - 1]) + hypotf(x[k + 1] - x[k], z[k + 1] - z[k]));
    return ds > 1e-3f ? gm_angle(h1 - h0) / ds : 0.0f;
}

static void gl_append_point(const gr_lib_t *lib, gl_tank_t *t, float x, float z, float kappa, int clr_byte)
{
    if (t->n >= GL_MAX_SAMPLES)
        return;
    if (t->n && hypotf(x - t->x[t->n - 1], z - t->z[t->n - 1]) < 0.05f)
        return;
    t->x[t->n] = x;
    t->z[t->n] = z;
    t->kappa[t->n] = kappa;
    t->clr[t->n] = (uint8_t)(clr_byte >= 0 ? clr_byte : (int)fminf(255.0f, floorf(gr_clearance(lib, x, z) * 10.0f + 1e-3f)));
    ++t->n;
}

/* Connector points (curvature from geometry, clearance from the map). */
static void gl_append_points(const gr_lib_t *lib, gl_tank_t *t, const gr_pt_t *p, int n)
{
    const int from = t->n;
    for (int k = 0; k < n; ++k)
        gl_append_point(lib, t, p[k].x, p[k].z, 0.0f, -1);
    for (int k = from; k < t->n; ++k)
        t->kappa[k] = gl_kappa_at(t->x, t->z, t->n, k);
    t->conn_n = t->n;
}

/* The library route from sample k0 to its end (reverse: from k0 down to its first sample). */
static void gl_append_route(const gr_lib_t *lib, gl_tank_t *t, int route, int k0, int reverse)
{
    const gr_route_t *r = &lib->routes[route];
    if (t->n > 1) /* the joint between a connector and the route */
        t->kappa[t->n - 1] = 0.0f;
    if (!reverse)
        for (int k = k0; k < (int)r->n; ++k)
            gl_append_point(lib, t, lib->px[r->first + k], lib->pz[r->first + k], lib->kappa[r->first + k],
                            lib->clr[r->first + k]);
    else
        for (int k = k0; k >= 0; --k)
            gl_append_point(lib, t, lib->px[r->first + k], lib->pz[r->first + k], -lib->kappa[r->first + k],
                            lib->clr[r->first + k]);
    t->route = route;
    t->route_k0 = k0;
    t->route_rev = reverse;
}

static gm_point_t gl_hull_centre(const gm_agent_t *a)
{
    return lp_centre(a->v, a->p.x, a->p.z, a->fwd);
}

/* The curve's end as the planner's arrival point (s24's standing-room mechanism: route_goal). */
static void gl_set_end(gm_agent_t *a, gl_tank_t *t)
{
    gm_commitment_t *c = &a->c;
    if (!t->n)
        return;
    c->key_goal = a->v->goal;
    c->route_goal = (gm_point_t){t->x[t->n - 1], t->z[t->n - 1]};
    c->route_goal_moved = !a->region;
    c->route_status = GM_NAV_READY;
}

static void gl_log_pick(const gm_snapshot_t *s, const gm_agent_t *a, const gl_tank_t *t, const char *why, double us)
{
    const gr_lib_t *lib = s->routes_lib;
    char what[240] = "";
    if (t->source == GL_SRC_NET) {
        /* s38: goal, edges, network metres and seconds, connector metres, exit distance from a free goal */
        char goal[48], exit[48] = "";
        if (t->target >= 0)
            snprintf(goal, sizeof goal, "target %s", lib->targets[t->target].id);
        else
            snprintf(goal, sizeof goal, "(%.1f, %.1f)", (double)t->goal.x, (double)t->goal.z);
        if (t->target < 0)
            snprintf(exit, sizeof exit, ", exit %.1f m (reach %.1f m) from the goal", (double)t->net_exit,
                     (double)t->net_exit_reach);
        char legs[64] = "";
        if (t->net_leg)
            snprintf(legs, sizeof legs, " (leg to the first: %.0f deg at node %d)", (double)t->net_turn_deg, t->net_leg_node);
        snprintf(what, sizeof what, " goal %s, %d edges, %.0f m / %.1f s, %d turns%s, connector %.0f m%s, searches "
                 "%.1f+%.1f+%.1f ms,", goal, t->net_edges, (double)t->net_len, (double)t->net_time, t->net_turns, legs,
                 (double)t->net_conn, exit, t->net_us_goal / 1000.0, t->net_us_hull / 1000.0, t->net_us_net / 1000.0);
    } else if (t->route >= 0 && t->source != GL_SRC_GRID && t->source != GL_SRC_NAV && t->source != GL_SRC_SLOT
        && t->source != GL_SRC_FINISH)
        snprintf(what, sizeof what, " route %d %s%s from sample %d,", t->route, lib->routes[t->route].id,
                 t->route_rev ? " (reversed)" : "", t->route_k0);
    gm_path_log(s, "lib: seat %llu %s:%s %d samples, %.0f m (connector %d), target %s, reason %s, search %.1f ms%s",
                (unsigned long long)a->v->id, gl_src_name[t->source], what, t->n, (double)(t->n ? t->arc[t->n - 1] : 0.0f),
                t->conn_n, t->target >= 0 ? lib->targets[t->target].id : "none", why ? why : "-", us / 1000.0,
                t->turn_pending ? (t->turn_gear == 2 ? ", start turn in reverse gear" : ", start turn") : "");
}

/* ---- events and the picks that need no search ---- */

static void gl_request(gl_tank_t *t, int need, const char *why, uint64_t ms)
{
    /* s27 fix 3: a failed search's retry time holds for every request (gl_events clears `failed` on a new goal and
     * when the retry time has come) */
    if (t->need == need || (t->failed && ms < t->retry_ms))
        return;
    t->need = need;
    t->need_since = ms;
    t->need_why = why;
}

static void gl_count_pick(int source, double us)
{
    if (source >= 0 && source < GL_SRC_COUNT)
        ++gl_stats.picks[source];
    if (us > 0.0) {
        ++gl_stats.searches;
        gl_stats.search_us_sum += us;
        if (us > gl_stats.search_us_max)
            gl_stats.search_us_max = us;
    }
}

/* s35: a capture order whose goal is a lane's slot inside the zone (a kind-1 target): its end is the tank's parking slot
 * (the capture region parks it there, as on a zone slot). */
static void gl_lane_slot(gm_agent_t *a, const gl_tank_t *t)
{
    if (!a->region || a->v->goal_target1 <= 0 || t->target != a->v->goal_target1 - 1 || !t->n)
        return;
    a->c.slot = (gm_point_t){t->x[t->n - 1], t->z[t->n - 1]};
    a->c.slot_valid = 1;
    a->c.slot_phase = 2;
}

/* DESIGNATED: standing on a library pose with a goal that maps to a target. 1 when taken. */
static int gl_try_designated(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t)
{
    const gr_lib_t *lib = s->routes_lib;
    if (t->target < 0 || fabsf(a->speed) >= GL_POSE_SPEED)
        return 0;
    const int pose = gr_pose_of(lib, a->p.x, a->p.z, a->v->heading, GL_POSE_DIST, GL_POSE_DEG);
    if (pose < 0)
        return 0;
    /* s32: the order's route variant (0 the designated / recommended corridor, 1 / 2 a flank corridor) */
    const int route = gr_route_variant(lib, pose, t->target, a->v->route_variant);
    if (route < 0)
        return 0;
    const gr_route_t *r = &lib->routes[route];
    t->n = t->conn_n = 0;
    t->finishing = 0;
    gl_append_route(lib, t, route, 0, 0);
    gl_finish_curve(t, GL_SRC_DESIGNATED, s->ms);
    /* the start turn: in the gear the library stores (forward when it stores none). s32: a pose with an unknown (NaN)
     * heading has no start-turn data: no start turn, the normal turn-on-the-spot rule (gl_decide) makes the first turn */
    const int known = isfinite(lib->poses[pose].heading_deg);
    /* a route without start-turn data (no gear stored, turn angle not finite) has no start turn either */
    t->turn_pending = known && (r->gear == 1 || r->gear == 2
                                || (!(r->flags & GR_FLAG_TANGENT) && isfinite(r->turn_deg) && fabsf(r->turn_deg) > GL_TURN_DEG));
    t->turn_gear = r->gear == 2 ? 2 : 1;
    if (a->v->route_variant)
        gm_path_log(s, "lib: seat %llu takes route variant %d (route %d, fallback %d, %.0f s)", (unsigned long long)a->v->id,
                    a->v->route_variant, route, (int)r->fallback, (double)r->total_s);
    gl_lane_slot(a, t);
    gl_set_end(a, t);
    gl_count_pick(GL_SRC_DESIGNATED, 0.0);
    gl_log_pick(s, a, t, "on pose", 0.0);
    return 1;
}

/* Per controlled tank, before following: goal identity and the events. */
static void gl_events(gm_state_t *state, const gm_snapshot_t *s, gm_agent_t *agents, int count)
{
    const gr_lib_t *lib = s->routes_lib;
    gl_session(state);
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        const gm_vehicle_t *v = a->v;
        a->lib = 1;
        if (!v->controlled)
            continue;
        gl_tank_t *t = gl_get(v->id, v->life_id, s->ms);
        t->seen_ms = s->ms;
        t->army = v->group;
        if (v->move && t->has_goal && !t->arrived_logged && (a->c.parked || a->c.arrived)) {
            t->arrived_logged = 1;
            gm_path_log(s, "lib: seat %llu %s at (%.1f, %.1f), %.1f m from the goal (%.1f, %.1f), target %s, last source %s",
                        (unsigned long long)v->id, a->c.parked ? "parked" : "arrived", (double)a->p.x, (double)a->p.z,
                        (double)gm_dist(a->p, v->goal), (double)v->goal.x, (double)v->goal.z,
                        t->target >= 0 ? lib->targets[t->target].id : "none", gl_src_name[t->source]);
        }
        if (!a->mobile || !v->move)
            continue;
        /* s35: an order naming its slot's library target (a plan lane) takes that target; else the goal is mapped */
        const int target = v->goal_target1 > 0 && (uint32_t)v->goal_target1 <= lib->n_targets ? v->goal_target1 - 1
                         : gr_target_of(lib, v->goal.x, v->goal.z, v->group, GL_POST_RADIUS);
        const char *why = NULL;
        if (!t->has_goal) {
            why = "first";
        } else if (target != t->target) {
            why = "goal changed";
        } else if (gm_dist(v->goal, t->goal) > GL_GOAL_MOVED) {
            if (t->sc_state)
                t->goal = v->goal;  /* s37: the same target during the street-closed decision: nothing restarts */
            else if (t->net_free && target < 0 && t->n
                     && gm_dist(v->goal, (gm_point_t){t->net_end_x, t->net_end_z}) <= GL_ATTACH_LEN)
                t->goal = v->goal;  /* s38: a free goal on a NET curve, still within GL_ATTACH_LEN of the path's end */
            else
                why = "goal changed";
        }
        if (why && target < 0 && s->ms < t->first_seen_ms + GL_ORDER_WAIT_MS && fabsf(a->speed) < GL_POSE_SPEED
            && gr_pose_of(lib, a->p.x, a->p.z, v->heading, GL_POSE_DIST, GL_POSE_DEG) >= 0) {
            /* the commander's first order applies 1-3 s after birth; until then the goal is a lane step: stand on the
             * spawn pose (no goal taken, no curve: gl_decide's no-curve branch holds) and look again next snapshot */
            if (!t->order_wait_logged) {
                t->order_wait_logged = 1;
                gm_path_log(s, "lib: seat %llu waits on its spawn pose for its first order (goal (%.1f, %.1f) maps to no "
                            "target)", (unsigned long long)v->id, (double)v->goal.x, (double)v->goal.z);
            }
            continue;
        }
        if (why) {
            t->has_goal = 1;
            t->goal = v->goal;
            t->target = target;
            t->n = 0;
            t->source = GL_SRC_NONE;
            t->route = -1;
            t->slot = -1;
            t->excluded_n = 0;
            t->av = GL_AV_IDLE;
            t->turn_pending = 0;
            t->finishing = 0;
            t->arrived_logged = 0;
            t->failed = 0;      /* s27: a new goal is not held by the old goal's retry time */
            t->pv = GL_PV_NONE; /* s27: the turn-on-the-spot decision belongs to the old curve */
            t->net_free = t->net_end = t->net_leg = t->net_leg_req = 0; /* s38: the old curve's NET state */
            if (target < 0)
                gm_path_log(s, "lib: seat %llu goal (%.1f, %.1f) maps to no library target (%s, army %d, %.0f m away)",
                            (unsigned long long)v->id, (double)v->goal.x, (double)v->goal.z, why, v->group,
                            (double)gm_dist(a->p, v->goal));
            a->c.route_goal_moved = 0;
            if (!a->region) {
                a->c.slot_valid = 0;
                a->c.slot_phase = 0;
            }
            if (t->sc_state) {
                /* s36 P1: a new goal during the street-closed decision ends the stand at once: with a closed box still
                 * alive, the street pick to the NEW target (the closed boxes left out; gl_decide's sc_state 1 hold
                 * covers the snapshots until it is served); none alive any more: the ordinary pick. s37: only a new
                 * target gets here (a goal moved with its target unchanged is taken above without a restart) */
                int alive = 0;
                for (int k = 0; k < t->cl_n; ++k)
                    alive |= s->ms < t->cl_until[k];
                if (alive && target >= 0) {
                    t->sc_state = 1;
                    t->sc_ms = 0;
                    gl_request(t, GL_NEED_STREET, "street closed: new goal", s->ms);
                    continue;
                }
                t->sc_state = 0;
                t->sc_ms = 0;
            }
            if (!gl_try_designated(s, a, t))
                gl_request(t, GL_NEED_PLAN, why, s->ms);
            continue;
        }
        if (t->failed && s->ms >= t->retry_ms && !t->need) {
            /* a failed search's retry time came: the same request again (a slot or finishing connector), else the
             * whole pick */
            const int again = t->failed == GL_NEED_SLOT || t->failed == GL_NEED_FINISH ? t->failed : GL_NEED_PLAN;
            t->failed = 0;
            t->excluded_n = 0;
            gl_request(t, again, "retry", s->ms);
        }
    }
}

static void gl_align_back(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i);

/* s34: the tank's remembered boxes now: its s27 marks and the streets it found closed (60 s each). */
static gr_box_t gl_mark_buf[GL_MAX_MARKS + 4];
static int gl_marks_now(const gl_tank_t *t, uint64_t ms, gr_box_t *out)
{
    int n = 0;
    for (int k = 0; k < t->marks_n; ++k)
        out[n++] = t->marks[k];
    for (int k = 0; k < t->cl_n; ++k)
        if (ms < t->cl_until[k])
            out[n++] = t->cl_box[k];
    return n;
}

/* ---- the searches (at most one per snapshot) ---- */

/* ATTACH: the least connector cost + remaining length to the target over every sample reached. */
static int gl_attach(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t, double *us)
{
    const gr_lib_t *lib = s->routes_lib;
    const gm_point_t h = gl_hull_centre(a);
    const double t0 = gm_clock_us();
    gr_info_t info;
    int ok = 0;
    /* s32: an order with a flank variant attaches only to that variant's route (from the life's spawn pose), when it exists */
    int only = -1;
    if (a->v->route_variant > 0 && a->v->route_pose >= 0 && (uint32_t)a->v->route_pose < lib->n_poses) {
        only = gr_route_variant(lib, a->v->route_pose, t->target, a->v->route_variant);
        if (only >= 0 && lib->routes[only].fallback != a->v->route_variant)
            only = -1;
    }
    const int nmk = gl_marks_now(t, s->ms, gl_mark_buf);    /* s34: marks and closed streets */
    if (gr_dijkstra(lib, h.x, h.z, GL_ATTACH_LEN, GL_RELOCATE, gl_mark_buf, nmk, &info) == GR_OK) { /* s27: marks */
        float best = INFINITY;
        int br = -1, bk = -1, brev = 0;
        for (int pass = only >= 0 ? 0 : 1; pass < 2 && br < 0; ++pass)
        for (int q = lib->ref_start[t->target]; q < lib->ref_start[t->target + 1]; ++q) {
            const gr_ref_t ref = lib->refs[q];
            if (!pass && ref.route != only)
                continue;               /* pass 0: the variant's route only; pass 1 (none reached): every route */
            const gr_route_t *r = &lib->routes[ref.route];
            const float total = lib->arc[r->first + r->n - 1];
            for (uint32_t k = 0; k < r->n; ++k) {
                const uint32_t j = r->first + k;
                if (fabsf(lib->px[j] - h.x) > GL_ATTACH_LEN + 6.0f || fabsf(lib->pz[j] - h.z) > GL_ATTACH_LEN + 6.0f)
                    continue;
                const float cost = gr_reach_cost(lib, lib->px[j], lib->pz[j]);
                if (!isfinite(cost))
                    continue;
                const float rest = ref.reverse ? lib->arc[j] : total - lib->arc[j];
                if (cost + rest < best) {
                    best = cost + rest;
                    br = ref.route;
                    bk = (int)k;
                    brev = ref.reverse;
                }
            }
        }
        if (br >= 0) {
            const gr_route_t *r = &lib->routes[br];
            const int n = gr_reach_path(lib, h.x, h.z, lib->px[r->first + (uint32_t)bk], lib->pz[r->first + (uint32_t)bk],
                                        gl_mark_buf, nmk, gl_buf, GL_CONNECT_CAP); /* s27: marks */
            if (n > 0) {
                t->n = 0;
                gl_append_points(lib, t, gl_buf, n);
                gl_append_route(lib, t, br, bk, brev);
                ok = 1;
            }
        }
    }
    *us = gm_clock_us() - t0;
    return ok;
}

/* s34: a sample inside one of the tank's closed boxes */
static int gl_in_closed(const gl_tank_t *t, uint64_t ms, float x, float z)
{
    for (int k = 0; k < t->cl_n; ++k)
        if (ms < t->cl_until[k]
            && lp_point_rect((gm_point_t){x, z}, (gm_point_t){t->cl_box[k].cx, t->cl_box[k].cz},
                             (gm_point_t){t->cl_box[k].fx, t->cl_box[k].fz}, t->cl_box[k].hl, t->cl_box[k].hw) < 0.5f)
            return 1;
    return 0;
}

/* s34 street closed: an attach to the SAME target over GL_STREET_LEN m of path, the closed boxes stamped into the search,
 * over every library route that leads to the target, rejecting a join whose remaining samples pass through a closed box. */
/* s36: the street search proper, shared by the driving side (gl_attach_street) and the command side's lane check
 * (gm_lib_street_open): from (hx, hz) over GL_STREET_LEN m of path with the tank's marks and closed boxes stamped, the
 * best join onto a route of `target` whose rest does not run through a closed box. 1 found (*br, *bk, *brev). */
static int gl_street_scan(const gr_lib_t *lib, const gl_tank_t *t, uint64_t ms, float hx, float hz, int target, int *br,
                          int *bk, int *brev, int *rejected)
{
    gr_info_t info;
    *rejected = 0;
    *br = *bk = -1;
    *brev = 0;
    const int nmk = gl_marks_now(t, ms, gl_mark_buf);
    if (gr_dijkstra(lib, hx, hz, GL_STREET_LEN, GL_RELOCATE, gl_mark_buf, nmk, &info) != GR_OK)
        return 0;
    float best = INFINITY;
    for (int q = lib->ref_start[target]; q < lib->ref_start[target + 1]; ++q) {
        const gr_ref_t ref = lib->refs[q];
        const gr_route_t *r = &lib->routes[ref.route];
        int first_hit = -1, last_hit = -1;
        for (uint32_t k = 0; k < r->n; ++k)
            if (gl_in_closed(t, ms, lib->px[r->first + k], lib->pz[r->first + k])) {
                if (first_hit < 0)
                    first_hit = (int)k;
                last_hit = (int)k;
            }
        *rejected += first_hit >= 0;
        const float total = lib->arc[r->first + r->n - 1];
        for (uint32_t k = 0; k < r->n; ++k) {
            if (first_hit >= 0 && (ref.reverse ? (int)k >= first_hit : (int)k <= last_hit))
                continue;   /* the rest of this route runs through the closed street */
            const uint32_t j = r->first + k;
            if (fabsf(lib->px[j] - hx) > GL_STREET_LEN + 6.0f || fabsf(lib->pz[j] - hz) > GL_STREET_LEN + 6.0f)
                continue;
            const float cost = gr_reach_cost(lib, lib->px[j], lib->pz[j]);
            if (!isfinite(cost))
                continue;
            const float rest = ref.reverse ? lib->arc[j] : total - lib->arc[j];
            if (cost + rest < best) {
                best = cost + rest;
                *br = ref.route;
                *bk = (int)k;
                *brev = ref.reverse;
            }
        }
    }
    return *br >= 0;
}

/* s36 (command side, lane change after "no other street"): can the tank (id, life) reach `target` from (x, z) without
 * passing through a street it has closed? The driving side's own street search (gl_street_scan). 1 open (also when it has
 * no closed street now), 0 not. *closed_until: when its last closed box expires (0: none open now). */
int gm_lib_street_open(const struct gr_lib *lib, uint64_t id, uint64_t life, int target, float x, float z, uint64_t ms,
                       uint64_t *closed_until)
{
    const gl_tank_t *t = gl_find(id, life);
    *closed_until = 0;
    for (int k = 0; t && k < t->cl_n; ++k)
        if (ms < t->cl_until[k] && t->cl_until[k] > *closed_until)
            *closed_until = t->cl_until[k];
    if (!lib || target < 0 || (uint32_t)target >= lib->n_targets)
        return 0;
    if (!t || !*closed_until)
        return 1;
    int br, bk, brev, rejected;
    return gl_street_scan(lib, t, ms, x, z, target, &br, &bk, &brev, &rejected);
}

static int gl_attach_street(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t, double *us, int *rejected)
{
    const gr_lib_t *lib = s->routes_lib;
    const gm_point_t h = gl_hull_centre(a);
    const double t0 = gm_clock_us();
    int ok = 0;
    const int nmk = gl_marks_now(t, s->ms, gl_mark_buf);
    {
        int br, bk, brev;
        gl_street_scan(lib, t, s->ms, h.x, h.z, t->target, &br, &bk, &brev, rejected);
        if (br >= 0) {
            const gr_route_t *r = &lib->routes[br];
            const int n = gr_reach_path(lib, h.x, h.z, lib->px[r->first + (uint32_t)bk], lib->pz[r->first + (uint32_t)bk],
                                        gl_mark_buf, nmk, gl_buf, GL_CONNECT_CAP);
            if (n > 0) {
                t->n = 0;
                gl_append_points(lib, t, gl_buf, n);
                gl_append_route(lib, t, br, bk, brev);
                ok = 1;
            }
        }
    }
    *us = gm_clock_us() - t0;
    return ok;
}

static float gl_grid_slack; /* s38: m beyond GL_GRID_MAX, only for the GRID search from a NET curve's end (else 0) */

/* A* from the hull centre to (gx, gz), into gl_buf. Stamped: `boxes` (the detour's standing hulls) and, s27 fix 2,
 * the tank's own marks (where it stuck against something the map does not have). relocate: start from the nearest
 * free cell within this (0: GR_START_BLOCKED when the own cell is not free: the detour's "no room to turn out"). */
static int gl_grid(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t, gm_point_t goal, const gr_box_t *boxes,
                   int nbox, float relocate, float goal_relocate, double *us, int *status)
{
    const gr_lib_t *lib = s->routes_lib;
    const gm_point_t h = gl_hull_centre(a);
    static gr_box_t all[GM_MAX_VEHICLES + GL_MAX_MARKS + 4];
    int nall = 0;
    for (int k = 0; k < nbox && nall < GM_MAX_VEHICLES; ++k)
        all[nall++] = boxes[k];
    nall += gl_marks_now(t, s->ms, all + nall);  /* s27 marks, s34 closed streets */
    const double t0 = gm_clock_us();
    gr_info_t info;
    int n = 0;
    gr_pt_t used;
    *status = gr_astar(lib, h.x, h.z, goal.x, goal.z, relocate, goal_relocate, GL_GRID_MAX + gl_grid_slack, all, nall,
                       gl_buf, GL_CONNECT_CAP, &n, &used, &info);
    *us = gm_clock_us() - t0;
    if (*status != GR_OK || n < 1)
        return 0;
    return n;
}

/* ---- s38 NET: a curve over the road network ---- */

static gr_pt_t gl_net_buf[GL_MAX_SAMPLES];
static uint8_t gl_net_clr[GL_MAX_SAMPLES];
static gr_box_t gl_net_closed[4];

/* NET. A free point first, in a snapshot of its own (return 2): gr_dijkstra from the goal over GL_ATTACH_LEN m of path
 * (round 3: the tank side's reach; start moved to a free cell within GL_GOAL_RELOCATE, as GRID moves its goal; marks and
 * closed streets stamped); the network samples it reaches are the exits (reach cost / GL_NET_SPEED s), kept for this
 * tank and goal. The next snapshot (exits still this tank's, goal moved by at most GL_GOAL_MOVED; else the goal side
 * again): gr_dijkstra from the hull over GL_ATTACH_LEN m of path (as ATTACH; the connector is taken from it): the network
 * samples it reached are the entries (reach cost / GL_NET_SPEED s). The goal of the network search is the tank's
 * library target (its end nodes) or the exits; an edge with a sample in one of the tank's live closed boxes is not used;
 * the turn rule holds at every node. The curve: the connector (gr_reach_path) and the network chain up to its first
 * counted turn (a leg; *leg 1, more follows) or to its end, curvature from the points (pieces may run against their
 * sample order). Loop guard: a leg asked at a leg end (t->net_leg_req) whose new first turn is at the node it stands
 * on is not taken (*loop 1). 1 built; 2 the goal side done (the rest next snapshot); 0 and *why. us_part: the goal-side, hull-side and network search times. */
static int gl_net(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t, double *us, const char **why, int *n_in,
                  int *n_out, int *loop)
{
    const gr_lib_t *lib = s->routes_lib;
    const gn_net_t *net = s->routes_net;
    const gm_point_t h = gl_hull_centre(a);
    const gm_point_t goal = a->v->goal;
    const double t0 = gm_clock_us();
    double t1 = t0, t2 = t0;
    gr_info_t info;
    int ok = 0, ne = 0, nx = 0;
    *why = NULL;
    *loop = 0;
    const int nmk = gl_marks_now(t, s->ms, gl_mark_buf);
    int ncl = 0;
    for (int k = 0; k < t->cl_n; ++k)
        if (s->ms < t->cl_until[k])
            gl_net_closed[ncl++] = t->cl_box[k];
    if (t->target < 0) {
        /* round 3: the exits by reach from the goal over GL_ATTACH_LEN of path, in a snapshot of their own; kept in the
         * network's exit buffer for this tank and goal, the next snapshot runs the hull side */
        const int kept = gl_net_exits_valid && gl_net_exits_id == a->v->id && gl_net_exits_life == a->v->life_id
                         && gm_dist(gl_net_exits_goal, goal) <= GL_GOAL_MOVED;
        if (kept) {
            nx = gl_net_exits_n;
        } else {
            gl_net_exits_valid = 0;
            if (gr_dijkstra(lib, goal.x, goal.z, GL_ATTACH_LEN, GL_GOAL_RELOCATE, gl_mark_buf, nmk, &info) != GR_OK) {
                *why = "the goal has no free cell within GL_GOAL_RELOCATE";
            } else {
                const int nn = gn_near(net, goal.x, goal.z, GL_ATTACH_LEN, net->near_buf, (int)net->n_samples);
                for (int k = 0; k < nn; ++k) {
                    const uint32_t i = net->near_buf[k];
                    const float cost = gr_reach_cost(lib, net->sx[i], net->sz[i]);
                    if (isfinite(cost))
                        net->ext_buf[nx++] = (gn_end_t){i, cost / GL_NET_SPEED};
                }
                if (!nx)
                    *why = "no network within reach of the goal";
            }
            t1 = gm_clock_us();
            *n_in = 0;
            *n_out = nx;
            *us = t1 - t0;
            t->net_us_goal = *us;
            if (*why)
                return 0;
            gl_net_exits_valid = 1;
            gl_net_exits_id = a->v->id;
            gl_net_exits_life = a->v->life_id;
            gl_net_exits_goal = goal;
            gl_net_exits_n = nx;
            return 2;           /* the hull side and the network search in the next snapshot */
        }
    }
    if (!*why) {
        const int st0 = gr_dijkstra(lib, h.x, h.z, GL_ATTACH_LEN, GL_RELOCATE, gl_mark_buf, nmk, &info);
        if (st0 != GR_OK) {
            *why = "the hull's search start is not free";
        } else {
            const int nn = gn_near(net, h.x, h.z, GL_ATTACH_LEN, net->near_buf, (int)net->n_samples);
            for (int k = 0; k < nn; ++k) {
                const uint32_t i = net->near_buf[k];
                const float cost = gr_reach_cost(lib, net->sx[i], net->sz[i]);
                if (isfinite(cost))
                    net->ent_buf[ne++] = (gn_end_t){i, cost / GL_NET_SPEED};
            }
            if (!ne)
                *why = "no network sample within the attach range";
        }
    }
    t2 = gm_clock_us();
    gn_result_t res;
    memset(&res, 0, sizeof res);
    if (!*why) {
        const int st = gn_search(net, net->ent_buf, ne, t->target, net->ext_buf, nx, gl_net_closed, ncl, gl_net_buf,
                                 gl_net_clr, GL_MAX_SAMPLES, &res);
        if (st == GN_TOO_LONG)
            *why = "buffer too small";
        else if (st == GN_NO_ENTRY)
            *why = "no network sample within the attach range (all on closed streets)";
        else if (st == GN_NO_EXIT)
            *why = t->target >= 0 ? "the target has no end node on the network (or all on closed streets)"
                                  : "no network within reach of the goal (all on closed streets)";
        else if (st != GN_OK)
            *why = "not joined (no path)";
    }
    /* round 2 legs: the curve ends at the first counted turn; the loop guard */
    const int leg = !*why && res.n_turns > 0;
    const int leg_n = leg ? res.turn_at[0] + 1 : res.n;
    if (!*why && leg && t->net_leg_req && res.turn_node[0] == t->net_leg_node) {
        *why = "the new leg would end again at the node the tank stands on";
        *loop = 1;
    }
    if (!*why) {
        const int n = gr_reach_path(lib, h.x, h.z, net->sx[res.entry], net->sz[res.entry], gl_mark_buf, nmk, gl_buf,
                                    GL_CONNECT_CAP);
        if (n <= 0) {
            *why = "connector to the entry sample failed";
        } else if (n + leg_n > GL_MAX_SAMPLES) {
            *why = "buffer too small";
        } else {
            float conn = 0.0f;
            for (int k = 1; k < n; ++k)
                conn += hypotf(gl_buf[k].x - gl_buf[k - 1].x, gl_buf[k].z - gl_buf[k - 1].z);
            t->n = 0;
            gl_append_points(lib, t, gl_buf, n);
            const int conn_n = t->n;
            for (int k = 0; k < leg_n; ++k)
                gl_append_point(lib, t, gl_net_buf[k].x, gl_net_buf[k].z, 0.0f, gl_net_clr[k]);
            t->net_leg_next = leg;  /* gl_net_pick sets net_leg after gl_finish_curve */
            t->net_leg_node = leg ? res.turn_node[0] : -1;
            t->net_turns = res.n_turns;
            t->net_turn_deg = leg ? res.turn_deg[0] : 0.0f;
            t->net_end_x = gl_net_buf[res.n - 1].x;
            t->net_end_z = gl_net_buf[res.n - 1].z;
            t->net_exit_reach = t->target < 0 ? res.exit_cost * GL_NET_SPEED : 0.0f;
            for (int k = 0; k < t->n; ++k)  /* as gl_adopt_nav: the composed curve's curvature from its points */
                t->kappa[k] = gl_kappa_at(t->x, t->z, t->n, k);
            t->conn_n = conn_n;
            t->route = -1;
            t->route_k0 = 0;
            t->route_rev = 0;
            t->net_edges = res.edges;
            t->net_time = res.time_s;
            t->net_len = res.length_m;
            t->net_conn = conn;
            t->net_exit = t->target < 0 ? hypotf(net->sx[res.exit] - goal.x, net->sz[res.exit] - goal.z) : 0.0f;
            ok = 1;
        }
    }
    *n_in = ne;
    *n_out = t->target >= 0 ? -1 : nx;
    const double t3 = gm_clock_us();
    *us = t3 - t0;
    if (t->target >= 0)
        t->net_us_goal = 0.0;   /* a free point's goal side ran in the snapshot before (kept from there) */
    t->net_us_hull = t2 - t1;
    t->net_us_net = t3 - t2;
    return ok;
}

/* s38: the NET curve taken. A library target, whole path: as after ATTACH to it (not finishing, the end published as
 * the arrival point, a lane's slot; gl_decide asks for the slot / finishing at GL_FINISH_M before the end, a post arrives
 * at the end). A free point: the end is not the goal (no arrival point published there); GRID takes it from the end.
 * A leg (round 2: the path turns on the spot further on): its end is no arrival of any kind (no arrival point, no
 * slot); gl_decide asks for the next NET leg there. */
static void gl_net_pick(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t, const char *why, double us)
{
    t->finishing = 0;
    gl_finish_curve(t, GL_SRC_NET, s->ms);
    t->net_leg = t->net_leg_next;
    gl_set_end(a, t);
    if (t->target < 0)
        t->net_free = 1;
    if (t->target >= 0 && !t->net_leg)
        gl_lane_slot(a, t);
    else
        a->c.route_goal_moved = 0;
    gl_count_pick(GL_SRC_NET, us);
    gl_log_pick(s, a, t, why, us);
}

/* round 3: the goal side of a free point's NET search done this snapshot (counted as this snapshot's search) */
static void gl_net_goal_side(const gm_snapshot_t *s, const gm_agent_t *a, int n_out, double us)
{
    gl_count_pick(-1, us);
    gm_path_log(s, "lib: seat %llu net: goal side to (%.1f, %.1f): %d network samples within %.0f m of path (%.1f ms); the "
                "rest next snapshot", (unsigned long long)a->v->id, (double)a->v->goal.x, (double)a->v->goal.z, n_out,
                (double)GL_ATTACH_LEN, us / 1000.0);
}

static void gl_net_fail_log(const gm_snapshot_t *s, const gm_agent_t *a, const gl_tank_t *t, const char *fail, int n_in,
                            int n_out, double us)
{
    char goal[48];
    if (t->target >= 0)
        snprintf(goal, sizeof goal, "target %s", s->routes_lib->targets[t->target].id);
    else
        snprintf(goal, sizeof goal, "(%.1f, %.1f)", (double)a->v->goal.x, (double)a->v->goal.z);
    char exits[32] = "";
    if (n_out >= 0)
        snprintf(exits, sizeof exits, ", %d exit samples", n_out);
    gm_path_log(s, "lib: seat %llu net found no curve to %s: %s (%d entry samples%s, %.1f ms)", (unsigned long long)a->v->id,
                goal, fail ? fail : "-", n_in, exits, us / 1000.0);
}

/* The nearest free slot of a zone (no other hull on it, not claimed by a tank of the own army). -1 none. */
static int gl_pick_slot(const gm_snapshot_t *s, const gm_agent_t *agents, int count, int i, const gl_tank_t *t)
{
    const gr_lib_t *lib = s->routes_lib;
    const gm_point_t h = gl_hull_centre(&agents[i]);
    int best = -1;
    float bd = INFINITY;
    for (uint32_t k = 0; k < lib->n_slots; ++k) {
        const gr_slot_t *sl = &lib->slots[k];
        if ((int)sl->target != t->target)
            continue;
        int free = 1;
        for (int e = 0; e < t->excluded_n && free; ++e)
            free = t->excluded[e] != (int)k;
        for (int j = 0; j < count && free; ++j) {
            if (j == i)
                continue;
            const gm_agent_t *b = &agents[j];
            const gm_point_t bc = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
            if (lp_point_rect((gm_point_t){sl->x, sl->z}, bc, b->fwd, b->half_length, b->half_width) < GL_SLOT_TAKEN)
                free = 0;
        }
        for (int g = 0; g < GM_MAX_VEHICLES && free; ++g) {
            const gl_tank_t *o = &gl_tanks[g];
            if (o->used && o != t && o->slot == (int)k && o->army == t->army && s->ms <= o->seen_ms + GM_RETAIN_MS)
                free = 0;
        }
        if (!free)
            continue;
        const float d = hypotf(sl->x - h.x, sl->z - h.z);
        if (d < bd) {
            bd = d;
            best = (int)k;
        }
    }
    return best;
}

/* The standing hulls within GL_DETOUR_RANGE, as stamps for the detour search. */
static int gl_stamps(const gm_agent_t *agents, int count, int i, gr_box_t *boxes, int cap)
{
    int n = 0;
    for (int j = 0; j < count && n < cap; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i || !b->stationary || gm_dist(b->p, agents[i].p) > GL_DETOUR_RANGE + b->radius)
            continue;
        const gm_point_t bc = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
        boxes[n++] = (gr_box_t){bc.x, bc.z, b->fwd.x, b->fwd.z, b->half_length, b->half_width};
    }
    return n;
}

/* s34: the avoidance layer reports "this street is closed" to the route layer: a box over the standing blocker and
 * GL_STREET_BEYOND m of the tank's curve beyond it, kept GL_STREET_KEEP_MS; then a new pick to the same target
 * (GL_NEED_STREET). A tank already inside its capture circle parks where it stands instead (it is capturing). 1 when it
 * decided, 0 when the blocker is not a standing hull it can see. */
static int gl_street_close(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i, gl_tank_t *t, uint64_t blocker,
                           const char *why)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    const int j = blocker ? gm_agent_index(agents, count, blocker) : -1;
    if (j < 0 || !agents[j].stationary || !t->n)
        return 0;
    const gm_agent_t *b = &agents[j];
    if (a->region && gm_dist(a->p, v->goal_region_center) < v->goal_region_radius) {
        a->c.parked = 1;
        a->c.slot = a->p;
        a->c.slot_valid = 1;
        a->c.slot_phase = 2;
        t->sc_ms = 0;
        gm_path_log(s, "avoid: seat %llu blocked on the way to its slot by standing seat %llu inside its capture circle (%s): "
                    "parks where it stands", (unsigned long long)v->id, (unsigned long long)blocker, why);
        return 1;
    }
    const gm_point_t bc = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
    int k0 = t->proj;
    float bd = INFINITY;
    for (int k = t->proj; k < t->n && t->arc[k] - t->arc[t->proj] < GL_STOP_MAX; ++k) {
        const float d = gm_dist((gm_point_t){t->x[k], t->z[k]}, bc);
        if (d < bd) {
            bd = d;
            k0 = k;
        }
    }
    int k1 = t->n - 1;
    for (int k = k0; k < t->n; ++k)
        if (t->arc[k] >= t->arc[k0] + b->half_length + GL_STREET_BEYOND) {
            k1 = k;
            break;
        }
    const gm_point_t end = {t->x[k1], t->z[k1]};
    gm_point_t dir = gm_sub(end, bc);
    if (gm_len(dir) < 0.5f)
        dir = b->fwd;
    dir = gm_unit(dir);
    const gm_point_t start = gm_sub(bc, gm_scale(dir, b->half_length));
    const float len = gm_dist(start, end);
    const gm_point_t mid = gm_scale(gm_add(start, end), 0.5f);
    int slot = t->cl_n < 4 ? t->cl_n++ : 0;
    for (int k = 0; k < t->cl_n; ++k)
        if (t->cl_until[k] < t->cl_until[slot])
            slot = k;       /* full: the oldest goes */
    t->cl_box[slot] = (gr_box_t){mid.x, mid.z, dir.x, dir.z, 0.5f * len, b->half_width + 0.5f};
    t->cl_until[slot] = s->ms + GL_STREET_KEEP_MS;
    t->sc_state = 1;
    t->sc_blocker = blocker;
    t->sc_ms = 0;
    t->av = GL_AV_PENDING;
    gl_request(t, GL_NEED_STREET, "street closed", s->ms);
    gm_path_log(s, "avoid: seat %llu: street closed at (%.1f, %.1f) by standing seat %llu (%s): picks again to %s",
                (unsigned long long)v->id, (double)bc.x, (double)bc.z, (unsigned long long)blocker, why,
                t->target >= 0 ? s->routes_lib->targets[t->target].id : "its goal");
    return 1;
}

/* After GRID (and s38 NET) found nothing: NAV (s24's run-time search) when the route layer runs, else a failure that
 * retries after GL_RETRY_MS. (s37's tail of the GRID case, unchanged, as a function: s38 NET reaches it too.) */
static void gl_last_resort(const gm_snapshot_t *s, gm_agent_t *a, gl_tank_t *t, int routed)
{
    if (routed) {
        /* NAV: s24's run-time search (gm_route_layer asks for it from now on) */
        t->need = 0;
        t->n = 0;
        t->source = GL_SRC_NAV;
        t->nav_wait = 1;
        t->nav_rev = a->c.nav_revision;
        a->c.replan = 1;
        ++gl_stats.nav_requests;
        gm_path_log(s, "lib: seat %llu asks the run-time route layer (NAV) for (%.1f, %.1f)", (unsigned long long)a->v->id,
                    (double)a->v->goal.x, (double)a->v->goal.z);
        return;
    }
    t->need = 0;
    t->failed = GL_NEED_PLAN;
    t->retry_ms = s->ms + GL_RETRY_MS;
    ++gl_stats.failures;
    a->c.route_status = GM_NAV_UNREACHABLE;
}

/* The tank to serve this snapshot: the oldest request (then the lowest seat). */
static void gl_serve(const gm_snapshot_t *s, gm_agent_t *agents, int count, int routed)
{
    const gr_lib_t *lib = s->routes_lib;
    int pick = -1;
    gl_tank_t *pt = NULL;
    for (int i = 0; i < count; ++i) {
        const gm_agent_t *a = &agents[i];
        if (!a->v->controlled || !a->mobile)
            continue;
        gl_tank_t *t = gl_find(a->v->id, a->v->life_id);
        if (!t || !t->need)
            continue;
        if (!pt || t->need_since < pt->need_since || (t->need_since == pt->need_since && a->v->id < agents[pick].v->id)) {
            pick = i;
            pt = t;
        }
    }
    if (pick < 0)
        return;
    gm_agent_t *a = &agents[pick];
    gl_tank_t *t = pt;
    const char *why = t->need_why;
    double us = 0.0;
    int status = GR_OK;
    switch (t->need) {
    case GL_NEED_PLAN: {
        /* DESIGNATED once more (it may stand on its pose now), ATTACH, GRID, NAV */
        if (gl_try_designated(s, a, t))
            return;
        const gm_point_t h = gl_hull_centre(a);
        if (t->target >= 0 && lib->targets[t->target].kind == 0
            && hypotf(h.x - lib->targets[t->target].x, h.z - lib->targets[t->target].z)
                   < fmaxf(lib->targets[t->target].inner_radius, 5.0f)) {
            /* already inside the zone: straight to the slot / the goal */
            t->n = 0;
            gl_request(t, a->region ? GL_NEED_SLOT : GL_NEED_FINISH, why, t->need_since);
            return;
        }
        if (t->target >= 0) {
            if (gl_attach(s, a, t, &us)) {
                t->finishing = 0;
                gl_finish_curve(t, GL_SRC_ATTACH, s->ms);
                gl_set_end(a, t);
                gl_lane_slot(a, t);     /* s35 */
                gl_count_pick(GL_SRC_ATTACH, us);
                gl_log_pick(s, a, t, why, us);
                return;
            }
            gl_count_pick(-1, us);
            gm_path_log(s, "lib: seat %llu attach found no route to %s within %.0f m of path (%.1f ms)",
                        (unsigned long long)a->v->id, lib->targets[t->target].id, (double)GL_ATTACH_LEN, us / 1000.0);
        }
        t->net_tried = 0;       /* s38 */
        t->net_from_grid = 0;
        t->net_leg_req = 0;
        /* the next snapshot's search (one per snapshot): s38 NET for a library target when the network is loaded */
        t->need = t->target >= 0 && s->routes_net ? GL_NEED_NET : GL_NEED_GRID;
        return;
    }
    case GL_NEED_NET: {
        /* s38: a library target after ATTACH found nothing (GRID next when this fails), or a free point after GRID (NAV
         * next when this fails) */
        /* round 2: also the next leg at a leg end ("net leg": GRID next when it fails, then the last resort; the loop
         * guard goes to the last resort at once) */
        const char *fail = NULL;
        int n_in = 0, n_out = 0, loop = 0;
        const int got = gl_net(s, a, t, &us, &fail, &n_in, &n_out, &loop);
        if (got == 2) {
            gl_net_goal_side(s, a, n_out, us);
            return;             /* round 3: the request stays; the hull side in the next snapshot */
        }
        if (got) {
            t->net_from_grid = 0;
            gl_net_pick(s, a, t, why, us);
            return;
        }
        t->net_leg_req = 0;
        gl_count_pick(-1, us);
        gl_net_fail_log(s, a, t, fail, n_in, n_out, us);
        if (loop) {
            t->net_from_grid = 0;
            gl_last_resort(s, a, t, routed);
            return;
        }
        if (!t->net_from_grid) {
            t->net_tried = 1;
            t->need = GL_NEED_GRID; /* the next snapshot's search */
            return;
        }
        t->net_from_grid = 0;
        gl_last_resort(s, a, t, routed);
        return;
    }
    case GL_NEED_GRID: {
        /* s38: the request made at a NET curve's end may start up to GL_NET_END_SLACK beyond GL_GRID_MAX; NET is not
         * tried again after it, nor when NET already failed for this pick */
        const int net_end = t->net_end, net_tried = t->net_tried;
        t->net_end = t->net_tried = 0;
        int searched = 0;
        if (gm_dist(a->p, a->v->goal) <= GL_GRID_MAX + (net_end ? GL_NET_END_SLACK : 0.0f)) {
            gl_grid_slack = net_end ? GL_NET_END_SLACK : 0.0f;
            const int n = gl_grid(s, a, t, a->v->goal, NULL, 0, GL_RELOCATE, GL_GOAL_RELOCATE, &us, &status);
            gl_grid_slack = 0.0f;
            searched = 1;
            gl_count_pick(-1, us);
            if (n) {
                t->n = 0;
                gl_append_points(lib, t, gl_buf, n);
                t->route = -1;
                t->finishing = 1;
                gl_finish_curve(t, GL_SRC_GRID, s->ms);
                gl_set_end(a, t);
                gl_count_pick(GL_SRC_GRID, 0.0);
                gl_log_pick(s, a, t, why, us);
                return;
            }
            gm_path_log(s, "lib: seat %llu grid search to (%.1f, %.1f) failed (status %d, %.1f ms)",
                        (unsigned long long)a->v->id, (double)a->v->goal.x, (double)a->v->goal.z, status, us / 1000.0);
        }
        if (s->routes_net && !net_end && !net_tried) {
            /* s38 NET before NAV: this snapshot when no search ran yet, else the next one */
            t->net_from_grid = 1;
            if (searched) {
                t->need = GL_NEED_NET;
                return;
            }
            const char *fail = NULL;
            int n_in = 0, n_out = 0, loop = 0;
            t->net_leg_req = 0;
            const int got = gl_net(s, a, t, &us, &fail, &n_in, &n_out, &loop);
            if (got == 2) {
                gl_net_goal_side(s, a, n_out, us);
                t->need = GL_NEED_NET; /* round 3: the hull side in the next snapshot (net_from_grid kept) */
                return;
            }
            if (got) {
                t->net_from_grid = 0;
                gl_net_pick(s, a, t, why, us);
                return;
            }
            t->net_from_grid = 0;
            gl_count_pick(-1, us);
            gl_net_fail_log(s, a, t, fail, n_in, n_out, us);
        }
        gl_last_resort(s, a, t, routed);
        return;
    }
    case GL_NEED_SLOT:
    case GL_NEED_FINISH: {
        const int kind = t->need;
        gm_point_t goal = a->v->goal;
        float relocate = GL_GOAL_RELOCATE;
        if (kind == GL_NEED_SLOT) {
            if (t->slot < 0)
                t->slot = gl_pick_slot(s, agents, count, pick, t);
            if (t->slot < 0) {
                /* no free slot: the centre of the zone, as far as it gets (the s24 parking rules hold there) */
                goal = (gm_point_t){lib->targets[t->target].x, lib->targets[t->target].z};
            } else {
                goal = (gm_point_t){lib->slots[t->slot].x, lib->slots[t->slot].z};
                relocate = 2.0f;
            }
        }
        const int n = gl_grid(s, a, t, goal, NULL, 0, GL_RELOCATE, relocate, &us, &status);
        gl_count_pick(-1, us);
        if (n) {
            t->n = 0;
            gl_append_points(lib, t, gl_buf, n);
            t->route = -1;
            t->finishing = 1;
            gl_finish_curve(t, kind == GL_NEED_SLOT ? GL_SRC_SLOT : GL_SRC_FINISH, s->ms);
            gl_set_end(a, t);
            if (t->source == GL_SRC_SLOT && t->slot >= 0) {
                a->c.slot = (gm_point_t){t->x[t->n - 1], t->z[t->n - 1]};
                a->c.slot_valid = 1;
                a->c.slot_phase = 2;
            }
            gl_count_pick(t->source, 0.0);
            gl_log_pick(s, a, t, t->source == GL_SRC_SLOT ? "slot" : "finish", us);
            if (t->source == GL_SRC_SLOT)
                gm_path_log(s, "lib: seat %llu takes slot %d (number %d) of %s at (%.1f, %.1f)", (unsigned long long)a->v->id,
                            t->slot, t->slot >= 0 ? (int)lib->slots[t->slot].number : -1, lib->targets[t->target].id,
                            (double)t->x[t->n - 1], (double)t->z[t->n - 1]);
            return;
        }
        if (kind == GL_NEED_SLOT && t->slot >= 0 && t->excluded_n < GL_MAX_EXCLUDED) {
            t->excluded[t->excluded_n++] = t->slot; /* that slot cannot be reached from here: the next one */
            t->slot = -1;
            t->need_since = s->ms;
            return;
        }
        t->need = 0;
        t->failed = kind;
        t->retry_ms = s->ms + GL_RETRY_MS;
        ++gl_stats.failures;
        gm_path_log(s, "lib: seat %llu %s connector failed (status %d): tries again in %.0f s", (unsigned long long)a->v->id,
                    kind == GL_NEED_SLOT ? "slot" : "finishing", status, 0.001 * (double)GL_RETRY_MS);
        return;
    }
    case GL_NEED_DETOUR: {
        /* the curve beyond the blocker: the first free sample at least GL_DETOUR_BEYOND past it */
        static gr_box_t boxes[GM_MAX_VEHICLES];
        const int nbox = gl_stamps(agents, count, pick, boxes, GM_MAX_VEHICLES);
        int join = -1;
        for (int k = t->proj; k < t->n; ++k)
            if (t->arc[k] >= t->block_arc + GL_DETOUR_BEYOND
                && gr_eff_clearance(lib, t->x[k], t->z[k], boxes, nbox) >= GR_CLEAR_MIN
                && gr_eff_clearance(lib, t->x[k], t->z[k], t->marks, t->marks_n) >= GR_CLEAR_MIN) { /* s27: marks */
                join = k;
                break;
            }
        if (join < 0) {
            /* the blocker stands on the end of the curve */
            t->need = 0;
            t->av = GL_AV_WAIT;
            t->av_until = s->ms + GL_RETRY_MS;
            gm_path_log(s, "avoid: seat %llu: seat %llu stands on the end of its curve: waits, decides again in %.0f s",
                        (unsigned long long)a->v->id, (unsigned long long)t->blocker, 0.001 * (double)GL_RETRY_MS);
            return;
        }
        const int n = gl_grid(s, a, t, (gm_point_t){t->x[join], t->z[join]}, boxes, nbox, 0.0f, 1.0f, &us, &status);
        gl_count_pick(-1, us);
        if (n) {
            const int tail = t->n - join;
            memcpy(gl_tmp_x, t->x + join, sizeof(float) * (size_t)tail);
            memcpy(gl_tmp_z, t->z + join, sizeof(float) * (size_t)tail);
            memcpy(gl_tmp_k, t->kappa + join, sizeof(float) * (size_t)tail);
            memcpy(gl_tmp_c, t->clr + join, (size_t)tail);
            const int finishing = t->finishing;
            t->n = 0;
            gl_append_points(lib, t, gl_buf, n);
            for (int k = 0; k < tail; ++k)
                gl_append_point(lib, t, gl_tmp_x[k], gl_tmp_z[k], gl_tmp_k[k], gl_tmp_c[k]);
            t->finishing = finishing;
            const int keep_source = t->source;
            gl_finish_curve(t, GL_SRC_DETOUR, s->ms);
            (void)keep_source;
            gl_set_end(a, t);
            if (t->net_free || t->net_leg)
                a->c.route_goal_moved = 0;  /* s38: a NET curve's end is not the free goal / a leg end (as gl_net_pick) */
            t->av = GL_AV_IDLE;
            gl_count_pick(GL_SRC_DETOUR, 0.0);
            gm_path_log(s, "avoid: seat %llu detour round %s%llu: %d samples, rejoins its curve %.0f m on (%.1f ms)",
                        (unsigned long long)a->v->id, t->blocker ? "seat " : "where it stuck ",
                        (unsigned long long)t->blocker, n, (double)(t->arc[t->n - 1] - t->arc[n > 0 ? n - 1 : 0]),
                        us / 1000.0);
            return;
        }
        t->need = 0;
        if (status == GR_START_BLOCKED) {
            const int backed_here = t->backed_ms && s->ms < t->backed_ms + GL_BACK_AGAIN_MS
                && gm_dist(a->p, t->backed_at) < GL_BACK_SAME;
            if (!backed_here) {
                t->av = GL_AV_BACK;
                t->back_kind = GL_BACK_DETOUR;
                t->back_from = a->p;
                t->av_until = s->ms + GL_BACK_MS;
                gm_path_log(s, "avoid: seat %llu has no room to turn out (search start not free): backs up %.0f m",
                            (unsigned long long)a->v->id, (double)GL_BACK_M);
                return;
            }
        }
        gm_path_log(s, "avoid: seat %llu finds no way round %s%llu (status %d, %.1f ms)", (unsigned long long)a->v->id,
                    t->blocker ? "seat " : "", (unsigned long long)t->blocker, status, us / 1000.0);
        if (gl_street_close(s, agents, count, pick, t, t->blocker, "no way round"))
            return;             /* s34: the street is closed: a new pick to the same target */
        t->av = GL_AV_WAIT;
        t->av_until = s->ms + GL_RETRY_MS;
        gm_path_log(s, "avoid: seat %llu waits, decides again in %.0f s", (unsigned long long)a->v->id,
                    0.001 * (double)GL_RETRY_MS);
        return;
    }
    case GL_NEED_STREET: {
        /* s34: the same target with the closed street left out */
        int rejected = 0;
        if (t->target >= 0 && gl_attach_street(s, a, t, &us, &rejected)) {
            t->finishing = 0;
            t->sc_state = 0;
            t->sc_ms = 0;
            t->av = GL_AV_IDLE;
            gl_finish_curve(t, GL_SRC_ATTACH, s->ms);
            gl_set_end(a, t);
            gl_lane_slot(a, t);         /* s35 */
            gl_count_pick(GL_SRC_ATTACH, us);
            gl_log_pick(s, a, t, why ? why : "street closed", us);   /* s36 P1: the request's reason */
            return;
        }
        gl_count_pick(-1, us);
        t->need = 0;
        t->sc_state = 2;
        t->sc_until = s->ms + GL_STREET_RETRY_MS;
        t->av = GL_AV_IDLE;
        gm_path_log(s, "avoid: seat %llu: no other street to %s (attach over %.0f m of path, %d route(s) through the closed street "
                    "left out, %.1f ms): stands at least %.0f m short of seat %llu, tries again in %.0f s",
                    (unsigned long long)a->v->id, t->target >= 0 ? lib->targets[t->target].id : "its goal (no target)",
                    (double)GL_STREET_LEN, rejected, us / 1000.0, (double)GL_STREET_BACK_M, (unsigned long long)t->sc_blocker,
                    0.001 * (double)GL_STREET_RETRY_MS);
        return;
    }
    default:
        t->need = 0;
        return;
    }
}

/* NAV source: the route layer's polyline as a curve once it is in. */
static void gl_adopt_nav(const gm_snapshot_t *s, gm_agent_t *agents, int count)
{
    const gr_lib_t *lib = s->routes_lib;
    for (int i = 0; i < count; ++i) {
        gm_agent_t *a = &agents[i];
        gm_commitment_t *c = &a->c;
        if (!a->v->controlled || !a->mobile)
            continue;
        gl_tank_t *t = gl_find(a->v->id, a->v->life_id);
        if (!t || t->source != GL_SRC_NAV)
            continue;
        if (c->route_status == GM_NAV_UNREACHABLE && !c->nav_count) {
            t->source = GL_SRC_NONE;
            t->nav_wait = 0;
            t->failed = GL_NEED_PLAN;
            t->retry_ms = s->ms + GL_RETRY_MS;
            ++gl_stats.failures;
            continue;
        }
        if (!c->nav_count || (t->n && c->nav_revision == t->nav_rev))
            continue;
        t->nav_rev = c->nav_revision;
        t->n = 0;
        const gm_point_t h = gl_hull_centre(a);
        gl_append_point(lib, t, h.x, h.z, 0.0f, -1);
        gm_point_t prev = h;
        for (int k = 0; k < c->nav_count; ++k) {
            const gm_point_t q = c->nav_points[k];
            const float len = gm_dist(prev, q);
            for (float at = GR_STEP; at < len; at += GR_STEP)
                gl_append_point(lib, t, prev.x + (q.x - prev.x) * at / len, prev.z + (q.z - prev.z) * at / len, 0.0f, -1);
            gl_append_point(lib, t, q.x, q.z, 0.0f, -1);
            prev = q;
        }
        for (int k = 0; k < t->n; ++k)
            t->kappa[k] = gl_kappa_at(t->x, t->z, t->n, k);
        t->conn_n = t->n;
        t->route = -1;
        t->finishing = 1;
        gl_finish_curve(t, GL_SRC_NAV, s->ms);
        t->nav_wait = 0;
        gl_set_end(a, t);
        gl_count_pick(GL_SRC_NAV, 0.0);
        gl_log_pick(s, a, t, "nav route in", 0.0);
    }
}

/* Is this tank's route request the run-time route layer's (NAV source)? (gm_route_layer) */
static int gl_wants_nav(const gm_agent_t *a)
{
    const gl_tank_t *t = gl_find(a->v->id, a->v->life_id);
    return t && t->source == GL_SRC_NAV;
}

/* ---- following: one rule for every source ---- */

/* The hull's pose after k of `steps` equal parts of a turn on the spot by `turn` (rad). Forward gear (gear 1, A or
 * D alone): the rotation centre moves on a circle of GM_PATH_PIVOT_R about a point GM_PATH_PIVOT_R to the steer
 * side. Reverse gear (gear 2, S + A / D; s27): that point on the other side, the hull moving backwards. */
static void gl_pivot_pose(const gm_agent_t *a, float turn, int gear, int k, int steps, gm_point_t *c, gm_point_t *f)
{
    const gm_vehicle_t *v = a->v;
    const float side = (turn > 0.0f ? 1.0f : -1.0f) * (gear == 2 ? -1.0f : 1.0f);
    const gm_point_t f0 = a->fwd;
    const gm_point_t centre = {a->p.x - side * f0.z * GM_PATH_PIVOT_R, a->p.z + side * f0.x * GM_PATH_PIVOT_R};
    const gm_point_t rel = gm_sub(a->p, centre);
    const float ang = turn * (float)k / (float)steps, cs = cosf(ang), sn = sinf(ang);
    const gm_point_t p = {centre.x + rel.x * cs - rel.z * sn, centre.z + rel.x * sn + rel.z * cs};
    *f = gm_axis(v->heading + ang);
    *c = lp_centre(v, p.x, p.z, *f);
}

/* Room among the other hulls for a turn on the spot by `turn` (rad) in `gear` (s24 gm_path_turn_room's rule): the
 * hull (grown by GM_SKIN) every GM_PATH_PIVOT_STEP along the arc; no more overlap with a hull than at the start.
 * 0 and *blocker when a hull is in the way. */
static int gl_pivot_room(const gm_agent_t *agents, int count, int i, float turn, int gear, uint64_t *blocker)
{
    const gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    const gm_point_t f0 = a->fwd;
    const gm_point_t c0 = lp_centre(v, a->p.x, a->p.z, f0);
    const float hl = a->half_length + GM_SKIN, hw = a->half_width + GM_SKIN;
    const int steps = (int)ceilf(fabsf(turn) / GM_PATH_PIVOT_STEP);
    for (int j = 0; j < count; ++j) {
        const gm_agent_t *b = &agents[j];
        if (j == i || gm_dist(a->p, b->p) > a->radius + b->radius + 2.0f * GM_PATH_PIVOT_R + 1.0f)
            continue;
        const gm_point_t bc = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
        const float pen0 = lp_box_pen(c0, f0, hl, hw, bc, b->fwd, b->half_length, b->half_width);
        for (int k = 1; k <= steps; ++k) {
            gm_point_t c, f;
            gl_pivot_pose(a, turn, gear, k, steps, &c, &f);
            if (lp_box_pen(c, f, hl, hw, bc, b->fwd, b->half_length, b->half_width) > fmaxf(pen0, 0.0f) + 0.02f) {
                *blocker = b->v->id;
                return 0;
            }
        }
    }
    return 1;
}

/* s27 fix 1: room against the walls (map.bin's hard cells) for the same turn: at every step of the arc the bare hull
 * may come no closer than GL_WALL_GAP to a hard cell, unless it is no closer than at the starting pose (a hull that
 * already touches does not block itself). 1 free. */
static int gl_pivot_walls(const gr_lib_t *lib, const gm_agent_t *a, float turn, int gear)
{
    const gm_point_t c0 = lp_centre(a->v, a->p.x, a->p.z, a->fwd);
    const float d0 = gr_hull_hard_distance(lib, c0.x, c0.z, a->v->heading, a->half_length, a->half_width, 0.5f);
    const int steps = (int)ceilf(fabsf(turn) / GM_PATH_PIVOT_STEP);
    for (int k = 1; k <= steps; ++k) {
        gm_point_t c, f;
        gl_pivot_pose(a, turn, gear, k, steps, &c, &f);
        const float d = gr_hull_hard_distance(lib, c.x, c.z, atan2f(f.z, f.x), a->half_length, a->half_width, 0.5f);
        if (d < GL_WALL_GAP && d < d0 - 1e-3f)
            return 0;
    }
    return 1;
}

/* The turn a turn on the spot makes: until the heading is within GL_TURN_DEG of the aim direction. */
static float gl_pivot_turn(float e)
{
    const float settle = GL_TURN_DEG * 0.017453293f;
    return fabsf(e) > settle ? e - (e > 0.0f ? settle : -settle) : 0.0f;
}

static void gl_follow(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gl_tank_t *t = gl_find(v->id, v->life_id);
    a->following = a->tight = a->reverse_leg = 0;
    a->target = a->aim = a->goal;
    a->cross = 0.0f;
    a->lib_source = t ? t->source : GL_SRC_NONE;
    a->lib_route = -1;
    a->lib_sample = -1;
    a->lib_cross = a->lib_clear = NAN;
    a->stop_vehicle = a->stop_turn = a->stop_end = INFINITY;
    a->lib_pivot_rev = 0;
    a->lib_turning = 0;
    a->path_blocker = 0;
    a->path_blocker_moving = 0;
    if (!t || !t->n) {
        a->remaining = gm_dist(a->p, a->goal);
        return;
    }
    a->lib_rev = t->rev;
    const gm_point_t h = gl_hull_centre(a);
    /* projection: forwards only, in a window from the last index */
    int k = t->proj < t->n ? t->proj : t->n - 1;
    float bd = INFINITY;
    for (int j = k; j < t->n && j <= t->proj + 80; ++j) {
        const float d = hypotf(t->x[j] - h.x, t->z[j] - h.z);
        if (d < bd) {
            bd = d;
            k = j;
        }
    }
    t->proj = k;
    const int k1 = k + 1 < t->n ? k + 1 : k, k0 = k1 == k ? (k > 0 ? k - 1 : k) : k;
    const float sx = t->x[k1] - t->x[k0], sz = t->z[k1] - t->z[k0], sl = hypotf(sx, sz);
    const float cross = sl > 1e-3f ? (sx * (h.z - t->z[k0]) - sz * (h.x - t->x[k0])) / sl : bd;
    a->following = 1;
    a->cross = fabsf(cross);
    a->lib_cross = cross;
    a->lib_clear = 0.1f * (float)t->clr[k];
    a->lib_sample = t->route >= 0 && k >= t->conn_n ? t->route_k0 + (t->route_rev ? -(k - t->conn_n) : (k - t->conn_n)) : -1;
    a->lib_route = k >= t->conn_n ? t->route : -1;
    const float arc0 = t->arc[k], end = t->arc[t->n - 1];
    a->remaining = end - arc0;
    /* aim */
    float L = gm_clamp(GL_AIM_K * fabsf(a->speed), GL_AIM_MIN, GL_AIM_MAX), kmax = 0.0f;
    for (int j = k; j < t->n && t->arc[j] <= arc0 + GL_AIM_LOOK; ++j)
        kmax = fmaxf(kmax, fabsf(t->kappa[j]));
    if (kmax > 1e-4f)
        L = fminf(L, GL_AIM_LAT / kmax);
    L = fmaxf(L, GL_AIM_FLOOR);
    int ai = t->n - 1;
    for (int j = k; j < t->n; ++j)
        if (t->arc[j] >= arc0 + L) {
            ai = j;
            break;
        }
    a->aim = a->target = (gm_point_t){t->x[ai], t->z[ai]};
    /* stop distance (c): the end of the curve */
    a->stop_end = a->remaining;
    /* (b): the first sample whose direction differs from the heading by more than 30 deg. The samples within
     * GL_TURN_NEAR of the projection are where the hull stands: a corner there is turned on the spot towards the
     * aim (the driver's rule at d <= 1 m), after which they no longer point where the hull does. */
    for (int j = k; j + 1 < t->n && t->arc[j] - arc0 <= GL_STOP_MAX; ++j) {
        if (t->arc[j] - arc0 < GL_TURN_NEAR)
            continue;
        const float dir = atan2f(t->z[j + 1] - t->z[j], t->x[j + 1] - t->x[j]);
        if (fabsf(gm_angle(dir - v->heading)) > GL_TURN_STOP) {
            a->stop_turn = t->arc[j] - arc0;
            break;
        }
    }
    /* (a): the own hull (grown) marched along the curve against every other hull */
    {
        const float hl = a->half_length + GL_HULL_GROW, hw = a->half_width + GL_HULL_GROW;
        static int near_j[GM_MAX_VEHICLES];
        static float pen0[GM_MAX_VEHICLES][2];
        static gm_point_t hc[GM_MAX_VEHICLES][2];
        int nn = 0;
        for (int j = 0; j < count; ++j) {
            const gm_agent_t *b = &agents[j];
            if (j == i || gm_dist(b->p, a->p) > GL_STOP_MAX + a->radius + b->radius + gm_len(b->vel) * GL_PREDICT_S)
                continue;
            const gm_point_t bc = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
            hc[nn][0] = bc;
            hc[nn][1] = b->stationary ? bc : gm_add(bc, gm_scale(b->vel, GL_PREDICT_S));
            for (int q = 0; q < 2; ++q)
                pen0[nn][q] = lp_box_pen(h, a->fwd, hl, hw, hc[nn][q], b->fwd, b->half_length, b->half_width);
            near_j[nn++] = j;
        }
        if (nn) {
            float free = 0.0f;
            int hit = 0;
            for (int j = k; j < t->n && !hit; ++j) {
                const float s_arc = t->arc[j] - arc0;
                if (s_arc > GL_STOP_MAX)
                    break;
                const int jn = j + 1 < t->n ? j + 1 : j, jp = jn == j ? (j > 0 ? j - 1 : j) : j;
                const gm_point_t f = gm_unit((gm_point_t){t->x[jn] - t->x[jp], t->z[jn] - t->z[jp]});
                const gm_point_t p = {t->x[j], t->z[j]};
                for (int q = 0; q < nn && !hit; ++q) {
                    const gm_agent_t *b = &agents[near_j[q]];
                    const int moving = !b->stationary;
                    /* right of way between two moving tanks: the lower seat ignores the higher seat's predicted hull */
                    const int ignore_predicted = !moving || (b->v->controlled && v->id < b->v->id);
                    for (int w = 0; w < (ignore_predicted ? 1 : 2) && !hit; ++w) {
                        const float pen = lp_box_pen(p, f, hl, hw, hc[q][w], b->fwd, b->half_length, b->half_width);
                        if (pen > fmaxf(pen0[q][w], 0.0f) + 0.02f) {
                            hit = 1;
                            a->path_blocker = b->v->id;
                            a->path_blocker_moving = moving;
                            t->block_arc = t->arc[j];
                        }
                    }
                }
                if (!hit)
                    free = s_arc;
            }
            if (hit)
                a->stop_vehicle = free;
        }
    }
    a->path_free = a->stop_vehicle;
    a->path_d = fminf(a->stop_vehicle, fminf(a->stop_turn, a->stop_end));
    /* the start turn of a designated route: on the spot, in the library's gear, until the nose points within 8 deg
     * of the point 5 m along */
    if (t->turn_pending) {
        int j5 = t->n - 1;
        for (int j = 0; j < t->n; ++j)
            if (t->arc[j] >= GL_TURN_AHEAD) {
                j5 = j;
                break;
            }
        const gm_point_t to = {t->x[j5] - a->p.x, t->z[j5] - a->p.z};
        const float e = gm_len(to) > 0.5f ? gm_angle(atan2f(to.z, to.x) - v->heading) : 0.0f;
        if (fabsf(e) * 57.29578f < GL_TURN_DEG) {
            t->turn_pending = 0;
            gm_path_log(s, "lib: seat %llu start turn done (%.1f deg off)", (unsigned long long)v->id,
                        (double)(e * 57.29578f));
        } else {
            a->aim = a->target = (gm_point_t){t->x[j5], t->z[j5]};
            a->path_d = 0.0f;
            a->lib_pivot_rev = t->turn_gear == 2;
            a->lib_turning = 1;
        }
    }
    /* a turn on the spot the driver will make (|e| > 30 deg, or no room ahead and |e| >= 8 deg: ground_drive.inc), and
     * the heading error to the aim; s27: its room (walls and hulls) is decided in gl_decide */
    a->lib_pivot_blocked = 0;
    a->lib_pivot_blocker = 0;
    {
        const gm_point_t to = gm_sub(a->aim, a->p);
        a->lib_pivot_e = gm_len(to) >= 1.0f ? gm_angle(atan2f(to.z, to.x) - v->heading) : 0.0f;
        const float deg = fabsf(a->lib_pivot_e) * 57.29578f;
        a->lib_will_pivot = deg > 30.0f || (a->path_d <= GM_PATH_NO_ROOM && deg >= GL_TURN_DEG);
    }
    (void)count;
    a->v_des = v->cruise_speed;
    a->path_cap = INFINITY;
}

/* Room straight behind (library map and hulls): up to 4 m. */
static float gl_room_behind(const gm_snapshot_t *s, const gm_agent_t *agents, int count, int i)
{
    const gm_agent_t *a = &agents[i];
    const gr_lib_t *lib = s->routes_lib;
    float free = 0.0f;
    for (float d = 0.5f; d <= 4.0f + 1e-3f; d += 0.5f) {
        const gm_point_t p = gm_sub(a->p, gm_scale(a->fwd, d));
        const gm_point_t c = lp_centre(a->v, p.x, p.z, a->fwd);
        if (gr_hull_hard_distance(lib, c.x, c.z, a->v->heading, a->half_length, a->half_width, 0.2f) < 0.2f)
            break;
        int hit = 0;
        for (int j = 0; j < count && !hit; ++j) {
            const gm_agent_t *b = &agents[j];
            if (j == i)
                continue;
            const gm_point_t bc = lp_centre(b->v, b->p.x, b->p.z, b->fwd);
            hit = lp_box_pen(c, a->fwd, a->half_length + 0.2f, a->half_width + 0.2f, bc, b->fwd, b->half_length,
                             b->half_width) > 0.0f;
        }
        if (hit)
            break;
        free = d;
    }
    return free;
}

/* s36: the standing hull a controlled tank stands for (it does not move, a standing hull on its curve within GL_BLOCKED_M
 * stops it, or its last blocked decision named one under 0.6 s ago), 0 when it stands for its own reason or moves. */
static uint64_t gl_standing_for(const gm_snapshot_t *s, const gm_agent_t *agents, int j)
{
    const gm_agent_t *b = &agents[j];
    if (!b->v->controlled || !b->mobile || fabsf(b->speed) >= 1.0f)
        return 0;
    if (b->path_blocker && !b->path_blocker_moving && b->stop_vehicle < GL_BLOCKED_M && b->stop_vehicle <= b->path_d + 1e-3f)
        return b->path_blocker;
    const gl_tank_t *t = gl_find(b->v->id, b->v->life_id);
    return t && t->veh_blocker && s->ms <= t->veh_blocker_ms + 600 ? t->veh_blocker : 0;
}

/* s36: the walk from tank i over the hulls each stands for (at most GL_RING_STEPS blockers). 1: i is in a ring
 * (chain[first..n-1], each blocked by the next, the last by chain[first]); 2: the walk ends in a ring i is queued behind;
 * 0: it ends at a tank standing for its own reason (or goes on longer). */
static int gl_ring_walk(const gm_snapshot_t *s, const gm_agent_t *agents, int count, int i, int *chain, int *n, int *first)
{
    int cur = i;
    *n = *first = 0;
    for (int step = 0; step <= GL_RING_STEPS; ++step) {
        const uint64_t nb = gl_standing_for(s, agents, cur);
        const int j = nb ? gm_agent_index(agents, count, nb) : -1;
        if (j < 0)
            return 0;
        chain[(*n)++] = cur;
        for (int p = 0; p < *n; ++p)
            if (chain[p] == j) {
                *first = p;
                return p == 0 ? 1 : 2;
            }
        cur = j;
    }
    return 0;
}

/* s36: one line per ring (members, who gives way, room); the same members are logged again after 10 s at most once */
static void gl_ring_log(const gm_snapshot_t *s, gl_tank_t *t, const gm_agent_t *agents, const int *chain, int n, int first,
                        const char *tail)
{
    uint64_t sig = 0;
    char members[96] = "";
    for (int k = first; k < n; ++k) {
        sig = sig * 131u + agents[chain[k]].v->id;
        const size_t len = strlen(members);
        snprintf(members + len, sizeof members - len, "%s%llu", k > first ? " -> " : "", (unsigned long long)agents[chain[k]].v->id);
    }
    if (sig == t->ring_log_sig && s->ms < t->ring_log_ms + 10000u)
        return;
    t->ring_log_sig = sig;
    t->ring_log_ms = s->ms;
    gm_path_log(s, "avoid: ring of %d tanks waiting for each other (seats %s, each blocked by the next): %s", n - first, members,
                tail);
}

/* ---- avoidance (vehicles only) and the publication ---- */

static void gl_decide(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    gm_commitment_t *c = &a->c;
    const uint64_t ms = s->ms;
    const gr_lib_t *lib = s->routes_lib;
    gl_tank_t *t = gl_find(v->id, v->life_id);
    a->path_go = a->path_rev = 0;
    a->lp_moving = 0;
    a->cmd_speed = a->cmd_yaw = a->cmd_curvature = 0.0f;
    a->cmd_heading = v->heading;
    a->blocker = 0;
    a->geometry = GM_CLEAR;
    a->firm = 0;
    a->reversing = 0;
    a->state = GM_DRIVING;
    if (gm_dist(a->aim, a->p) > 0.3f)
        a->cmd_heading = atan2f(a->aim.z - a->p.z, a->aim.x - a->p.x);
    if (!t || !t->n) {
        /* no curve yet (a search owed, or NAV pending): stand until it comes */
        a->state = GM_WAIT_GEOMETRY;
        a->geometry = GM_UNKNOWN;
        if (t && t->failed)
            c->route_status = GM_NAV_UNREACHABLE;
        else if (!t || t->source != GL_SRC_NAV)
            c->route_status = GM_NAV_PENDING;
        return;
    }
    c->route_status = GM_NAV_READY;
    /* s38 round 2: at the end of a NET leg (the post's arrival criterion below: within GM_STOP_RADIUS of the curve's
     * end along it and slower than 1 m/s) the next leg is searched from where the tank stands; that end is no arrival
     * (the two rules below and the net-end rule leave a leg alone) */
    if (t->net_leg && !t->need && a->remaining <= GM_STOP_RADIUS && fabsf(a->speed) < 1.0f) {
        gl_request(t, GL_NEED_NET, "net leg", ms);
        if (t->need == GL_NEED_NET) {
            t->net_leg_req = 1;
            t->net_from_grid = 0;
        }
    }
    /* arrival at the end of a curve that ends on the goal (post, grid goal, finishing): as s24 reports it */
    if (!t->net_leg && !a->region && t->finishing + (t->target >= 0 && lib->targets[t->target].kind == 1) > 0
        && a->remaining <= GM_STOP_RADIUS && fabsf(a->speed) < 1.0f) {
        c->arrived = 1;
        a->state = GM_ARRIVED;
        return;
    }
    /* the zone's slot (or, a goal that is not a capture, the goal itself) once the route's end is near */
    if (!t->net_leg && !t->finishing && !t->need && t->target >= 0 && lib->targets[t->target].kind == 0
        && a->remaining <= GL_FINISH_M)
        gl_request(t, a->region ? GL_NEED_SLOT : GL_NEED_FINISH, "zone reached", ms);
    /* the end reached but not the goal (pushed off, or a slot that was not parked on): pick again */
    if (t->finishing && !t->need && a->remaining < 1.0f && gm_dist(gl_hull_centre(a), (gm_point_t){t->x[t->n - 1], t->z[t->n - 1]}) > 3.0f)
        gl_request(t, a->region ? GL_NEED_SLOT : GL_NEED_PLAN, "end missed", ms);
    /* s38: a NET curve to a free point: the GRID search to the goal once the end is near (GL_FINISH_M) and the goal
     * within GL_GRID_MAX of the hull, or at the end */
    if (t->net_free && !t->net_leg && !t->need && t->target < 0
        && ((a->remaining <= GL_FINISH_M && gm_dist(gl_hull_centre(a), v->goal) <= GL_GRID_MAX) || a->remaining < 1.0f)) {
        gl_request(t, GL_NEED_GRID, "net end", ms);
        t->net_end = t->need == GL_NEED_GRID;
    }
    /* off the curve: an event */
    if (a->cross > GL_OFF_CURVE && !t->need && t->av == GL_AV_IDLE) {
        gm_path_log(s, "lib: seat %llu is %.1f m off its curve: picks again from here", (unsigned long long)v->id,
                    (double)a->cross);
        if (t->source == GL_SRC_NAV)
            c->replan = 1;
        else
            gl_request(t, t->finishing && t->source != GL_SRC_DETOUR ? (a->region ? GL_NEED_SLOT : GL_NEED_GRID)
                                                                     : GL_NEED_PLAN, "off curve", ms);
    }

    /* s34 street closed. (1) A decision under way: a pick owed (hold until the route layer serves it), or no other
     * street (stand at least GL_STREET_BACK_M short of the blocker, backing straight when there is room; again after
     * GL_STREET_RETRY_MS). (2) Blocked-time accounting against one standing hull on the curve ahead: blocked decisions,
     * waits and detours that do not get the tank past all count; GL_STREET_MS in all closes the street. */
    a->lib_street = t->sc_state == 2;
    if (t->sc_state == 1) {
        a->state = GM_WAIT_GEOMETRY;
        a->blocker = t->sc_blocker;
        return;
    }
    if (t->sc_state == 2) {
        if (ms >= t->sc_until) {
            t->sc_state = 1;
            gl_request(t, GL_NEED_STREET, "street closed: tries again", ms);
            a->state = GM_WAIT_GEOMETRY;
            return;
        }
        const int bj = gm_agent_index(agents, count, t->sc_blocker);
        if (bj >= 0 && gm_dist(a->p, agents[bj].p) < GL_STREET_BACK_M && gl_room_behind(s, agents, count, i) >= 0.5f) {
            gl_align_back(s, agents, count, i);
            return;
        }
        a->state = GM_WAIT_TRAFFIC;
        a->blocker = t->sc_blocker;
        return;
    }

    /* s36 give-way (a ring of tanks waiting for each other): (1) backing straight, one hull length + 2 m or until the room
     * ends; (2) then holding, not advancing while the tank it blocked is within GM_YIELD_RANGE ahead, GM_YIELD_MS at most
     * (or until that tank stands blocked by it again: the ring again) */
    if (t->av == GL_AV_BACK && t->back_kind == GL_BACK_YIELD) {
        const float room = gl_room_behind(s, agents, count, i);
        if (gm_dist(a->p, t->back_from) >= t->yield_back_m || ms >= t->av_until || room < 0.5f) {
            t->av = GL_AV_IDLE;
            t->yield_hold_until = ms + GM_YIELD_MS;
            t->low_since = t->stuck_since = 0;
            t->sc_ms = 0;
            t->sc_last_ms = ms;
            a->state = GM_WAIT_TRAFFIC;
            a->blocker = t->yield_for;
            return;
        }
        t->sc_last_ms = ms;
        a->path_go = a->path_rev = 1;
        a->reversing = 1;
        a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
        a->path_d = room;
        a->path_cap = INFINITY;
        a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
        a->cmd_heading = v->heading;
        return;
    }
    if (t->yield_hold_until) {
        const int pj = gm_agent_index(agents, count, t->yield_for);
        const char *end = NULL;
        if (pj < 0 || gm_dist(a->p, agents[pj].p) >= GM_YIELD_RANGE || gm_dot(a->fwd, gm_sub(agents[pj].p, a->p)) <= 0.0f)
            end = "passed";
        else if (gl_standing_for(s, agents, pj) == v->id)
            end = "the ring again";
        else if (ms >= t->yield_hold_until)
            end = "time over";
        if (!end) {
            t->sc_last_ms = ms;
            a->state = GM_WAIT_TRAFFIC;
            a->blocker = t->yield_for;
            return;
        }
        gm_path_log(s, "avoid: seat %llu give-way for seat %llu ends: %s (%.1f s after it began)", (unsigned long long)v->id,
                    (unsigned long long)t->yield_for, end, 0.001 * (double)(ms - t->yield_ms));
        t->yield_hold_until = 0;
        t->low_since = t->stuck_since = 0;
    }

    /* s36 ring: blocked by a standing hull that stands for another standing hull ... back to this tank (or into a ring it
     * is queued behind). The highest seat of the ring with GL_BACK_MIN room behind gives way; with none, the highest goes
     * on as before (detour, street closing). The same highest seat gave way for a member of this ring less than
     * GL_RING_AGAIN_MS ago: no second give-way, it goes on as before; the others wait until it picks again or
     * GL_RING_REPEAT_WAIT_MS passed. Everyone else of the ring and behind it waits: no detour, no street closing, the
     * 2 s / 6 s blocked clocks stand still. */
    int ring = GL_RING_NONE;
    if (t->av == GL_AV_IDLE && !t->need) {
        int chain[GL_RING_STEPS + 2], n = 0, first = 0;
        const int kind = gl_ring_walk(s, agents, count, i, chain, &n, &first);
        int high = -1, give = -1, again = 0;
        const gl_tank_t *th = NULL;
        for (int k = first; kind && k < n; ++k)
            if (high < 0 || agents[chain[k]].v->id > agents[high].v->id)
                high = chain[k];
        if (kind == 1) {
            th = gl_find(agents[high].v->id, agents[high].v->life_id);
            for (int k = first; th && th->yield_ms && ms < th->yield_ms + GL_RING_AGAIN_MS && k < n && !again; ++k)
                again = agents[chain[k]].v->id == th->yield_for;
            for (int k = first; !again && k < n; ++k)
                if ((give < 0 || agents[chain[k]].v->id > agents[give].v->id)
                    && gl_room_behind(s, agents, count, chain[k]) >= GL_BACK_MIN)
                    give = chain[k];
        }
        char tail[200];
        if (kind == 2) {
            ring = GL_RING_WAIT;
        } else if (kind == 1 && again) {
            if (i == high) {
                ring = GL_RING_ON;
                snprintf(tail, sizeof tail, "again %.1f s after seat %llu gave way for seat %llu: no second give-way, seat %llu "
                         "goes on as before (detour, street closing)", 0.001 * (double)(ms - th->yield_ms),
                         (unsigned long long)v->id, (unsigned long long)th->yield_for, (unsigned long long)v->id);
                gl_ring_log(s, t, agents, chain, n, first, tail);
            } else {
                if (!t->ring_since || t->ring_high != agents[high].v->id) {
                    t->ring_since = ms;
                    t->ring_high = agents[high].v->id;
                    t->ring_rev0 = th->rev;
                }
                ring = ms - t->ring_since >= GL_RING_REPEAT_WAIT_MS || th->rev != t->ring_rev0 ? GL_RING_ON : GL_RING_WAIT;
            }
        } else if (kind == 1 && give < 0) {
            ring = i == high ? GL_RING_ON : GL_RING_WAIT;
            if (i != high) {
                /* s36 P2: the others wait until the highest seat picks again or GL_RING_REPEAT_WAIT_MS passed */
                const gl_tank_t *hi = gl_find(agents[high].v->id, agents[high].v->life_id);
                if (!t->ring_since || t->ring_high != agents[high].v->id) {
                    t->ring_since = ms;
                    t->ring_high = agents[high].v->id;
                    t->ring_rev0 = hi ? hi->rev : 0;
                }
                if (ms - t->ring_since >= GL_RING_REPEAT_WAIT_MS || (hi && hi->rev != t->ring_rev0))
                    ring = GL_RING_ON;
            }
            if (i == high) {
                snprintf(tail, sizeof tail, "no member has %.0f m room behind: seat %llu goes on as before (detour, street "
                         "closing)", (double)GL_BACK_MIN, (unsigned long long)v->id);
                gl_ring_log(s, t, agents, chain, n, first, tail);
            }
        } else if (kind == 1) {
            ring = i == give ? GL_RING_GIVE : GL_RING_WAIT;
        }
        if (!(kind == 1 && (again || give < 0)))
            t->ring_since = 0;
        if (ring == GL_RING_GIVE) {
            /* the tank it blocked: the member before it in the ring (chain[k] stands for chain[k + 1]) */
            const int blocked_by_me = chain[n - 1];
            const float room = gl_room_behind(s, agents, count, i);
            t->av = GL_AV_BACK;
            t->back_kind = GL_BACK_YIELD;
            t->back_from = a->p;
            t->av_until = ms + 2u * GL_BACK_MS;
            t->yield_back_m = 2.0f * a->half_length + 2.0f;
            t->yield_for = agents[blocked_by_me].v->id;
            t->yield_ms = ms;
            t->yield_hold_until = 0;
            t->low_since = t->stuck_since = 0;
            t->sc_ms = 0;
            t->sc_last_ms = ms;
            snprintf(tail, sizeof tail, "seat %llu gives way to seat %llu: backs straight %.1f m (room behind %.1f m, more as it "
                     "goes), then holds while seat %llu is within %.0f m ahead, %.0f s at most", (unsigned long long)v->id,
                     (unsigned long long)t->yield_for, (double)t->yield_back_m, (double)room, (unsigned long long)t->yield_for,
                     (double)GM_YIELD_RANGE, 0.001 * (double)GM_YIELD_MS);
            gl_ring_log(s, t, agents, chain, n, first, tail);
            a->path_go = a->path_rev = 1;
            a->reversing = 1;
            a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
            a->path_d = room;
            a->path_cap = INFINITY;
            a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
            a->cmd_heading = v->heading;
            return;
        }
        if (ring == GL_RING_WAIT) {
            /* the clocks stand still: the blocked clock's start moves on with the time, the street clock adds nothing */
            if (t->low_since && t->sc_last_ms && ms > t->sc_last_ms)
                t->low_since += ms - t->sc_last_ms;
            t->sc_last_ms = ms;
            a->state = GM_WAIT_TRAFFIC;
            a->blocker = gl_standing_for(s, agents, i);
            if (a->blocker && a->blocker == t->veh_blocker)
                t->veh_blocker_ms = ms;     /* still standing for it (the blocked section that records it is skipped) */
            return;
        }
    }
    {
        const int bj = t->blocker ? gm_agent_index(agents, count, t->blocker) : -1;
        int ahead = 0;
        if (bj >= 0 && agents[bj].stationary && gm_dist(a->p, agents[bj].p) < GL_STREET_NEAR) {
            const gm_point_t bc = lp_centre(agents[bj].v, agents[bj].p.x, agents[bj].p.z, agents[bj].fwd);
            for (int k = t->proj; k < t->n && t->arc[k] - t->arc[t->proj] < GL_STREET_NEAR + 10.0f && !ahead; ++k)
                ahead = gm_dist((gm_point_t){t->x[k], t->z[k]}, bc) < agents[bj].radius + 2.0f;
            /* the pivot-blocked case ("stop distance inf"): the blocker beside the hull that its turn sweeps into */
            ahead = ahead || gm_dist(a->p, agents[bj].p) < a->radius + agents[bj].radius + 3.0f;
        }
        if (ahead && fabsf(a->speed) < 1.0f && fabsf(v->yaw_rate) < 0.1f) {    /* not getting past it, not turning */
            if (t->sc_blocker != t->blocker || gm_dist(a->p, t->sc_from) > 8.0f) {
                t->sc_blocker = t->blocker;
                t->sc_ms = 0;
                t->sc_from = a->p;
            } else if (t->sc_last_ms && ms > t->sc_last_ms) {
                t->sc_ms += ms - t->sc_last_ms < 500 ? ms - t->sc_last_ms : 500;
            }
            if (t->sc_ms >= GL_STREET_MS && t->need != GL_NEED_STREET
                && gl_street_close(s, agents, count, i, t, t->blocker, "6 s")) {
                t->sc_last_ms = ms;
                a->state = a->c.parked ? GM_PARKED : GM_WAIT_GEOMETRY;
                return;
            }
        } else if (!ahead && t->sc_blocker) {
            t->sc_blocker = 0;
            t->sc_ms = 0;
        }
        t->sc_last_ms = ms;
    }

    /* backing up: straight back 3 m (GL_BACK_DETOUR / GL_BACK_MARK: then the detour search; GL_BACK_TURN, s27 fix 1:
     * in 3 m steps until an arc of the turn is free, the room behind ends, or GL_BACK_TURN_MAX were backed) */
    if (t->av == GL_AV_BACK) {
        const float room = gl_room_behind(s, agents, count, i);
        const float moved = gm_dist(a->p, t->back_from);
        if (moved >= GL_BACK_M || ms >= t->av_until || room < 0.5f) {
            if (t->back_kind == GL_BACK_TURN) {
                t->back_total += moved;
                const float turn = gl_pivot_turn(a->lib_pivot_e);
                const int fwd = turn == 0.0f || gl_pivot_walls(lib, a, turn, 1);
                const int rev = !fwd && gl_pivot_walls(lib, a, turn, 2);
                t->pv_at = a->p;
                t->pv_stalled = 0;
                t->av = GL_AV_IDLE;
                if (fwd || rev) {
                    t->pv = fwd ? GL_PV_FWD : GL_PV_REV;
                    t->pv_rev_free = rev || gl_pivot_walls(lib, a, turn, 2);
                    gm_path_log(s, "avoid: seat %llu backed %.1f m: room to turn %.0f deg in %s gear", (unsigned long long)v->id,
                                (double)t->back_total, (double)(turn * 57.29578f), fwd ? "forward" : "reverse");
                } else if (t->back_total < GL_BACK_TURN_MAX && room >= GL_BACK_MIN) {
                    t->pv = GL_PV_BACK;
                    t->av = GL_AV_BACK;
                    t->back_from = a->p;
                    t->av_until = ms + GL_BACK_MS;
                    gm_path_log(s, "avoid: seat %llu backed %.1f m: turn %.0f deg still blocked by a wall in both gears, backs "
                                "%.0f m more", (unsigned long long)v->id, (double)t->back_total,
                                (double)(turn * 57.29578f), (double)GL_BACK_M);
                } else {
                    t->pv = GL_PV_HOLD;
                    t->pv_until = ms + GL_PV_HOLD_MS;
                    gm_path_log(s, "avoid: seat %llu backed %.1f m, turn %.0f deg still blocked by a wall in both gears and "
                                "%s: holds, decides again in %.0f s", (unsigned long long)v->id, (double)t->back_total,
                                (double)(turn * 57.29578f), room < GL_BACK_MIN ? "no room behind" : "backed the most",
                                0.001 * (double)GL_PV_HOLD_MS);
                }
                a->state = GM_WAIT_GEOMETRY;
                if (t->av == GL_AV_IDLE)
                    return;
            } else {
                gm_path_log(s, "avoid: seat %llu backed up %.1f m: decides again", (unsigned long long)v->id, (double)moved);
                t->backed_ms = ms;
                t->backed_at = t->back_from;
                t->av = GL_AV_PENDING;
                gl_request(t, GL_NEED_DETOUR, "after backing up", ms);
                a->state = GM_WAIT_GEOMETRY;
                return;
            }
        }
        a->path_go = a->path_rev = 1;
        a->reversing = 1;
        a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
        a->path_d = room;
        a->path_cap = INFINITY;
        a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
        a->cmd_heading = v->heading;
        return;
    }
    /* no way round: wait, then decide again */
    if (t->av == GL_AV_WAIT) {
        if (ms < t->av_until) {
            a->blocker = t->blocker;
            a->state = t->blocker ? GM_WAIT_TRAFFIC : GM_STATIC_BLOCKED;
            return;
        }
        t->av = GL_AV_IDLE;
        t->low_since = t->stuck_since = 0;
    }
    if (t->av == GL_AV_PENDING && !t->need)
        t->av = GL_AV_IDLE;

    /* nose to nose: the lower seat decides first, the higher waits */
    const int j = a->path_blocker ? gm_agent_index(agents, count, a->path_blocker) : -1;
    if (j >= 0 && agents[j].v->controlled && agents[j].mobile && agents[j].path_blocker == v->id
        && a->stop_vehicle < GL_BLOCKED_M && agents[j].stop_vehicle < GL_BLOCKED_M
        && gm_dot(a->fwd, agents[j].fwd) < -0.5f && v->id > agents[j].v->id) {
        if (t->waits_for != agents[j].v->id)
            gm_path_log(s, "avoid: seat %llu waits for seat %llu (nose to nose: the lower seat decides first)",
                        (unsigned long long)v->id, (unsigned long long)agents[j].v->id);
        t->waits_for = agents[j].v->id;
        t->low_since = 0;
        a->blocker = agents[j].v->id;
        a->state = GM_WAIT_TRAFFIC;
        a->path_go = 1;
        a->path_d = fminf(a->path_d, a->stop_vehicle);
        return;
    }
    t->waits_for = 0;

    /* s27 fix 1: a turn on the spot (the driver turns while |e| >= 8 deg once it started): room against the walls,
     * decided once per pose (forward arc, else reverse arc, else back straight in steps, else hold 4 s), and against
     * the hulls every snapshot (reverse arc tried first when the forward one meets a hull) */
    const float settle = GL_TURN_DEG * 0.017453293f;
    const int pivoting = !a->lib_turning
        && (a->lib_will_pivot || ((t->pv == GL_PV_FWD || t->pv == GL_PV_REV) && fabsf(a->lib_pivot_e) >= settle));
    int gear = a->lib_turning ? (a->lib_pivot_rev ? 2 : 1) : 1;
    if (!pivoting) {
        if (!a->lib_turning)
            t->pv = GL_PV_NONE;
        t->pv_still_since = 0;
        t->pv_stalled = 0;
    } else {
        const float turn = gl_pivot_turn(a->lib_pivot_e);
        /* a turn that was published and does not turn (no rotation, no motion) for GL_PV_STILL_MS: that arc is blocked by
         * something the map does not have */
        int stalled = 0;
        if (t->pv_published && (t->pv == GL_PV_FWD || t->pv == GL_PV_REV) && fabsf(v->yaw_rate) < 0.05f
            && fabsf(a->speed) < GL_STUCK_MPS) {
            if (!t->pv_still_since)
                t->pv_still_since = ms;
            if (ms - t->pv_still_since >= GL_PV_STILL_MS) {
                t->pv_stalled |= t->pv == GL_PV_FWD ? 1 : 2;
                stalled = 1;
            }
        } else {
            t->pv_still_since = 0;
        }
        if (t->pv == GL_PV_NONE || t->pv == GL_PV_BACK || gm_dist(a->p, t->pv_at) > GL_PV_MOVED || stalled
            || (t->pv == GL_PV_HOLD && ms >= t->pv_until)) {
            if (gm_dist(a->p, t->pv_at) > GL_PV_MOVED)
                t->pv_stalled = 0;
            const int fwd = !(t->pv_stalled & 1) && gl_pivot_walls(lib, a, turn, 1);
            const int rev = !(t->pv_stalled & 2) && gl_pivot_walls(lib, a, turn, 2);
            const float room = fwd || rev ? 0.0f : gl_room_behind(s, agents, count, i);
            t->pv_at = a->p;
            t->pv_rev_free = rev;
            t->pv_still_since = 0;
            const char *chosen;
            if (fwd) {
                t->pv = GL_PV_FWD;
                chosen = "turns in forward gear";
            } else if (rev) {
                t->pv = GL_PV_REV;
                chosen = "turns in reverse gear";
            } else if (room >= GL_BACK_MIN) {
                t->pv = GL_PV_BACK;
                chosen = "backs straight to get room";
            } else {
                t->pv = GL_PV_HOLD;
                t->pv_until = ms + GL_PV_HOLD_MS;
                chosen = "holds (no room behind), decides again in 4 s";
            }
            gm_path_log(s, "avoid: seat %llu turn on the spot %.0f deg: forward arc %s, reverse arc %s: %s",
                        (unsigned long long)v->id, (double)(turn * 57.29578f),
                        (t->pv_stalled & 1) ? "did not turn" : fwd ? "free" : "wall",
                        (t->pv_stalled & 2) ? "did not turn" : rev ? "free" : fwd ? "not checked" : "wall", chosen);
            if (fwd)
                t->pv_rev_free = gl_pivot_walls(lib, a, turn, 2) && !(t->pv_stalled & 2);
            if (t->pv == GL_PV_BACK) {
                t->av = GL_AV_BACK;
                t->back_kind = GL_BACK_TURN;
                t->back_total = 0.0f;
                t->back_from = a->p;
                t->av_until = ms + GL_BACK_MS;
                t->pv_published = 0;
                a->path_go = a->path_rev = 1;
                a->reversing = 1;
                a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
                a->path_d = room;
                a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
                return;
            }
        }
        if (t->pv == GL_PV_HOLD) {
            t->pv_published = 0;
            a->state = GM_STATIC_BLOCKED;
            a->geometry = GM_BLOCKED;
            return;
        }
        gear = t->pv == GL_PV_REV ? 2 : 1;
    }
    /* the hulls on the arc of the gear in use (a start turn: the library's gear) */
    if (pivoting || a->lib_turning) {
        const float turn = a->lib_turning ? a->lib_pivot_e : gl_pivot_turn(a->lib_pivot_e);
        if (turn != 0.0f && !gl_pivot_room(agents, count, i, turn, gear, &a->lib_pivot_blocker)) {
            uint64_t other = 0;
            if (pivoting && gear == 1 && t->pv_rev_free && gl_pivot_room(agents, count, i, turn, 2, &other)) {
                gm_path_log(s, "avoid: seat %llu turn on the spot %.0f deg: forward arc meets seat %llu, reverse arc free: "
                            "turns in reverse gear", (unsigned long long)v->id, (double)(turn * 57.29578f),
                            (unsigned long long)a->lib_pivot_blocker);
                t->pv = GL_PV_REV;
                gear = 2;
                a->lib_pivot_blocker = 0;
            } else {
                a->lib_pivot_blocked = 1;
            }
        }
        if (pivoting)
            a->lib_pivot_rev = gear == 2;
    }

    /* blocked: the stop distance from a standing hull under 2 m for 2 s, or a turn on the spot that a standing hull
     * leaves no room for; stuck (s27: only while driving, not turning on the spot): W held, no vehicle limits it, it
     * does not move */
    const int pj = a->lib_pivot_blocked ? gm_agent_index(agents, count, a->lib_pivot_blocker) : -1;
    const int pivot_standing = pj >= 0 && agents[pj].stationary;
    const int standing_block = (a->path_blocker && !a->path_blocker_moving && a->stop_vehicle < GL_BLOCKED_M
                                && a->stop_vehicle <= a->path_d + 1e-3f)
        || pivot_standing;
    if (standing_block) {   /* s36: what it stands for (the ring walk of the others reads it) */
        t->veh_blocker = pivot_standing ? a->lib_pivot_blocker : a->path_blocker;
        t->veh_blocker_ms = ms;
    }
    const int stuck_now = !a->lib_turning && !pivoting && !(a->stop_vehicle <= a->path_d + 1e-3f)
        && a->path_d > GM_PATH_NO_ROOM && fabsf(a->speed) < GL_STUCK_MPS && fabsf(v->yaw_rate) < 0.1f;
    if (t->av == GL_AV_IDLE && !t->need) {
        t->low_since = standing_block ? (t->low_since ? t->low_since : ms) : 0;
        t->stuck_since = stuck_now ? (t->stuck_since ? t->stuck_since : ms) : 0;
        const int blocked = t->low_since && ms - t->low_since >= GL_BLOCKED_MS;
        const int stuck = t->stuck_since && ms - t->stuck_since >= GL_STUCK_MS;
        if (blocked || stuck) {
            const int k = blocked ? j : -1;
            /* arrival short of an occupied end, and parking in the capture region, as s24 has it */
            if (k >= 0 && t->n && gm_dist((gm_point_t){t->x[t->n - 1], t->z[t->n - 1]}, agents[k].p)
                                      < agents[k].half_length + a->half_length + 2.0f && !a->region && t->finishing) {
                c->arrived = c->arrived_short = 1;
                a->state = GM_ARRIVED;
                gm_path_log(s, "avoid: seat %llu stops short of its target: seat %llu stands on it", (unsigned long long)v->id,
                            (unsigned long long)agents[k].v->id);
                return;
            }
            if (a->region && gm_dist(a->p, v->goal_region_center) < v->goal_region_radius - 3.0f) {
                c->parked = 1;
                c->slot = a->p;
                c->slot_valid = 1;
                c->slot_phase = 2;
                a->state = GM_PARKED;
                gm_path_log(s, "avoid: seat %llu parks where it stands in its capture region", (unsigned long long)v->id);
                return;
            }
            t->blocker = blocked ? (a->lib_pivot_blocked ? a->lib_pivot_blocker : a->path_blocker) : 0;
            if (stuck || a->lib_pivot_blocked)
                t->block_arc = t->arc[t->proj];
            t->low_since = t->stuck_since = 0;
            if (blocked && a->lib_pivot_blocked) {
                /* no room to turn for a standing hull: straight back 3 m first (s24's decision c), then the search */
                const int backed_here = t->backed_ms && ms < t->backed_ms + GL_BACK_AGAIN_MS
                    && gm_dist(a->p, t->backed_at) < GL_BACK_SAME;
                if (!backed_here && gl_room_behind(s, agents, count, i) >= GL_BACK_MIN) {
                    t->av = GL_AV_BACK;
                    t->back_kind = GL_BACK_DETOUR;
                    t->back_from = a->p;
                    t->av_until = ms + GL_BACK_MS;
                    gm_path_log(s, "avoid: seat %llu has no room to turn for standing seat %llu: backs up %.0f m",
                                (unsigned long long)v->id, (unsigned long long)t->blocker, (double)GL_BACK_M);
                    a->path_go = 1;
                    a->path_rev = 1;
                    a->reversing = 1;
                    a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
                    a->path_d = gl_room_behind(s, agents, count, i);
                    a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
                    return;
                }
            }
            if (stuck) {
                /* s27 fix 2: W held without displacement: something stands there that map.bin does not have. Marked for
                 * this life (stamped into every later search of this tank), back 3 m, then the detour. */
                const gm_point_t h = gl_hull_centre(a);
                const float ahead = a->half_length + 1.5f;
                if (t->marks_n == GL_MAX_MARKS) {
                    memmove(t->marks, t->marks + 1, sizeof t->marks[0] * (GL_MAX_MARKS - 1));
                    --t->marks_n;
                }
                t->marks[t->marks_n++] = (gr_box_t){h.x + a->fwd.x * ahead, h.z + a->fwd.z * ahead, a->fwd.x, a->fwd.z, 1.5f,
                                                    a->half_width + 0.3f};
                const float room = gl_room_behind(s, agents, count, i);
                gm_path_log(s, "avoid: seat %llu stuck against something the map does not have at (%.1f, %.1f): marked, %s, "
                            "goes round", (unsigned long long)v->id, (double)(h.x + a->fwd.x * ahead),
                            (double)(h.z + a->fwd.z * ahead), room >= GL_BACK_MIN ? "backs up" : "no room to back up");
                if (room >= GL_BACK_MIN) {
                    t->av = GL_AV_BACK;
                    t->back_kind = GL_BACK_MARK;
                    t->back_from = a->p;
                    t->av_until = ms + GL_BACK_MS;
                    a->path_go = a->path_rev = 1;
                    a->reversing = 1;
                    a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
                    a->path_d = room;
                    a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
                    return;
                }
            }
            t->av = GL_AV_PENDING;
            gl_request(t, GL_NEED_DETOUR, blocked ? "blocked" : "stuck", ms);
            if (blocked)
                gm_path_log(s, "avoid: seat %llu blocked by standing seat %llu (stop distance %.1f m for 2 s): asks for a "
                            "detour", (unsigned long long)v->id, (unsigned long long)t->blocker, (double)a->stop_vehicle);
        }
    }

    /* a turn on the spot into a hull: hold (no A / D into it) while the decision above waits for its 2 s */
    if (a->lib_pivot_blocked) {
        t->pv_published = 0;
        a->blocker = a->lib_pivot_blocker;
        a->state = GM_WAIT_TRAFFIC;
        return;
    }
    t->pv_published = pivoting;
    /* go: aim, stop distance */
    a->path_go = 1;
    a->cmd_speed = v->cruise_speed;
    if (a->lib_turning || pivoting) {
        a->state = GM_TURNING;
    } else if (a->stop_vehicle <= GM_PATH_NO_ROOM && a->stop_vehicle <= a->path_d + 1e-3f) {
        a->blocker = a->path_blocker;
        a->state = GM_WAIT_TRAFFIC;
    } else if (a->path_d <= GM_PATH_NO_ROOM && a->stop_turn <= a->path_d + 1e-3f) {
        a->state = GM_TURNING;
    }
}

/* s29: the published straight-back leg of an align that needs room (the same leg gl_decide drives). */
static void gl_align_back(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    a->path_go = a->path_rev = 1;
    a->reversing = 1;
    a->aim = gm_sub(a->p, gm_scale(a->fwd, 10.0f));
    a->path_d = gl_room_behind(s, agents, count, i);
    a->cmd_speed = -fminf(v->reverse_speed > 0.0f ? v->reverse_speed : GM_ESCAPE_SPEED, GM_ESCAPE_SPEED);
    a->cmd_heading = v->heading;
    a->state = GM_DRIVING;
}

/* s29: an align (the tank holds, its hull to be turned to v->align_heading: the reaction layer's facing) in library
 * mode, under s27's turn-on-the-spot rule instead of the planner's bare GM_TURNING: room against the walls decided
 * once per pose (forward-gear arc, else reverse-gear arc, else straight back in GL_BACK_M steps up to
 * GL_BACK_TURN_MAX while there is room behind, else hold and decide again after GL_PV_HOLD_MS), an arc that does not
 * turn for GL_PV_STILL_MS counted as blocked, and the hulls on the arc every snapshot (the reverse arc when the
 * forward one meets a hull, else wait). Published as a path-mode input: go, the aim point 10 m along the heading,
 * stop distance 0 (no driving, only the turn on the spot), the gear; within GL_TURN_DEG it holds. */
static void gl_align(const gm_snapshot_t *s, gm_agent_t *agents, int count, int i)
{
    gm_agent_t *a = &agents[i];
    const gm_vehicle_t *v = a->v;
    const uint64_t ms = s->ms;
    const gr_lib_t *lib = s->routes_lib;
    gl_tank_t *t = gl_get(v->id, v->life_id, ms);
    t->seen_ms = ms;
    if (!t->al_ms || ms < t->al_ms || ms - t->al_ms > 1000) {
        t->al_pv = GL_PV_NONE;          /* a new align: nothing decided yet */
        t->al_stalled = t->al_published = 0;
        t->al_still_since = 0;
        t->al_back_total = 0.0f;
    }
    t->al_ms = ms;
    a->lib_align = 1;
    a->path_go = a->path_rev = 0;
    a->reversing = 0;
    a->lp_moving = 0;
    a->cmd_speed = a->cmd_yaw = a->cmd_curvature = 0.0f;
    a->cmd_heading = v->align_heading;
    a->blocker = 0;
    a->geometry = GM_CLEAR;
    a->firm = 0;
    a->path_cap = INFINITY;
    a->path_d = 0.0f;
    a->state = GM_TURNING;
    const float e = gm_angle(v->align_heading - v->heading);
    a->lib_pivot_e = e;
    const float turn = gl_pivot_turn(e);
    if (turn == 0.0f) {
        /* within GL_TURN_DEG: done, hold */
        t->al_pv = GL_PV_NONE;
        t->al_published = 0;
        t->al_back_total = 0.0f;
        a->state = GM_PARKED;
        return;
    }
    if (t->al_pv == GL_PV_BACK) {
        const float room = gl_room_behind(s, agents, count, i), moved = gm_dist(a->p, t->al_back_from);
        if (moved < GL_BACK_M && ms < t->al_back_until && room >= 0.5f) {
            gl_align_back(s, agents, count, i);
            return;
        }
        t->al_back_total += moved;
        t->al_pv = GL_PV_NONE;          /* backed: decide again from here */
    }
    int stalled = 0;
    if (t->al_published && (t->al_pv == GL_PV_FWD || t->al_pv == GL_PV_REV) && fabsf(v->yaw_rate) < 0.05f
        && fabsf(a->speed) < GL_STUCK_MPS) {
        if (!t->al_still_since)
            t->al_still_since = ms;
        if (ms - t->al_still_since >= GL_PV_STILL_MS) {
            t->al_stalled |= t->al_pv == GL_PV_FWD ? 1 : 2;
            stalled = 1;
        }
    } else {
        t->al_still_since = 0;
    }
    if (t->al_pv == GL_PV_NONE || gm_dist(a->p, t->al_at) > GL_PV_MOVED || stalled
        || (t->al_pv == GL_PV_HOLD && ms >= t->al_until)) {
        if (gm_dist(a->p, t->al_at) > GL_PV_MOVED)
            t->al_stalled = 0;
        const int fwd = !(t->al_stalled & 1) && gl_pivot_walls(lib, a, turn, 1);
        const int rev = !(t->al_stalled & 2) && gl_pivot_walls(lib, a, turn, 2);
        const float room = fwd || rev ? 0.0f : gl_room_behind(s, agents, count, i);
        t->al_at = a->p;
        t->al_still_since = 0;
        const char *chosen;
        if (fwd) {
            t->al_pv = GL_PV_FWD;
            chosen = "turns in forward gear";
        } else if (rev) {
            t->al_pv = GL_PV_REV;
            chosen = "turns in reverse gear";
        } else if (room >= GL_BACK_MIN && t->al_back_total < GL_BACK_TURN_MAX) {
            t->al_pv = GL_PV_BACK;
            chosen = "backs straight to get room";
        } else {
            t->al_pv = GL_PV_HOLD;
            t->al_until = ms + GL_PV_HOLD_MS;
            chosen = room < GL_BACK_MIN ? "holds (no room behind), decides again in 4 s"
                                        : "holds (backed the most), decides again in 4 s";
        }
        gm_path_log(s, "avoid: seat %llu align %.0f deg (backed %.1f m): forward arc %s, reverse arc %s: %s",
                    (unsigned long long)v->id, (double)(turn * 57.29578f), (double)t->al_back_total,
                    (t->al_stalled & 1) ? "did not turn" : fwd ? "free" : "wall",
                    (t->al_stalled & 2) ? "did not turn" : rev ? "free" : fwd ? "not checked" : "wall", chosen);
        if (t->al_pv == GL_PV_BACK) {
            t->al_back_from = a->p;
            t->al_back_until = ms + GL_BACK_MS;
            t->al_published = 0;
            gl_align_back(s, agents, count, i);
            return;
        }
    }
    if (t->al_pv == GL_PV_HOLD) {
        t->al_published = 0;
        a->state = GM_STATIC_BLOCKED;
        a->geometry = GM_BLOCKED;
        return;
    }
    int gear = t->al_pv == GL_PV_REV ? 2 : 1;
    uint64_t blocker = 0;
    if (!gl_pivot_room(agents, count, i, turn, gear, &blocker)) {
        uint64_t other = 0;
        if (gear == 1 && !(t->al_stalled & 2) && gl_pivot_walls(lib, a, turn, 2)
            && gl_pivot_room(agents, count, i, turn, 2, &other)) {
            gm_path_log(s, "avoid: seat %llu align %.0f deg: forward arc meets seat %llu, reverse arc free: turns in reverse "
                        "gear", (unsigned long long)v->id, (double)(turn * 57.29578f), (unsigned long long)blocker);
            t->al_pv = GL_PV_REV;
            gear = 2;
        } else {
            t->al_published = 0;
            a->blocker = blocker;
            a->state = GM_WAIT_TRAFFIC;
            return;
        }
    }
    a->path_go = 1;
    a->lib_pivot_rev = gear == 2;
    a->aim = gm_add(a->p, gm_scale(gm_axis(v->align_heading), 10.0f));
    a->path_d = 0.0f;
    t->al_published = 1;
}
