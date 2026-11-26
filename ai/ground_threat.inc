/* ---- Fire-threat field (stage three, 2026-11-15) ------------------------------
 * Finding from the battle logs (Poland, 2026-11-14): a team charged a point down the open
 * main road into the other team's lines of fire. The route had been chosen
 * blind: position_value() counts only enemies already known, and before the
 * first contact none is.
 *
 * TH(cell) answers "how many enemy tanks could put a round into this G32 cell",
 * in expectation, from team knowledge only:
 *   K  known enemies: the EN grid of this army's last strat_update (confidence
 *      x class x range weight x fire line, hit-bearing wedges included);
 *   H  hidden enemies: where the ones we have not located may stand (the
 *      strategy layer's own prior: their spawn, the points that are not ours,
 *      nothing we watched in the last 10 s; its denser part, at least
 *      TH_SOURCE_MIN of its maximum), every such cell projected through
 *      the same fire-line matrix and range weight as K. A cell seen from much of
 *      the ground the enemy may hold scores high before anyone has shown up;
 *   Z  memory: own tanks hit or lost at the cell in the last minutes. The first
 *      tank through a kill zone pays for the lesson, the ones behind it do not.
 * During the opening the whole enemy roster counts as alive and hidden: the
 * engine spawns it over several seconds and the first routes are chosen before.
 *
 * Cost: H needs one visibility stamp per source cell (about 3 us each, 2500 of
 * them on Poland). It is computed in slices: every TH_SLICE_MS per army a slice
 * of at most TH_SLICE_US, the result published when a pass is complete. The
 * first pass of a battle gets TH_FIRST_US a slice so that it is there within
 * about a second. K is copied by strat_update, Z decays once a second.
 * Replayed offline by tools/run_analysis/threat_replay.py (same formulas).
 * Fault-guarded like the layers it reads: 3 faults switch it off (TH = 0, which
 * is the behaviour before this layer). */
#define TH_SLICE_MS 100
#define TH_SLICE_US 1500.0
#define TH_FIRST_US 4000.0
#define TH_SOURCE_MIN 0.2f      /* source cells: prior at least this share of its maximum (the rest is renormalised away) */
#define TH_OPENING_MS 90000     /* until then the enemy roster counts as alive and hidden */
#define TH_MEM_TAU_S 75.0f      /* a hit fades to a third in this time */
#define TH_MEM_HIT 0.5f
#define TH_MEM_LOSS 1.0f
#define TH_LOG_MS 60000

static struct {
    int n, off;
    float *k[3], *h[3], *z[3], *q[3];   /* per army: known, hidden (published), memory, source density of the pass */
    float *acc[3];                      /* padded accumulator of the running pass */
    float *load;                        /* cells of the team-mates' routes, for one approach search at a time */
    int load_on;
    float scale[3], floor[3];
    int cursor[3], ready[3], running[3], sources[3], passes[3], roster[3];
    uint64_t next_ms[3], decay_ms[3], pass_ms[3], pass_took_ms[3], battle_ms, log_ms;
    double us_sum, us_max;
    int slices;
    int was_alive[64];                  /* per slot: seen alive in the life counted in `life` */
    uint64_t life[64];
    float last_x[64], last_z[64];
    volatile long faults;
} th;

static int threat_on(void)
{
    return th.n > 0 && !th.off && strat.state == 1;
}

/* The field is usable for this army: K alone before the first hidden pass. */
static int threat_ready(int army)
{
    return threat_on() && army >= 1 && army <= 2 && th.ready[army];
}

static float threat_cell(int army, int c)
{
    if (c < 0 || c >= th.n || army < 1 || army > 2 || !threat_on())
        return 0.0f;
    return th.k[army][c] + th.h[army][c] + th.z[army][c];
}

static float threat_at(int army, float x, float z)
{
    return threat_on() ? threat_cell(army, strat_cellof(x, z)) : 0.0f;
}

/* strat_update leaves its EN grid in a buffer both armies share. */
static void threat_keep_known(int army)
{
    if (!threat_on() || army < 1 || army > 2)
        return;
    float *k = th.k[army];
    for (int i = 0; i < th.n; ++i)
        k[i] = strat.en[strat_pad(i)];
}

static int approach_on(void)
{
    return cfg.approach && threat_on();
}

/* An own tank was hit (weight TH_MEM_HIT) or lost (TH_MEM_LOSS) at (x, z). */
static void threat_note(int army, float x, float z, float weight)
{
    if (!threat_on() || army < 1 || army > 2)
        return;
    int c = strat_cellof(x, z);
    if (c < 0)
        return;
    int r0 = c / strat.nx, c0 = c % strat.nx;
    for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
            int r = r0 + dr, cc = c0 + dc;
            if (r < 0 || r >= strat.nz || cc < 0 || cc >= strat.nx)
                continue;
            th.z[army][r * strat.nx + cc] += dr || dc ? 0.5f * weight : weight;
        }
}

static void threat_hit(int army, float x, float z)
{
    threat_note(army, x, z, TH_MEM_HIT);
}

static int threat_alloc(void *host, uint64_t ms)
{
    const int n = strat.n, padded = strat.PX * strat.PZ;
    float *cells = (float *)calloc((size_t)n * 9, sizeof(float));
    float *pads = (float *)calloc((size_t)padded * 2, sizeof(float));
    if (!cells || !pads) {
        th.off = 1;
        event(host, "threat: off (out of memory)");
        return 0;
    }
    for (int a = 1; a <= 2; ++a) {
        th.k[a] = cells + (size_t)(a - 1) * 4 * n;
        th.h[a] = th.k[a] + n;
        th.z[a] = th.k[a] + 2 * n;
        th.q[a] = th.k[a] + 3 * n;
        th.acc[a] = pads + (size_t)(a - 1) * padded;
        th.roster[a] = 0;
    }
    th.load = cells + (size_t)8 * n;
    for (int s = 1; s < MAX_SEATS; ++s)
        if (authored_spawn_seat_army[s] >= 1 && authored_spawn_seat_army[s] <= 2)
            ++th.roster[authored_spawn_seat_army[s]];
    th.n = n;
    th.battle_ms = ms;      /* the caller's clock: a later reading could lie ahead of the ms passed down */
    event(host, "threat: field on, %d cells, rosters %d / %d", n, th.roster[1], th.roster[2]);
    return 1;
}

/* Source density of a new pass: the strat_update prior of where hidden enemies
 * may be, scaled so that it sums to the number of enemies not located. */
static void threat_begin_pass(void *host, int army, uint64_t ms)
{
    const strat_meta_t *m = strat.m;
    const int enemy = 3 - army;
    const uint32_t now_s = (uint32_t)((ms - t0) / 1000) + 1;
    const uint32_t *seen = strat.seen[army];
    int owner[8], alive = 0;
    for (uint32_t k = 0; k < m->ncap && k < 8; ++k)
        owner[k] = zone_owner(host, m->cap[k][0], m->cap[k][1]);
    for (int s = 0; s < 64; ++s)
        alive += slot_alive[s] && slot_army[s] == enemy;
    if (ms < th.battle_ms + TH_OPENING_MS && alive < th.roster[enemy])
        alive = th.roster[enemy];
    int hidden = alive - sview[army].ne;
    hidden = hidden < 0 ? 0 : hidden;
    float *q = th.q[army], sum = 0.0f, top = 0.0f;
    for (int i = 0; i < th.n; ++i) {
        float v = 0.0f;
        if ((strat.cell[i].flags & 1) && (!seen[i] || now_s - seen[i] > STRAT_SEEN_S)) {
            v = strat.prior[army][i];
            for (uint32_t k = 0; k < m->ncap && k < 6; ++k)
                if (owner[k] != army)
                    v += 0.8f * strat.capg[k][i];
        }
        q[i] = v;
        top = v > top ? v : top;
    }
    /* Only the denser part of the prior is projected (a stamp per cell is the
     * cost), carrying all of the hidden tanks: the count stays right when few
     * are left, where an absolute cut would drop every cell. */
    th.floor[army] = TH_SOURCE_MIN * top;
    for (int i = 0; i < th.n; ++i)
        if (q[i] >= th.floor[army])
            sum += q[i];
    th.scale[army] = sum > 1e-9f ? (float)hidden / sum : 0.0f;
    th.cursor[army] = 0;
    th.sources[army] = 0;
    th.running[army] = 1;
    th.pass_ms[army] = ms;
    memset(th.acc[army], 0, (size_t)strat.PX * strat.PZ * sizeof(float));
}

static void threat_slice(void *host, int army, uint64_t ms)
{
    if (!th.running[army])
        threat_begin_pass(host, army, ms);
    const double start = strat_now_us(), budget = th.ready[army] ? TH_SLICE_US : TH_FIRST_US;
    const float *q = th.q[army], scale = th.scale[army], floor = th.floor[army];
    float *acc = th.acc[army];
    int i = th.cursor[army], stamps = 0;
    for (; i < th.n; ++i) {
        const float mass = q[i] * scale;
        if (q[i] < floor || !(mass > 0.0f))
            continue;
        strat_stamp(acc, i, mass, 0);
        ++th.sources[army];
        if (!(++stamps & 31) && strat_now_us() - start > budget) {
            ++i;
            break;
        }
    }
    th.cursor[army] = i;
    if (i >= th.n) {
        float *h = th.h[army];
        for (int c = 0; c < th.n; ++c)
            h[c] = acc[strat_pad(c)];
        th.running[army] = 0;
        th.ready[army] = 1;
        ++th.passes[army];
        th.pass_took_ms[army] = ms - th.pass_ms[army];
    }
    const double us = strat_now_us() - start;
    th.us_sum += us;
    th.us_max = us > th.us_max ? us : th.us_max;
    ++th.slices;
}

/* Own tanks lost since the last look leave a mark where they stood. A life
 * ends when scan_units counts a new one for the slot: at the death, or (a wreck
 * keeps the engine's alive flags) when the seat respawns. Either way the mark
 * goes where that life was last seen alive, not to the new spawn. */
static void threat_losses(int army)
{
    for (int s = 0; s < 64; ++s) {
        if (slot_army[s] != army)
            continue;
        if (slot_life[s] != th.life[s]) {
            if (th.was_alive[s])
                threat_note(army, th.last_x[s], th.last_z[s], TH_MEM_LOSS);
            th.life[s] = slot_life[s];
            th.was_alive[s] = 0;
        }
        if (slot_alive[s] && slot_unit[s]) {
            const float *p = gh_unit_pos(slot_unit[s]);
            if (isfinite(p[0]) && isfinite(p[2])) {
                th.last_x[s] = p[0];
                th.last_z[s] = p[2];
                th.was_alive[s] = 1;
            }
        }
    }
}

/* s49: the guarded block of threat_tick, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; int army; uint64_t ms; } threat_run_ctx_t;
static void threat_run(void *ctx)
{
    threat_run_ctx_t *const context = ctx;
    void *const host = context->host;
    const int army = context->army;
    const uint64_t ms = context->ms;
    {
        if (ms >= th.decay_ms[army]) {
            const float keep = th.decay_ms[army] ? expf(-(float)(ms + 1000 - th.decay_ms[army]) / (1000.0f * TH_MEM_TAU_S)) : 1.0f;
            float *z = th.z[army];
            for (int i = 0; i < th.n; ++i)
                z[i] = z[i] * keep < 0.01f ? 0.0f : z[i] * keep;
            th.decay_ms[army] = ms + 1000;
            threat_losses(army);
        }
        threat_slice(host, army, ms);
    }
}

/* Called from lanes_tick under `lock`, any number of times a tick. */
static void threat_tick(void *host, int army, uint64_t ms)
{
    if (army < 1 || army > 2 || th.off || strat.state != 1 || !sview[army].ok)
        return;
    if (!th.n && !threat_alloc(host, ms))
        return;
    if (ms < th.next_ms[army])
        return;
    th.next_ms[army] = ms + TH_SLICE_MS;
    threat_run_ctx_t run = {host, army, ms};
    if (gh_guard(threat_run, &run)) {
        th.running[army] = 0;
        if (gh_atomic_inc(&th.faults) >= 3) {
            th.off = 1;
            event(host, "threat: off (faults)");
        }
    }
    if (ms >= th.log_ms) {
        if (th.log_ms)
            event(host, "threat: last 60 s %d slices, %.0f us avg, %.0f us max | passes %d / %d, last took %llu / %llu ms "
                  "from %d / %d source cells | faults %ld", th.slices, th.slices ? th.us_sum / th.slices : 0.0, th.us_max,
                  th.passes[1], th.passes[2], (unsigned long long)th.pass_took_ms[1], (unsigned long long)th.pass_took_ms[2],
                  th.sources[1], th.sources[2], th.faults);
        th.log_ms = ms + TH_LOG_MS;
        th.slices = 0;
        th.us_sum = th.us_max = 0.0;
        th.passes[1] = th.passes[2] = 0;
    }
}

/* ---- Routes under the field ----------------------------------------------------
 * A route costs its driving time plus AP_ALPHA seconds for every second spent
 * where one enemy can fire: with 3, a detour pays as long as it is no more than
 * three times longer than the stretch under fire it avoids. Samples every
 * AP_SAMPLE_M (half a G32 cell), 8 m/s like the commander's A*. */
#define AP_ALPHA 3.0f
#define AP_LOAD_CELL_S 3.0f     /* approach searches: a cell on a team-mate's route costs this much more per mate */
#define AP_LOAD_S 12.0f         /* choice among tactical routes: a team-mate on the first half of one */
#define AP_PICK_TAU_S 25.0f     /* ... and the old pick weights w as -tau ln w seconds */
#define AP_SAMPLE_M 16.0f
#define AP_SPEED 8.0f
#define AP_TH_HI 0.6f           /* a stretch is exposed from here ... */
#define AP_TH_LO 0.35f          /* ... and covered again below this */
typedef struct { float length, seconds, threat, peak, exposed; } threat_measure_t;

static void threat_measure_leg(int army, threat_measure_t *sum, float x0, float z0, float x1, float z1)
{
    const float d = dist2d(x0, z0, x1, z1);
    if (!(d > 0.01f) || !isfinite(d))
        return;
    const int steps = (int)ceilf(d / AP_SAMPLE_M);
    const float ds = d / (float)steps;
    for (int k = 0; k < steps; ++k) {
        const float f = ((float)k + 0.5f) / (float)steps;
        const float t = threat_at(army, x0 + (x1 - x0) * f, z0 + (z1 - z0) * f);
        sum->threat += ds / AP_SPEED * t;
        sum->peak = fmaxf(sum->peak, t);
        if (t >= AP_TH_HI)
            sum->exposed += ds;
    }
    sum->length += d;
    sum->seconds += d / AP_SPEED;
}

/* From (x, z) over waypoints first .. count - 1 of r. */
static threat_measure_t threat_measure_route(int army, float x, float z, const troute_t *r, int first, int count)
{
    threat_measure_t sum = {0};
    for (int k = first; k < count && k < MAX_TWP; ++k) {
        threat_measure_leg(army, &sum, x, z, r->x[k], r->z[k]);
        x = r->x[k];
        z = r->z[k];
    }
    return sum;
}

static float threat_cost(const threat_measure_t *m)
{
    return m->seconds + AP_ALPHA * m->threat;
}
