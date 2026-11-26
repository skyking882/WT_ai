/* s32 team plan (ground_teamplan.inc): the mission's plan pack (bot_trees/routes/<mission>.plan.json, WTPLAN001), read by
 * ground_brain.py (plan_lines) and handed over as config lines, in this order:
 *   tp_spawn=army|ID|pose;pose;...                       a spawn area and its library poses
 *   tp_zone=ID|target|x|z|r[|capacity|front1|front2]     a capture zone: its id, library target and circle; s33: how
 *                                                        many tanks the circle takes (0 / missing: no limit) and the
 *                                                        front corridors per army (log only)
 *   tp_area=army|ID|cls|target:x:z:face_deg;...          a watch area (cls M / F / S) and its slots (s33: only areas
 *                                                        a plan's watch entries or the support table name)
 *   tp_support=army|ZONE|AREA;AREA;...                   s33: the zone's fire-support areas of that army, preferred first
 *   tp_plan=army|ID|kind|weight|zones|watch|flank|why    one plan of an army:
 *       zones  ZONE:role:share,...    role main / contest / delay / skip, share 0..1 of the army's tanks
 *       watch  AREA<SPAWN:count,...   (empty: none)
 *       flank  ZONE,ZONE,...          the zones whose plan says flank true (empty: none)
 *       why    free text to the end of the line
 * A plan line naming an unknown zone / area / spawn drops that item (counted in the load line). */
#define TP_MAX_SPAWNS 16
#define TP_MAX_POSES 64             /* s43: 16 -> 64 (packs up to 52 poses per spawn area; tp_spawn line < 600 chars) */
#define TP_MAX_PLANS 24
#define TP_MAX_WATCH 8              /* s33: 6 -> 8 (an F entry per spawn area, plus M entries) */
enum { TPR_MAIN, TPR_CONTEST, TPR_DELAY, TPR_SKIP, TPR_N };
static const char *const tp_role_name[TPR_N] = {"main", "contest", "delay", "skip"};
typedef struct { int army, n; char id[16]; int pose[TP_MAX_POSES]; } tp_spawn_t;
typedef struct {
    int army, nz, nw;
    char id[32], kind[16], why[160];
    float weight;
    struct { int zone, role; float share; } z[TP_MAX_ZONES];
    struct { int area, from, count; } w[TP_MAX_WATCH];
    int flank[TP_MAX_ZONES];    /* by zone index */
} tp_plan_t;
tp_lane_t tp_lanes[TP_MAX_LANES];
int ntp_lanes, tp_format = 1;
static struct { int ok, sectors; float ux, uz, ox, oz, width; } tp_axes[3];
static tp_spawn_t tp_spawns[TP_MAX_SPAWNS];
static tp_zone_t tp_zones[TP_MAX_ZONES];
tp_area_t tp_areas[TP_MAX_AREAS];
static tp_plan_t tp_plans[TP_MAX_PLANS];
static int ntp_spawns, ntp_zones, ntp_areas, ntp_plans, tp_dropped;

/* Next '|'-separated field of *p (NUL-terminated in place); "" at the end. */
static char *tp_field(char **p)
{
    char *s = *p, *bar = s ? strchr(s, '|') : NULL;
    if (!s)
        return "";
    if (bar) {
        *bar = 0;
        *p = bar + 1;
    } else {
        *p = NULL;
    }
    return s;
}

static void tp_copy(char *dst, size_t n, const char *src)
{
    strncpy(dst, src, n - 1);
    dst[n - 1] = 0;
}

static int tp_zone_index(const char *id)
{
    for (int i = 0; i < ntp_zones; ++i)
        if (!strcmp(tp_zones[i].id, id))
            return i;
    return -1;
}

static int tp_area_index(const char *id)
{
    for (int i = 0; i < ntp_areas; ++i)
        if (!strcmp(tp_areas[i].id, id))
            return i;
    return -1;
}

static int tp_spawn_index(const char *id)
{
    for (int i = 0; i < ntp_spawns; ++i)
        if (!strcmp(tp_spawns[i].id, id))
            return i;
    return -1;
}

/* One tp_* config line (k without the "tp_" prefix, v the value; v is modified). */
void tp_parse(const char *k, char *v)
{
    char *p = v;
    if (!strcmp(k, "format")) {
        tp_format = atoi(v) == 2 ? 2 : 1;
        return;
    }
    if (!strcmp(k, "axis")) {
        const int army = atoi(tp_field(&p));
        if (army < 1 || army > 2)
            return;
        tp_axes[army].ux = (float)atof(tp_field(&p));
        tp_axes[army].uz = (float)atof(tp_field(&p));
        tp_axes[army].ox = (float)atof(tp_field(&p));
        tp_axes[army].oz = (float)atof(tp_field(&p));
        tp_axes[army].width = (float)atof(tp_field(&p));
        tp_axes[army].sectors = atoi(tp_field(&p));
        tp_axes[army].ok = tp_axes[army].sectors > 0 && tp_axes[army].sectors <= 16;
        return;
    }
    if (!strcmp(k, "lane")) {
        if (ntp_lanes >= TP_MAX_LANES) {
            ++tp_dropped;
            return;
        }
        tp_lane_t *l = &tp_lanes[ntp_lanes];
        memset(l, 0, sizeof *l);
        l->army = atoi(tp_field(&p));
        tp_copy(l->id, sizeof l->id, tp_field(&p));
        l->from = tp_spawn_index(tp_field(&p));
        const char *serves = tp_field(&p), *end = tp_field(&p);
        l->serves = strcmp(serves, "-") ? tp_zone_index(serves) : -1;
        l->end = end[0] == 'c' ? TLE_CIRCLE : end[0] == 's' ? TLE_SUPPORT : TLE_GUARD;
        l->sector = atoi(tp_field(&p));
        l->capacity = atoi(tp_field(&p));
        l->time_s = (float)atof(tp_field(&p));
        for (char *q = tp_field(&p); *q && l->nslot < TP_LANE_SLOTS;) {
            tp_slot_t *sl = &l->slot[l->nslot];
            if (sscanf(q, "%d:%f:%f:%f", &sl->target, &sl->x, &sl->z, &sl->face) == 4 && isfinite(sl->x) && isfinite(sl->z))
                ++l->nslot;
            char *semi = strchr(q, ';');
            if (!semi)
                break;
            q = semi + 1;
        }
        for (char *q = tp_field(&p); *q && l->nbound < TP_LANE_BOUNDS;) {
            tp_bound_t *b = &l->bound[l->nbound];
            if (sscanf(q, "%f:%f:%f:%f", &b->x, &b->z, &b->face, &b->at) == 4 && isfinite(b->x) && isfinite(b->z))
                ++l->nbound;
            char *semi = strchr(q, ';');
            if (!semi)
                break;
            q = semi + 1;
        }
        for (char *q = tp_field(&p); *q && l->nnb < TP_LANE_NB;) {
            char *semi = strchr(q, ';');
            if (semi)
                *semi = 0;
            tp_copy(l->nb_id[l->nnb], sizeof l->nb_id[0], q);
            l->nb[l->nnb++] = -1;
            if (!semi)
                break;
            q = semi + 1;
        }
        for (char *q = tp_field(&p); *q && l->nop < TP_LANE_OP;) {     /* s36: opposing lanes, optional field */
            char *semi = strchr(q, ';');
            if (semi)
                *semi = 0;
            tp_copy(l->op_id[l->nop], sizeof l->op_id[0], q);
            l->op[l->nop++] = -1;
            if (!semi)
                break;
            q = semi + 1;
        }
        l->value = (float)atof(tp_field(&p));      /* s37: optional last field ("" at the end: 0) */
        if (!isfinite(l->value) || l->value < 0.0f)
            l->value = 0.0f;
        if (l->capacity < 1)
            l->capacity = 1;
        if ((l->army == 1 || l->army == 2) && l->id[0] && l->from >= 0 && l->nslot && (l->end == TLE_GUARD || l->serves >= 0))
            ++ntp_lanes;
        else
            ++tp_dropped;
        return;
    }
    if (!strcmp(k, "spawn") && ntp_spawns < TP_MAX_SPAWNS) {
        tp_spawn_t *s = &tp_spawns[ntp_spawns];
        memset(s, 0, sizeof *s);
        s->army = atoi(tp_field(&p));
        tp_copy(s->id, sizeof s->id, tp_field(&p));
        for (char *q = tp_field(&p); *q && s->n < TP_MAX_POSES;) {
            s->pose[s->n++] = atoi(q);
            char *semi = strchr(q, ';');
            if (!semi)
                break;
            q = semi + 1;
        }
        if ((s->army == 1 || s->army == 2) && s->id[0])
            ++ntp_spawns;
    } else if (!strcmp(k, "zone") && ntp_zones < TP_MAX_ZONES) {
        tp_zone_t *z = &tp_zones[ntp_zones];
        memset(z, 0, sizeof *z);
        tp_copy(z->id, sizeof z->id, tp_field(&p));
        z->target = atoi(tp_field(&p));
        z->x = (float)atof(tp_field(&p));
        z->z = (float)atof(tp_field(&p));
        z->r = (float)atof(tp_field(&p));
        z->capacity = atoi(tp_field(&p));   /* s33 (missing: 0, no limit) */
        z->front[1] = atoi(tp_field(&p));
        z->front[2] = atoi(tp_field(&p));
        if (z->capacity < 0)
            z->capacity = 0;
        if (z->id[0] && isfinite(z->x) && isfinite(z->z))
            ++ntp_zones;
    } else if (!strcmp(k, "support")) {
        /* s33: after the zones and areas it names */
        const int army = atoi(tp_field(&p));
        const int zi = tp_zone_index(tp_field(&p));
        if (zi < 0 || army < 1 || army > 2) {
            ++tp_dropped;
            return;
        }
        for (char *q = tp_field(&p); *q;) {
            char *semi = strchr(q, ';');
            if (semi)
                *semi = 0;
            const int ai = tp_area_index(q);
            if (ai >= 0 && tp_zones[zi].nsup[army] < TP_MAX_SUPPORT)
                tp_zones[zi].sup[army][tp_zones[zi].nsup[army]++] = ai;
            else
                ++tp_dropped;
            if (!semi)
                break;
            q = semi + 1;
        }
    } else if (!strcmp(k, "area") && ntp_areas < TP_MAX_AREAS) {
        tp_area_t *a = &tp_areas[ntp_areas];
        memset(a, 0, sizeof *a);
        a->army = atoi(tp_field(&p));
        tp_copy(a->id, sizeof a->id, tp_field(&p));
        a->cls = tp_field(&p)[0];
        for (char *q = tp_field(&p); *q && a->nslot < TP_MAX_SLOTS;) {
            tp_slot_t *sl = &a->slot[a->nslot];
            if (sscanf(q, "%d:%f:%f:%f", &sl->target, &sl->x, &sl->z, &sl->face) == 4 && isfinite(sl->x) && isfinite(sl->z))
                ++a->nslot;
            char *semi = strchr(q, ';');
            if (!semi)
                break;
            q = semi + 1;
        }
        if ((a->army == 1 || a->army == 2) && a->id[0] && a->nslot)
            ++ntp_areas;
    } else if (!strcmp(k, "plan") && ntp_plans < TP_MAX_PLANS) {
        tp_plan_t *pl = &tp_plans[ntp_plans];
        memset(pl, 0, sizeof *pl);
        pl->army = atoi(tp_field(&p));
        tp_copy(pl->id, sizeof pl->id, tp_field(&p));
        tp_copy(pl->kind, sizeof pl->kind, tp_field(&p));
        pl->weight = (float)atof(tp_field(&p));
        char *zones = tp_field(&p), *watch = tp_field(&p), *flank = tp_field(&p);
        tp_copy(pl->why, sizeof pl->why, p ? p : "");
        for (char *q = zones; q && *q && pl->nz < TP_MAX_ZONES;) {
            char *comma = strchr(q, ','), zid[8] = "", role[16] = "";
            float share = 0.0f;
            if (comma)
                *comma = 0;
            if (sscanf(q, "%7[^:]:%15[^:]:%f", zid, role, &share) == 3) {
                const int zi = tp_zone_index(zid);
                int r = -1;
                for (int j = 0; j < TPR_N; ++j)
                    if (!strcmp(role, tp_role_name[j]))
                        r = j;
                if (zi >= 0 && r >= 0 && isfinite(share)) {
                    pl->z[pl->nz].zone = zi;
                    pl->z[pl->nz].role = r;
                    pl->z[pl->nz++].share = share < 0.0f ? 0.0f : share > 1.0f ? 1.0f : share;
                } else {
                    ++tp_dropped;
                }
            }
            q = comma ? comma + 1 : NULL;
        }
        for (char *q = watch; q && *q && pl->nw < TP_MAX_WATCH;) {
            char *comma = strchr(q, ','), aid[16] = "", sid[16] = "";
            int count = 0;
            if (comma)
                *comma = 0;
            if (sscanf(q, "%15[^<]<%15[^:]:%d", aid, sid, &count) == 3) {
                const int ai = tp_area_index(aid), si = tp_spawn_index(sid);
                if (ai >= 0 && si >= 0 && count > 0) {
                    pl->w[pl->nw].area = ai;
                    pl->w[pl->nw].from = si;
                    pl->w[pl->nw++].count = count > 2 ? 2 : count;
                } else {
                    ++tp_dropped;
                }
            }
            q = comma ? comma + 1 : NULL;
        }
        for (char *q = flank; q && *q;) {
            char *comma = strchr(q, ',');
            if (comma)
                *comma = 0;
            const int zi = tp_zone_index(q);
            if (zi >= 0)
                pl->flank[zi] = 1;
            else
                ++tp_dropped;
            q = comma ? comma + 1 : NULL;
        }
        if ((pl->army == 1 || pl->army == 2) && pl->id[0] && pl->weight > 0.0f && pl->nz)
            ++ntp_plans;
        else
            ++tp_dropped;
    }
}
