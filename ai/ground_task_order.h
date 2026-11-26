#ifndef WT_GROUND_TASK_ORDER_H
#define WT_GROUND_TASK_ORDER_H

#include <stdint.h>

enum { GTO_ROUTINE, GTO_TIME_CRITICAL };
enum {
    GFM_STOP_FIRE = 1u << 0, GFM_HULL_ALIGN = 1u << 1, GFM_ADVANCE = 1u << 2,
    GFM_RETURN_COVER = 1u << 3, GFM_HOLD = 1u << 4
};
enum { GTR_ACCEPTED = 1, GTR_DEFERRED, GTR_REJECTED };
enum {
    GTR_NONE, GTR_STALE, GTR_INVALID, GTR_CLEARANCE, GTR_COMBAT, GTR_NO_ROUTE,
    GTR_GEOMETRY_UNKNOWN, GTR_TRAFFIC_BLOCKED, GTR_REJOIN_FAILED,
    GTR_OBJECTIVE_LOST, GTR_POINT_CHANGED, GTR_POST_INVALID, GTR_DISABLED,
    GTR_UNIT_LOST, GTR_IMMOBILIZED, GTR_EXPIRED, GTR_SOFT_EXIT, GTR_CAPTURE_COMPLETE,
    GTR_SUPERSEDED,
    GTR_POST_IDLE, GTR_POST_HIT     /* v13 choke / overwatch posts: no enemy in 60 s; hit by an unknown shooter */
};
enum {
    GNR_NEED_ROUTE, GNR_PLANNING, GNR_READY, GNR_FOLLOWING, GNR_WAIT_TRAFFIC,
    GNR_RECOVERING, GNR_REPLAN_PENDING, GNR_ARRIVED, GNR_BLOCKED, GNR_UNREACHABLE,
    GNR_CANCELLED
};
enum { GNR_REACH_UNKNOWN, GNR_REACH_LOCAL, GNR_REACH_CONFIRMED, GNR_REACH_NO_ROUTE };

typedef struct {
    uint64_t task_id, deadline_ms;
    uint32_t revision, allowed, short_stop_budget_ms;
    int urgency;
} ground_engagement_policy_t;

typedef struct {
    uint64_t task_id, life_id, epoch_id;
    uint64_t issued_at_ms, apply_at_ms, min_commit_until_ms, deadline_ms, objective_life;
    uint32_t revision;
    int seat, type, objective, lane, support_slot, post_option, reason;
    float x, z, radius, value_rank;
    ground_engagement_policy_t engagement;
    int route_variant;          /* s32 teamplan: 0 recommended corridor, 1 / 2 a flank corridor (gr_route_variant) */
    int goal_target1;           /* s35 lanes: 1 + the library target of the order's slot (0: the goal is mapped as before) */
} ground_task_order_t;

typedef struct {
    uint64_t life_id, task_id, sampled_ms, blocked_since_ms, retry_after_ms;
    uint32_t revision, route_id, generation;
    int seat, state, reason, reachability;
    float completed_m, actual_m, remaining_m;
} ground_task_report_t;

#endif
