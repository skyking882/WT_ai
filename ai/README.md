# Ground-vehicle bot AI (public part)

C source of a decision layer for bot tanks in a team capture-point battle. Each bot is driven like a player
drives: the AI decides where it goes, which way, how it gives way to others, and which keys (W / S / A / D) it
presses. The layers, one authority per decision:

- **command**: which tank goes to which capture point / support / watch position, over which lane; point quotas,
  area caps, lanes ordered by how many tanks already use them (no bunching), what happens after a point is taken,
  rear-area alerts;
- **route**: which curve to follow from where the tank stands: a pre-generated route, a join onto one, a road
  network search, a short grid search;
- **avoidance**: the stop distance along the heading (other vehicles, turns, route end); wait, back off, detour;
- **driver**: W / S / A / D from the route aim point and the stop distance;
- **tactics**: sensing, rear alert, wait / peek, target choice with inertia;
- **fire control**: turret pre-aim, stabiliser emulation, aim-point choice, fire gate.

Routes, the road network and lane plans are generated offline per map ("map packs"); in a battle the AI only
chooses among them. The design is described in `docs/ground_ai_algorithm_20261120.md` (Chinese); a draft of a
message interface around the same core is in `docs/core_interface_draft_20261120.md` (Chinese).

## What is not here

The host layer: the code that reads the vehicles, capture points and sight information from the environment and
hands the keys back. The AI reaches its environment only through the functions and constants declared in
`ground_host.h` (about 235 `gh_*` functions: OS services, single facts, a few call sequences). An integrator
implements every one of them. `host_stub/ground_host_stub.c` lists them all with a one-sentence description and a
trivial body; it is what the library is link-tested against.

The entry points the host calls are declared at the end of `ground_host.h` (`ground_plan`, `ground_hit`,
`ground_visible`, `ground_path`, `ground_los_interval`, `ground_control_begin`, `ground_control_after`,
`ground_observe`, `ground_controls`, `ground_aim`, `ground_fire`).

The map packs are not here either: they come from the separate generator directory.

## Files

| File | Layer | What it is |
|---|---|---|
| `ground_host.h` | interface | everything the AI needs from its host, and the AI's entry points |
| `ground_brain.c` | all | main translation unit: config, per-tick loop, team knowledge, lanes executor; includes the `.inc` files below |
| `ground_command.c` / `.h` | command | the command layer as its own unit; shared types and tables |
| `ground_command_core.inc` | command | team commander: roles, objectives, orders, posts |
| `ground_command_loop.inc` | command | the commander's tick, post leave rules |
| `ground_command_team.inc` | command | team planner: offensive share, rear class |
| `ground_task_command.inc` | command | task orders, receipts, navigation evidence |
| `ground_task_order.h` | command | the task-order record |
| `ground_teamplan.inc` | command | team plan: one plan per army from the plan pack, zone quotas, area caps |
| `ground_teamplan_data.inc` | command | parsing of the plan pack's `tp_*` config lines |
| `ground_threat.inc` | command | fire-threat field over the map |
| `ground_approach.inc` | command | covered approach to an objective using the threat field |
| `ground_routes.c` / `.h` | route | route library loader and grid searches on its clearance grid |
| `ground_network.c` / `.h` | route | road network loader, spatial index and search |
| `ground_nav.c` / `.h` | route | lattice route search validated by the swept hull footprint |
| `ground_navigation.h` / `.inc` | route | goal arbitration and route progress |
| `ground_motion.c` / `.h` | avoidance | fleet motion planner: lanes, yielding, stop distance, spawn exit queue |
| `ground_motion_library.inc` | route / avoidance | library mode: follows pre-generated routes, turns on the spot, street closures |
| `ground_motion_control.c` / `.h` | driver | speed and yaw-rate control helpers |
| `ground_vehicle_control.inc` | driver | publishes the fleet plan; owns the final control bytes |
| `ground_drive.inc` | driver | the W / S / A / D driver |
| `ground_control_graft.inc` | driver | the single write point of the keys, around the host's input layer; probe CSV |
| `ground_geometry.inc` | route / avoidance | obstacle samples and collision evidence of the map |
| `ground_terrain.h` | route | water and slope from a decoded height field |
| `ground_lidar.c` / `.h`, `ground_lidar_adapter.inc` | avoidance | live local ray perception per tank (optional) |
| `ground_overhead.c` / `.h`, `ground_overhead_adapter.inc` | avoidance | overhead probes into a shared occupancy cache (optional) |
| `ground_phys_rate.inc` | host tuning | asks the host for a bot physics tick rate |
| `ground_tactics.inc` | tactics | sense, rear alert, take-over, wait / peek |
| `ground_fire.inc` | fire control | pre-aim, stabiliser, aim-point choice, fire gate |
| `host_stub/ground_host_stub.c` | interface | host skeleton: every `gh_*` function, trivial bodies |
| `build_public.cmd` | build | builds the library and the link test |

## Build (MSVC, x64)

Run `build_public.cmd` in this directory (it finds the Visual Studio C tools with `vswhere`). It compiles the nine
`.c` files with `/std:c11 /W4 /WX /O2 /MT` into `build\ground_ai.lib`, then links
`host_stub\ground_host_stub.c` against the whole library (`/WHOLEARCHIVE`) into `build\host_stub_link.exe`. The
link succeeds only if the skeleton provides every symbol the library needs; the exe does nothing.

The code has not been built with gcc or clang. What it uses that is MSVC- or x86-specific:

- `__declspec(thread)` (`ground_vehicle_control.inc`);
- `_BitScanForward64` and `<intrin.h>` (`ground_brain.c`, `ground_motion.c`);
- SSE / SSE2 intrinsics (`<emmintrin.h>`: `_mm_*`, `__m128`, `__m128i`) in `ground_brain.c` and `ground_lidar.c`;
- `#pragma pack(push, 1)` (`ground_lidar.c`, `ground_overhead.c`);
- `MAX_PATH` (defined as 260 where the Windows header is absent) and `_CRT_SECURE_NO_WARNINGS`.

## Configuration

The AI reads a `key=value` file whose path is in the environment variable `WT_GROUND_AI_CONFIG` (through
`gh_env`). The keys are parsed in `init()` near the top of `ground_brain.c` (map pack paths: `routes_file`,
`routes_map_file`, `routes_net_file`; plan-pack lines `tp_*`, parsed in `ground_teamplan_data.inc`; vehicle
classes `cls.<seat>=`; lanes `lane=`; switches such as `lanes`, `teamplan`, `routes`, `graft`). A number of
switches are also read directly from `WT_GROUND_*` environment variables, right after the config file in the same
function, and in `ground_fire.inc` (`WT_GROUND_PENAIM`, `WT_GROUND_WEAKSPOT`, ...). With `log_dir=` set, the AI
writes its event log and probe CSV files there.
