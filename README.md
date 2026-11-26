# WT_ai

A ground-vehicle bot AI for team capture-point battles, and the offline generator of the map data it drives on.

| Directory | What it is |
|---|---|
| [`ai/`](ai/README.md) | The AI itself, in C: command (who goes where, over which lane, without bunching), route, avoidance, driver (W / S / A / D like a player), tactics, fire control. It reaches its environment only through the functions declared in `ai/ground_host.h`. |
| [`generator/`](generator/README.txt) | Python + C tools that build, per map, the four files the AI reads: drivable map, route library, road network, lane plan. |

## What is not here

- The host layer: the code that implements `ai/ground_host.h` for a particular environment. `ai/host_stub/ground_host_stub.c` lists every function an integrator has to provide.
- Generated map packs and battle logs.

## State

- The AI sources build as a static library with MSVC (`ai/build_public.cmd`) and link against the stub host. They have not been built with gcc or clang.
- The public split of the sources was checked by build and by line-by-line comparison with the version it was made from; this exact tree has not been run in a battle.
- Three generator scripts that depend on third-party packages are written but were never run (see `generator/README.txt`). One generator input, the obstacle scan, cannot be produced offline; without it maps with bridges are incomplete.

The design is described in `ai/docs/ground_ai_algorithm_20261120.md` (Chinese).
