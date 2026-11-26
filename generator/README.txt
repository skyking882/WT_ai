Ground-route generator (lane packs + road network), Windows and Linux
=====================================================================

What it does
  For one ground mission it reads the mission's prepared base layers (class map, sight layers, foliage), its zones,
  spawn points and spawn-protection areas, and the speeds measured in logged battles, and writes four files per
  mission into out/<mission>/ -- the files a route-following program reads:

  <mission>.plan.json     JSON, format "WTPLAN002": the armies' axes and sectors, spawn areas, zones, the lanes (one
                          corridor from a spawn area to one end: a zone circle, a fire-support position or a position
                          watching enemy traffic; with slots, bounds, neighbours, opposing lanes) and the plans (per
                          army: role and share of every zone).
  <mission>.routes.bin    binary, magic "WTRTE004": spawn poses, targets (the zones and the lane slots), slots, a rank table
                          and the route curves (0.5 m samples with clearance and curvature). Layout in gen/binary_v4.py.
  <mission>.map.bin       binary, magic "WTMAP004": the merged map as a 0.25 m hard-cell bitmap and a 0.5 m clearance
                          grid. Layout in gen/binary_v4.py.
  <mission>.network.bin   binary, magic "WTNET001": a road network built from the route curves: nodes, edges (sample
                          runs, length, time, minimum clearance, use count), adjacency and the nodes of every target.
                          Layout in gen/network38.py (write_network_bin / read_network_bin).

  Also written: read_check.txt, lanes_summary.json, network_report.txt, net_queries.txt, run_report.json, lanes.log,
  network.log, pictures (lanes_army*.png, zone_*_lanes.png, positions_*.png, plans_*.png, cover_*.png, network.png)
  and a work folder out/<mission>/cache/.

Requirements
  * Python 3.10 or later (tested with 3.12) with numpy and Pillow. Nothing else (no scipy, no matplotlib).
  * A C compiler with OpenMP for the helpers (built on first use into build/, rebuilt when a source is newer):
      Linux:   gcc with OpenMP (libgomp), e.g. Ubuntu: sudo apt install build-essential
               (gcc -O2 -fopenmp -shared -fPIC -> build/s30_core.so; read_check -> build/read_check);
               another compiler: set CC
      Windows: Microsoft Visual C++ (cl, found with vswhere) -> build/s30_core.dll, build/read_check.exe
  * Memory: about 6-8 GB per map with 13 worker processes; disk: about 500 MB of out/ per map.

The two commands
  python run_map.py <mission> [--workers N] [--check]
      lanes (gen/lanes37.py), then the road network (gen/network38.py), for one mission; outputs in out/<mission>/.
      --check: inputs present, zones and spawn areas readable, anything unusual (out/<mission>/check.json); fast.
      --workers: worker processes (default min(13, CPUs)); also the OpenMP thread count of the C helpers.
      Exit code 0 = ok, 1 = failed (the reason is in run_report.json).
  python run_all.py [mission ...] [--workers N] [--check]
      the same for a list (default: every mission with data/maps/<mission>/map.npz), one after the other; a failure
      does not stop the batch. out/batch_report.txt per mission: ok / failed and why, phase times, lane counts,
      limits, read_check, network checks, pack sizes.
  Environment: WT_RUNS_EXCLUDE (comma-separated run folder names left out of the speed measurement; default none).

Layout (all paths come from paths.py, relative to this folder)
  paths.py      the root and every location
  run_map.py    one mission;  run_all.py  a list
  gen/          the generator: lanes37.py (lanes, targets, routes.bin / map.bin / plan.json), network38.py (network),
                their modules, C sources (routes_core.c, s30_vis.c, lanes_core.c, read_check.c), build_core.py
  prep/         builds the inputs and base layers from an installed copy of the game: Windows only, not needed to
                generate (build_battle_areas.py, blk_to_text.py, extract_spawn_protection.py, lnav_extract.py,
                extract_terrain.py + gen_terrain.py, prep_base.py and their modules)
  tools/        compare_base.py (base layers vs another folder), core_selftest.py (C helper checksums),
                notes_summary.py (lanes_summary.json notes), plans_only.py (the plan step alone), speeds.py
                (measured speeds), wsl_probe.sh / wsl_test.sh (Linux environment probe and build check)
  data/         inputs; every file, its format and where it comes from: data/README.txt
  out/, build/  generated

Inputs (details in data/README.txt)
  data/common/ground_battle_areas.json   zones, spawn points, battle rectangle and level of every mission (required)
  data/common/spawn_protection.json      spawn-protection (kill) areas (optional)
  data/maps/<mission>/                   base layers map.npz, sight.npz, foliage.npz, map_report.json (required)
  data/runs/<run>/                       logged battles: measured road / off-road speeds (optional)
  data/sweden/routes.json                sweden_dom only, optional: spawn poses with headings (without it sweden_dom
                                         uses its spawn points with unknown heading, like every other mission)
  data/closed/<mission>.closed.json      optional: lanes switched off by position
  prep only: data/common/rendinst_dmg.blk, data/terrain/<mission>.bin, data/lnav/<level>.bin (optional),
             data/obstacles/<mission>.bin (optional), and the installed game

Base layers (prep, Windows only)
  Every prep input except the obstacle scan is written by a script in prep/. In order, from the tree's folder, with
  the environment variable WT_GAME_DIR set to the installed game's folder (prep stops with a message when it is not
  set; the generator itself never opens game files):
    python prep/build_battle_areas.py <missions dir>     data/common/ground_battle_areas.json; <missions dir> =
                                                         the tank mission files of mis.vromfs.bin unpacked to JSON by
                                                         a BLK unpacker such as wt_ext_cli
    python prep/blk_to_text.py                           data/common/rendinst_dmg.blk (from aces.vromfs.bin)
    python prep/extract_spawn_protection.py <m> [...]    data/common/spawn_protection.json (optional)
    python prep/lnav_extract.py <game>/levels/<level>.bin data/lnav/<level>.bin     (optional, per level)
    python prep/gen_terrain.py <m> [...] [--runs <dir>]  data/terrain/<m>.bin (HM2 heights of the level; deck layer
                                                         only from logged battles with ground_samples.csv)
    python prep/prep_base.py <m> [...]                   data/maps/<m>/ (reads the level's rendinst placements and
                                                         content/base/res through the game CDK's daKernel-dev.dll)
  Exact inputs, formats and what each step reads: data/README.txt.

  Cannot be produced offline: data/obstacles/<m>.bin, the 2 m obstacle scan, made inside a running game by code that
  is not part of this tree. prep works without it (the same grid, from the battle rectangle + 150 m); then the map
  has no scan classes (hard_static_scan, unknown_ri_scan, unknown_edge_scan, overhead) and no bridge decks: static
  scene objects that are not placed instances (bridges and some structures) are missing, bridges over deep water are
  water, and map_report.json says so ("obstacle_scan").

Third-party tools you need for prep
  the game CDK's codec DLL (daKernel-dev.dll, part of the installed game); a BLK unpacker that writes mission files
  as JSON (e.g. wt_ext_cli); the Python packages vromfs and blk (for blk_to_text.py and extract_spawn_protection.py).
  None of them is part of this tree.
