# LNS4MAPF

**A from-scratch anytime Multi-Agent Path Finding solver with an interactive visualizer that replays every step the search took.**

![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![SFML](https://img.shields.io/badge/GUI-SFML%20%2B%20ImGui-8CC445)
![License](https://img.shields.io/badge/license-MIT-blue)

<p align="center">
  <img src="docs/media/solution.gif" width="608" alt="60 agents crossing random-32-32-20 without a single collision">
</p>
<p align="center"><em>60 agents crossing <code>random-32-32-20</code>. Every agent follows the path the solver planned for it; no two of them ever share a cell or swap places.</em></p>

This repository is the code of a diploma thesis (Czech Technical University in Prague, FEE, 2025) on using **Large Neighbourhood Search** for the **Multi-Agent Path Finding** problem. It contains a complete solver, a visual debugger for it, an experiment harness, and the reference implementations the work was compared against.

---

## Table of contents

- [The problem](#the-problem)
- [What is in here](#what-is-in-here)
- [Visual tour](#visual-tour)
- [How the solver works](#how-the-solver-works)
- [Getting started](#getting-started)
- [Command line options](#command-line-options)
- [Tests, benchmarks, profiling and docs](#tests-benchmarks-profiling-and-docs)
- [Experiments](#experiments)
- [Regenerating the media](#regenerating-the-media)
- [Repository layout](#repository-layout)
- [Benchmarks and related work](#benchmarks-and-related-work)
- [License](#license)

---

## The problem

**Multi-Agent Path Finding (MAPF)** asks for collision-free paths for many agents at once.

- The world is a 4-connected grid; black cells are obstacles.
- Time is discrete. In each timestep an agent either moves to one of the four neighbouring cells or waits.
- Every agent *i* has a start cell and a goal cell and must stay at its goal once it arrives.
- Two paths collide if they use the same cell in the same timestep (**vertex conflict**) or traverse the same edge in opposite directions in the same timestep (**edge conflict**).

The objective used throughout this project is the **sum of costs**: the sum over all agents of the number of timesteps until the agent reaches its goal for good. Finding a collision-free plan is easy in principle; finding a *cheap* one is NP-hard, which is why anytime methods like LNS are attractive — they produce a feasible plan quickly and then keep improving it for as long as you let them run.

## What is in here

| | |
|---|---|
| **An anytime MAPF-LNS solver** | Prioritized planning for the initial plan, then a destroy-and-repair loop that keeps replanning small groups of agents while the plan gets cheaper. C++17, no solver framework — the search, the safe-interval structures and the constraint tables are all part of this repository. |
| **An interactive visualizer** | Not just a solution player: it replays every LNS iteration and lets you step through the **low-level search node by node**, showing the expanded position, its safe interval and the *g*/*h*/*f* values of each expansion. Built with SFML + Dear ImGui (docking branch). |
| **An experiment harness** | Nine experiment binaries that sweep instances, agent counts, seeds and algorithm variants, write one JSON file per run, and a set of Python scripts that load those into SQLite and plot them. |
| **Reference implementations** | The original MAPF-LNS / MAPF-LNS2 / ADDRESS code used as a baseline lives under [`papers/`](papers) and [`my_code/`](my_code). |

## Visual tour

### The GUI

<p align="center">
  <img src="docs/media/gui_overview.png" width="900" alt="The solver GUI: left panel with SIPP node info, the map, the right panel with LNS info and the console">
</p>

1. **Left panel** — the selected agent (start, goal, current position) and its *low-level search*: a playback bar over the search nodes plus the details of the node expanded in the current step (position, safe interval, *g*, *h*, *h2*, *h3*, *f*, whether the cell is somebody's goal, nodes generated/expanded and which agents were already planned at that moment).
2. **Map** — the grid, the agents (filled circle = current position, hollow circle = goal) and their paths. Mouse wheel zooms, dragging pans, clicking an agent selects it.
3. **Right panel** — two independent playback bars: one over the *timesteps* of the displayed solution, one over the *LNS iterations*. For the selected iteration it shows which destroy operator ran, which agents were destroyed, whether the candidate was accepted and by how much the cost changed. "Highlight destroyed" paints the replanned paths red.
4. **Console** — the solver's log, streamed from the computation thread while it works.

The solver runs the search in one thread and the GUI in another, so the visualization stays responsive on large instances and you can watch the cost drop live. The *Instance* menu loads a different map or scenario (or changes the number of agents) and re-solves without leaving the window.

### Stepping through the low-level search

<p align="center">
  <img src="docs/media/sipp.gif" width="900" alt="SIPP expanding safe intervals for one agent">
</p>

The selected agent's SIPP search, replayed expansion by expansion: the shaded cells are the ones already expanded, and the left panel shows the safe interval and the heuristics of each node as the frontier spreads towards the goal. The paths drawn on the map are exactly the constraints this search has to respect (the agents listed under *Already planned*).

### Watching LNS destroy and repair

<p align="center">
  <img src="docs/media/lns.gif" width="900" alt="LNS iterations: destroyed neighbourhoods highlighted in red">
</p>

Twelve consecutive LNS iterations. In each one, a destroy operator removes a neighbourhood of paths (red) and the repair step replans them one by one; the panel reports the operator that was chosen, the agents in the neighbourhood, and whether the candidate plan was accepted (`Accepted: 1`) or thrown away. Because the adaptive operator picks a strategy per iteration, the reported strategy changes as you step forward.

### Larger instances

<p align="center">
  <img src="docs/media/large_instance.png" width="900" alt="250 agents on den312d">
</p>

250 agents on `den312d`, solved with the `BLOCKED` destroy operator and the bounded-suboptimal low-level search — what `./scripts/run_specific_instance.sh den312d -w 1.2` runs. The console shows the anytime behaviour: a feasible plan of cost 17359 is found first and then improved iteration after iteration.

<details>
<summary>The exact commands behind the figures</summary>

```bash
cd my_code/my_solver

# solution.gif, sipp.gif, lns.gif, gui_overview.png
./build/simple_MAPF_solver \
  -m ../assets/mapf-map/random-32-32-20.map \
  -a ../assets/mapf-scen-random/scen-random/random-32-32-20-random-1.scen \
  -k 60 -s 1 -i 500 -t 10

# large_instance.png
./build/simple_MAPF_solver \
  -m ../assets/mapf-map/den312d.map \
  -a ../assets/mapf-scen-random/scen-random/den312d-random-1.scen \
  -k 250 -s 0 -i 500 -t 10 \
  --destroy_operator BLOCKED --sipp_implementation SIPP_suboptimal -w 1.2
```

</details>

## How the solver works

```
             ┌──────────────────────── initial solution ────────────────────────┐
 instance ──►│ prioritized planning: plan agents one by one in random order,    │
             │ each avoiding the paths already planned (restart on failure)     │
             └──────────────────────────────┬──────────────────────────────────┘
                                            ▼
             ┌──────────────────────── improvement loop ────────────────────────┐
             │  destroy: pick a neighbourhood of n agents and delete their paths│
             │  repair : replan them one by one with SIPP against the rest      │
             │  accept the candidate only if it is strictly cheaper             │
             └───── repeat until --maxIterations or --timeLimit is reached ─────┘
```

**Initial solution — prioritized planning.** Agents are planned one at a time in a random order; each agent treats the already-planned paths as moving obstacles. Prioritized planning is incomplete, so a failed attempt is retried with a fresh random order (`-r/--Restarts`).

**Low level — SIPP.** Instead of searching over (cell, timestep) pairs, [Safe Interval Path Planning](#benchmarks-and-related-work) searches over (cell, *safe interval*) pairs, where a safe interval is a maximal span of time in which the cell is free. That collapses long stretches of waiting into a single node and makes replanning against many moving obstacles cheap. The variants selectable with `--sipp_implementation` are:

| Value | What it is |
|---|---|
| `SIPP_mine` | the optimal implementation written for this thesis (default) |
| `SIPP_mine_ap` | the same search with the additional (multi-heuristic) tie-breaking used in the parameter studies |
| `SIPP_suboptimal` | bounded-suboptimal search: expands nodes with *f* ≤ *w* · *f*<sub>min</sub>, trading optimality of the single path for speed (`-w`) |
| `SIPP_suboptimal_ap` | the multi-heuristic variant of the above |
| `SIPP_mapf_lns` | a port of the SIPP used in the original MAPF-LNS code, kept for comparison |

The paths a search has to avoid live in two incrementally maintained structures: a **constraint table** (which agent occupies which cell and which edge at which time, plus the per-cell agent counts the destroy operators need) and a **safe interval table** (the free intervals of every cell). Inserting or removing a path updates them in place — in parallel over the timepoints of the path, with one OpenMP lock per cell — so an LNS iteration never rebuilds them from scratch.

**Destroy operators** (`--destroy_operator`, neighbourhood size `-n`):

| Operator | Which agents it removes |
|---|---|
| `RANDOM` | a uniformly random set of agents |
| `RANDOMWALK` | the agents met by a random walk that starts at a random point of a random agent's path — i.e. agents that are close in space *and* time and therefore likely to interact |
| `INTERSECTION` | the agents that meet at a busy cell (one visited by at least three agents), picked at random and grown outwards until the neighbourhood is full |
| `BLOCKED` | starts from an agent that cannot reach its goal at its earliest possible time, adds the agents blocking it, then the agents blocking those — a chain of blame |
| `RANDOM_CHOOSE` | picks one of the above uniformly at random in every iteration |
| `ADAPTIVE` | keeps a weight per operator, samples proportionally to the weights and reinforces whatever produced improvements (the default) |

**Repair and acceptance.** The destroyed paths are replanned one by one against the surviving paths. The candidate is kept only if it is feasible *and* strictly cheaper than the incumbent (measured in sum of delays); otherwise the constraint structures are rolled back and the incumbent survives. Either way the outcome feeds back into the adaptive weights, so operators that stop paying off are chosen less often. Because the plan is feasible from the very first iteration, the solver can be stopped at any time — that is the *anytime* property, and it is what the LNS playback bar in the GUI lets you inspect after the fact.

## Getting started

### 1. Dependencies

```bash
sudo apt install libsfml-dev nlohmann-json3-dev libeigen3-dev sqlite3 libsqlite3-dev libgtk-3-dev -y
sudo apt install libboost-all-dev -y
git submodule update --init --recursive
```

or simply `./my_code/my_solver/scripts/install_dependencies.sh`, which runs exactly the three commands above.

<details>
<summary>Versions the project was developed with</summary>

SFML 2.5.1 · Boost 1.71.0 (program_options, system, filesystem, regex) · nlohmann-json 3.7.3 · Eigen3 · OpenGL · OpenMP · GTK+3 (native file dialogs) · SQLite3 (experiments only) · GoogleTest 1.10 / GoogleMock / Google Benchmark 1.5 (fetched by CMake if missing).

Vendored under `my_code/my_solver/third_party_libraries/`: Dear ImGui, ImGui-SFML, magic_enum, indicators, and `nativefiledialog-extended` (a git submodule — hence `git submodule update --init --recursive`).

</details>

### 2. Build

```bash
cd my_code/my_solver
./scripts/compile.sh                 # relwithdebinfo + tests (defaults)
./scripts/compile.sh release notest  # optimized, no tests
```

The first argument is the build type (`debug` — no optimizations, asserts on; `relwithdebinfo` — optimized with debug info and asserts, the default; `release` — optimized, asserts off), the second whether to build the unit tests and benchmarks (`tests` / `notest`).

Plain CMake works too:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF
cmake --build build -j
```

### 3. Run

```bash
./scripts/run.sh -m <map> -a <scenario> -k <number of agents>   # ./build/simple_MAPF_solver
```

for example

```bash
./scripts/run.sh -m ../assets/mapf-map/random-32-32-20.map \
                 -a ../assets/mapf-scen-random/scen-random/random-32-32-20-random-1.scen \
                 -k 60 -i 500 -t 10
```

Prepared instances (`./scripts/run_specific_instance.sh <name> [extra options]`, extra options are appended to the command line):

| Name | Instance |
|---|---|
| `dummy` | 3×3 toy map, 1 agent |
| `empty` / `empty-8-8` / `empty-32-32` | empty grids (16×16, 8×8, 32×32), 60 / 32 / 400 agents |
| `random-32-32-10` | random obstacles, 100 agents |
| `den312d`, `den520d` | game maps, 250 / 500 agents (`den520d` is the default) |
| `room` | `room-64-64-8`, 500 agents |
| `warehouse` | `warehouse-10-20-10-2-1`, 400 agents |
| `ost` | `ost003d`, 400 agents |
| `Paris` | `Paris_1_256`, 700 agents |

The harder ones already carry the settings they need (`BLOCKED` + `SIPP_suboptimal`) in the script. Add `-G false` to run headless (no window, no visualization overhead) and `--output_paths <file>` to write the resulting plan to disk.

## Command line options

`./build/simple_MAPF_solver -h`

| Option | Default | Meaning |
|---|---|---|
| `-m, --map <file>` | — | map file in the [MAPF benchmark](#benchmarks-and-related-work) `.map` format |
| `-a, --agents <file>` | — | scenario file (`.scen`) with the start/goal pairs |
| `-k, --agentNum <int>` | `0` | how many agents to take from the scenario |
| `-G, --GUI <bool>` | `1` | open the visualizer; with `false` the solver runs headless and prints to stdout |
| `-i, --maxIterations <int>` | `10` | number of LNS iterations (0 = initial solution only) |
| `-t, --timeLimit <sec>` | `30` | wall-clock budget for the whole search |
| `-n, --neighborhood_size <int>` | `10` | how many paths the destroy operator removes per iteration |
| `--destroy_operator <name>` | `ADAPTIVE` | `RANDOM`, `RANDOMWALK`, `INTERSECTION`, `ADAPTIVE`, `RANDOM_CHOOSE`, `BLOCKED` |
| `--sipp_implementation <name>` | `SIPP_mine` | `SIPP_mine`, `SIPP_mine_ap`, `SIPP_suboptimal`, `SIPP_suboptimal_ap`, `SIPP_mapf_lns` |
| `-w, --sipp_suboptimality <double>` | `1` | suboptimality factor of the bounded-suboptimal low-level search |
| `-r, --Restarts <bool>` | `1` | restart the initial planning with a new random order when it fails |
| `-s, --seed <int>` | `-1` | seed of the random generators; negative = non-reproducible |
| `--output_paths <file>` | — | write the final plan to this file |
| `-h, --help` | — | print the option list |

An unknown `--destroy_operator` or `--sipp_implementation` is not fatal: the solver prints a warning and falls back to the default. The names are matched case-insensitively.

## Tests, benchmarks, profiling and docs

```bash
./scripts/run_tests.sh                     # all unit tests (GoogleTest)
./scripts/run_specific_test.sh 'SIPP*'     # a subset, gtest filter syntax
./scripts/run_benchmark.sh sipp_bm         # sipp_bm | vector_operations_bm | path_conv_bm
./scripts/run_valgrind.sh den520d          # memory check on den520d with 500 agents
./scripts/clang_tidy.sh                    # static analysis
./scripts/profile.sh                       # callgrind profile, view with kcachegrind
./scripts/generate_documentation.sh        # Doxygen -> documentation/html/
./scripts/show_documentation.sh            # open the generated docs
```

(The last two need `sudo apt install doxygen graphviz` — the Doxyfile has `HAVE_DOT = YES`, so the call graphs are drawn with graphviz.)

The unit tests cover the safe-interval table, the constraint table, SIPP, the LNS loop, instances, solutions and the utilities (`tests/src/`), using the small maps in `tests/test_maps/` together with recorded reference solutions in `tests/test_solutions/`. The benchmark scripts set the CPU governor to `performance` first, so they ask for `sudo`. For profiling, GCC gives better debug info in kcachegrind than clang:

```bash
export CC=/usr/bin/gcc CXX=/usr/bin/g++
```

## Experiments

Every experiment is its own binary in `src/experiments/`, built next to the solver:

| Binary | Question it answers |
|---|---|
| `feasible_experiment`, `feasible_restarts_experiment` | how often (and how fast) a feasible initial solution is found, with and without restarts |
| `sipp_pp_experiment` | this SIPP + prioritized planning against the original MAPF-LNS implementation |
| `destroy_experiment`, `repair_experiment` | the destroy operators and the repair step in isolation |
| `influence_of_w_experiment`, `influence_of_p_experiment`, `influence_of_ap_experiment` | parameter studies of the bounded-suboptimal search |
| `overall_improvement_experiment` | the full solver: cost over time |

```bash
cd my_code/my_solver
./scripts/run_experiment.sh feasible_experiment   # pins the CPU governor, so it asks for sudo
./scripts/run_all_experiments.sh                  # all of them, one after another
```

Each run writes one JSON file per (instance, agent count, algorithm, seed) into
`my_code/my_solver/experiments/<experiment name>/<map>/`, containing the settings, the initial cost and makespan, node counts, timings and the whole cost-over-time trace. The Python side then collects them from there:

```bash
cd experiments
python3 add_all_experiments.py       # JSON files -> experiment_database.db (set experiment_name at the top)
python3 add_experiment_csv.py        # import the CSV produced by the original MAPF-LNS code
python3 feasible_results.py          # plots; one *_results.py per experiment
```

`experiments/experiment_database.db` is a SQLite database with one table per experiment, so the analysis scripts are plain SQL queries plus matplotlib/seaborn. `remove_experiments.py` and `reset_experiment.py` clean up individual runs or whole tables.

## Regenerating the media

All GIFs and screenshots in this README are produced by one script, so they can be refreshed after a UI change without a desktop session:

```bash
python3 -m venv .venv && .venv/bin/pip install python-xlib
sudo apt install xvfb ffmpeg imagemagick -y
.venv/bin/python docs/media/record_demo.py            # all clips
.venv/bin/python docs/media/record_demo.py lns        # solution | sipp | lns | overview | large
```

It starts an off-screen X server (Xvfb, 1600×900), launches the solver on it, drives the ImGui widgets with synthetic XTEST mouse events, records with `ffmpeg -f x11grab` and converts the result into a palette-optimized GIF. See [`docs/media/record_demo.py`](docs/media/record_demo.py).

## Repository layout

```
my_code/my_solver/        the solver and the visualizer
  inc/, src/              LNS, SIPP, ConstraintTable, SafeIntervalTable, Instance, Map,
                          Solution, Computation (worker thread), Visualizer (GUI thread),
                          SharedData (their handshake)
  src/experiments/        one .cpp per experiment binary
  tests/                  unit tests, benchmarks, test maps and reference solutions
  scripts/                build / run / test / profile / documentation helpers
  third_party_libraries/  ImGui, ImGui-SFML, magic_enum, indicators, nativefiledialog
experiments/              JSON -> SQLite import and the plotting scripts
my_code/                  reference implementations used as baselines
my_map/                   hand-made instances (dummy)
MAPF-benchmark/           the standard MAPF benchmark maps and scenarios
papers/                   related work, incl. the ADDRESS / MAPF-LNS sources
docs/media/               README figures and the script that records them
```

## Benchmarks and related work

The instances come from the standard MAPF benchmark set of Stern et al., *Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks* (SoCS 2019), bundled in [`MAPF-benchmark/`](MAPF-benchmark).

The algorithms this work builds on:

- Phillips & Likhachev, *SIPP: Safe Interval Path Planning for Dynamic Environments* (ICRA 2011) — the low-level search.
- Li, Ruml & Koenig, *MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search* (AAAI 2022) — the destroy/repair scheme and the neighbourhood operators.
- Li et al., *Anytime Multi-Agent Path Finding via Large Neighborhood Search* (IJCAI 2021) — the anytime improvement loop.
- Huang et al., *Anytime Multi-Agent Path Finding via Machine Learning-Guided Large Neighborhood Search* / ADDRESS — a further baseline, sources in `papers/ADDRESS`.

## License

MIT — see [`LICENSE`](LICENSE). The vendored libraries and the reference implementations under `papers/` and `my_code/` keep their own licenses.

Copyright (c) 2025 Jan Chleboun · <chlebja3@fel.cvut.cz>


# MAPF solver with interactive visualizations

This directory holds the solver itself: an anytime MAPF-LNS solver with SIPP as the low-level
planner and an interactive SFML + ImGui visualizer (C++17).

**The documentation lives in the [README at the root of the repository](../../README.md)** —
dependencies, build, the command line options, the prepared instances, the tests and benchmarks,
the experiment harness and a visual tour of the GUI.

The short version:

```bash
./scripts/install_dependencies.sh   # apt packages + git submodules
./scripts/compile.sh                # -> ./build/simple_MAPF_solver
./scripts/run_specific_instance.sh den312d
./scripts/run.sh -m <map> -a <scenario> -k <number of agents>
./scripts/run.sh -h                 # all options
./scripts/run_tests.sh
```

## License

MIT, see [`LICENSE`](../../LICENSE).

## Author

Jan Chleboun (2025), <chlebja3@fel.cvut.cz>
