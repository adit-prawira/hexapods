# hexapods

A native hexapod robotics playground: C++20 brain, MuJoCo physics, MJCF XML body.
macOS, Apple Silicon.

<img width="1077" height="784" alt="Screenshot 2026-11-21 at 10 32 07 pm" src="https://github.com/user-attachments/assets/f34ef1ae-8391-4041-860a-2313f38882c5" />

## Setup — zero to running, step by step

Three steps, in this order. Steps 1–2 are once per machine; step 3 is once
per clone.

1. **Toolchain** — install the build tools first (step 2 needs CMake):

   ```
   brew install cmake glfw clang-format
   ```

2. **MuJoCo 4.0.1** — the physics engine, built from source and installed to
   `~/.mujoco` (~10–20 min; its dependencies are fetched automatically):

   ```
   git clone https://github.com/google-deepmind/mujoco.git ~/src/mujoco
   cd ~/src/mujoco
   git checkout 4.0.1
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=$HOME/.mujoco/mujoco-4.0.1
   cmake --build build --parallel
   cmake --install build
   ```

   Non-default location? Build hexapods with
   `MUJOCO_ROOT=/path/to/mujoco make build`.

3. **Build and run** — back in this repo:

   ```
   make build
   make run
   ```

   The first `make build` also fetches Dear ImGui (the HUD library) from
   GitHub automatically — pinned to `v1.93.0`, nothing to install, no
   network needed after the first time. If you see ImGui files compiling
   under `build/_deps/imgui-src/`, that's this step working.

There are no other dependencies. The binary finds MuJoCo through the
`MUJOCO_ROOT` prefix (default `~/.mujoco/mujoco-4.0.1`).

## Daily commands

- `make run` — build if needed, open the 3D viewer
- `make test` — headless scenario gates (smoke, determinism)
- `make build` — compile only
- `make format` — clang-format over `src/` and `tests/` (run before
  committing; CI enforces it)
- `make clean` — remove `build/` (the next build re-fetches ImGui)

## Third-party code

Dear ImGui (the HUD) is fetched at configure time (CMake `FetchContent`),
pinned in `CMakeLists.txt`. The first configure needs network once; it is
cached in `build/_deps` afterwards. Upgrading = bump the `GIT_TAG` pin.

## Controls

- drag (left / right / middle): orbit / pan / zoom
- scroll: zoom
- `space`: pause / resume
- `R`: reset (the robot stands up again)
- HUD, top-left: fps, steps, joint tracking error, trunk pitch and height —
  and the permanent **actuators** toggle; untick it and gravity wins

## Session journal

Every run writes `journal.log` (gitignored): physics-affecting inputs stamped
by sim step. Phase 4 ghost replays consume it.
