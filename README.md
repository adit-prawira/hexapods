# hexapods

A native hexapod robotics playground: C++20 brain, MuJoCo physics, MJCF XML body.
macOS, Apple Silicon.

## Setup (once)

1. Build MuJoCo from source and install it to `~/.mujoco` (~10–20 min; deps are
   fetched automatically):

   ```
   git clone https://github.com/google-deepmind/mujoco.git ~/src/mujoco
   cd ~/src/mujoco && git checkout 4.0.1
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=$HOME/.mujoco/mujoco-4.0.1
   cmake --build build --parallel
   cmake --install build
   ```

2. Toolchain: `brew install cmake glfw clang-format`

## Build & run

```
make run
# or explicitly:
cmake -S . -B build -DMUJOCO_ROOT=$HOME/.mujoco/mujoco-4.0.1
cmake --build build
./build/hexapods
```

## Controls

- drag (left / right / middle): orbit / pan / zoom
- scroll: zoom
- `space`: pause / resume
- `R`: reset (the box drops again)

## Session journal

Every run writes `journal.log`: physics-affecting inputs (reset, keys, later
perturbations and sliders) stamped by sim step. Camera and pause are viewer-level
and never journaled. Ghost replays (Phase 4) will replay this file.

Scenario gates (`--scenario <name>`) arrive with story #10.
