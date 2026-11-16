cmake_minimum_required(VERSION 3.25)
project(hexapods LANGUAGES CXX)

# find_package honors the MUJOCO_ROOT cache variable directly (CMake >= 3.27).
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MuJoCo location: source build installed via `cmake --install`
set(MUJOCO_ROOT "" CACHE PATH "MuJoCo install prefix, e.g. ~/.mujoco/mujoco-4.0.1")
if(NOT MUJOCO_ROOT)
  message(FATAL_ERROR
    "MUJOCO_ROOT not set. Build MuJoCo once (README 'Setup'), then configure with:\n"
    " cmake -S . -B build -DMUJOCO_ROOT=$HOME/.mujoco/mujoco-4.0.1")
endif()
list(APPEND CMAKE_PREFIX_PATH "${MUJOCO_ROOT}")
find_package(mujoco CONFIG REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

# Dear ImGui: ships no CMake package, so fetch the pinned release at
# configure time (network needed once; cached in build/_deps afterwards).
include(FetchContent)
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.93.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)

add_executable(hexapods
  src/main.cpp
  src/control/control.cpp
  src/world/world.cpp
  src/session/clock.cpp
  src/events/journal.cpp
  src/presentation/viewer.cpp
  src/verification/scenario.cpp
  tests/scenarios.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
)
target_include_directories(hexapods PRIVATE src ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(hexapods PRIVATE mujoco::mujoco glfw OpenGL::GL)

set_target_properties(hexapods PROPERTIES 
  BUILD_WITH_INSTALL_RPATH ON 
  INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)
