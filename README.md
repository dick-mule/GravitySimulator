# GravitySimulator

A real-time, GPU-accelerated gravity simulator featuring a deformable visualization grid rendered with Vulkan. The project demonstrates high-performance compute shaders, physically faithful gravitational warping across multiple geometries, a rippling elastic-membrane grid, traced null geodesics (light rays), and a clean comparison between GPU and multi-threaded CPU implementations.

## Features

- **Three Spatial Geometries**
  - **Flat**: Classic Newtonian-style gravitational wells
  - **Spherical**: Gravity visualized on the surface of a sphere (with proper geodesic distances and tangent-plane projections)
  - **Hyperbolic**: Gravity on a saddle/paraboloid surface with characteristic negative curvature warping

- **GPU-Accelerated Grid Deformation**
  - Custom Vulkan compute shader (`flat_grid.comp`) that generates the base surface and applies per-geometry gravitational warping in real time
  - Bodies are uploaded to the GPU each frame via storage buffers
  - Full pipeline barriers and descriptor management for safe, high-performance execution

- **Dynamic Membrane (Rippling Grid)**
  - The grid is an elastic sheet: every vertex carries a height + velocity and integrates a damped wave equation on the GPU, with the gravitational-well shape as the forcing term
  - Moving bodies and merges radiate ripples across the surface — the rubber-sheet analogy made literal
  - Ping-pong storage buffers keep the wave Laplacian race-free; live-tunable stiffness, wave speed, and damping

- **Depth Heat-Map Coloring**
  - The grid is colored by warp depth — a multi-stop white → sky → blue → navy gradient — so well depth (and the membrane ripples) read at a glance

- **Null Geodesics (Light Rays)**
  - Trace a fan of massless rays along the geodesics of the active manifold
  - **Lensing on** — rays are deflected by the masses (light-bending); **off** — pure geodesics, so a parallel fan reveals the manifold's curvature (geodesic deviation: parallel on Flat, converging on Spherical, diverging on Hyperbolic)
  - **Dynamic mode** — rays travel against the live, moving field; **Static mode** — the bodies (and the curvature they produce) are frozen, but the rays still animate one step per frame, so you can isolate the lensing pattern of a single snapshot without losing the motion
  - Bounded-rotation deflection: the lensing pull is applied as a per-step-clamped rotation of the ray direction, so even the intense field next to a heavy mass turns the ray smoothly (it can curve sharply or even loop) instead of collapsing or flipping the direction vector in one step
  - Rays ride the warped surface so each deflection's cause is visible, at constant speed (a true null geodesic)

- **High-Performance CPU Fallback**
  - OpenMP-parallelized warping on all three geometries
  - OpenMP-parallelized N-body Verlet integration for the underlying massive body simulation
  - Useful for direct performance comparison and validation

- **Interactive N-Body Simulation**
  - 41 massive bodies (1 dominant central mass + 40 orbiting bodies with randomized masses)
  - Velocity Verlet integration with geometry-aware distance and force calculations
  - Stable circular orbits with proper initial conditions for each geometry
  - Optional inelastic merging — bodies that touch coalesce, conserving mass and momentum (accretion)

- **Scientific Visualization UI**
  - Clean, dark-themed interface with live performance metrics
  - Real-time switching between geometries and GPU/CPU rendering modes
  - Camera controls, simulation parameters, and diagnostic information
  - Live conservation plots — total energy and angular momentum over time, validating the symplectic integrator

## Performance

The simulator is designed to highlight the massive performance advantage of GPU compute for this class of problem.

| Path          | Flat   | Spherical | Hyperbolic | Notes                          |
|---------------|--------|-----------|------------|--------------------------------|
| **GPU**       | ~120 FPS | ~120 FPS | ~120 FPS   | Locked to vsync on modern hardware |
| **CPU (OpenMP)** | ~24 FPS | ~14 FPS | ~18-20 FPS | 800×800 grid + 41 bodies       |

The GPU path stays at the display refresh rate even with a dense 800×800 vertex grid (~640k vertices) and 40+ bodies contributing to the warping field. The CPU path, while significantly accelerated by OpenMP, remains the limiting factor for very large grids or body counts.

## Technical Highlights

- **Vulkan Compute Pipeline**
  - Storage buffers for vertex data and body parameters
  - Push constants for grid parameters, geometry type, and body count
  - Explicit memory barriers between compute and vertex stages
  - Robust descriptor management with runtime safety checks

- **Unified Mass-Driven Warping**
  - One model across all three geometries:
    `depth = wellDepth · tanh(warpGain · Σ pow(mass, 0.4) / softenedDist)`
  - The contributions are summed *inside* the `tanh`, so the warp is bounded by `wellDepth` —
    a cluster of bodies can't stack deeper than a single well — and deep wells round into
    smooth basins instead of clipping flat
  - Mass enters as `mass^0.4`: a compression firm enough to tame the ~600× central-vs-orbiter
    range, gentle enough to keep the central body clearly dominant
  - `radialInfluence` couples well *width* to mass — heavy bodies get proportionally wider
    wells, so the central reads as a broad basin rather than a narrow cone
  - A smooth edge taper fades the warp to zero at the grid boundary (Flat/Hyperbolic), and
    Spherical/Hyperbolic subtract a re-centering offset so the manifold doesn't drift
  - `Well Depth`, `Warp Gain`, and `Radial Influence` are live-tunable from the controls panel
  - The CPU (`Geometry::warpGrid`) and GPU (`flat_grid.comp`) paths share the identical model;
    `Geometry::warpedPosition` is the single source of truth for the per-geometry displacement

- **Dynamic Membrane**
  - The GPU path integrates a damped wave equation per grid vertex each frame:
    `accel = stiffness·(wellShape − h) + waveSpeed·∇²h − damping·v`, in per-step units
  - The well shape is the forcing term, so the grid chases it with inertia and radiates ripples
  - Two ping-pong storage buffers (read previous frame / write current) keep the 4-neighbour
    Laplacian race-free; the buffers are zero-initialized once and never recreated
  - `Wave Speed` is capped just below the CFL stability limit (0.5) so the explicit scheme stays stable

- **Null Geodesics**
  - A light ray is a massless tracer integrated along the manifold geodesic — straight lines on
    Flat, exact great-circle rotation on Spherical, constrained-particle steps on Hyperbolic
  - With lensing enabled it also feels a `∝ mass/dist²` deflection. Only the component
    perpendicular to travel is used (the parallel component would change speed), and it is
    applied as a **rotation of `dir` by a per-step-clamped angle** rather than added to the
    direction vector — so even the intense field next to a heavy mass turns the ray smoothly
    instead of collapsing or flipping it in one step. Combined with constant speed, the path
    stays a true null geodesic that can curve sharply or even loop around a mass.
  - Two modes: **Dynamic** ticks the rays against the live field every frame; **Static** keeps
    the rays animating but stops the N-body update inside `step()`, so the orbits and the
    curvature they produce are frozen while the rays trace through that fixed snapshot.
  - Each traced point is lifted onto the warped surface **once**, when added, so a dense fan of
    rays costs only one lift per ray per frame rather than re-lifting the whole trail history

- **Robust Development Practices**
  - Early development used strict safety gates (`m_UseGPUGrid`) after an initial GPU hang caused by out-of-bounds writes in the compute shader
  - Comprehensive bounds checking, descriptor validation, and debug logging during bring-up

- **Cross-Platform**
  - Runs on macOS (via MoltenVK), Windows, and Linux from a single Vulkan code path
  - Instance creation **feature-detects** the portability extensions MoltenVK needs
    (`VK_KHR_portability_enumeration` / `VK_KHR_portability_subset`) and skips them on
    native drivers — the same logic boots on a Mac and on a native NVIDIA/AMD driver
  - Device selection prefers a **discrete GPU** (and logs the choice), so laptops with
    switchable graphics don't silently run the sim on a weak integrated GPU
  - MSVC-specific portability: signed loop indices for MSVC's OpenMP 2.0, and
    `<windows.h>` macro hygiene (`NOMINMAX`, `#undef MemoryBarrier`) so the Windows
    headers don't clobber `std::max` / `vk::MemoryBarrier`

## Project Structure

```
src/
├── main.cpp          # Entry point — constructs and runs VulkanApp
├── VulkanApp.*       # Vulkan instance/device/swapchain/render pass, window + input, frame loop
├── GridRenderer.*    # Grid buffers & pipelines, GPU compute dispatch, membrane buffers, UI panel
├── Simulation.*      # N-body Verlet integration, body merging, conserved quantities, light rays
├── Geometry.*        # Flat / Spherical / Hyperbolic surfaces: grid generation, CPU warping,
│                     #   the warp/displacement model, and coordinate/velocity conversion
├── CameraController.* # Orbit camera: input, view/projection, controls UI
├── VulkanBuffer.*    # RAII wrapper for a Vulkan buffer + its device memory
├── Types.*           # Shared POD types: Vertex, Object, Camera, PushConstants, Shape/Cube/Sphere
├── ImGuiHandler.*    # Dear ImGui lifecycle and per-frame UI hooks
├── shaders/          # GLSL: grid.vert/frag + per-geometry compute warping shaders
└── imgui/            # Vendored Dear ImGui (not project code)
```

### Architecture

The simulation is split into two layers:

- **N-body layer** — A small set of massive bodies (1 central + 40 orbiters) integrated with
  velocity Verlet. This always runs on the CPU (OpenMP-parallelized pairwise forces).
- **Grid (visualization) layer** — A dense vertex grid warped by the bodies' gravitational
  potential. This runs **either** on the GPU (`*_grid.comp` compute shaders writing the vertex
  storage buffer directly) **or** on the CPU (`Geometry::warpGrid`), selectable at runtime.

`Geometry` is the key abstraction: each of `FlatGeometry`, `SphericalGeometry`, and
`HyperbolicGeometry` implements grid generation, distance, integration, and warping for its
surface. The compute shaders are a hand-port of the CPU `warpGrid` math, so the two paths
should produce visually matching results.

## Rendering Pipeline & Frame Sequencing

### Initialization order (`VulkanApp::run`)

Setup is ordered around one hard dependency — **a framebuffer's depth attachment comes
from the renderer, so framebuffers cannot exist until the renderer does**:

1. `initWindow` — GLFW window (no GL context) + input callbacks.
2. `initVulkan` — instance, surface, physical device, then the **logical device**. The
   device enables `VK_KHR_swapchain`, and — because macOS/MoltenVK is a portability driver —
   `VK_KHR_portability_subset` whenever it is advertised, plus the `fillModeNonSolid` feature
   the wireframe grid's `polygonMode = LINE` requires. Then the swapchain, image views,
   render pass, command pool, command buffers, and sync objects.
3. `initGridRenderer` — constructs `GridRenderer`: grid geometry, the graphics and compute
   pipelines, the ping-pong membrane buffers, and the **per-swapchain-image depth resources**.
4. `createFramebuffers` — each framebuffer binds attachment 0 = swapchain colour view,
   attachment 1 = the renderer's depth view. It runs *after* step 3 so the depth view exists;
   running it earlier bound a colour image into the depth slot (a validation error and a
   wrong-layout present). `recreateSwapchain` follows the same order on window resize.
5. `initImGui` — ImGui context, the GLFW + Vulkan backends, and ImGui's **own** descriptor
   pool (separate from the renderer's compute pool) for its font-atlas texture.

### Per-frame sequence

Each iteration of `VulkanApp::mainLoop` has two halves — **CPU/UI preparation**, then
`drawFrame`, which owns all GPU work.

**In `mainLoop` (before `drawFrame`):**

1. Compute `deltaTime`, poll GLFW events, service a pending resize.
2. `ImGuiHandler::newFrame()` — begin the ImGui frame.
3. `updateCamera` — fold this frame's mouse/keyboard input into the orbit camera.
4. `renderCameraControls` + `renderUI` — build the ImGui widget tree. This is pure CPU work:
   it reads/writes the simulation parameters the widgets are bound to and produces an ImGui
   *draw-data* list. **No Vulkan calls happen here.**

**In `drawFrame(deltaTime)`** — the governing rule is that *nothing which frees or
rewrites a GPU buffer may run before the in-flight fence is waited*, because with a single
frame in flight the previous frame's command buffer may still be reading those buffers:

1. `waitForFences` / `resetFences` — block until the previous frame's GPU work has finished.
2. `updateSimulation` — now safe: one N-body Verlet step, body merges, and one geodesic step
   per light ray; it also **rebuilds the trail vertex/index buffers** (and, on a merge, the
   grid buffers). This must run here, after the wait — doing it in `mainLoop` freed buffers a
   still-executing frame referenced, which lost the device.
3. `updateBodiesBuffer` — upload body positions/masses to the bodies SSBO (host-visible,
   written directly — no staging copy).
4. `updateGrid` — CPU warp path re-uploads the vertex buffer; a no-op on the GPU path.
5. `acquireNextImageKHR` — obtain the next swapchain image index, signalling
   `m_ImageAvailableSemaphore`; an out-of-date result triggers `recreateSwapchain`.
6. Begin recording that image's command buffer.
7. `recordComputeWork` (GPU path) — bind the compute pipeline and descriptors (ping-ponging
   the two membrane buffers), push constants, **dispatch the warp + membrane compute shader**
   over the grid, then a memory barrier so the writes are visible to (a) vertex input for the
   upcoming draw and (b) next frame's compute read of the membrane buffer.
8. `beginRenderPass` — clears the colour and depth attachments.
9. `GridRenderer::draw` — records the scene: the warped grid, the bodies, motion trails, and
   the light-ray paths.
10. `ImGuiHandler::renderDrawData` — records the ImGui draw-data list (built back in step 4)
    into the **same** command buffer, **inside the same render pass**, so the UI composites
    on top of the scene.
11. `endRenderPass`, end the command buffer.
12. `submit` — waits on `m_ImageAvailableSemaphore`, signals `m_RenderFinishedSemaphore`, and
    signals `m_InFlightFence` when the GPU completes.
13. `presentKHR` — waits on `m_RenderFinishedSemaphore`, then presents the image.

### Vulkan ↔ ImGui interplay

ImGui is deliberately split into a **build** half and a **record** half:

- The **build** half — `newFrame()` plus every `ImGui::` widget call in
  `renderCameraControls` / `renderUI` — runs in `mainLoop` and touches only CPU-side ImGui
  state, emitting a draw-data list. Widgets bind directly to simulation/renderer fields, so
  the panel both displays and edits live state.
- The **record** half — `ImGui_ImplVulkan_RenderDrawData` via `renderDrawData` — runs inside
  `drawFrame`'s render pass and turns that draw-data list into Vulkan draw calls appended to
  the frame's command buffer.

Because ImGui records into the *same* command buffer and render pass as the scene, the UI is
simply the last thing drawn — it composites on top with no extra pass or command buffer. The
ImGui Vulkan backend owns its own descriptor pool for font/texture descriptors; on shutdown
`ImGui_ImplVulkan_Shutdown()` must run **before** that pool is destroyed (it frees descriptor
sets from it), and the `ImGuiHandler` destructor is ordered accordingly.

### Synchronization model

- **One frame in flight** — a single `m_InFlightFence` and one
  `m_ImageAvailableSemaphore` / `m_RenderFinishedSemaphore` pair. Frame N+1 cannot start GPU
  work until frame N's fence signals; that fence is the linchpin of the "buffer updates only
  after the wait" rule above.
- **Within a frame** — a compute→vertex-input memory barrier orders the warp compute's
  vertex-buffer writes before the draw's vertex fetch.
- **The membrane** ping-pongs two storage buffers: frame N writes buffer B while reading
  buffer A. The barrier (extended to also cover `eShaderRead` by the compute stage) plus the
  per-frame fence make B's writes visible to frame N+1, which then reads B and writes A.

## Controls

- **Geometry Selector**: Switch between Flat, Spherical, and Hyperbolic in real time
- **Use GPU Grid**: Toggle between the high-performance compute shader path and the parallel CPU implementation
- **Body Merging**: Toggle inelastic merging (accretion) of bodies that collide
- **Camera Controls**: Orbit, zoom, pan, and adjust FOV (near/far planes auto-fit the scene)
- **Simulation Parameters**: Gravity strength, time step, orbit speed factor, etc.
- **Warp & Membrane**: Well depth, warp gain, radial influence, and the membrane's stiffness / wave speed / damping
- **Light Rays**: pick **Ray Mode** (Dynamic — live field; Static — frozen orbits, animated rays), Emit / Clear a ray fan, toggle Lensing, and tune ray count, speed, and lens strength

## Building

### Requirements

- C++23 compiler (the code uses `std::print` and other C++23 features; `CMAKE_CXX_STANDARD` is 23) —
  Apple Clang, GCC, or MSVC 19.4x+ (Visual Studio 2022 17.10+)
- Vulkan SDK (LunarG on Windows/Linux; on macOS this provides MoltenVK + `glslc`)
- GLFW and GLM
- Dear ImGui (vendored in `src/imgui/`)
- OpenMP (libomp on macOS via Homebrew; built into MSVC and GCC)

### macOS / Linux

```bash
git clone <repo-url>
cd GravitySimulator
mkdir build && cd build
cmake ..
make -j
```

On macOS you may need to point CMake at Homebrew's `libomp`:

```bash
cmake -DOpenMP_C_LIB_NAMES="omp" -DOpenMP_CXX_LIB_NAMES="omp" ..
```

GLFW and GLM are expected on the compiler's default search path
(e.g. `brew install glfw glm`).

### Windows (MSVC + vcpkg)

Windows uses a **native** Vulkan driver (no MoltenVK) and [vcpkg](https://github.com/microsoft/vcpkg)
to supply GLFW and GLM. The `vcpkg.json` manifest in the repo root lists those
dependencies, so vcpkg installs them automatically on the first CMake configure.

1. **Vulkan SDK** — install the [LunarG SDK](https://vulkan.lunarg.com/sdk/home#windows)
   (headers, `vulkan-1.lib`, the `glslc` shader compiler, validation layers). It sets the
   `VULKAN_SDK` environment variable.
   ```powershell
   winget install --id KhronosGroup.VulkanSDK
   ```
   Restart your shell/IDE afterward so `VULKAN_SDK` is visible (or bake it into the preset below).
2. **Visual Studio 2022** with the *Desktop development with C++* workload (MSVC + Windows SDK).
3. **vcpkg** — a bootstrapped checkout. CLion ships one under `~/.vcpkg-clion/vcpkg`; otherwise
   `git clone https://github.com/microsoft/vcpkg && .\vcpkg\bootstrap-vcpkg.bat`.
4. **`CMakeUserPresets.json`** — create this in the repo root. It is **git-ignored** because it
   holds your machine's absolute paths. Copy the template below and edit the two paths
   (`CMAKE_TOOLCHAIN_FILE` → your vcpkg, `VULKAN_SDK` → your SDK version):
   ```json
   {
     "version": 3,
     "configurePresets": [
       {
         "name": "windows-release",
         "generator": "Ninja",
         "binaryDir": "${sourceDir}/cmake-build-release",
         "cacheVariables": {
           "CMAKE_BUILD_TYPE": "Release",
           "CMAKE_TOOLCHAIN_FILE": "C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
         },
         "environment": { "VULKAN_SDK": "C:/VulkanSDK/<version>" }
       }
     ]
   }
   ```
   Add a matching `windows-debug` preset (`CMAKE_BUILD_TYPE: Debug`, `cmake-build-debug`) if you
   want a debug profile. Baking `VULKAN_SDK` into the preset means CMake finds Vulkan even if your
   IDE was launched before the SDK was installed.
5. **Configure & build.** From a *Developer PowerShell for VS* (so `cl.exe` is on `PATH`):
   ```powershell
   cmake --preset windows-release
   cmake --build cmake-build-release --target GravitySimulator
   ```
   Or just open the folder in **CLion** / **Visual Studio**, which provide the MSVC environment
   automatically — in CLion, enable the preset under
   *Settings → Build, Execution, Deployment → CMake* and Run.

> **Benchmarking note:** always use the **Release** preset. A Debug MSVC build is several times
> slower (no optimization + checked-iterator overhead in the OpenMP hot loops), which makes
> CPU-side frame costs dominate and is not representative of real performance.

### Selecting the GPU

`VulkanApp::pickPhysicalDevice()` prefers a **discrete** GPU and prints its choice
(`Selected GPU: ...`) at startup. On a laptop with switchable graphics this prevents the sim from
running on a weak integrated GPU. If you have several GPUs and want a specific one, that function
is where to change the selection.

## Development Notes

This project evolved from a desire to explore robust, production-grade Vulkan compute shader programming while building something visually compelling. Early development included surviving (and learning from) a full GPU device loss caused by unsafe storage buffer writes. The experience drove the adoption of strict safety gates, validation layers, and careful descriptor/pipeline barrier discipline.

The final result is a stable, high-performance demonstration of how modern GPUs can dramatically outperform even well-threaded CPU implementations for grid-based gravitational visualization.

## Known Limitations & Cleanup Backlog

The simulator is fully working; the items below are known rough edges, kept here so they
aren't forgotten (see the in-repo review notes for detail):

- **`GridRenderer` is still the largest class** (~1200 lines, down from ~1900). The N-body
  physics (`Simulation`), the orbit camera (`CameraController`), and Vulkan buffer ownership
  (`VulkanBuffer`) have been extracted into their own units. What remains is genuinely
  rendering: pipelines, the compute path, depth resources, draw, and the controls panel —
  the panel could still move into its own type.
- **Single frame in flight** — one fence + one semaphore pair; frames cannot overlap. The
  per-frame GPU-buffer updates are now correctly ordered after the fence wait, so moving to
  N frames in flight mainly needs per-frame command buffers, sync objects, and bodies buffers.
- **Shader discovery** — `readFile` probes a hard-coded list of candidate paths; shaders
  should instead be copied next to the executable at build time.
- **Validation layers are disabled** — despite the bring-up history, there is no debug build
  toggle to re-enable them.

## Future Directions

- Larger particle counts and a dedicated accretion-disk preset (merging already models coalescence)
- A CPU-path dynamic membrane to match the GPU rippling grid
- Improved lighting and shading on the deformed grid
- Export of simulation data for analysis
- Further optimization of the CPU path and exploration of GPU-accelerated N-body integration

---

Built with Vulkan, ImGui, and a healthy respect for the dangers of raw compute shaders.