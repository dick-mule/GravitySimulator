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
  - Speed is renormalized each step, so the path stays a constant-speed (null) geodesic; rays ride the warped surface so each deflection's cause is visible

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
  - With lensing enabled it also feels a `∝ mass/dist²` deflection; speed is renormalized each
    step so it bends without speeding up — a true null geodesic
  - Each traced point is lifted onto the warped surface **once**, when added, so a dense fan of
    rays costs only one lift per ray per frame rather than re-lifting the whole trail history

- **Robust Development Practices**
  - Early development used strict safety gates (`m_UseGPUGrid`) after an initial GPU hang caused by out-of-bounds writes in the compute shader
  - Comprehensive bounds checking, descriptor validation, and debug logging during bring-up

- **Cross-Platform**
  - Primary development on macOS (MoltenVK)
  - Designed for easy portability to Windows and Linux via Vulkan

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

### Per-frame flow (`VulkanApp::mainLoop` → `drawFrame`)

1. `updateCamera` — apply mouse/keyboard input to the orbit camera.
2. `updateSimulation` — advance the N-body system one Verlet step (CPU/OpenMP), apply
   merges, then advance any traced light rays one geodesic step.
3. If GPU mode: `updateBodiesBuffer` uploads body positions/masses to an SSBO.
4. `updateGrid` — CPU mode warps + re-uploads the vertex buffer; GPU mode is a no-op here.
5. `recordComputeWork` — GPU mode dispatches the warping compute shader into the frame's
   command buffer, with a compute→vertex-input memory barrier.
6. `draw` — render the grid, bodies, and motion trails; ImGui draws on top.

## Controls

- **Geometry Selector**: Switch between Flat, Spherical, and Hyperbolic in real time
- **Use GPU Grid**: Toggle between the high-performance compute shader path and the parallel CPU implementation
- **Body Merging**: Toggle inelastic merging (accretion) of bodies that collide
- **Camera Controls**: Orbit, zoom, pan, and adjust FOV (near/far planes auto-fit the scene)
- **Simulation Parameters**: Gravity strength, time step, orbit speed factor, etc.
- **Warp & Membrane**: Well depth, warp gain, radial influence, and the membrane's stiffness / wave speed / damping
- **Light Rays**: Emit / clear a ray fan, toggle lensing, and tune ray count, speed, and lens strength

## Building

### Requirements

- C++23 compiler (the code uses `std::print` and other C++23 features; `CMAKE_CXX_STANDARD` is set to 23)
- Vulkan SDK
- GLFW
- GLM
- Dear ImGui (included in `src/imgui/`)
- OpenMP (libomp on macOS via Homebrew recommended)

### Build Steps

```bash
git clone <repo-url>
cd GravitySimulator
mkdir build && cd build
cmake ..
make -j
```

On macOS, you may need to explicitly link against Homebrew's `libomp`:

```bash
cmake -DOpenMP_C_LIB_NAMES="omp" -DOpenMP_CXX_LIB_NAMES="omp" ..
```

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