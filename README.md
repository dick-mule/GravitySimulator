# GravitySimulator

A real-time, GPU-accelerated gravity simulator featuring a deformable visualization grid rendered with Vulkan. The project demonstrates high-performance compute shaders, physically faithful gravitational warping across multiple geometries, and a clean comparison between GPU and multi-threaded CPU implementations.

## Features

- **Three Spatial Geometries**
  - **Flat**: Classic Newtonian-style gravitational wells
  - **Spherical**: Gravity visualized on the surface of a sphere (with proper geodesic distances and tangent-plane projections)
  - **Hyperbolic**: Gravity on a saddle/paraboloid surface with characteristic negative curvature warping

- **GPU-Accelerated Grid Deformation**
  - Custom Vulkan compute shader (`flat_grid.comp`) that generates the base surface and applies per-geometry gravitational warping in real time
  - Bodies are uploaded to the GPU each frame via storage buffers
  - Full pipeline barriers and descriptor management for safe, high-performance execution

- **High-Performance CPU Fallback**
  - OpenMP-parallelized warping on all three geometries
  - OpenMP-parallelized N-body Verlet integration for the underlying massive body simulation
  - Useful for direct performance comparison and validation

- **Interactive N-Body Simulation**
  - 41 massive bodies (1 dominant central mass + 40 orbiting bodies with randomized masses)
  - Velocity Verlet integration with geometry-aware distance and force calculations
  - Stable circular orbits with proper initial conditions for each geometry

- **Scientific Visualization UI**
  - Clean, dark-themed interface with live performance metrics
  - Real-time switching between geometries and GPU/CPU rendering modes
  - Camera controls, simulation parameters, and diagnostic information

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

- **Per-Geometry Warping**
  - Each geometry has its own base surface generation + distinct gravitational warping implementation inside the compute shader
  - Faithful port of the CPU reference math (including visual tuning factors such as radius scaling on the sphere)

- **Robust Development Practices**
  - Early development used strict safety gates (`m_UseGPUGrid`) after an initial GPU hang caused by out-of-bounds writes in the compute shader
  - Comprehensive bounds checking, descriptor validation, and debug logging during bring-up

- **Cross-Platform**
  - Primary development on macOS (MoltenVK)
  - Designed for easy portability to Windows and Linux via Vulkan

## Controls

- **Geometry Selector**: Switch between Flat, Spherical, and Hyperbolic in real time
- **Use GPU Grid**: Toggle between the high-performance compute shader path and the parallel CPU implementation
- **Camera Controls**: Orbit, zoom, pan, and adjust FOV
- **Simulation Parameters**: Gravity strength, time step, orbit speed factor, etc.

## Building

### Requirements

- C++17 compiler
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

## Future Directions

- Larger particle counts and more sophisticated accretion disk models
- Improved lighting and shading on the deformed grid
- Export of simulation data for analysis
- Further optimization of the CPU path and exploration of GPU-accelerated N-body integration

---

Built with Vulkan, ImGui, and a healthy respect for the dangers of raw compute shaders.