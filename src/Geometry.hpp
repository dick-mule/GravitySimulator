//
// Created by Richard Mule on 3/23/25.
//

#pragma once

#include "Types.hpp" // Include shared types

enum class GeometryType { Flat, Spherical, Hyperbolic };

// =============================================================================
// Grid warping (gravitational-well visualisation)
//
// The warp is a unified, mass-driven model shared by all three geometries.
// Each body contributes  m04 / sqrt(rEff^2 + softening^2)  where:
//
//   m04   = pow(mass, 0.4)                        — compressed mass
//   width = 1 + radialInfluence * m04             — heavier bodies are wider
//   rEff  = dist / width                          — mass-scaled distance
//
// The contributions are summed, then  depth = wellDepth * tanh(warpGain * sum).
// Summing inside the tanh bounds the depth by wellDepth (a cluster cannot stack
// past one well), mass^0.4 keeps the central body's depth dominant, and
// `radialInfluence` widens heavy bodies so the central reads as a broad basin
// rather than a narrow cone. A smooth edge taper fades the warp to zero at the
// grid boundary; Spherical/Hyperbolic also subtract a re-centering offset (the
// mean warp depth) so the manifold does not drift under a global potential bias.
//
// On the GPU path the warp also drives a *dynamic membrane*: rather than the
// grid snapping to the well shape, each vertex carries a height + velocity and
// integrates a damped wave equation that chases the well shape as a forcing
// term — so moving bodies and merges radiate ripples across the sheet. The CPU
// fallback keeps the instant (static) warp.
// =============================================================================

// Tunable parameters for the grid-warp visualisation.
struct WarpParams
{
    float wellDepth       = 40.0f;   // saturated depth of the warp (world units)
    float warpGain        = 0.115f;  // how quickly depth saturates with mass
    float softening       = 1.0f;    // avoids a singular 1/d at a body's centre
    float radialInfluence = 0.045f;  // couples well width to mass (0 = uniform width)

    // Dynamic-membrane dials (GPU path only). The grid integrates
    //   accel = stiffness*(wellShape - h) + waveSpeed*laplacian(h) - damping*v
    // once per frame, in per-step units (no separate dt).
    bool  membraneEnabled   = true;  // false = instant warp (the old behaviour)
    float membraneStiffness = 0.10f; // spring pull toward the well shape
    float membraneWaveSpeed = 0.25f; // ripple propagation (CFL-stable below 0.5)
    float membraneDamping   = 0.06f; // bleeds energy so ripples settle
};

// Purely visual / debug tuning knobs that don't affect physics.
// These are passed through to the shader so you can tweak appearance
// without recompiling. Grouped here so they can be exposed in the UI.
struct VisualTuning
{
    float edgeFadeStart   = 0.85f;   // where the edge taper begins (0.0 = center, 1.0 = outer edge)
    float edgeFadeEnd     = 1.0f;
    float gridFrequency   = 18.0f;   // how many grid lines across the screen
    float gridIntensity   = 0.04f;   // strength of the background grid
};

class Geometry
{
protected:
    int m_GridSize;
    float m_GridScale;

    // Cached, un-warped base surface. warpGrid() reuses this instead of
    // regenerating the full (~640k-vertex) grid every frame. Invalidated
    // whenever the grid parameters change (see setGridParams).
    std::vector<Vertex> m_BaseGrid;
    bool m_BaseGridDirty = true;
    const std::vector<Vertex>& baseGrid();

public:
    Geometry(int grid_size, float grid_scale);
    virtual ~Geometry() = default;
    virtual void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) = 0;
    virtual float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const = 0;
    virtual void updatePosition(Object& obj, float deltaTime, float radius, bool verlet_half) const = 0;

    /// Total warp depth at a base-grid position: tanh of the summed potential
    /// of every body. The base implementation uses Euclidean (chord) distance;
    /// HyperbolicGeometry overrides it to measure along the paraboloid.
    virtual float warpDepth(const glm::vec3& basePos,
                            const std::vector<std::shared_ptr<Shape>>& bodies,
                            const WarpParams& warp) const;

    /// Smooth taper, 1 in the interior falling to 0 at the grid boundary, so
    /// the edges stay flat regardless of body positions. SphericalGeometry
    /// overrides it to 1 (a closed surface has no edges).
    virtual float edgeFade(const glm::vec3& basePos) const;

    /// Mean warp depth over a coarse sample of the base grid — the re-centering
    /// offset for Spherical / Hyperbolic. (Callers pass 0 for Flat.)
    float computeRecenterOffset(const std::vector<std::shared_ptr<Shape>>& bodies,
                                const WarpParams& warp);

    /// Warps `vertices` from the base grid using the model above.
    virtual void warpGrid(std::vector<Vertex>& vertices,
                          const std::vector<std::shared_ptr<Shape>>& bodies,
                          const WarpParams& warp,
                          float recenterOffset) = 0;

    /// Displaces a single base-surface point onto the warped surface, given its
    /// raw warp depth (from warpDepth()). The per-geometry displacement model —
    /// warpGrid() and the light-ray lift both go through this.
    virtual glm::vec3 warpedPosition(const glm::vec3& basePos, float rawDepth,
                                     float recenterOffset) const = 0;

    void setGridParams(int grid_size, float grid_scale)
    {
        // Only invalidate the cached base grid when the parameters actually
        // change — this is called every frame on the CPU warp path.
        if ( grid_size != m_GridSize || grid_scale != m_GridScale )
        {
            m_GridSize = grid_size;
            m_GridScale = grid_scale;
            m_BaseGridDirty = true;
        }
    }
};

class FlatGeometry : public Geometry
{
public:
    using Geometry::Geometry;
    ~FlatGeometry() override = default;

    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) override;
    float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& bodies, const WarpParams& warp, float recenterOffset) override;
    glm::vec3 warpedPosition(const glm::vec3& basePos, float rawDepth, float recenterOffset) const override;
};

class SphericalGeometry : public Geometry
{
public:
    using Geometry::Geometry;
    ~SphericalGeometry() override = default;

    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) override;
    float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const override;
    // The sphere is a closed surface — no edges to taper.
    float edgeFade(const glm::vec3& basePos) const override { return 1.0f; }
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& bodies, const WarpParams& warp, float recenterOffset) override;
    glm::vec3 warpedPosition(const glm::vec3& basePos, float rawDepth, float recenterOffset) const override;
};

class HyperbolicGeometry : public Geometry
{
public:
    using Geometry::Geometry;
    ~HyperbolicGeometry() override = default;

    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) override;
    float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const override;
    // Measures the warp distance along the paraboloid surface.
    float warpDepth(const glm::vec3& basePos,
                    const std::vector<std::shared_ptr<Shape>>& bodies,
                    const WarpParams& warp) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& bodies, const WarpParams& warp, float recenterOffset) override;
    glm::vec3 warpedPosition(const glm::vec3& basePos, float rawDepth, float recenterOffset) const override;
};

std::shared_ptr<Geometry> geometryFactory(GeometryType type, int grid_size, float grid_scale);
glm::vec3 convertCoordinates(
    const glm::vec3& coordinates,
    GeometryType start_type,
    GeometryType end_type,
    float radius = 1.0f,
    const std::shared_ptr<Geometry>& geometry = nullptr);
// Helper function to convert velocity
glm::vec3 convertVelocity(
    const glm::vec3& oldPos,
    const glm::vec3& oldVel,
    GeometryType start_type,
    GeometryType end_type,
    float radius,
    float dist,
    float mu,
    const std::shared_ptr<Geometry>& calculator = nullptr);