//
// CPU N-body gravitational simulation: bodies, integration, energy, trails.
//

#pragma once

#include "Types.hpp"
#include "Geometry.hpp"

#include <deque>
#include <memory>
#include <vector>

// A motion trail: the recent positions of one body.
struct Trail
{
    std::deque<glm::vec3> positions;
    static constexpr size_t maxPoints = 100;
};

// A null geodesic: a massless light ray that travels at constant speed along a
// geodesic of the active manifold. With lensing enabled it is also deflected by
// the masses (light-bending); with it off the path is a pure geodesic, so a fan
// of rays reveals the manifold's curvature (geodesic deviation).
struct LightRay
{
    glm::vec3 position{0.0f};
    glm::vec3 direction{1.0f, 0.0f, 0.0f};   // unit tangent to the surface
    std::deque<glm::vec3> trail;
    bool active = true;                      // false once it leaves the grid
    static constexpr size_t maxPoints = 600;
};

// The gravitational simulation, extracted from GridRenderer: the massive
// bodies, their velocity-Verlet integration on the active geometry, conserved-
// energy bookkeeping, and motion trails. Pure CPU — owns no Vulkan resources.
class Simulation
{
public:
    // Defaults double as the single source of truth for the grid dimensions;
    // GridRenderer reads them back via gridSize() / gridScale().
    Simulation(GeometryType type = GeometryType::Flat,
               int gridSize = 800,
               float gridScale = 320.0f,
               float gravity = 0.2f);

    // Advance the simulation by one frame (the step is clamped to timeStep()).
    void step(float deltaTime);

    // Switch geometry, converting body + trail coordinates onto the new one.
    void switchGeometry(GeometryType type);

    // Body-set presets driven by the controls panel.
    void resetTwoBody();   // "Reset Simulation": a cube + a sphere
    void resetToOrbit();   // re-seed a clean two-body orbit

    // Null-geodesic (light-ray) demo: emit a fan of parallel rays appropriate
    // to the active geometry, or clear them.
    void emitRays();
    void clearRays();

    // GridRenderer pushes the current warp config here each frame so newly
    // traced ray points can be lifted onto the warped surface once, when
    // added — instead of re-lifting the whole trail history every frame.
    void setRayWarp(const WarpParams& warp, float recenterOffset)
    {
        m_RayWarp = warp;
        m_RayRecenter = recenterOffset;
    }

    // --- Queries -----------------------------------------------------------
    // The bodies are returned by const reference, but each Shape behind the
    // shared_ptr stays mutable (the per-body UI edits mass/velocity/position).
    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& bodies() const { return m_Bodies; }
    [[nodiscard]] const std::vector<Trail>& trails() const { return m_Trails; }
    [[nodiscard]] const std::vector<LightRay>& rays() const { return m_Rays; }
    [[nodiscard]] const std::shared_ptr<Geometry>& geometry() const { return m_Geometry; }
    [[nodiscard]] GeometryType geometryType() const { return m_GeometryType; }
    [[nodiscard]] int   gridSize()  const { return m_GridSize; }
    [[nodiscard]] float gridScale() const { return m_GridScale; }

    [[nodiscard]] float kineticEnergy() const    { return m_KineticEnergy; }
    [[nodiscard]] float potentialEnergy() const  { return m_PotentialEnergy; }
    [[nodiscard]] float totalEnergy() const      { return m_TotalEnergy; }
    [[nodiscard]] float angularMomentum() const  { return m_AngularMomentum; }

    // True (once) if step() merged bodies this frame — the renderer must then
    // rebuild its grid/body buffers. Reading it clears the flag.
    [[nodiscard]] bool consumeBodiesChanged()
    {
        const bool changed = m_BodiesChanged;
        m_BodiesChanged = false;
        return changed;
    }

    // Mutable handles so the ImGui sliders / checkboxes can bind directly.
    [[nodiscard]] float& gravity()        { return m_Gravity; }
    [[nodiscard]] float& timeStep()       { return m_TimeStep; }
    [[nodiscard]] bool&  mergeEnabled()   { return m_MergeEnabled; }
    [[nodiscard]] bool&  lensingEnabled() { return m_LensingEnabled; }
    [[nodiscard]] int&   rayCount()       { return m_RayCount; }
    [[nodiscard]] float& rayLensStrength(){ return m_RayLensStrength; }
    [[nodiscard]] float& raySpeed()       { return m_RaySpeed; }
    [[nodiscard]] float  gravity() const  { return m_Gravity; }

private:
    void spawnBodies(GeometryType type);
    void advanceRays(float deltaTime);   // one geodesic step for every light ray
    glm::vec3 liftRayPoint(const glm::vec3& basePos) const;  // base -> warped surface

    std::vector<std::shared_ptr<Shape>> m_Bodies;
    std::vector<Trail>                  m_Trails;   // one per body
    std::shared_ptr<Geometry>           m_Geometry;
    GeometryType                        m_GeometryType;

    int   m_GridSize;
    float m_GridScale;
    float m_Gravity;
    float m_TimeStep    = 0.05f;
    float m_OrbitFactor = 1.0f;

    float m_KineticEnergy   = 0.0f;
    float m_PotentialEnergy = 0.0f;
    float m_TotalEnergy     = 0.0f;
    float m_AngularMomentum = 0.0f;

    bool  m_MergeEnabled  = true;   // bodies that touch coalesce (accretion)
    bool  m_BodiesChanged = false;  // a merge happened this step

    // Null-geodesic (light-ray) demo state.
    std::vector<LightRay> m_Rays;
    WarpParams m_RayWarp;             // warp config used to lift ray points
    float m_RayRecenter = 0.0f;       // matching re-centering offset
    bool  m_LensingEnabled  = true;   // deflect rays by the masses (vs pure geodesic)
    int   m_RayCount        = 200;     // rays per emitted fan
    float m_RayLensStrength = 0.05f;  // how strongly masses bend the rays
    float m_RaySpeed        = 30.0f;  // constant ray speed (world units / sec)
};
