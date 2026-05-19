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

    // --- Queries -----------------------------------------------------------
    // The bodies are returned by const reference, but each Shape behind the
    // shared_ptr stays mutable (the per-body UI edits mass/velocity/position).
    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& bodies() const { return m_Bodies; }
    [[nodiscard]] const std::vector<Trail>& trails() const { return m_Trails; }
    [[nodiscard]] const std::shared_ptr<Geometry>& geometry() const { return m_Geometry; }
    [[nodiscard]] GeometryType geometryType() const { return m_GeometryType; }
    [[nodiscard]] int   gridSize()  const { return m_GridSize; }
    [[nodiscard]] float gridScale() const { return m_GridScale; }

    [[nodiscard]] float kineticEnergy() const   { return m_KineticEnergy; }
    [[nodiscard]] float potentialEnergy() const { return m_PotentialEnergy; }
    [[nodiscard]] float totalEnergy() const     { return m_TotalEnergy; }

    // Mutable handles so the ImGui sliders can bind directly.
    [[nodiscard]] float& gravity()  { return m_Gravity; }
    [[nodiscard]] float& timeStep() { return m_TimeStep; }
    [[nodiscard]] float  gravity() const { return m_Gravity; }

private:
    void spawnBodies(GeometryType type);

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
};
