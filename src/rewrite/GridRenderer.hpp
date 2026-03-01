//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#include <memory>
#include <deque>
#include "Geometry.hpp"
#include <glm/gtc/type_ptr.hpp>
#include "VkResourceManager.hpp"


struct SimulationConfig
{
    bool make_central_object = true;
    size_t n_orbiters = 5;
    float warp_strength = 1.0f;
    float zoom_level = 0.0f;
    float g_factor = 0.2f;
    int grid_size = 500;
    float grid_scale = 250.0f;
    float time_step = 0.05f;
    float max_displacement = 100.0f;
    float min_displacement = 0.0f;
    float softening_length = 1.0f;

    [[nodiscard]] float baseRadius() const { return grid_scale * 0.05; } // 5% of grid scale as starting radius
    [[nodiscard]] float radiusStep(const size_t i = 0) const { return i * grid_scale * 0.5 / n_orbiters;}  // Spread across 10% of grid
    [[nodiscard]] float angle(const size_t i = 0) const { return static_cast<float>(i) * 2.0 * glm::pi<float>() / n_orbiters; }
    [[nodiscard]] float radius(const size_t i = 0) const { return i * grid_scale; }

    WarpPushConstants warpConstants() const
    {
        return {
            static_cast<int>(n_orbiters + make_central_object),
            grid_size,
            grid_scale,
            warp_strength,
            max_displacement,
            min_displacement,
            softening_length,
            g_factor
        };
    }
};

struct SimulationState
{
    float TotalEnergy = 0.0f;
    float KineticEnergy = 0.0f;
    float PotentialEnergy = 0.0f;
};

struct Trail
{
    std::deque<glm::vec3> positions;
    static constexpr size_t maxPoints = 100; // Number of points in the trail
};

class GridRenderer 
{
public:
    explicit GridRenderer(
        std::shared_ptr<VkResourceManager> resourceManager,
        GeometryType geometryType = GeometryType::Flat,
        const SimulationConfig& simulationConfig = {});
    ~GridRenderer();

private:
    void makeCentralObject();
    void makeOrbiters();
    void initializeObjects();

public:
    void initialize() const;
    void draw();
    void updateCamera();

protected:
    std::shared_ptr<VkResourceManager> m_ResourceManager;
    std::shared_ptr<Geometry> m_Geometry;
    GeometryType m_GeometryType;
    SimulationConfig m_SimulationConfig;
    std::vector<std::shared_ptr<Shape>> m_MassiveObjects;
    std::vector<Trail> m_Trails; // One trail per object
    // PushConstants m_PushConstants;
    Camera m_Camera;

};
