//
// Created by Richard Mule on 3/22/25.
//

#pragma once
#include <memory>
#include <deque>
#include "Geometry.hpp"
#include <glm/gtc/type_ptr.hpp>

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
private:
    void makeCentralObject();
    void makeOrbiters();
    void initializeObjects();
public:
    explicit GridRenderer(
        const std::shared_ptr<VulkanResourceManager>& manager,
        GeometryType geometryType = GeometryType::Flat,
        const SimulationConfig& simulationConfig = {});
    ~GridRenderer();
    void init();
    void draw(vk::CommandBuffer commandBuffer) const;
    void updateCamera();
    // void updateGrid() const; // New method to warp grid
    void updateSimulation(float deltaTime);
    void renderCameraControls();
    void createDepthResources();
    void createComputePipeline();
    vk::Format getDepthFormat() const { return m_DepthFormat; }
    const std::vector<vk::ImageView>& getDepthImageViews() const { return m_DepthImageViews; }

private:
    std::shared_ptr<VulkanResourceManager> m_ResourceManager;
    std::shared_ptr<Geometry> m_Geometry;
    std::shared_ptr<GeometryShader> m_ShaderManager;
    GeometryType m_CurrentGeometryType;
    SimulationConfig m_SimulationConfig;
    std::vector<std::shared_ptr<Shape>> m_MassiveObjects;
    std::shared_ptr<Shape> m_CentralObj;
    std::vector<Trail> m_Trails; // One trail per object
    std::vector<Vertex> m_TrailVertices;
    std::vector<uint32_t> m_TrailIndices;
    vk::Buffer m_TrailVertexBuffer;
    vk::DeviceMemory m_TrailVertexBufferMemory;
    vk::Buffer m_TrailIndexBuffer;
    vk::DeviceMemory m_TrailIndexBufferMemory;
    vk::Fence m_TrailUpdateFence;
    uint32_t m_VertexCount = 0;
    uint32_t m_IndexCount = 0;

    void generateGrid();
    void createVertexBuffer();
    void createIndexBuffer();
    void createGraphicsPipeline();
    void generateObjects();
    void createObjectComputePipeline();
    void updateGeometry(GeometryType type);
    void addShape(const std::shared_ptr<Shape>& shape) { m_MassiveObjects.push_back(shape); }
    void updateTrails();

    vk::Buffer createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage) const;
    vk::DeviceMemory allocateBufferMemory(vk::Buffer buffer, vk::MemoryPropertyFlags properties) const;
    void copyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer, vk::DeviceSize size) const;
    vk::CommandBuffer beginSingleTimeCommands(const vk::CommandPool& commandPool) const;
    void endSingleTimeCommands(vk::CommandBuffer commandBuffer, const vk::Queue& queue) const;

    vk::Format findSupportedFormat(
        const std::vector<vk::Format>& candidates,
        const vk::ImageTiling& tiling,
        const vk::FormatFeatureFlags& features);
    vk::Format findDepthFormat();
    uint32_t findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties) const;
    void transitionImageLayout(
        const vk::Image& image,
        const vk::Format& format,
        const vk::ImageLayout& oldLayout,
        const vk::ImageLayout& newLayout);

    void copyBufferWithFence(const vk::Buffer srcBuffer, const vk::Buffer dstBuffer, const vk::DeviceSize size) const;

    friend class VulkanApp;
};

