//
// Created by Richard Mule on 3/22/25.
//

#pragma once

#include <vulkan/vulkan.hpp>
#include <array>
#include <vector>
#include <memory>

#include "Geometry.hpp"
#include "VulkanBuffer.hpp"
#include "CameraController.hpp"
#include "Simulation.hpp"
#include <glm/gtc/type_ptr.hpp>


class GridRenderer
{
public:
    GridRenderer(
        const vk::Device& device,
        const vk::PhysicalDevice& physicalDevice,
        const vk::CommandPool& commandPool,
        const vk::Queue& graphicsQueue,
        const vk::RenderPass& renderPass,
        const std::vector<vk::Image>& swapchainImages,
        const vk::Extent2D& swapchainExtent,
        Simulation& simulation);
    ~GridRenderer();

    void init();
    void draw(vk::CommandBuffer commandBuffer) const;
    void updateCamera();
    void updateGrid(); // New method to warp grid
    void updateSimulation(float deltaTime);
    void renderCameraControls();
    void createDepthResources();

    // === GPU Grid Toggle ===
    // GPU is now the default high-performance path for grid warping.
    // When adding new compute shaders or changing dispatch/descriptor logic, validate first
    // with small dispatches + bounds checks + GPU-Assisted Validation before scaling up.
    void setUseGPUGrid(bool enabled);
    bool getUseGPUGrid() const { return m_UseGPUGrid; }

    void setVisualTuning(const VisualTuning& tuning) { m_Visual = tuning; }

    void updateBodiesBuffer();   // populates the bodies SSBO for the compute shader

    // Ensures the compute descriptor set has valid bindings for both the vertex buffer (0)
    // and bodies buffer (1). Call this after the descriptor set exists and whenever
    // the underlying buffers change.
    void ensureComputeDescriptors();

    void recordComputeWork(vk::CommandBuffer commandBuffer);

    vk::Format getDepthFormat() const { return m_DepthFormat; }
    const std::vector<vk::ImageView>& getDepthImageViews() const { return m_DepthImageViews; }

private:
    // Grid dimensions are cached from the Simulation in the constructor (it is
    // the single source of truth); the defaults here are just placeholders.
    int m_GridSize = 800;
    float m_GridScale = 320.0f;
    float m_VelocityAngle = 0.0f;   // unused diagnostic slider

    // Grid-warp (gravitational-well visualisation) dials — UI-tunable.
    WarpParams m_Warp;
    VisualTuning m_Visual;          // new tuning struct for purely visual parameters
    float m_RecenterOffset = 0.0f;  // mean warp depth, recomputed each frame

    // Rolling history for the conserved-quantity plots in the controls panel.
    static constexpr int kHistorySize = 180;
    std::array<float, kHistorySize> m_EnergyHistory{};
    std::array<float, kHistorySize> m_AngMomHistory{};
    std::array<float, kHistorySize> m_MembraneEnergyHistory{};   // new
    int m_HistoryHead = 0;

    // The CPU N-body simulation — owned by VulkanApp and referenced here, so
    // it outlives this renderer across swapchain recreation (window resize).
    Simulation& m_Simulation;

    // GPU grid deformation is now the default (high-performance path).
    // The checkbox in the Controls window can still toggle back to CPU fallback for comparison.
    bool m_UseGPUGrid = true;

    // Bodies buffer for GPU compute (position + mass), uploaded each frame.
    VulkanBuffer m_BodiesBuffer;

    glm::vec3 m_CenterOfMass = glm::vec3(0.0f);  // computed when GPU mode is active

    // Dynamic-membrane state: one (height, velocity) per grid vertex. Two
    // buffers are ping-ponged so the wave Laplacian reads a consistent
    // snapshot — m_MembraneRead indexes the buffer the compute shader reads;
    // it writes the other, then recordComputeWork() swaps the index.
    VulkanBuffer m_MembraneBuffer[2];
    int m_MembraneRead = 0;
    void createMembraneBuffers();   // allocates + zeroes both membrane buffers

    std::shared_ptr<Geometry> m_Geometry;
    GeometryType m_CurrentGeometryType;
    vk::Device m_Device;
    vk::PhysicalDevice m_PhysicalDevice;
    vk::CommandPool m_CommandPool;
    vk::Queue m_GraphicsQueue;
    vk::RenderPass m_RenderPass;
    std::vector<vk::Image> m_SwapchainImages;
    vk::Extent2D m_SwapchainExtent;

    std::vector<Vertex> m_Vertices;
    std::vector<uint32_t> m_Indices;
    VulkanBuffer m_VertexBuffer;
    VulkanBuffer m_IndexBuffer;
    vk::PipelineLayout m_PipelineLayout;
    vk::Pipeline m_GraphicsPipeline, m_TrianglePipeline;

    // Compute pipeline for GPU-driven grid (positions + warping)
    vk::PipelineLayout m_ComputePipelineLayout;
    vk::Pipeline m_ComputePipeline;
    vk::DescriptorSetLayout m_ComputeDescriptorSetLayout;
    vk::DescriptorPool m_ComputeDescriptorPool;
    vk::DescriptorSet m_ComputeDescriptorSet;
    PushConstants m_PushConstants;
    CameraController m_CameraController;
    std::vector<vk::Image> m_DepthImages;
    std::vector<vk::DeviceMemory> m_DepthImageMemory;
    std::vector<vk::ImageView> m_DepthImageViews;
    vk::Format m_DepthFormat;

    Object m_GridObject;
    std::vector<Vertex> m_TrailVertices;
    std::vector<uint32_t> m_TrailIndices;
    VulkanBuffer m_TrailVertexBuffer;
    VulkanBuffer m_TrailIndexBuffer;

    void generateGrid();
    void createVertexBuffer();
    void createIndexBuffer();
    void createGraphicsPipeline();
    void createComputePipeline();   // GPU grid generation / warping
    void updateGeometry(GeometryType type);
    void updateTrails();

    // Allocates a VulkanBuffer on this renderer's device / physical device.
    VulkanBuffer makeBuffer(vk::DeviceSize size,
                            vk::BufferUsageFlags usage,
                            vk::MemoryPropertyFlags properties) const;
    void copyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer, vk::DeviceSize size);
    vk::CommandBuffer beginSingleTimeCommands() const;
    void endSingleTimeCommands(vk::CommandBuffer commandBuffer) const;

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
};
