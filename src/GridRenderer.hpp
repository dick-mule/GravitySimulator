//
// Created by Richard Mule on 3/22/25.
//

#pragma once

#include <vulkan/vulkan.hpp>
#include <vector>
#include <memory>

#include "Geometry.hpp"
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
        GeometryType geometryType);
    ~GridRenderer();

    void init();
    void draw(vk::CommandBuffer commandBuffer) const;
    void updateCamera();
    void updateGrid(); // New method to warp grid
    void updateSimulation(float deltaTime) const;
    void renderCameraControls();
    void createDepthResources();
    vk::Format getDepthFormat() const { return m_DepthFormat; }
    const std::vector<vk::ImageView>& getDepthImageViews() const { return m_DepthImageViews; }

private:
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
    vk::Buffer m_VertexBuffer;
    vk::DeviceMemory m_VertexBufferMemory;
    vk::Buffer m_IndexBuffer;
    vk::DeviceMemory m_IndexBufferMemory;
    vk::PipelineLayout m_PipelineLayout;
    vk::Pipeline m_GraphicsPipeline, m_LinePipeline, m_TrianglePipeline;
    PushConstants m_PushConstants;
    Camera m_Camera;
    std::vector<vk::Image> m_DepthImages;
    std::vector<vk::DeviceMemory> m_DepthImageMemory;
    std::vector<vk::ImageView> m_DepthImageViews;
    vk::Format m_DepthFormat;

    Object m_GridObject;
    std::vector<std::shared_ptr<Shape>> m_MassiveObjects;
    float m_ZoomLevel = 0.;
    float m_Gravity = 0.2f;
    int m_GridSize = 200;
    float m_GridScale = 100.0f;

    void generateGrid();
    void createVertexBuffer();
    void createIndexBuffer();
    void createGraphicsPipeline();
    void updateGeometry(GeometryType type);
    void addShape(const std::shared_ptr<Shape>& shape) { m_MassiveObjects.push_back(shape); }

    vk::Buffer createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage) const;
    vk::DeviceMemory allocateBufferMemory(vk::Buffer buffer, vk::MemoryPropertyFlags properties) const;
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
