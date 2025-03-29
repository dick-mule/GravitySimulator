//
// Created by Richard Mule on 3/23/25.
//

#pragma once

#include "Types.hpp" // Include shared types
#include "utils/ShaderLoader.hpp"
#include <unordered_map>

enum class GeometryType { Flat, Spherical, Hyperbolic };

std::ostream& operator<<(std::ostream& os, const GeometryType& type);

struct ShaderProgram
{
    vk::ShaderModule vertexShader;
    vk::ShaderModule fragmentShader;
};

class GeometryShader
{
protected:
    vk::Device m_Device;
    vk::PhysicalDevice m_PhysicalDevice;
    std::unordered_map<GeometryType, ShaderProgram> m_ShaderPrograms;

    // GPU buffers (to be set by the renderer)
    vk::CommandPool m_CommandPool;
    vk::Buffer m_VertexBuffer;
    vk::DeviceMemory m_VertexBufferMemory;
    vk::Buffer m_IndexBuffer;
    vk::DeviceMemory m_IndexBufferMemory;
    vk::Queue m_ComputeQueue;
    vk::Pipeline m_ComputePipeline;
    vk::PipelineLayout m_PipelineLayout;
    vk::DescriptorSetLayout m_DescriptorSetLayout;
    vk::DescriptorSet m_DescriptorSet;

public:
    GeometryShader(
        const vk::Device& device,
        const vk::PhysicalDevice& physicalDevice);
    ~GeometryShader();

private:
    void loadShader(
        GeometryType type,
        const std::string& vertexPath,
        const std::string& fragmentPath);

public:
    void initializeAll();
    [[nodiscard]] const ShaderProgram& getShaderProgram(GeometryType type) const;
    [[nodiscard]] const vk::Device& getDevice() const { return m_Device; }

    // Set GPU buffers (called by the renderer)
    void setGPUBuffers(
        const vk::Buffer& vertexBuffer,
        const vk::DeviceMemory& vertexBufferMemory,
        const vk::Buffer& indexBuffer,
        const vk::DeviceMemory& indexBufferMemory);

    // Set compute pipeline (set by renderer)
    void setComputePipeline(
        const vk::CommandPool& commandPool,
        const vk::Queue& computeQueue,
        const vk::Pipeline& pipeline,
        const vk::PipelineLayout& pipelineLayout,
        const vk::DescriptorSetLayout& descriptorSetLayout,
        const vk::DescriptorSet& descriptorSet);

    [[nodiscard]] vk::CommandBuffer beginSingleTimeCommands() const;
    void endSingleTimeCommands(vk::CommandBuffer commandBuffer) const;

    void dispatchComputeShader(size_t gridSize, float gridScale) const;

    // New method to copy GPU buffers to CPU
    void copyBuffersToCPU(
        std::vector<Vertex>& vertices,
        std::vector<uint32_t>& indices,
        size_t vertexCount,
        size_t indexCount) const;

    // Buffer management methods
    [[nodiscard]] vk::Buffer createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage) const;
    [[nodiscard]] vk::DeviceMemory allocateBufferMemory(
        vk::Buffer buffer,
        vk::MemoryPropertyFlags properties) const;
    static void copyBuffer(
        const vk::Device& device,
        const vk::CommandPool& commandPool,
        const vk::Queue& queue,
        const vk::Buffer& srcBuffer,
        const vk::Buffer& dstBuffer,
        const vk::DeviceSize& size);

    // Getters for rendering
    [[nodiscard]] const vk::Buffer& getVertexBuffer() const { return m_VertexBuffer; }
    [[nodiscard]] const vk::Buffer& getIndexBuffer() const { return m_IndexBuffer; }
};


class Geometry
{
protected:
    int m_GridSize;
    float m_GridScale, m_WarpStrength;
    std::shared_ptr<GeometryShader> m_Shader;
    GeometryType m_Type = GeometryType::Flat;

public:
    Geometry(int grid_size, float grid_scale, float warp_strength = 1.);
    virtual ~Geometry() = default;
    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) const;
    [[nodiscard]] virtual float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const = 0;
    virtual void updatePosition(Object& obj, float deltaTime, float radius, bool verlet_half) const = 0;
    virtual void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) = 0;
    [[nodiscard]] size_t vertexCount() const { return (m_GridSize + 1) * (m_GridSize + 1); }
    [[nodiscard]] virtual size_t indexCount() const = 0;
    void setShader(const std::shared_ptr<GeometryShader>& shader) { m_Shader = shader; }
    void setGridParams(int grid_size, float grid_scale) { m_GridSize = grid_size; m_GridScale = grid_scale; }
    void setWarpStrength(float warp_strength) { m_WarpStrength = warp_strength; }
};

class FlatGeometry : public Geometry
{
protected:
    GeometryType m_Type = GeometryType::Flat;
public:
    using Geometry::Geometry;
    ~FlatGeometry() override = default;

    [[nodiscard]] float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) override;
    [[nodiscard]] size_t indexCount() const override { return m_GridSize * m_GridSize * 4; }
};

class SphericalGeometry : public Geometry
{
protected:
    GeometryType m_Type = GeometryType::Spherical;
public:
    using Geometry::Geometry;
    ~SphericalGeometry() override = default;

    [[nodiscard]] float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) override;
    [[nodiscard]] size_t indexCount() const override { return (m_GridSize - 2) * (2 * m_GridSize - 1) * 2; }
};

class HyperbolicGeometry : public Geometry
{
protected:
    GeometryType m_Type = GeometryType::Hyperbolic;
public:
    using Geometry::Geometry;
    ~HyperbolicGeometry() override = default;

    [[nodiscard]] float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) override;
    [[nodiscard]] size_t indexCount() const override { return m_GridSize * m_GridSize * 4; }
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