//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#include "Types.hpp"

#include "../utils/ShaderLoader.hpp"
#include "VkResourceManager.hpp"
#include <unordered_map>
#include <ranges>
#include <sstream>

enum class GeometryType : int { Flat, Spherical, Hyperbolic };

std::ostream& operator<<(std::ostream& os, const GeometryType& type);

class GeometryShader
{
public:
    GeometryShader(std::shared_ptr<VkResourceManager> resourceManager);
    ~GeometryShader();

    void initializeAll();
    void dispatchShader(size_t grid_size, float grid_scale);
    [[nodiscard]] const ShaderProgram& getShaderProgram(GeometryType type) const;
    [[nodiscard]] std::shared_ptr<VkResourceManager> operator->() const { return m_ResourceManager; }
private:
    void loadShader(
        GeometryType type,
        const std::string& vertexPath,
        const std::string& fragmentPath);

protected:
    std::shared_ptr<VkResourceManager> m_ResourceManager;
    std::unordered_map<GeometryType, ShaderProgram> m_ShaderPrograms;

};

class Geometry
{
public:
    Geometry(
        std::shared_ptr<VkResourceManager> resourceManager,
        GeometryType geometryType,
        int grid_size,
        float grid_scale,
        float warp_strength = 1.);
    virtual ~Geometry() = default;

protected:
    std::shared_ptr<GeometryShader> m_Shader;
    GeometryType m_GeometryType;
    int m_GridSize;
    float m_GridScale, m_WarpStrength;
    std::vector<Vertex> m_GridVertices;
    std::vector<uint32_t> m_GridIndices;

protected:
    void bindVertices();
    void bindIndices();
public:
    void createGeometryPipeline();
    void generateGrid() const;
    [[nodiscard]] virtual float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const = 0;
    virtual void updatePosition(Object& obj, float deltaTime, float radius, bool verlet_half) const = 0;
    virtual void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) = 0;
    [[nodiscard]] size_t vertexCount() const
    {
        if ( m_GridVertices.empty() )
            return (m_GridSize + 1) * (m_GridSize + 1);
        return m_GridVertices.size();
    }
    [[nodiscard]] virtual size_t indexCount() const = 0;
    void setShader(const std::shared_ptr<GeometryShader>& shader) { m_Shader = shader; }
    void setGridParams(int grid_size, float grid_scale) { m_GridSize = grid_size; m_GridScale = grid_scale; }
    void setWarpStrength(float warp_strength) { m_WarpStrength = warp_strength; }
    const std::shared_ptr<GeometryShader>& shader() { return m_Shader; }
    const std::shared_ptr<GeometryShader>& operator->() { return m_Shader; }

    const std::vector<Vertex>& vertices() { return m_GridVertices; }
    const std::vector<uint32_t>& indices() { return m_GridIndices; }
    const vk::Buffer& vertexBuffer() const { return (*m_Shader)->wrappedBuffers(ResourceTypes::VERTEX).buffer; }
    const vk::Buffer& indexBuffer() const { return (*m_Shader)->wrappedBuffers(ResourceTypes::INDEX).buffer; }
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

std::shared_ptr<Geometry> geometryFactory(
    std::shared_ptr<VkResourceManager> resource_manager,
    GeometryType type,
    int grid_size,
    float grid_scale);
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