//
// Created by Richard Mule on 3/23/25.
//

#pragma once

#include "Types.hpp" // Include shared types

enum class GeometryType { Flat, Spherical, Hyperbolic };

class Geometry
{
protected:
    int m_GridSize;
    float m_GridScale;
public:
    Geometry(int grid_size, float grid_scale);
    virtual ~Geometry() = default;
    virtual void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) = 0;
    virtual float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const = 0;
    virtual void updatePosition(Object& obj, float deltaTime, float radius) const = 0;
    virtual void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) = 0;
    void setGridParams(int grid_size, float grid_scale) { m_GridSize = grid_size; m_GridScale = grid_scale; }
};

class FlatGeometry : public Geometry
{
public:
    using Geometry::Geometry;
    ~FlatGeometry() override = default;

    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) override;
    float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) override;
};

class SphericalGeometry : public Geometry
{
public:
    using Geometry::Geometry;
    ~SphericalGeometry() override = default;

    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) override;
    float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) override;
};

class HyperbolicGeometry : public Geometry
{
public:
    using Geometry::Geometry;
    ~HyperbolicGeometry() override = default;

    void generateGrid(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices, uint32_t gridSize, float scale) override;
    float computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const override;
    void updatePosition(Object& obj, float deltaTime, float radius) const override;
    void warpGrid(std::vector<Vertex>& vertices, const std::vector<std::shared_ptr<Shape>>& massiveObjects, float G, float maxDisplacement, float minDistSquared, float softeningLength) override;
};

std::shared_ptr<Geometry> geometryFactory(GeometryType type, int grid_size, float grid_scale);
glm::vec3 convertCoordinates(
    const glm::vec3& coordinates,
    GeometryType start_type,
    GeometryType end_type,
    float radius = 1.0f);