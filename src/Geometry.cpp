//
// Created by Richard Mule on 3/23/25.
//

#include "Geometry.hpp"

#include <iostream>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/fast_trigonometry.hpp>

Geometry::Geometry(int grid_size, float grid_scale)
    : m_GridSize(grid_size)
    , m_GridScale(grid_scale)
{
}

void FlatGeometry::generateGrid(
    std::vector<Vertex>& vertices,
    std::vector<uint32_t>& indices,
    uint32_t gridSize,
    float scale)
{
    const float spacing = scale / gridSize;
    for ( int i = 0; i <= gridSize; ++i )
    {
        for ( int j = 0; j <= gridSize; ++j )
        {
            Vertex vertex{};
            vertex.position = glm::vec3(
                i * spacing - gridSize * spacing / 2.0f,
                0.0f,
                j * spacing - gridSize * spacing / 2.0f
            );
            vertex.color = glm::vec3(1.0f);
            vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            vertices.push_back(vertex);
        }
    }

    for ( int i = 0; i < gridSize; ++i )
    {
        for ( int j = 0; j < gridSize; ++j )
        {
            uint32_t idx = i * (gridSize + 1) + j;
            indices.push_back(idx); indices.push_back(idx + 1);
            indices.push_back(idx); indices.push_back(idx + (gridSize + 1));
        }
    }
}

float FlatGeometry::computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const
{
    return glm::length(pos2 - pos1);
}

void FlatGeometry::updatePosition(Object& obj, float deltaTime, float /*radius not used*/) const
{
    obj.velocity += obj.acceleration * deltaTime;
    glm::vec3 translation = obj.velocity * deltaTime;
    obj.modelMatrix[3] += glm::vec4(translation, 0.0f);
}

static std::string vecToString(const glm::vec3& vec)
{
    std::stringstream ss;
    ss << vec.x << " " << vec.y << " " << vec.z;
    return ss.str();
}

void FlatGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<glm::vec3>& massivePositions,
    const std::vector<float>& masses,
    float G,
    float maxDisplacement,
    float minDistSquared,
    float softeningLength)
{
    // Reset vertices to base positions
    std::vector<Vertex> baseVertices(0);
    std::vector<uint32_t> dummyIndices(0);
    generateGrid(baseVertices, dummyIndices, m_GridSize, m_GridScale); // Match m_GridSize and m_GridScale
    for ( Vertex& vertex : baseVertices )
    {
        float totalDisplacement = 0.0f;
        for ( size_t k = 0; k < massivePositions.size(); ++k )
        {
            if ( masses[k] <= 0.0f )
                continue;
            float distSquared = glm::length2(vertex.position - massivePositions[k]);
            if ( distSquared < minDistSquared )
                distSquared = minDistSquared;
            const float softenedDist = distSquared + softeningLength * softeningLength;
            const float displacement = G * masses[k] / softenedDist;
            totalDisplacement += displacement;
        }
        totalDisplacement = std::min(totalDisplacement, maxDisplacement);
        vertex.position.y -= totalDisplacement;
    }

    for ( size_t i = 0; i < baseVertices.size(); ++i )
        vertices[i].position = baseVertices[i].position;
}

void SphericalGeometry::generateGrid(
    std::vector<Vertex>& vertices,
    std::vector<uint32_t>& indices,
    uint32_t gridSize,
    float scale)
{
    constexpr float pi = glm::pi<float>();
    const float R = scale / 2.0f;

    // Generate vertices
    for ( int i = 0; i <= gridSize; ++i )
    {
        const float theta = pi * i / gridSize;
        for ( int j = 0; j <= gridSize; ++j )
        {
            const float phi = 2.0f * pi * j / gridSize;
            Vertex vertex{};
            vertex.position = glm::vec3(
                R * sin(theta) * cos(phi),
                R * sin(theta) * sin(phi),
                R * cos(theta)
            );
            vertex.color = glm::vec3(1.0f);
            vertex.normal = glm::normalize(vertex.position);
            vertices.push_back(vertex);
        }
    }

    // Generate indices as lines
    for ( int i = 0; i < gridSize; ++i )
    {
        for ( int j = 0; j < gridSize; ++j )
        {
            uint32_t idx = i * (gridSize + 1) + j;
            uint32_t idxRight = idx + 1;
            uint32_t idxBottom = idx + (gridSize + 1);

            // Horizontal lines (constant theta, varying phi) - latitude
            if ( i != 0 && i != gridSize - 1 ) // Skip poles
            {
                if ( j == gridSize - 1 )
                {
                    uint32_t idxFirstInRow = i * (gridSize + 1);
                    indices.push_back(idx);
                    indices.push_back(idxFirstInRow);
                }
                else
                {
                    indices.push_back(idx);
                    indices.push_back(idxRight);
                }
            }

            // Vertical lines (constant phi, varying theta) - longitude
            if ( j != gridSize - 1 ) // Skip the last column
            {
                // Only draw vertical lines between non-pole vertices
                if (i != 0 && i != gridSize - 1)
                {
                    indices.push_back(idx);
                    indices.push_back(idxBottom);
                }
            }
        }
    }

}

float SphericalGeometry::computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const
{
    const float R = glm::length(pos1);
    return R * acos(glm::dot(glm::normalize(pos1), glm::normalize(pos2)));
}

void SphericalGeometry::updatePosition(Object& obj, float deltaTime, float radius) const
{
    glm::vec3 pos = obj.modelMatrix[3];
    float theta = acos(pos.z / radius);
    float phi = atan2(pos.y, pos.x);

    obj.velocity += obj.acceleration * deltaTime;

    const float dTheta = obj.velocity.z / (radius * sin(theta));
    const float dPhi = (obj.velocity.x * cos(phi) + obj.velocity.y * sin(phi)) / (radius * sin(theta));

    theta += dTheta * deltaTime;
    phi += dPhi * deltaTime;
    theta = glm::clamp(theta, 0.01f, glm::pi<float>() - 0.01f);
    phi = fmod(phi, 2.0f * glm::pi<float>());
    if ( phi < 0 )
        phi += 2.0f * glm::pi<float>();

    pos = glm::vec3(
        radius * sin(theta) * cos(phi),
        radius * sin(theta) * sin(phi),
        radius * cos(theta)
    );
    obj.modelMatrix[3] = glm::vec4(pos, 1.0f);

    const glm::vec3 normal = glm::normalize(pos);
    obj.velocity -= glm::dot(obj.velocity, normal) * normal;
}

void SphericalGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<glm::vec3>& massivePositions,
    const std::vector<float>& masses,
    float G,
    float maxDisplacement,
    float minDistSquared,
    float softeningLength)
{
    std::vector<Vertex> baseVertices;
    std::vector<uint32_t> dummyIndices;
    generateGrid(baseVertices, dummyIndices, m_GridSize, m_GridScale);
    const float R = glm::length(vertices[0].position);
    for ( Vertex& vertex : baseVertices )
    {
        const glm::vec3 normal = glm::normalize(vertex.position);
        float totalDisplacement = 0.0f;
        for ( size_t k = 0; k < massivePositions.size(); ++k )
        {
            if ( masses[k] <= 0.0f )
                continue;
            float dist = R * acos(glm::dot(normal, glm::normalize(massivePositions[k])));
            if ( dist * dist < minDistSquared )
                dist = sqrt(minDistSquared);
            const float softenedDist = dist * dist + softeningLength * softeningLength;
            const float displacement = G * masses[k] / softenedDist;
            totalDisplacement += displacement;
        }
        totalDisplacement = std::min(totalDisplacement, maxDisplacement);
        vertex.position = normal * (R - totalDisplacement);
    }

    for (size_t i = 0; i < baseVertices.size(); ++i)
        vertices[i].position = baseVertices[i].position;
}

void HyperbolicGeometry::generateGrid(
    std::vector<Vertex>& vertices,
    std::vector<uint32_t>& indices,
    uint32_t gridSize,
    float scale)
{
    // Generate vertices
    for ( int i = 0; i <= gridSize; ++i )
    {
        for ( int j = 0; j <= gridSize; ++j )
        {
            Vertex vertex{};
            // Map i, j to x, y coordinates in the range [-scale/2, scale/2]
            const float x = (static_cast<float>(i) / gridSize - 0.5f) * scale;
            const float y = (static_cast<float>(j) / gridSize - 0.5f) * scale;
            // Hyperbolic paraboloid: z = (x^2 - y^2) / k
            const float k = scale; // Adjust this to control curvature (smaller k = more pronounced saddle)
            const float z = (x * x - y * y) / k;
            vertex.position = glm::vec3(x, y, z);
            vertex.color = glm::vec3(1.0f);

            // Compute the normal (partial derivatives of z = (x^2 - y^2) / k)
            const float dz_dx = (2.0f * x) / k; // ∂z/∂x = 2x/k
            const float dz_dy = (-2.0f * y) / k; // ∂z/∂y = -2y/k
            const auto tangent_x = glm::vec3(1.0f, 0.0f, dz_dx);
            const auto tangent_y = glm::vec3(0.0f, 1.0f, dz_dy);
            vertex.normal = glm::normalize(glm::cross(tangent_y, tangent_x)); // Normal is cross product of tangents
            vertices.push_back(vertex);
        }
    }

    // Generate indices as lines
    for ( int i = 0; i < gridSize; ++i )
    {
        for ( int j = 0; j < gridSize; ++j )
        {
            uint32_t idx = i * (gridSize + 1) + j;
            uint32_t idxRight = idx + 1;
            uint32_t idxBottom = idx + (gridSize + 1);

            // Horizontal line (constant i, varying j)
            indices.push_back(idx);
            indices.push_back(idxRight);

            // Vertical line (constant j, varying i)
            indices.push_back(idx);
            indices.push_back(idxBottom);
        }
    }
}

float HyperbolicGeometry::computeDistance(const glm::vec3& pos1, const glm::vec3& pos2) const
{
    const float R = 10.0f;
    const float x1 = pos1.x / R, y1 = pos1.y / R;
    const float x2 = pos2.x / R, y2 = pos2.y / R;
    const float d = 2.0f * ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) /
              ((1.0f - x1 * x1 - y1 * y1) * (1.0f - x2 * x2 - y2 * y2));
    return R * acosh(1.0f + d);
}

void HyperbolicGeometry::updatePosition(Object& obj, float deltaTime, float /*radius not used*/) const
{
    obj.velocity += obj.acceleration * deltaTime;
    const glm::vec3 pos = obj.modelMatrix[3];
    glm::vec2 posDisk(pos.x, pos.y);
    glm::vec2 velDisk(obj.velocity.x, obj.velocity.y);

    posDisk += velDisk * deltaTime;

    constexpr float R = 10.0f;
    if ( glm::length(posDisk) > 0.99f * R )
    {
        posDisk = glm::normalize(posDisk) * 0.99f * R;
        velDisk = glm::vec2(0.0f);
    }

    obj.modelMatrix[3] = glm::vec4(posDisk.x, posDisk.y, 0.0f, 1.0f);
    obj.velocity = glm::vec3(velDisk, 0.0f);
}

void HyperbolicGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<glm::vec3>& massivePositions,
    const std::vector<float>& masses,
    float G,
    float maxDisplacement,
    float minDistSquared,
    float softeningLength)
{
    std::vector<Vertex> baseVertices;
    std::vector<uint32_t> dummyIndices;
    generateGrid(baseVertices, dummyIndices, m_GridSize, m_GridScale);

    constexpr float R = 10.0f;
    for ( Vertex& vertex : baseVertices )
    {
        float totalDisplacement = 0.0f;
        for ( size_t k = 0; k < massivePositions.size(); ++k )
        {
            if ( masses[k] <= 0.0f )
                continue;
            float dist = computeDistance(vertex.position, massivePositions[k]);
            if ( dist * dist < minDistSquared )
                dist = sqrt(minDistSquared);
            const float softenedDist = dist * dist + softeningLength * softeningLength;
            const float displacement = G * masses[k] / softenedDist;
            totalDisplacement += displacement;
        }
        totalDisplacement = std::min(totalDisplacement, maxDisplacement);
        vertex.position.z -= totalDisplacement;
    }

    for ( size_t i = 0; i < baseVertices.size(); ++i )
        vertices[i].position = baseVertices[i].position;
}

std::shared_ptr<Geometry> geometryFactory(GeometryType type, int grid_size, float grid_scale)
{
    switch ( type )
    {
    case GeometryType::Spherical:
        return std::make_shared<SphericalGeometry>(grid_size, grid_scale);
    case GeometryType::Hyperbolic:
        return std::make_shared<HyperbolicGeometry>(grid_size, grid_scale);
    default:
        return std::make_shared<FlatGeometry>(grid_size, grid_scale);
    }
}

glm::vec3 convertCoordinates(
    const glm::vec3& coordinates,
    GeometryType start_type,
    GeometryType end_type,
    float radius)
{
    switch ( start_type )
    {
    case GeometryType::Flat:
    {
        switch ( end_type )
        {
            case GeometryType::Spherical:
            {
                // Convert flatPos1 to spherical
                float r1 = glm::length(coordinates);
                if ( r1 < 0.01f )
                    r1 = 0.01f; // Avoid division by zero
                const float theta1 = acos(coordinates.z / r1);
                const float phi1 = atan2(coordinates.y, coordinates.x);
                return glm::vec3(
                    radius * sin(theta1) * cos(phi1),
                    radius * sin(theta1) * sin(phi1),
                    radius * cos(theta1)
                );
            }
            case GeometryType::Hyperbolic:
                return coordinates;
            default:
                return coordinates;
        }
    }
    case GeometryType::Hyperbolic:
    {
        switch ( end_type )
        {
            case GeometryType::Flat:
                return glm::vec3(coordinates.x, coordinates.y, coordinates.z);
            case GeometryType::Spherical:
                return glm::vec3(coordinates.x, coordinates.y, coordinates.z);
            case GeometryType::Hyperbolic:
                return coordinates;
        }
    }
    default:
    {
        switch ( end_type )
        {
        case GeometryType::Spherical:
            return glm::vec3(coordinates.x, coordinates.y, coordinates.z);
        case GeometryType::Hyperbolic:
            return glm::vec3(coordinates.x, coordinates.y, coordinates.z);
        default:
            return coordinates;
        }
    }
    }
}
