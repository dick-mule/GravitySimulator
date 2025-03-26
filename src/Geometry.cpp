//
// Created by Richard Mule on 3/23/25.
//

#include "Geometry.hpp"

#include <iostream>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/fast_trigonometry.hpp>

Geometry::Geometry(int grid_size, float grid_scale, float warp_strength)
    : m_GridSize(grid_size)
    , m_GridScale(grid_scale)
    , m_WarpStrength(warp_strength)
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

void FlatGeometry::updatePosition(Object& obj, float deltaTime, float /*radius not used*/, bool apply_verlet_half) const
{
    const glm::vec3 accelTerm = 0.5f * obj.acceleration * deltaTime;
    const glm::vec3 halfV = (obj.velocity + accelTerm) * deltaTime;
    obj.velocity += accelTerm; // v(t + 0.5 * dt)
    if ( apply_verlet_half )
    {
        obj.position += halfV; // x(t + dt) = x(t) + halfV
        obj.modelMatrix = glm::translate(glm::mat4(1.0), obj.position);
    }
}

static std::string vecToString(const glm::vec3& vec)
{
    std::stringstream ss;
    ss << vec.x << " " << vec.y << " " << vec.z;
    return ss.str();
}

void FlatGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<std::shared_ptr<Shape>>& massiveObjects,
    float G,
    float maxDisplacement,
    float minDistSquared,
    float softeningLength)
{
    // // Reset vertices to base positions
    std::vector<Vertex> baseVertices(0);
    std::vector<uint32_t> dummyIndices(0);
    generateGrid(baseVertices, dummyIndices, m_GridSize, m_GridScale); // Match m_GridSize and m_GridScale

    // Compute the center of mass and relative velocity for ripple effect
    glm::vec3 centerOfMass(0.0f);
    float totalMass = 0.0f;
    float maxMass = 0.0;
    for ( auto& shape : massiveObjects )
    {
        float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        if ( mass > maxMass )
            maxMass = mass;
        centerOfMass += mass * glm::vec3(shape->m_Object.modelMatrix[3]);
        totalMass += mass;
    }
    if ( totalMass > 0.0f )
        centerOfMass /= totalMass;

    // Estimate orbital frequency for ripple effect (if two masses)
    float orbitalFrequency = 0.0f;
    if ( massiveObjects.size() == 2 )
    {
        const glm::vec3 pos1 = massiveObjects[0]->m_Object.modelMatrix[3];
        const glm::vec3 pos2 = massiveObjects[0]->m_Object.modelMatrix[3];
        const float dist = computeDistance(pos1, pos2);
        if ( dist > 0.01f )
        {
            const float mu = G * (massiveObjects[0]->m_Object.mass + massiveObjects[1]->m_Object.mass);
            orbitalFrequency = sqrt(mu / (dist * dist * dist)); // Kepler's third law approximation
        }
    }

    // Current time for ripple animation
    static float time = 0.0f;
    time += 0.016f; // Assuming ~60 FPS, adjust based on actual deltaTime

    for ( size_t v = 0; v < baseVertices.size(); ++v )
    {
        Vertex& vertex = baseVertices[v];
        // Compute gravitational potential at the vertex
        float potential = 0.0f;
        for ( size_t i = 0; i < massiveObjects.size(); ++i )
        {
            auto& obj = massiveObjects[i];
            const float mass = obj->m_Object.mass;
            if ( mass <= 0.0f )
                continue;
            const float dist = computeDistance(vertex.position, obj->m_Object.modelMatrix[3]);
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            potential -= mass / softenedDist; // Gravitational potential
        }

        // Base displacement from potential
        float displacement = -potential * m_WarpStrength; // Negative potential -> downward displacement

        // Add ripple effect
        if ( massiveObjects.size() == 2 && orbitalFrequency > 0.0f )
        {
            glm::vec3 p = vertex.position;
            glm::vec3 a = massiveObjects[0]->m_Object.modelMatrix[3];
            glm::vec3 b = massiveObjects[0]->m_Object.modelMatrix[3];
            glm::vec3 ab = b - a;
            glm::vec3 ap = p - a;
            const float abLengthSquared = glm::dot(ab, ab);
            if ( abLengthSquared < 1e-6f )
                continue;

            float t = glm::dot(ap, ab) / abLengthSquared;
            t = glm::clamp(t, 0.0f, 1.0f);
            glm::vec3 closestPoint = a + t * ab;
            float distToLine = computeDistance(vertex.position, closestPoint);

            float rippleAmplitude = 0.3f * m_WarpStrength * totalMass / (distToLine + 1.0f);
            float ripplePhase = orbitalFrequency * time - 0.1f * distToLine;
            displacement += rippleAmplitude * sin(ripplePhase);
        }

        // Cap the displacement
        displacement = std::min(displacement, maxDisplacement);
        displacement = std::max(displacement, -maxDisplacement);

        // Apply displacement along the z-axis
        vertex.position.y -= displacement;  // Positive potential -> upward, negative -> downward

        // Update normal after warping (approximate normal for flat geometry)
        vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f); // Simplified for flat geometry
    }

    for ( size_t i = 0; i < baseVertices.size(); ++i )
        vertices[i] = baseVertices[i];

    // Log average displacement for debugging
    // float avgDisplacement = 0.0f;
    // for ( const auto& vertex : vertices )
    //     avgDisplacement += vertex.position.z;

    // avgDisplacement /= vertices.size();
    // std::cout << "Average grid displacement: " << avgDisplacement << std::endl;
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
    float r1 = glm::length(pos1);
    float r2 = glm::length(pos2);
    if ( r1 < 0.01f || r2 < 0.01f )
    {
        // std::cout << "Warning: Small radius in computeDistance - r1: " << r1 << ", r2: " << r2 << std::endl;
        return 0.01f;
    }

    const float theta1 = atan2(pos1.z, pos1.x);
    const float phi1 = acos(glm::clamp(pos1.y / r1, -1.0f, 1.0f));
    const float theta2 = atan2(pos2.z, pos2.x);
    const float phi2 = acos(glm::clamp(pos2.y / r2, -1.0f, 1.0f));

    const float dtheta = theta2 - theta1;
    const float dphi = phi2 - phi1;
    float angularDist = sqrt(dtheta * dtheta + dphi * dphi);
    if ( angularDist < 0.01f )
    {
        // std::cout << "Warning: Small angular distance - dtheta: " << dtheta << ", dphi: " << dphi << ", angularDist: " << angularDist << std::endl;
        angularDist = 0.01f;
    }

    const float dist = angularDist * (m_GridScale / 2.0f);
    // std::cout << "Computed distance: " << dist << std::endl;
    return dist;
}

void SphericalGeometry::updatePosition(Object& obj, float deltaTime, float radius, bool apply_verlet_half) const
{
    glm::vec3 pos = obj.position; // Use obj.position instead of modelMatrix[3] for consistency
    float r = glm::length(pos);
    if ( r < 0.01f )
    {
        obj.position = glm::vec3(0.0f, 0.0f, radius); // Reset to sphere surface
        obj.modelMatrix = glm::translate(glm::mat4(1.0f), obj.position);
        obj.velocity = glm::vec3(0.0f);
        return;
    }
    glm::vec3 normal = pos / r; // Unit normal (radial direction)

    // Project acceleration onto the tangent plane
    glm::vec3 accel = obj.acceleration;
    float radialAccel = glm::dot(accel, normal);
    accel -= radialAccel * normal; // Tangential acceleration only

    // Verlet half-step: Compute velocity at t + dt/2
    const glm::vec3 accelTerm = 0.5f * accel * deltaTime;
    const glm::vec3 halfV = obj.velocity + accelTerm;

    // Update velocity to t + dt/2 (tangential)
    obj.velocity = halfV;
    float radialVel = glm::dot(obj.velocity, normal);
    obj.velocity -= radialVel * normal; // Ensure velocity remains tangential

    if ( apply_verlet_half )
    {
        // Update position using half-step velocity
        glm::vec3 newPos = pos + halfV * deltaTime;

        // Enforce spherical constraint: Project onto sphere
        newPos = glm::normalize(newPos) * radius;
        obj.position = newPos;
        obj.modelMatrix = glm::translate(glm::mat4(1.0f), obj.position);

        // Recompute normal and ensure velocity remains tangential
        normal = glm::normalize(newPos);
        radialVel = glm::dot(obj.velocity, normal);
        obj.velocity -= radialVel * normal;
    }
}

void SphericalGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<std::shared_ptr<Shape>>& massiveObjects,
    float G,
    float maxDisplacement,
    float minDistSquared,
    float softeningLength)
{
    std::vector<Vertex> baseVertices;
    std::vector<uint32_t> dummyIndices;
    generateGrid(baseVertices, dummyIndices, m_GridSize, m_GridScale);

    glm::vec3 centerOfMass(0.0f);
    float totalMass = 0.0f;
    float maxMass = 0.0;
    for ( auto& shape : massiveObjects )
    {
        float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        if ( mass > maxMass )
            maxMass = mass;
        centerOfMass += mass * glm::vec3(shape->m_Object.modelMatrix[3]);
        totalMass += mass;
    }
    if ( totalMass > 0.0f )
        centerOfMass /= totalMass;

    float orbitalFrequency = 0.0f;
    if ( massiveObjects.size() == 2 )
    {
        glm::vec3 pos1 = massiveObjects[0]->m_Object.modelMatrix[3];
        glm::vec3 pos2 = massiveObjects[1]->m_Object.modelMatrix[3];
        const float dist = computeDistance(pos1, pos2);
        if ( dist > 0.01f )
        {
            float mu = G * (massiveObjects[0]->m_Object.mass + massiveObjects[1]->m_Object.mass);
            orbitalFrequency = sqrt(mu / (dist * dist * dist));
        }
    }

    static float time = 0.0f;
    time += 0.016f;
    softeningLength = std::max(softeningLength, 10.0f);

    for ( Vertex& vertex : baseVertices )
    {
        float potential = 0.0f;
        for ( const auto& obj : massiveObjects )
        {
            float mass = obj->m_Object.mass;
            if ( mass <= 0.0f )
                continue;
            float dist = computeDistance(vertex.position, obj->m_Object.modelMatrix[3]);
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            potential -= (1.0 + std::log(maxMass / mass)) * std::pow(mass / softenedDist, 0.8); // Gravitational potential
        }

        float displacement = -potential * m_WarpStrength;

        if ( massiveObjects.size() == 2 && orbitalFrequency > 0.0f )
        {
            glm::vec3 p = vertex.position;
            glm::vec3 a = massiveObjects[0]->m_Object.modelMatrix[3];
            glm::vec3 b = massiveObjects[1]->m_Object.modelMatrix[3];
            glm::vec3 ab = b - a;
            glm::vec3 ap = p - a;
            float abLengthSquared = glm::dot(ab, ab);
            if ( abLengthSquared < 1e-6f )
                continue;

            float t = glm::dot(ap, ab) / abLengthSquared;
            t = glm::clamp(t, 0.0f, 1.0f);
            glm::vec3 closestPoint = a + t * ab;
            float distToLine = computeDistance(vertex.position, closestPoint);

            float rippleAmplitude = 0.3f * m_WarpStrength * totalMass / (distToLine + 1.0f);
            float ripplePhase = orbitalFrequency * time - 0.1f * distToLine;
            displacement += rippleAmplitude * sin(ripplePhase);
        }

        displacement = std::min(displacement, maxDisplacement);
        displacement = std::max(displacement, -maxDisplacement);

        vertex.position -= displacement * glm::normalize(vertex.position); // Displace along normal (outward for negative potential)
        vertex.normal = glm::normalize(vertex.position);
    }

    for ( size_t i = 0; i < baseVertices.size(); ++i )
        vertices[i] = baseVertices[i];
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
    const float R = m_GridScale / 2.0f;
    const float x1 = pos1.x / R, y1 = pos1.y / R;
    const float x2 = pos2.x / R, y2 = pos2.y / R;

    const float len1 = x1 * x1 + y1 * y1;
    const float len2 = x2 * x2 + y2 * y2;
    if ( len1 > 0.99f || len2 > 0.99f )
    {
        // std::cout << "Warning: Positions outside Poincaré disk - len1: " << len1 << ", len2: " << len2 << std::endl;
        return std::numeric_limits<float>::max();
    }

    const float denom1 = 1.0f - len1;
    const float denom2 = 1.0f - len2;
    if ( denom1 <= 0.0f || denom2 <= 0.0f )
    {
        // std::cout << "Warning: Invalid denominator in hyperbolic distance - denom1: " << denom1 << ", denom2: " << denom2 << std::endl;
        return std::numeric_limits<float>::max();
    }

    const float d = 2.0f * ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) / (denom1 * denom2);
    float result = R * acosh(std::max(1.0f, 1.0f + d));
    if ( !std::isfinite(result) )
    {
        // std::cout << "Warning: Hyperbolic distance computation resulted in non-finite value - d: " << d << std::endl;
        result = std::numeric_limits<float>::max();
    }
    return result;}

void HyperbolicGeometry::updatePosition(Object& obj, float deltaTime, float /*radius not used*/, bool apply_verlet_half) const
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
    const std::vector<std::shared_ptr<Shape>>& massiveObjects,
    float G,
    float maxDisplacement,
    float minDistSquared,
    float softeningLength)
{
    std::vector<Vertex> baseVertices;
    std::vector<uint32_t> dummyIndices;
    generateGrid(baseVertices, dummyIndices, m_GridSize, m_GridScale);

    glm::vec3 centerOfMass(0.0f);
    float totalMass = 0.0f;
    const float R = m_GridScale / 2.0f; // Radius of the Poincaré disk (50.0f)
    const float k = m_GridScale; // Scale factor for z = (x^2 - y^2) / k (100.0f)
    float maxMass = 0.0;
    for ( auto& shape : massiveObjects )
    {
        float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        if ( mass > maxMass )
            maxMass = mass;
        centerOfMass += mass * glm::vec3(shape->m_Object.modelMatrix[3]);
        totalMass += mass;
    }
    if ( totalMass > 0.0f )
        centerOfMass /= totalMass;

    float orbitalFrequency = 0.0f;
    if ( massiveObjects.size() == 2 )
    {
        const glm::vec3 pos1 = massiveObjects[0]->m_Object.modelMatrix[3];
        const glm::vec3 pos2 = massiveObjects[1]->m_Object.modelMatrix[3];
        const float dist = computeDistance(pos1, pos2);
        if ( dist > 0.01f )
        {
            float mu = G * (massiveObjects[0]->m_Object.mass + massiveObjects[1]->m_Object.mass);
            orbitalFrequency = sqrt(mu / (dist * dist * dist));
        }
    }

    static float time = 0.0f;
    time += 0.016f;

    softeningLength = std::max(softeningLength, 10.0f);

    for ( Vertex& vertex : baseVertices )
    {
        // Compute damping factor based on distance from center
        const float rSquared = vertex.position.x * vertex.position.x + vertex.position.y * vertex.position.y;
        float damping = 1.0f - rSquared / (R * R);
        if ( damping < 0.0f )
            damping = 0.0f;

        float potential = 0.0f;
        for ( const auto& obj : massiveObjects )
        {
            const float mass = obj->m_Object.mass;
            if ( mass <= 0.0f )
                continue;

            const float dist = computeDistance(vertex.position, obj->m_Object.modelMatrix[3]);
            if ( dist >= std::numeric_limits<float>::max() )
                continue; // Skip if distance computation failed
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            potential -= (1.0 + std::log(maxMass / mass)) * mass / softenedDist / (m_GridScale / 10.); // Gravitational potential
        }

        float displacement = -potential * m_WarpStrength;
        // Add ripple effect emanating from the line between the two masses
        if ( massiveObjects.size() == 2 && orbitalFrequency > 0.0f )
        {
            const glm::vec3 pos1 = massiveObjects[0]->m_Object.modelMatrix[3];
            const glm::vec3 pos2 = massiveObjects[1]->m_Object.modelMatrix[3];
            // Compute the shortest distance from the vertex to the line segment between pos1 and pos2
            glm::vec3 p = vertex.position;
            glm::vec3 a = pos1;
            glm::vec3 b = pos2;
            glm::vec3 ab = b - a;
            glm::vec3 ap = p - a;
            float abLengthSquared = glm::dot(ab, ab);
            if ( abLengthSquared < 1e-6f )
                continue; // Avoid division by zero

            float t = glm::dot(ap, ab) / abLengthSquared;
            t = glm::clamp(t, 0.0f, 1.0f); // Clamp to the line segment
            glm::vec3 closestPoint = a + t * ab;
            float distToLine = computeDistance(vertex.position, closestPoint);
            if ( distToLine >= std::numeric_limits<float>::max() )
                continue;

            float rippleAmplitude = 0.3f * m_WarpStrength * totalMass / (distToLine + 1.0f) * damping;
            float ripplePhase = orbitalFrequency * time - 0.1f * distToLine;
            displacement += rippleAmplitude * sin(ripplePhase);
        }

        displacement = std::min(displacement, maxDisplacement);
        displacement = std::max(displacement, -maxDisplacement);

        // Compute the normal to the hyperbolic surface z = (x^2 - y^2) / k
        float x = vertex.position.x;
        float y = vertex.position.y;
        glm::vec3 normal(-2.0f * x / k, 2.0f * y / k, 1.0f);
        normal = glm::normalize(normal);

        // Displace along the normal
        vertex.position -= displacement * normal;

        // Update the normal after displacement
        x = vertex.position.x;
        y = vertex.position.y;
        normal = glm::vec3(-2.0f * x / k, 2.0f * y / k, 1.0f);
        vertex.normal = glm::normalize(normal);
    }

    for ( size_t i = 0; i < baseVertices.size(); ++i )
        vertices[i] = baseVertices[i];
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
                // Flat distance from origin (0, 0, 0)
                float flatDist = glm::length(coordinates);
                if ( flatDist < 0.01f )
                    flatDist = 0.01f; // Avoid division by zero

                // Desired geodesic distance on sphere (preserve flat distance)
                const float d = flatDist; // e.g., 10
                const float alpha = d / radius; // Angular distance from Shape 0 (at north pole)

                // Direction in flat space (normalized)
                const glm::vec3 flatDir = glm::normalize(coordinates);
                const float flatTheta = atan2(coordinates.y, coordinates.x); // Azimuthal angle in flat plane
                const float flatPhi = acos(glm::clamp(coordinates.z / flatDist, -1.0f, 1.0f)); // Polar angle

                // New position: Place at angular distance alpha from (0, 0, R), preserve flat direction
                // Spherical coordinates: theta = flatTheta, phi = alpha
                const float phi = alpha; // Polar angle from north pole
                const float theta = flatTheta; // Preserve azimuthal direction

                return glm::vec3(
                    radius * sin(phi) * cos(theta), // x
                    radius * sin(phi) * sin(theta), // y
                    radius * cos(phi)              // z
                );
            }
            case GeometryType::Hyperbolic:
            {
                // Assume hyperbolic plane in xy-plane, z as embedding height
                // Map flat Euclidean distance to hyperbolic distance via Poincaré disk
                float flatDist = glm::length(glm::vec2(coordinates.x, coordinates.y));
                if ( flatDist < 0.01f )
                    flatDist = 0.01f;

                // Hyperbolic radius k (curvature scale, often radius-related)
                const float k = radius; // Adjust based on your hyperbolic model
                float hypDist = k * tanh(flatDist / k); // Approximate hyperbolic distance
                if ( hypDist >= k ) // Cap at disk boundary
                    hypDist = k * 0.999f;

                // Direction preserved
                const float theta = atan2(coordinates.y, coordinates.x);
                return glm::vec3(
                    hypDist * cos(theta), // x
                    hypDist * sin(theta), // y
                    0.0f                  // z (2D plane)
                );
            }
            default:
                return coordinates;
        }
    }
    case GeometryType::Spherical:
    {
        switch ( end_type )
        {
            case GeometryType::Flat:
            {
                // Spherical position assumed relative to (0, 0, R)
                const glm::vec3 fromNorthPole = coordinates - glm::vec3(0, 0, radius);
                float r = glm::length(fromNorthPole);
                if ( r < 0.01f )
                    r = 0.01f;

                // Convert to spherical coordinates relative to north pole
                const float phi = acos(glm::clamp(fromNorthPole.z / r, -1.0f, 1.0f)); // Angle from north pole
                const float theta = atan2(fromNorthPole.y, fromNorthPole.x);

                // Flat distance = arc length from (0, 0, R)
                const float flatDist = radius * phi;
                return glm::vec3(
                    flatDist * sin(phi) * cos(theta), // Approximate flat projection
                    flatDist * sin(phi) * sin(theta),
                    flatDist * cos(phi)
                );
            }
            case GeometryType::Hyperbolic:
            {
                // Spherical to hyperbolic: Project onto xy-plane, convert arc length
                const glm::vec3 fromNorthPole = coordinates - glm::vec3(0, 0, radius);
                float r = glm::length(fromNorthPole);
                if ( r < 0.01f )
                    r = 0.01f;

                const float phi = acos(glm::clamp(fromNorthPole.z / r, -1.0f, 1.0f));
                const float theta = atan2(fromNorthPole.y, fromNorthPole.x);
                const float arcDist = radius * phi; // Geodesic distance from north pole

                const float k = radius;
                float hypDist = k * tanh(arcDist / k); // Hyperbolic distance
                if ( hypDist >= k )
                    hypDist = k * 0.999f;

                return glm::vec3(
                    hypDist * cos(theta),
                    hypDist * sin(theta),
                    0.0f
                );
            }
            default:
                return coordinates;
        }
    }
    default:
    {
        switch ( end_type )
        {
        case GeometryType::Flat:
        {
            // Hyperbolic (Poincaré disk) to flat: Approximate inverse hyperbolic map
            float hypDist = glm::length(glm::vec2(coordinates.x, coordinates.y));
            if ( hypDist < 0.01f )
                hypDist = 0.01f;

            const float k = radius;
            const float flatDist = k * atanh(hypDist / k); // Inverse tanh to flat distance
            const float theta = atan2(coordinates.y, coordinates.x);

            return glm::vec3(
                flatDist * cos(theta),
                flatDist * sin(theta),
                0.0f
            );
        }
        case GeometryType::Spherical:
        {
            // Hyperbolic to spherical: Map hyperbolic distance to arc length
            float hypDist = glm::length(glm::vec2(coordinates.x, coordinates.y));
            if ( hypDist < 0.01f )
                hypDist = 0.01f;

            const float k = radius;
            const float flatDist = k * atanh(hypDist / k); // Convert to flat-like distance
            float alpha = flatDist / radius; // Angular distance from (0, 0, R)
            if ( alpha > M_PI )
                alpha = M_PI;

            const float theta = atan2(coordinates.y, coordinates.x);
            const float phi = alpha;

            return glm::vec3(
                radius * sin(phi) * cos(theta),
                radius * sin(phi) * sin(theta),
                radius * cos(phi)
            );
        }
        default:
            return coordinates; // No conversion needed
        }
    }
    }
}

glm::vec3 convertVelocity(
    const glm::vec3& oldPos,
    const glm::vec3& oldVel,
    GeometryType start_type,
    GeometryType end_type,
    float radius,
    float dist,
    float mu)
{
    if ( start_type == end_type )
        return oldVel;

    float vMag = glm::length(oldVel);
    if ( vMag < 0.01f )
        return glm::vec3(0.0f);

    glm::vec3 tangent;
    glm::vec3 newPos = convertCoordinates(oldPos, start_type, end_type, radius);

    switch ( start_type )
    {
    case GeometryType::Flat:
        switch ( end_type )
        {
            case GeometryType::Spherical:
            {
                // New position relative to Shape 0 at (0, 0, R)
                glm::vec3 normal = glm::normalize(newPos - glm::vec3(0, 0, radius));
                float theta = atan2(oldPos.y, oldPos.x);
                tangent = glm::vec3(-sin(theta), cos(theta), 0); // theta-direction
                float v_circ = sqrt(mu / dist); // Circular velocity at original distance
                return v_circ * tangent;
            }
            case GeometryType::Hyperbolic:
            {
                float theta = atan2(oldPos.y, oldPos.x);
                tangent = glm::vec3(-sin(theta), cos(theta), 0);
                return vMag * tangent; // Preserve magnitude, adjust direction
            }
            default:
                return oldVel;
        }
    case GeometryType::Spherical:
        switch ( end_type )
        {
            case GeometryType::Flat:
            {
                glm::vec3 fromNorth = oldPos - glm::vec3(0, 0, radius);
                float theta = atan2(fromNorth.y, fromNorth.x);
                tangent = glm::vec3(-sin(theta), cos(theta), 0);
                return vMag * tangent; // Approximate flat plane velocity
            }
            case GeometryType::Hyperbolic:
            {
                float theta = atan2(oldPos.y, oldPos.x);
                tangent = glm::vec3(-sin(theta), cos(theta), 0);
                return vMag * tangent;
            }
            default:
                return oldVel;
        }
    case GeometryType::Hyperbolic:
        switch ( end_type )
        {
            case GeometryType::Flat:
                return oldVel; // Already in xy-plane, preserve
            case GeometryType::Spherical:
            {
                glm::vec3 normal = glm::normalize(newPos - glm::vec3(0, 0, radius));
                float theta = atan2(oldPos.y, oldPos.x);
                tangent = glm::vec3(-sin(theta), cos(theta), 0);
                float v_circ = sqrt(mu / dist);
                return v_circ * tangent;
            }
            default:
                return oldVel;
        }
    default:
        return oldVel;
    }
    return glm::vec3(0.0f); // Fallback
}
