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
            const float dist = computeDistance(vertex.position, obj->m_Object.position);
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            // Logarithmic scaling
            potential -= mass / softenedDist; // Gravitational potential
        }

        // Base displacement from potential
        float displacement = -potential * m_WarpStrength; // Negative potential -> downward displacement
        // displacement = maxDisplacement * (1.0f / (1 + exp(-displacement / maxDisplacement)) - 0.5);

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
                R * cos(theta),
                R * sin(theta) * sin(phi)
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
    const float r1 = glm::length(pos1);
    const float r2 = glm::length(pos2);
    if ( r1 < 0.01f || r2 < 0.01f )
    {
        // std::cout << "Warning: Small radius in computeDistance - r1: " << r1 << ", r2: " << r2 << std::endl;
        return 0.01f;
    }

    const float theta1 = atan2(pos1.z, pos1.x);
    const float phi1 = acos(glm::clamp(pos1.y / r1, -1.0f, 1.0f));
    const float theta2 = atan2(pos2.z, pos2.x);
    const float phi2 = acos(glm::clamp(pos2.y / r2, -1.0f, 1.0f));

    const float d_theta = theta2 - theta1;
    const float d_phi = phi2 - phi1;
    float angularDist = sqrt(d_theta * d_theta + d_phi * d_phi);
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
    const glm::vec3 pos = obj.position;
    const glm::vec3 pole(0.0f, radius, 0.0f); // Pole at (0, R, 0)
    const glm::vec3 fromPole = pos - pole;
    float r = glm::length(fromPole);
    if ( r < 0.01f )
    {
        obj.position = pole; // Reset to sphere surface
        obj.velocity = glm::vec3(0.0f);
        obj.modelMatrix = glm::translate(glm::mat4(1.0f), obj.position);
        return;
    }
    glm::vec3 normal = fromPole / r; // Unit normal (radial direction)
    // Project acceleration onto the tangent plane
    glm::vec3 accel = obj.acceleration;
    // float radialAccel = glm::dot(accel, normal);
    // accel -= radialAccel * normal; // Tangential acceleration only

    // Verlet half-step: Compute velocity at t + dt/2
    const glm::vec3 accelTerm = 0.5f * accel * deltaTime;
    glm::vec3 halfV = (obj.velocity + accelTerm) * deltaTime;

    // Update velocity to t + dt/2 (tangential)
    // float radialVel = glm::dot(halfV, normal);
    // halfV -= radialVel * normal;
    obj.velocity += accelTerm;

    if ( apply_verlet_half )
    {
        // Update position using half-step velocity
        // glm::vec3 newPos = pos + halfV * deltaTime;

        // Enforce spherical constraint: Project onto sphere
        obj.position += halfV;
        obj.position = glm::normalize(obj.position) * radius;
        obj.modelMatrix = glm::translate(glm::mat4(1.0f), obj.position);

        // Recompute normal and ensure velocity remains tangential
        // normal = glm::normalize(newPos - pole);
        // radialVel = glm::dot(obj.velocity, normal);
        // obj.velocity -= radialVel * normal;
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

    float maxMass = 0.0;
    float totalMass = 0.0f;
    glm::vec3 centerOfMass(0.0f);
    for ( auto& shape : massiveObjects )
    {
        const float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        if ( mass > maxMass )
            maxMass = mass;
        centerOfMass += mass * shape->m_Object.position;
        totalMass += mass;
    }
    if ( totalMass > 0.0f )
        centerOfMass /= totalMass;

    static float time = 0.0f;
    time += 0.016f;
    // softeningLength = softeningLength;
    // Use the base radius of the sphere for scaling
    const float R = m_GridScale / 2.0f; // e.g., 125

    static bool log_first = true;

    for ( Vertex& vertex : baseVertices )
    {
        float potential = 0.0f;
        for ( const auto& obj : massiveObjects )
        {
            const float mass = obj->m_Object.mass;
            if ( mass <= 0.0f )
                continue;
            const float r1 = glm::length(vertex.position);
            const float r2 = glm::length(obj->m_Object.position);
            if ( r1 < 0.01f || r2 < 0.01f )
                continue;

            float dist = computeDistance(vertex.position, obj->m_Object.position);
            if ( dist < 0.01f )
                dist = 0.01f;
            const float dist_from_mass = glm::length(vertex.position - obj->m_Object.position);
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            const float damping = 1.0 / std::max(1.0f, sqrt(dist_from_mass)); ///std::pow(std::clamp(glm::dot(norm1, norm2), 0.0f, 1.0f), 4.0);
            potential -= 3.0f * obj->getSize() * damping * dist / softenedDist; // Gravitational potential
        }

        float displacement = potential * m_WarpStrength;
        displacement = std::min(displacement, maxDisplacement);
        displacement = std::max(displacement, -maxDisplacement);

        // Displace along the radial direction
        // vertex.position is already on the sphere of radius R, as generated
        glm::vec3 radialDir = glm::normalize(vertex.position); // Matches vertex.normal from generation
        // Reduce the radius by the displacement (inward dip)
        const float newRadius = std::max(R + 30.0f * displacement, 0.5f * R);
        // newRadius = glm::max(newRadius, 0.1f * baseRadius); // Prevent collapsing to center
        // vertex.normal = glm::normalize(vertex.position);
        vertex.position = radialDir * newRadius;
    }

    log_first = false;

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
            // RECALL y <-> z for graphics, comments will show x, y, z standard formulas but graphics switches y/z!
            // Map i, j to x, y coordinates in the range [-scale/2, scale/2]
            const float x = (static_cast<float>(i) / gridSize - 0.5f) * scale;
            const float z = (static_cast<float>(j) / gridSize - 0.5f) * scale;
            // Hyperbolic paraboloid: z = (x^2 - y^2) / k  <- not using graphics coordinates
            const float k = scale; // Adjust this to control curvature (smaller k = more pronounced saddle)
            const float y = (x * x - z * z) / k;
            vertex.position = glm::vec3(x, y, z);
            vertex.color = glm::vec3(1.0f);

            // Compute the normal (partial derivatives of z = (x^2 - y^2) / k)
            const float dy_dx = 2.0f * x / k; // ∂y/∂x = 2x/k <- Graphics adjusted
            const float dy_dz = -2.0f * z / k; // ∂y/∂z = -2z/k <- Graphics adjusted
            const auto tangent_x = glm::vec3(1.0f, 0.0f, dy_dx);
            const auto tangent_y = glm::vec3(0.0f, 1.0f, dy_dz);
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
    const float k = m_GridScale; // 250 to match grid
    // Ensure y matches paraboloid (should already from convertCoordinates)
    const float y1 = (pos1.x * pos1.x - pos1.z * pos1.z) / k;
    const float y2 = (pos2.x * pos2.x - pos2.z * pos2.z) / k;

    const float dx = pos1.x - pos2.x;
    const float dy = y1 - y2; // Use paraboloid y, not input y
    const float dz = pos1.z - pos2.z;

    const float distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance;
}

void HyperbolicGeometry::updatePosition(Object& obj, float deltaTime, float /*radius not used*/, bool apply_verlet_half) const
{
    const float k = m_GridScale; // 250
    const glm::vec3 accelTerm = 0.5f * obj.acceleration * deltaTime;
    obj.velocity += accelTerm;

    if ( apply_verlet_half )
    {
        glm::vec2 posDisk(obj.position.x, obj.position.z);
        const glm::vec2 velDisk(obj.velocity.x, obj.velocity.z);

        // Simple Euler step in xz (no Poincaré scaling)
        posDisk += velDisk * deltaTime;

        // Constrain to paraboloid
        const float newY = (posDisk.x * posDisk.x - posDisk.y * posDisk.y) / k;
        obj.position = glm::vec3(posDisk.x, newY, posDisk.y);

        // Update vy to match surface
        obj.velocity.y = (2.0f / k) * (obj.position.x * obj.velocity.x - obj.position.z * obj.velocity.z);

        obj.modelMatrix = glm::translate(glm::mat4(1.0f), obj.position);
    }
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
    const float k = m_GridScale; // Scale factor for z = (x^2 - y^2) / k (100.0f) <- Non graphics formula
    float maxMass = 0.0;
    for ( auto& shape : massiveObjects )
    {
        float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        if ( mass > maxMass )
            maxMass = mass;
        centerOfMass += mass * shape->m_Object.position;
        totalMass += mass;
    }
    if ( totalMass > 0.0f )
        centerOfMass /= totalMass;

    static float time = 0.0f;
    time += 0.016f;

    // softeningLength = std::max(softeningLength, 10.0f);

    for ( Vertex& vertex : baseVertices )
    {
        // Compute damping factor based on distance from center
        const float rSquared = vertex.position.x * vertex.position.x + vertex.position.z * vertex.position.z;
        float damping = 1.0f - rSquared / (R * R);
        if ( damping < 0.0f )
            damping = 0.0f;

        float potential = 0.0f;
        for ( const auto& obj : massiveObjects )
        {
            const float mass = obj->m_Object.mass;
            if ( mass <= 0.0f )
                continue;

            glm::vec3 pos1 = vertex.position;
            glm::vec3 pos2 = obj->m_Object.position;
            glm::vec3 diff(
                pos2.x - pos1.x,
                (pos2.x * pos2.x - pos2.z * pos2.z) / k - (pos1.x * pos1.x - pos1.z * pos1.z) / k,
                pos2.z - pos1.z);
            const float dist = glm::length(diff);
            // const float dist = computeDistance(vertex.position, obj->m_Object.position);
            if ( dist >= std::numeric_limits<float>::max() )
                continue; // Skip if distance computation failed
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            potential -= mass / softenedDist; // Gravitational potential
        }

        float displacement = -potential * m_WarpStrength * glm::clamp(
            1.0f + std::log(std::abs(glm::length(vertex.position))), 1.0f, 5.0f);
        displacement = std::min(displacement, maxDisplacement);
        displacement = std::max(displacement, -maxDisplacement);

        // Compute the true normal to the hyperbolic surface
        const float x = vertex.position.x;
        const float z = vertex.position.z;
        glm::vec3 trueNormal(-2.0f * x / k, 1.0f, 2.0f * z / k);
        trueNormal = glm::normalize(trueNormal);

        // Compute a visual normal by blending with the y-direction
        float w = 1.0f; // Blend factor (0 = pure y-direction, 1 = true normal)
        glm::vec3 visualNormal = glm::normalize(w * trueNormal + (1.0f - w) * glm::vec3(0.0f, 1.0f, 0.0f));

        // Displace along the visual normal
        vertex.position -= displacement * visualNormal;

        // // Project back to the hyperbolic surface
        // vertex.position.y = (vertex.position.x * vertex.position.x - vertex.position.z * vertex.position.z) / k;

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
    float radius,
    const std::shared_ptr<Geometry>& geometry)
{
    switch ( start_type )
    {
    case GeometryType::Flat:
    {
        switch ( end_type )
        {
            case GeometryType::Spherical:
            {
                const float flatDistXZ = glm::length(glm::vec2(coordinates.x, coordinates.z));
                float flatDist = glm::length(coordinates);
                if ( flatDist < 0.01f )
                    flatDist = 0.01f;
                const float alpha = flatDistXZ / radius; // xz distance to sphere
                const float theta = atan2(coordinates.z, coordinates.x);
                const float flatPhi = acos(glm::clamp(coordinates.y / flatDist, -1.0f, 1.0f));
                return glm::vec3(
                    radius * sin(alpha) * cos(theta),
                    radius * cos(alpha) + coordinates.y, // Preserve y offset
                    radius * sin(alpha) * sin(theta)
                );
            }
            case GeometryType::Hyperbolic:
            {
                const float k = 2.0f * radius; // Must match grid's scale, e.g., 250
                const float x = coordinates.x;
                const float z = coordinates.z;
                const float yHeight = (x * x - z * z) / k + coordinates.y; // Paraboloid height + flat y offset

                return glm::vec3(
                    x,       // Preserve flat x
                    yHeight, // Hyperbolic paraboloid y
                    z        // Preserve flat z
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
                float distXZ = glm::length(glm::vec2(coordinates.x, coordinates.z));
                if ( distXZ < 0.01f )
                    distXZ = 0.01f;
                const float sinAlpha = distXZ / radius;
                const float alpha = asin(glm::clamp(sinAlpha, 0.0f, 1.0f));
                const float theta = atan2(coordinates.z, coordinates.x);
                const float flatDistXZ = alpha * radius;
                const float yOffset = coordinates.y - radius * cos(alpha);
                const glm::vec3 result(
                    flatDistXZ * cos(theta),
                    yOffset,
                    flatDistXZ * sin(theta)
                );
                return result;
            }
            case GeometryType::Hyperbolic:
            {
                const auto flat_coords = convertCoordinates(coordinates, start_type, GeometryType::Flat, radius, geometry);
                return convertCoordinates(flat_coords, GeometryType::Flat, end_type, radius);
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
            const float k = 2.0 * radius;
            const float x = coordinates.x;
            const float z = coordinates.z;
            const float flatY = coordinates.y - (x * x - z * z) / k; // Remove paraboloid height
            return glm::vec3(
                x,
                flatY,
                z
            );
        }
        case GeometryType::Spherical:
        {
            const auto flatCoords = convertCoordinates(coordinates, start_type, GeometryType::Flat, radius, geometry);
            return convertCoordinates(flatCoords, GeometryType::Flat, end_type, radius);
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
    float mu,
    const std::shared_ptr<Geometry>& calculator)
{
    if ( start_type == end_type )
        return oldVel;

    float vMag = glm::length(oldVel);
    if ( vMag < 0.01f )
        return glm::vec3(0.0f);

    glm::vec3 tangent;

    switch ( start_type )
    {
    case GeometryType::Flat:
        switch ( end_type )
        {
            case GeometryType::Spherical:
            {
                float flatDistXZ = glm::length(glm::vec2(oldPos.x, oldPos.z));
                if ( flatDistXZ < 0.01f )
                    flatDistXZ = 0.01f;
                const float alpha = flatDistXZ / radius;
                const float theta = atan2(oldPos.z, oldPos.x);

                // Tangent vectors
                const glm::vec3 thetaTangent(-sin(theta), 0, cos(theta)); // Azimuthal
                const glm::vec3 alphaTangent(cos(alpha) * cos(theta), 0.0, cos(alpha) * sin(theta)); // Polar

                // Project flat velocity onto spherical directions
                const float vTheta = glm::dot(oldVel, thetaTangent); // Azimuthal component
                const float vAlpha = glm::dot(oldVel, alphaTangent); // Polar component
                const float vRadial = oldVel.y; // y-velocity affects radial offset

                // Scale to preserve speed in tangential plane, add radial component
                return vAlpha * alphaTangent + vTheta * thetaTangent + vRadial * glm::vec3(0, 1, 0);
            }
            case GeometryType::Hyperbolic:
            {
                float flatDist = glm::length(glm::vec2(oldPos.x, oldPos.z));
                if ( flatDist < 0.01f )
                    flatDist = 0.01f;

                // Tangent in xz-plane
                const glm::vec2 posDisk(oldPos.x, oldPos.z);
                const glm::vec2 radial = glm::normalize(posDisk);
                const glm::vec2 tangentXZ(-radial.y, radial.x); // 90° CCW
                const glm::vec3 hypTangentXZ(tangentXZ.x, 0, tangentXZ.y);

                // Use flatDist (no hyperbolic scaling needed for paraboloid)
                const float h = flatDist * vMag;
                const float vHyp = h / flatDist; // vMag, preserving speed

                // Compute vy from paraboloid: y = (x^2 - z^2) / k
                const float k = 2.0f * radius; // 500
                const float vy = (oldPos.x * oldVel.x - oldPos.z * oldVel.z) / k;

                // Combine xz tangential velocity with vy
                return vHyp * hypTangentXZ + glm::vec3(0, vy, 0);
            }
            default:
                return oldVel;
        }
    case GeometryType::Spherical:
        switch ( end_type )
        {
            case GeometryType::Flat:
            {
                float distXZPrime = glm::length(glm::vec2(oldPos.x, oldPos.z));
                if ( distXZPrime < 0.01f )
                    distXZPrime = 0.01f;
                const float sinAlpha = distXZPrime / radius;
                const float alpha = asin(glm::clamp(sinAlpha, 0.0f, 1.0f));
                const float theta = atan2(oldPos.z, oldPos.x);

                // Tangent vectors
                const glm::vec3 thetaTangent(-sin(theta), 0, cos(theta));
                const glm::vec3 alphaTangent(cos(alpha) * cos(theta), -sin(alpha), cos(alpha) * sin(theta));

                // Project spherical velocity
                const float vTheta = glm::dot(oldVel, thetaTangent);
                const float vAlpha = glm::dot(oldVel, alphaTangent);
                const float vY = oldVel.y; // Radial component maps to flat y

                // Reconstruct flat velocity
                return vAlpha * alphaTangent + vTheta * thetaTangent + vY * glm::vec3(0, 1, 0);
            }
            case GeometryType::Hyperbolic:
            {
                    const auto convertedFlatCoord = convertCoordinates(oldPos, start_type, GeometryType::Flat, radius, calculator);
                    const auto convertedFlat = convertVelocity(oldPos, oldVel, start_type, GeometryType::Flat, radius, dist, mu);
                    return convertVelocity(convertedFlatCoord, convertedFlat, GeometryType::Flat, end_type, radius, dist, mu);
            }
            default:
                return oldVel;
        }
    case GeometryType::Hyperbolic:
        switch ( end_type )
        {
            case GeometryType::Flat:
            {
                float hypDist = glm::length(glm::vec2(oldPos.x, oldPos.z));
                if ( hypDist < 0.01f )
                    hypDist = 0.01f;

                vMag = glm::length(glm::vec2(oldVel.x, oldVel.z)); // xz speed
                float h = hypDist * vMag;

                const glm::vec2 posDisk(oldPos.x, oldPos.z);
                const glm::vec2 radial = glm::normalize(posDisk);
                const glm::vec2 tangentXZ(-radial.y, radial.x);
                tangent = glm::vec3(tangentXZ.x, 0, tangentXZ.y);

                float vFlat = h / hypDist; // Preserve xz speed
                return vFlat * tangent; // vy = 0 in flat
            }
            case GeometryType::Spherical:
            {
                const auto convertedFlatCoord = convertCoordinates(oldPos, start_type, GeometryType::Flat, radius, calculator);
                const auto convertedFlat = convertVelocity(oldPos, oldVel, start_type, GeometryType::Flat, radius, dist, mu);
                return convertVelocity(convertedFlatCoord, convertedFlat, GeometryType::Flat, end_type, radius, dist, mu);
            }
            default:
                return oldVel;
        }
    default:
        return oldVel;
    }
    return glm::vec3(0.0f); // Fallback
}
