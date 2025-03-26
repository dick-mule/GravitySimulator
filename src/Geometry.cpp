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
            potential -= mass / softenedDist; // Gravitational potential
        }

        // Base displacement from potential
        float displacement = -potential * m_WarpStrength; // Negative potential -> downward displacement

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
    float radialAccel = glm::dot(accel, normal);
    accel -= radialAccel * normal; // Tangential acceleration only

    // Verlet half-step: Compute velocity at t + dt/2
    const glm::vec3 accelTerm = 0.5f * accel * deltaTime;
    glm::vec3 halfV = obj.velocity + accelTerm;

    // Update velocity to t + dt/2 (tangential)
    float radialVel = glm::dot(halfV, normal);
    halfV -= radialVel * normal;
    obj.velocity = halfV;

    if ( apply_verlet_half )
    {
        // Update position using half-step velocity
        glm::vec3 newPos = pos + halfV * deltaTime;

        // Enforce spherical constraint: Project onto sphere
        // newPos = pole + glm::normalize(newPos - pole) * radius;
        obj.position = newPos;
        obj.modelMatrix = glm::translate(glm::mat4(1.0f), obj.position);

        // Recompute normal and ensure velocity remains tangential
        normal = glm::normalize(newPos - pole);
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
        centerOfMass += mass * shape->m_Object.position;
        totalMass += mass;
    }
    if ( totalMass > 0.0f )
        centerOfMass /= totalMass;

    static float time = 0.0f;
    time += 0.016f;
    softeningLength = std::max(softeningLength, 10.0f);

    for ( Vertex& vertex : baseVertices )
    {
        float potential = 0.0f;
        for ( const auto& obj : massiveObjects )
        {
            const float mass = obj->m_Object.mass;
            if ( mass <= 0.0f )
                continue;
            const float dist = computeDistance(vertex.position, obj->m_Object.position);
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            potential -= std::log(1 + mass / softenedDist); // Gravitational potential
        }

        float displacement = -potential * m_WarpStrength;

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
    const float R = m_GridScale / 2.0f;
    const float x1 = pos1.x / R, z1 = pos1.z / R;
    const float x2 = pos2.x / R, z2 = pos2.z / R;

    const float len1 = x1 * x1 + z1 * z1;
    const float len2 = x2 * x2 + z2 * z2;
    if ( len1 >= 0.99f || len2 >= 0.99f )
    {
        // std::cout << "Warning: Positions outside Poincaré disk - len1: " << len1 << ", len2: " << len2 << std::endl;
        return std::numeric_limits<float>::max();
    }

    const float denom1 = 1.0f - len1;
    const float denom2 = 1.0f - len2;
    if ( denom1 <= 0.0f || denom2 <= 0.0f )
    {
        // std::cout << "Warning: Invalid denominator - denom1: " << denom1 << ", denom2: " << denom2 << std::endl;
        return std::numeric_limits<float>::max();
    }

    const float d = 2.0f * ((x1 - x2) * (x1 - x2) + (z1 - z2) * (z1 - z2)) / (denom1 * denom2);
    const float result = R * acosh(std::max(1.0f, 1.0f + d));
    if ( !std::isfinite(result) )
    {
        // std::cout << "Warning: Non-finite distance - d: " << d << std::endl;
        return std::numeric_limits<float>::max();
    }
    return result;
}

void HyperbolicGeometry::updatePosition(Object& obj, float deltaTime, float /*radius not used*/, bool apply_verlet_half) const
{
    const float R = m_GridScale / 2.0f;
    const glm::vec3 accelTerm = 0.5f * obj.acceleration * deltaTime;
    obj.velocity += accelTerm; // v(t + 0.5 * dt) or v(t + dt) incrementally
    if ( apply_verlet_half )
    {
        const glm::vec3 pos = obj.position;
        glm::vec2 posDisk(pos.x, pos.z);
        const glm::vec2 velDisk(obj.velocity.x, obj.velocity.z);

        float len = glm::length(posDisk) / (m_GridScale / 2.0f);
        if ( len < 0.01f )
            len = 0.01f;
        const float scale = 1.0f / (1.0f - len * len);
        posDisk += velDisk * deltaTime * scale; // x(t + dt) using v(t + 0.5 * dt)

        if ( glm::length(posDisk) > 0.99f * R )
        {
            posDisk = glm::normalize(posDisk) * 0.99f * R;
            obj.velocity = glm::vec3(0, obj.velocity.y, 0); // Reset xz velocity
        }

        obj.position = glm::vec3(posDisk.x, pos.y, posDisk.y);
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

    softeningLength = std::max(softeningLength, 10.0f);

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

            const float dist = computeDistance(vertex.position, obj->m_Object.position);
            if ( dist >= std::numeric_limits<float>::max() )
                continue; // Skip if distance computation failed
            const float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);
            potential -= std::log(1 + mass / softenedDist); // Gravitational potential
        }

        float displacement = -potential * m_WarpStrength;

        displacement = std::min(displacement, maxDisplacement);
        displacement = std::max(displacement, -maxDisplacement);

        // Compute the normal to the hyperbolic surface z = (x^2 - y^2) / k
        float x = vertex.position.x; // remember to graphics adjust y <-> z
        float z = vertex.position.z; // remember to graphics adjust y <-> z
        glm::vec3 normal(-2.0f * x / k, 1.0f, 2.0f * z / k);
        normal = glm::normalize(normal);

        // Displace along the normal
        vertex.position -= displacement * normal;

        // Update the normal after displacement
        x = vertex.position.x;
        z = vertex.position.z;
        normal = glm::vec3(-2.0f * x / k, 1.0f, 2.0f * z / k);
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
                const float alpha = flatDist / radius; // Angular distance from Shape 0 (at north pole)

                // Direction in flat space (normalized)
                const float flatTheta = atan2(coordinates.z, coordinates.x); // Azimuthal angle in flat plane
                const float flatPhi = acos(glm::clamp(coordinates.y / flatDist, -1.0f, 1.0f)); // Polar angle

                // New position: Place at angular distance alpha from (0, 0, R), preserve flat direction
                // Spherical coordinates: theta = flatTheta, phi = alpha
                const float phi = alpha; // Polar angle from north pole
                const float theta = flatTheta; // Preserve azimuthal direction

                return glm::vec3(
                    radius * sin(phi) * cos(theta), // x
                    radius * cos(phi),              // z
                    radius * sin(phi) * sin(theta)  // y
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
                // Vector from Shape 0 at (0, R, 0) to the point GRAPHICS
                const glm::vec3 fromNorthPole = coordinates - glm::vec3(0, radius, 0);
                float chordDist = glm::length(fromNorthPole);
                if ( chordDist < 0.01f )
                    chordDist = 0.01f;

                // Compute central angle (alpha) and arc length
                const float alpha = 2.0f * asin(chordDist / (2.0f * radius));
                const float flatDist = radius * alpha; // Distance in flat space

                // Direction in spherical space relative to north pole (Z-Y coordinates switched GRAPHICS!)
                const float theta = atan2(fromNorthPole.z, fromNorthPole.x); // Azimuthal angle
                const float phi = acos(glm::clamp(fromNorthPole.y / chordDist, -1.0f, 1.0f)); // Polar angle from north pole

                // Map to flat space, preserving xz-plane if y was 0
                const glm::vec3 flatDir(
                    sin(phi) * cos(theta), // x-component
                    cos(phi),              // y-component (0 if theta keeps y = 0)
                    sin(phi) * sin(theta)  // z-component (Recall y<->z in graphics)
                );
                return flatDist * flatDir; // Scale by flat distance
            }
            case GeometryType::Hyperbolic:
            {
                const auto flat_coords = convertCoordinates(coordinates, start_type, GeometryType::Flat, radius);
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
            const auto flatcoords = convertCoordinates(coordinates, start_type, GeometryType::Flat, radius);
            return convertCoordinates(flatcoords, GeometryType::Flat, end_type, radius);
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
    glm::vec3 newPos = convertCoordinates(oldPos, start_type, end_type, radius);

    switch ( start_type )
    {
    case GeometryType::Flat:
        switch ( end_type )
        {
            case GeometryType::Spherical:
            {
                // Flat xz-plane to spherical (pole at (0, R, 0))
                const float theta = atan2(oldPos.z, oldPos.x); // Azimuthal angle in xz-plane
                // Tangent for theta-direction (azimuthal around y-axis)
                tangent = glm::vec3(-sin(theta), 0, cos(theta));
                // float v_circ = sqrt(mu / dist); // Preserve circular velocity *Deprecated*
                return vMag * tangent;
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
                const float vy = 2.0f * (oldPos.x * oldVel.x - oldPos.z * oldVel.z) / k;

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
                glm::vec3 fromPole = oldPos - glm::vec3(0, radius, 0);
                float theta = atan2(fromPole.z, fromPole.x);
                tangent = glm::vec3(-sin(theta), 0, cos(theta));
                return vMag * tangent;
            }
            case GeometryType::Hyperbolic:
            {
                const auto convertedVel = convertVelocity(oldVel, oldVel, start_type, GeometryType::Flat, radius, dist, mu);
                return convertVelocity(convertedVel, oldVel, GeometryType::Flat, end_type, radius, dist, mu);
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
                const auto convertedFlat = convertVelocity(oldPos, oldVel, start_type, GeometryType::Flat, radius, dist, mu);
                return convertVelocity(convertedFlat, oldVel, GeometryType::Flat, end_type, radius, dist, mu);
            }
            default:
                return oldVel;
        }
    default:
        return oldVel;
    }
    return glm::vec3(0.0f); // Fallback
}
