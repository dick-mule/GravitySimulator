//
// Created by Richard Mule on 3/23/25.
//

#include "Geometry.hpp"

#include <cmath>
#include <iostream>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/fast_trigonometry.hpp>
#include <algorithm>

namespace
{
    // Mass enters the warp as mass^kMassExponent — a compression gentle enough
    // to keep the central body clearly dominant over orbiter clusters, yet firm
    // enough to tame the ~600x central-vs-orbiter mass range. Must match the
    // exponent hard-coded in flat_grid.comp.
    constexpr float kMassExponent = 0.4f;

    // Heat-map colour for the warp: white where the grid is flat, fading to a
    // dark blue at full well depth. Must match the gradient in flat_grid.comp.
    glm::vec3 depthColor(float depth, float wellDepth)
    {
        const float t = glm::clamp(depth / std::max(wellDepth, 1e-3f), 0.0f, 1.0f);
        return glm::mix(glm::vec3(1.0f), glm::vec3(0.15f, 0.30f, 0.65f), t);
    }
}

Geometry::Geometry(int grid_size, float grid_scale)
    : m_GridSize(grid_size)
    , m_GridScale(grid_scale)
{
}

// Lazily (re)builds the un-warped base surface. warpGrid() warps a copy of
// this rather than regenerating the whole grid every frame.
const std::vector<Vertex>& Geometry::baseGrid()
{
    if ( m_BaseGridDirty )
    {
        m_BaseGrid.clear();
        std::vector<uint32_t> dummyIndices;
        generateGrid(m_BaseGrid, dummyIndices, m_GridSize, m_GridScale);
        m_BaseGridDirty = false;
    }
    return m_BaseGrid;
}

// Total warp depth at a base position: the sum of every body's smooth,
// saturating well. Euclidean distance — HyperbolicGeometry overrides this to
// measure along the paraboloid surface instead.
float Geometry::warpDepth(
    const glm::vec3& basePos,
    const std::vector<std::shared_ptr<Shape>>& bodies,
    const WarpParams& warp) const
{
    // Sum every body's contribution. mass^kMassExponent compresses the huge
    // central-vs-orbiter range; radialInfluence widens heavier bodies' wells.
    float sum = 0.0f;
    for ( const auto& shape : bodies )
    {
        const float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        const float m04   = std::pow(mass, kMassExponent);
        const float width = 1.0f + warp.radialInfluence * m04;
        const float rEff  = glm::length(basePos - shape->m_Object.position) / width;
        const float soft  = std::sqrt(rEff * rEff + warp.softening * warp.softening);
        sum += m04 / soft;
    }
    // One tanh of the *summed* field: bounded by wellDepth, so a cluster of
    // bodies cannot stack past a single well's depth.
    return warp.wellDepth * std::tanh(warp.warpGain * sum) * edgeFade(basePos);
}

float Geometry::edgeFade(const glm::vec3& basePos) const
{
    // Smoothstep falloff over the outer 15% of the grid, so the boundary is
    // always flat no matter where the bodies have drifted.
    const float half = m_GridScale * 0.5f;
    const float t = std::max(std::abs(basePos.x), std::abs(basePos.z)) / half;
    const float f = glm::clamp((t - 0.85f) / 0.15f, 0.0f, 1.0f);
    return 1.0f - f * f * (3.0f - 2.0f * f);
}

// Mean warp depth over a coarse sample of the base grid. The warp field is
// smooth, so a strided sample is an accurate, cheap estimate of the mean —
// used as the re-centering offset on Spherical / Hyperbolic.
float Geometry::computeRecenterOffset(
    const std::vector<std::shared_ptr<Shape>>& bodies,
    const WarpParams& warp)
{
    const std::vector<Vertex>& base = baseGrid();
    if ( base.empty() )
        return 0.0f;

    const size_t stride = std::max<size_t>(1, base.size() / 4096);
    double sum = 0.0;
    size_t count = 0;
    for ( size_t v = 0; v < base.size(); v += stride )
    {
        sum += warpDepth(base[v].position, bodies, warp);
        ++count;
    }
    return count ? static_cast<float>(sum / static_cast<double>(count)) : 0.0f;
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
        obj.position += halfV; // x(t + dt) = x(t) + halfV
}

void FlatGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<std::shared_ptr<Shape>>& bodies,
    const WarpParams& warp,
    float recenterOffset)
{
    const std::vector<Vertex>& base = baseGrid();

#pragma omp parallel for
    for ( size_t v = 0; v < base.size(); ++v )
    {
        Vertex vertex = base[v];
        const float rawDepth = warpDepth(vertex.position, bodies, warp);

        vertex.position.y -= rawDepth - recenterOffset;   // wells dip downward
        vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        vertex.color = depthColor(rawDepth, warp.wellDepth);

        vertices[v] = vertex;
    }
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

        // Enforce spherical constraint: project back onto the sphere. Guard
        // the normalize — if a body's step lands it exactly at the centre,
        // normalize() would yield NaN and the body would vanish.
        obj.position += halfV;
        const float len = glm::length(obj.position);
        if ( len > 1e-4f )
            obj.position = obj.position / len * radius;
        else
            obj.position = pole;   // degenerate: snap to a pole instead of NaN

        // Recompute normal and ensure velocity remains tangential
        // normal = glm::normalize(newPos - pole);
        // radialVel = glm::dot(obj.velocity, normal);
        // obj.velocity -= radialVel * normal;
    }
}

void SphericalGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<std::shared_ptr<Shape>>& bodies,
    const WarpParams& warp,
    float recenterOffset)
{
    const std::vector<Vertex>& base = baseGrid();
    const float R = m_GridScale / 2.0f; // base sphere radius

#pragma omp parallel for
    for ( size_t v = 0; v < base.size(); ++v )
    {
        Vertex vertex = base[v];

        // Wells dimple the sphere inward. The re-centering offset removes the
        // (near-uniform) contribution of the central mass — which sits at the
        // sphere's centre — so the manifold keeps its average radius R.
        const float rawDepth = warpDepth(vertex.position, bodies, warp);

        const glm::vec3 radialDir = glm::normalize(vertex.position);
        const float newRadius = std::max(R - (rawDepth - recenterOffset), 0.3f * R);
        vertex.position = radialDir * newRadius;
        vertex.normal = radialDir;
        vertex.color = depthColor(rawDepth, warp.wellDepth);

        vertices[v] = vertex;
    }
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

    }
}

// Hyperbolic measures the warp distance along the paraboloid surface rather
// than the straight-line Euclidean distance the base implementation uses.
float HyperbolicGeometry::warpDepth(
    const glm::vec3& basePos,
    const std::vector<std::shared_ptr<Shape>>& bodies,
    const WarpParams& warp) const
{
    const float k = m_GridScale; // paraboloid scale: y = (x^2 - z^2) / k
    float sum = 0.0f;
    for ( const auto& shape : bodies )
    {
        const float mass = shape->m_Object.mass;
        if ( mass <= 0.0f )
            continue;
        const glm::vec3 b = shape->m_Object.position;
        const glm::vec3 diff(
            b.x - basePos.x,
            (b.x * b.x - b.z * b.z) / k - (basePos.x * basePos.x - basePos.z * basePos.z) / k,
            b.z - basePos.z);
        const float m04   = std::pow(mass, kMassExponent);
        const float width = 1.0f + warp.radialInfluence * m04;
        const float rEff  = glm::length(diff) / width;
        const float soft  = std::sqrt(rEff * rEff + warp.softening * warp.softening);
        sum += m04 / soft;
    }
    return warp.wellDepth * std::tanh(warp.warpGain * sum) * edgeFade(basePos);
}

void HyperbolicGeometry::warpGrid(
    std::vector<Vertex>& vertices,
    const std::vector<std::shared_ptr<Shape>>& bodies,
    const WarpParams& warp,
    float recenterOffset)
{
    const std::vector<Vertex>& base = baseGrid();
    const float k = m_GridScale; // paraboloid scale: y = (x^2 - z^2) / k

#pragma omp parallel for
    for ( size_t v = 0; v < base.size(); ++v )
    {
        Vertex vertex = base[v];
        const float rawDepth = warpDepth(vertex.position, bodies, warp);

        // Displace along the surface normal of the paraboloid.
        const float x = vertex.position.x;
        const float z = vertex.position.z;
        const glm::vec3 surfaceNormal = glm::normalize(glm::vec3(-2.0f * x / k, 1.0f, 2.0f * z / k));
        vertex.position -= (rawDepth - recenterOffset) * surfaceNormal;
        vertex.normal = surfaceNormal;
        vertex.color = depthColor(rawDepth, warp.wellDepth);

        vertices[v] = vertex;
    }
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
                const float sinAlpha = glm::clamp(distXZ / radius, 0.0f, 1.0f);
                // Polar angle from the north pole. asin only covers [0, π/2];
                // for the southern hemisphere (y < 0) continue past the equator
                // so a point on the sphere maps to flat y = 0 in BOTH halves.
                float alpha = asin(sinAlpha);
                if ( coordinates.y < 0.0f )
                    alpha = glm::pi<float>() - alpha;
                const float theta = atan2(coordinates.z, coordinates.x);
                const float flatDistXZ = alpha * radius;
                // cos(alpha) is now signed, so on-sphere bodies get yOffset 0
                // instead of the spurious ~-2R that dropped southern bodies.
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
