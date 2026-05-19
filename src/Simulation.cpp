//
// CPU N-body gravitational simulation: bodies, integration, energy, trails.
//

#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <glm/gtc/constants.hpp>

namespace
{
    std::string vecToString(const glm::vec3& v)
    {
        std::stringstream ss;
        ss << v.x << " " << v.y << " " << v.z;
        return ss.str();
    }
}

Simulation::Simulation(GeometryType type, int gridSize, float gridScale, float gravity)
    : m_Geometry(geometryFactory(type, gridSize, gridScale))
    , m_GeometryType(type)
    , m_GridSize(gridSize)
    , m_GridScale(gridScale)
    , m_Gravity(gravity)
{
    spawnBodies(type);

    m_Trails.resize(m_Bodies.size());
}

void Simulation::spawnBodies(GeometryType type)
{
    // Central massive body at the origin.
    Object centralObj{};
    centralObj.mass = 20000.0; // Very massive
    centralObj.position = glm::vec3(0.0, 0.0, 0.0);
    centralObj.velocity = glm::vec3(0.0); // Stationary
    auto centralShape = std::make_shared<Sphere>(centralObj);
    m_Bodies.push_back(centralShape);
    centralShape->setSize(1.0 + std::log(centralShape->m_Object.mass / std::sqrt(m_GridScale)));

    // Orbiting bodies (many, to stress the GPU compute warp loop).
    constexpr int numOrbiters = 40;
    const float G = m_Gravity;
    const float centralMass = centralObj.mass;

    // Keep the whole orbiter spread inside the grid (half-extent 0.5*gridScale):
    // the outermost orbiter lands near 0.40*gridScale, with margin to spare.
    const float baseRadius = m_GridScale * 0.06f;
    const float radiusStep = (m_GridScale * 0.34f) / numOrbiters;

    // Randomised orbiter masses for more natural / interesting warping wells.
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> massDist(3.0f, 72.0f);

    for ( int i = 0; i < numOrbiters; ++i )
    {
        Object orbiter{};
        orbiter.mass = massDist(rng);

        // Circular orbit parameters.
        float radius = baseRadius + i * radiusStep;
        float angle = static_cast<float>(i) * 2.0 * glm::pi<float>() / numOrbiters;
        glm::vec3 rawPosition(radius * cos(angle), 0.0, radius * sin(angle));
        glm::vec3 position = convertCoordinates(
            rawPosition, GeometryType::Flat, type, m_GridSize / 2, m_Geometry);
        orbiter.position = position;

        float actualRadius = glm::length(position);
        float v = std::sqrt(G * centralMass / actualRadius);
        glm::vec3 radialDir = glm::normalize(position);
        glm::vec3 tangentDir(-radialDir.z, 0.0, radialDir.x);
        if ( glm::length(tangentDir) < 1e-6 )
            tangentDir = glm::vec3(0.0, 0.0, 1.0);
        orbiter.velocity = m_OrbitFactor * v * glm::normalize(tangentDir);

        m_Bodies.push_back(std::make_shared<Sphere>(orbiter));
        auto& obj = m_Bodies.back();
        obj->setSize(1.0 + std::log(obj->m_Object.mass / std::sqrt(m_GridScale)));
        std::cout << "Object " << obj->getName()
            << " Mass: " << obj->m_Object.mass << " w Radius: " << actualRadius
            << " G = " << G << " Orbit = " << centralMass
            << " Pos: " << vecToString(rawPosition) << " vs. " << vecToString(tangentDir)
            << " Vel: " << vecToString(obj->m_Object.velocity) << " / " << v << "\n";
    }
}

void Simulation::step(float deltaTime)
{
    const float R = m_GridScale / 2.0f;
    deltaTime = std::min(deltaTime, m_TimeStep);

    // Step 1: velocity-Verlet half-step (kick + drift) and reset accelerations.
    #pragma omp parallel for schedule(dynamic)
    for ( size_t i = 0; i < m_Bodies.size(); ++i )
    {
        auto& obj = m_Bodies[i]->m_Object;
        m_Geometry->updatePosition(obj, deltaTime, R, true);
        obj.acceleration = glm::vec3(0.0f);
    }

    // Step 2: symmetric pairwise gravitational forces (OpenMP, thread-safe).
    constexpr float softeningLength = 1.0f;
    const size_t nBodies = m_Bodies.size();
    std::vector<glm::vec3> deltaAcc(nBodies, glm::vec3(0.0f));

    #pragma omp parallel for schedule(dynamic)
    for ( size_t i = 0; i < nBodies; ++i )
    {
        for ( size_t j = i + 1; j < nBodies; ++j )
        {   // Only j > i, to avoid double-counting.
            auto& shape1 = m_Bodies[i];
            auto& shape2 = m_Bodies[j];

            glm::vec3 pos1 = shape1->m_Object.position;
            glm::vec3 pos2 = shape2->m_Object.position;
            float dist = m_Geometry->computeDistance(pos1, pos2);
            if ( dist < 0.01f )
                dist = 0.01f;

            glm::vec3 direction;
            if ( m_GeometryType == GeometryType::Spherical )
            {
                // Chord direction (straight line in 3D space).
                glm::vec3 r = pos2 - pos1;
                float chordDist = glm::length(r);
                if ( chordDist < 0.01f )
                    continue;

                // Geodesic (great-circle) distance. The dot of two unit
                // vectors can drift just past ±1 from float error, and
                // acos() of that is NaN — which would poison the whole
                // force chain and fling the body off the sphere. Clamp it.
                const float cosAngle =
                    glm::clamp(glm::dot(glm::normalize(pos1), glm::normalize(pos2)), -1.0f, 1.0f);
                float centralAngle = acos(cosAngle);
                dist = R * centralAngle;
                if ( dist < 0.01f )
                    dist = 0.01f;

                // Project the force direction onto the tangent plane at pos1.
                direction = glm::normalize(r);
                glm::vec3 normal1 = glm::normalize(pos1);
                float radialComponent = glm::dot(direction, normal1);
                if ( std::isnan(radialComponent) )
                    radialComponent = 0.0f;
                direction -= radialComponent * normal1;
                if ( glm::length(direction) < 1e-6f )
                    continue;
                direction = glm::normalize(direction);
            }
            else if ( m_GeometryType == GeometryType::Hyperbolic )
            {
                const float k = 2.0f * m_GridScale;
                glm::vec3 diff(pos2.x - pos1.x,
                               (pos2.x * pos2.x - pos2.z * pos2.z) / k - (pos1.x * pos1.x - pos1.z * pos1.z) / k,
                               pos2.z - pos1.z);
                direction = glm::normalize(diff);

                // Project onto the paraboloid tangent plane.
                glm::vec3 normal(2.0f * pos1.x / k, -1.0f, -2.0f * pos1.z / k);
                normal = glm::normalize(normal);
                direction -= glm::dot(direction, normal) * normal;
                direction = glm::normalize(direction);
            }
            else
            {
                direction = glm::normalize(pos2 - pos1);
            }

            float softenedDist = std::sqrt(dist * dist + softeningLength * softeningLength);

            // Force on shape1 due to shape2.
            float force1 = m_Gravity * shape2->m_Object.mass / (softenedDist * softenedDist);
            deltaAcc[i] += force1 * direction;

            // Equal and opposite force on shape2.
            float force2 = m_Gravity * shape1->m_Object.mass / (softenedDist * softenedDist);
            #pragma omp atomic
            deltaAcc[j].x -= (force2 * direction).x;
            #pragma omp atomic
            deltaAcc[j].y -= (force2 * direction).y;
            #pragma omp atomic
            deltaAcc[j].z -= (force2 * direction).z;
        }
    }

    // Merge the accumulated accelerations back into the bodies.
    #pragma omp parallel for schedule(dynamic)
    for ( size_t i = 0; i < nBodies; ++i )
        m_Bodies[i]->m_Object.acceleration += deltaAcc[i];

    // Step 3: second Verlet half-step + append to motion trails.
    for ( size_t i = 0; i < m_Bodies.size(); ++i )
    {
        auto& shape = m_Bodies[i];
        m_Geometry->updatePosition(shape->m_Object, deltaTime, R, false);

        auto& positions = m_Trails[i].positions;
        positions.push_back(shape->m_Object.position);
        if ( positions.size() > Trail::maxPoints )
            positions.pop_front();
    }

    // Conserved-quantity bookkeeping.
    m_KineticEnergy = 0.0f;
    m_PotentialEnergy = 0.0f;
    for ( const auto& shape : m_Bodies )
    {
        const float speedSquared = glm::length2(shape->m_Object.velocity);
        m_KineticEnergy += 0.5f * shape->m_Object.mass * speedSquared;
    }
    for ( size_t i = 0; i < m_Bodies.size(); ++i )
    {
        for ( size_t j = i + 1; j < m_Bodies.size(); ++j )
        {
            const glm::vec3 pos1 = m_Bodies[i]->m_Object.position;
            const glm::vec3 pos2 = m_Bodies[j]->m_Object.position;
            float dist = m_Geometry->computeDistance(pos1, pos2);
            float softenedDist = std::sqrt(dist * dist + softeningLength * softeningLength);
            m_PotentialEnergy -=
                m_Gravity * m_Bodies[i]->m_Object.mass * m_Bodies[j]->m_Object.mass / softenedDist;
        }
    }
    m_TotalEnergy = m_KineticEnergy + m_PotentialEnergy;
}

void Simulation::switchGeometry(GeometryType type)
{
    if ( m_GeometryType == type )
        return;

    constexpr float mu = 0.0f;
    constexpr float dist = 0.0f;

    m_Geometry = geometryFactory(type, m_GridSize, m_GridScale);
    m_Geometry->setGridParams(m_GridSize, m_GridScale);
    const float R = m_GridScale / 2.0f;

    // Convert body positions and velocities onto the new geometry.
    for ( auto& shape : m_Bodies )
    {
        Object& obj = shape->m_Object;
        const glm::vec3 oldPos = obj.position;
        const glm::vec3 oldVel = obj.velocity;

        const glm::vec3 newPos = convertCoordinates(oldPos, m_GeometryType, type, R, m_Geometry);
        const glm::vec3 newVel =
            convertVelocity(oldPos, oldVel, m_GeometryType, type, R, dist, mu, m_Geometry);

        obj.position = newPos;
        obj.velocity = newVel;
        obj.acceleration = glm::vec3(0.0f);

        std::cout << "Object at (" << oldPos.x << ", " << oldPos.y << ", " << oldPos.z
                  << ") -> (" << newPos.x << ", " << newPos.y << ", " << newPos.z
                  << "), Vel (" << oldVel.x << ", " << oldVel.y << ", " << oldVel.z
                  << ") -> (" << newVel.x << ", " << newVel.y << ", " << newVel.z << ")"
                  << " w radius = " << glm::length(newPos) << "\n";
    }

    // Convert the trail histories too, so they stay on-surface.
    for ( auto& trail : m_Trails )
        for ( auto& pos : trail.positions )
            pos = convertCoordinates(pos, m_GeometryType, type, R, m_Geometry);

    m_GeometryType = type;
}

void Simulation::resetTwoBody()
{
    m_Bodies.clear();

    Object cubeObj{};
    cubeObj.mass = 10.0f;
    cubeObj.position = glm::vec3(-5.0f, 2.0f, 0.0f);
    cubeObj.velocity = glm::vec3(0.5f, 0.0f, 0.0f);
    m_Bodies.push_back(std::make_shared<Cube>(cubeObj));

    Object sphereObj{};
    sphereObj.mass = 5.0f;
    sphereObj.position = glm::vec3(5.0f, 2.0f, 0.0f);
    sphereObj.velocity = glm::vec3(-0.5f, 0.0f, 0.0f);
    m_Bodies.push_back(std::make_shared<Sphere>(sphereObj));

    // Keep one trail per body (previously left stale at the old body count).
    m_Trails.assign(m_Bodies.size(), Trail{});
}

void Simulation::resetToOrbit()
{
    if ( m_Bodies.size() < 2 )
        return;

    // Center of mass.
    glm::vec3 com(0.0f);
    float totalMass = 0.0f;
    for ( const auto& shape : m_Bodies )
    {
        com += shape->m_Object.mass * shape->m_Object.position;
        totalMass += shape->m_Object.mass;
    }
    if ( totalMass > 0.0f )
        com /= totalMass;

    // Recenter the bodies on the COM (and re-project onto the sphere).
    for ( auto& shape : m_Bodies )
    {
        shape->m_Object.position -= com;

        if ( m_GeometryType == GeometryType::Spherical )
        {
            const float R = static_cast<float>(m_GridSize) / 2.0f;
            const glm::vec3 current = shape->m_Object.position;
            const float len = glm::length(current);
            shape->m_Object.position = ( len < 0.01f )
                ? glm::vec3(0.0f, 0.0f, R)
                : glm::normalize(current) * R;
        }
    }

    // Give the first two bodies an orbital velocity about their shared COM.
    const float mu = m_Gravity * (m_Bodies[0]->m_Object.mass + m_Bodies[1]->m_Object.mass);
    float r = m_Geometry->computeDistance(m_Bodies[0]->m_Object.position,
                                          m_Bodies[1]->m_Object.position);
    if ( r < 0.01f )
        r = 0.01f;

    const glm::vec3 pos0 = m_Bodies[0]->m_Object.position;
    const glm::vec3 pos1 = m_Bodies[1]->m_Object.position;
    const glm::vec3 r_vec = pos1 - pos0;

    glm::vec3 tangent;
    if ( m_GeometryType == GeometryType::Spherical )
    {
        const glm::vec3 avgRadial =
            glm::normalize(glm::normalize(pos0) + glm::normalize(pos1));
        tangent = glm::normalize(glm::cross(r_vec, avgRadial));
    }
    else
    {
        // Flat / Hyperbolic: tangent in the x-y plane.
        tangent = glm::normalize(glm::vec3(-r_vec.y, r_vec.x, 0.0f));
    }

    const float v = std::sqrt(mu / r);
    m_Bodies[0]->m_Object.velocity =  tangent * (v * m_Bodies[1]->m_Object.mass / totalMass);
    m_Bodies[1]->m_Object.velocity = -tangent * (v * m_Bodies[0]->m_Object.mass / totalMass);

    for ( size_t i = 2; i < m_Bodies.size(); ++i )
        m_Bodies[i]->m_Object.velocity = glm::vec3(0.0f);
}
