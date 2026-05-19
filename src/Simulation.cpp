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

    // Step 4: inelastic merging — bodies that touch coalesce into one. This
    // doubles as accretion and as a cure for close-encounter slingshots. One
    // merge per step keeps the index bookkeeping simple (merges are rare).
    if ( m_MergeEnabled && m_Bodies.size() > 1 )
    {
        const float mergeDist = m_GridScale * 0.02f;   // collision radius
        bool merged = false;
        for ( size_t i = 0; i < m_Bodies.size() && !merged; ++i )
        {
            for ( size_t j = i + 1; j < m_Bodies.size() && !merged; ++j )
            {
                Object& a = m_Bodies[i]->m_Object;
                Object& b = m_Bodies[j]->m_Object;
                if ( glm::length(a.position - b.position) >= mergeDist )
                    continue;

                // Conserve mass and momentum; the merged body keeps slot i.
                const float m = a.mass + b.mass;
                a.position = (a.mass * a.position + b.mass * b.position) / m;
                a.velocity = (a.mass * a.velocity + b.mass * b.velocity) / m;
                a.acceleration = glm::vec3(0.0f);
                a.mass = m;
                m_Bodies[i]->setSize(std::max(1.0f + std::log(m / std::sqrt(m_GridScale)), 0.6f));

                m_Bodies.erase(m_Bodies.begin() + static_cast<long>(j));
                m_Trails.erase(m_Trails.begin() + static_cast<long>(j));
                m_BodiesChanged = true;
                merged = true;
            }
        }
    }

    // Conserved-quantity bookkeeping (post-merge).
    m_KineticEnergy = 0.0f;
    m_PotentialEnergy = 0.0f;
    glm::vec3 angMom(0.0f);
    for ( const auto& shape : m_Bodies )
    {
        const Object& obj = shape->m_Object;
        m_KineticEnergy += 0.5f * obj.mass * glm::length2(obj.velocity);
        angMom += obj.mass * glm::cross(obj.position, obj.velocity);
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
    m_AngularMomentum = glm::length(angMom);

    // Step 5: advance the null geodesics (light rays).
    advanceRays(deltaTime);
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

    // Rays were traced on the old manifold — drop them on a geometry change.
    clearRays();
}

// ============================ NULL GEODESICS ================================

void Simulation::clearRays()
{
    m_Rays.clear();
}

// Emits a fan of parallel light rays suited to the active geometry: a straight
// line of rays crossing the grid (Flat / Hyperbolic), or a spray of meridians
// rising from the equator (Spherical).
void Simulation::emitRays()
{
    m_Rays.clear();
    const int count = std::max(m_RayCount, 1);
    const float R = m_GridScale * 0.75f;

    for ( int k = 0; k < count; ++k )
    {
        const float frac = count > 1 ? static_cast<float>(k) / (count - 1) : 0.75f;
        LightRay ray;

        if ( m_GeometryType == GeometryType::Spherical )
        {
            // Meridians: start spread along the equator, all heading north.
            // Parallel at the equator, they converge at the pole.
            const float lon = (frac - 0.5f) * 1.4f;   // longitude spread (rad)
            ray.position  = R * glm::vec3(std::cos(lon), 0.0f, std::sin(lon));
            ray.direction = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        else
        {
            // Flat / Hyperbolic: a line of rays on one side, crossing in +x.
            const float spread = 0.30f * m_GridScale;
            const float z = (frac - 0.5f) * 2.0f * spread;
            const float x = -0.42f * m_GridScale;
            if ( m_GeometryType == GeometryType::Hyperbolic )
            {
                const float kPara = m_GridScale;
                ray.position  = glm::vec3(x, (x * x - z * z) / kPara, z);
                ray.direction = glm::normalize(glm::vec3(1.0f, 2.0f * x / kPara, 0.0f));
            }
            else
            {
                ray.position  = glm::vec3(x, 0.0f, z);
                ray.direction = glm::vec3(1.0f, 0.0f, 0.0f);
            }
        }

        ray.trail.push_back(liftRayPoint(ray.position));
        m_Rays.push_back(std::move(ray));
    }
}

// Lifts a ray's base-manifold position onto the warped surface (slightly clear
// of the grid) so the traced path rides the potential wells. Called once per
// point as it is traced — never re-evaluated for the whole trail history.
glm::vec3 Simulation::liftRayPoint(const glm::vec3& basePos) const
{
    const float rayLift = m_GridScale * 0.005f;
    const float raw = m_Geometry->warpDepth(basePos, m_Bodies, m_RayWarp);
    return m_Geometry->warpedPosition(basePos, raw - rayLift, m_RayRecenter);
}

// Advances every active ray one geodesic step along the manifold, optionally
// deflected by the masses (lensing). Speed is renormalised each step so the
// path stays a constant-speed (null) geodesic — light bends but never speeds up.
void Simulation::advanceRays(float deltaTime)
{
    if ( m_Rays.empty() )
        return;

    const float R = m_GridScale * 0.5f;
    const float kPara = m_GridScale;
    const float step = m_RaySpeed * deltaTime;
    const float bound = 0.55f * m_GridScale;
    constexpr float soft2 = 4.0f;   // softening^2 for the lensing acceleration
#pragma omp parallel for
    for ( auto& ray : m_Rays )
    {
        if ( !ray.active )
            continue;

        glm::vec3 pos = ray.position;
        glm::vec3 dir = ray.direction;

        // --- Lensing: deflect the direction toward the masses ---------------
        if ( m_LensingEnabled )
        {
            glm::vec3 accel(0.0f);
            for ( const auto& shape : m_Bodies )
            {
                const Object& b = shape->m_Object;
                const glm::vec3 toBody = b.position - pos;
                const float d2 = glm::dot(toBody, toBody) + soft2;
                accel += m_RayLensStrength * b.mass * toBody / (d2 * std::sqrt(d2));
            }
            dir += accel * deltaTime;   // renormalised below, so speed is unchanged
        }

        // --- Geodesic step on the active manifold ---------------------------
        if ( m_GeometryType == GeometryType::Spherical )
        {
            // Keep the direction tangent to the sphere, then rotate position
            // and direction together along the great circle (exact).
            const glm::vec3 n = glm::normalize(pos);
            dir = dir - glm::dot(dir, n) * n;
            if ( glm::length(dir) < 1e-6f ) { ray.active = false; continue; }
            dir = glm::normalize(dir);

            const float theta = step / R;
            const glm::vec3 newPos = pos * std::cos(theta) + R * dir * std::sin(theta);
            const glm::vec3 newDir = dir * std::cos(theta) - (pos / R) * std::sin(theta);
            pos = newPos;
            dir = glm::normalize(newDir);
        }
        else if ( m_GeometryType == GeometryType::Hyperbolic )
        {
            dir = glm::normalize(dir);
            pos += dir * step;
            pos.y = (pos.x * pos.x - pos.z * pos.z) / kPara;   // snap to paraboloid
            // Re-tangent the direction to the new surface point.
            const glm::vec3 nrm =
                glm::normalize(glm::vec3(-2.0f * pos.x / kPara, 1.0f, 2.0f * pos.z / kPara));
            dir = dir - glm::dot(dir, nrm) * nrm;
            if ( glm::length(dir) < 1e-6f ) { ray.active = false; continue; }
            dir = glm::normalize(dir);
        }
        else
        {
            // Flat: a straight line in the y = 0 plane.
            dir.y = 0.0f;
            if ( glm::length(dir) < 1e-6f ) { ray.active = false; continue; }
            dir = glm::normalize(dir);
            pos += dir * step;
            pos.y = 0.0f;
        }

        ray.position  = pos;
        ray.direction = dir;
        ray.trail.push_back(liftRayPoint(pos));   // lift once, store lifted
        if ( ray.trail.size() > LightRay::maxPoints )
            ray.trail.pop_front();

        // Open manifolds: stop a ray once it leaves the grid footprint.
        if ( m_GeometryType != GeometryType::Spherical &&
             (std::abs(pos.x) > bound || std::abs(pos.z) > bound) )
            ray.active = false;
    }
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
