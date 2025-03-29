//
// Created by Richard Mule on 3/23/25.
//

#include "Geometry.hpp"
#include <iostream>
#include <ranges>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/fast_trigonometry.hpp>

std::ostream& operator<<(std::ostream& os, const GeometryType& type)
{
    const char* geometryItems[] = {"Flat", "Spherical", "Hyperbolic"};
    int currentGeometry = static_cast<int>(type);
    os << geometryItems[currentGeometry];
    return os;
}

GeometryShader::GeometryShader(const vk::Device& device, const vk::PhysicalDevice& physicalDevice)
    : m_Device(device)
    , m_PhysicalDevice(physicalDevice)
{
    initializeAll();
}

GeometryShader::~GeometryShader()
{
    for ( const auto& [vertexShader, fragmentShader] : m_ShaderPrograms | std::views::values )
    {
        m_Device.destroyShaderModule(vertexShader);
        m_Device.destroyShaderModule(fragmentShader);
    }
}

void GeometryShader::loadShader(const GeometryType type, const std::string& vertexPath, const std::string& fragmentPath)
{
    const std::vector<char> vertexCode = utils::readFile(vertexPath);
    const std::vector<char> fragmentCode = utils::readFile(fragmentPath);

    vk::ShaderModuleCreateInfo vertexInfo{};
    vertexInfo.sType = vk::StructureType::eShaderModuleCreateInfo;
    vertexInfo.codeSize = vertexCode.size();
    vertexInfo.pCode = reinterpret_cast<const uint32_t*>(vertexCode.data());
    const vk::ShaderModule vertexShader = m_Device.createShaderModule(vertexInfo);

    vk::ShaderModuleCreateInfo fragmentInfo{};
    fragmentInfo.sType = vk::StructureType::eShaderModuleCreateInfo;
    fragmentInfo.codeSize = fragmentCode.size();
    fragmentInfo.pCode = reinterpret_cast<const uint32_t*>(fragmentCode.data());
    const vk::ShaderModule fragmentShader = m_Device.createShaderModule(fragmentInfo);

    m_ShaderPrograms[type] = { vertexShader, fragmentShader };
}

void GeometryShader::initializeAll()
{
    loadShader(GeometryType::Flat, "flat_grid.comp.spv", "grid.frag.spv");
    loadShader(GeometryType::Spherical, "spherical_grid.comp.spv", "grid.frag.spv");
    loadShader(GeometryType::Hyperbolic, "hyperbolic_grid.comp.spv", "grid.frag.spv");
}

const ShaderProgram& GeometryShader::getShaderProgram(const GeometryType type) const
{
    const auto it = m_ShaderPrograms.find(type);
    std::cout << "GETTING SHADER PROGRAM: " << type << std::endl;
    if ( it != m_ShaderPrograms.end() )
        return it->second;
    throw std::runtime_error("Shader program not found for geometry type");
}

void GeometryShader::setGPUBuffers(
    const vk::Buffer& vertexBuffer,
    const vk::DeviceMemory& vertexBufferMemory,
    const vk::Buffer& indexBuffer,
    const vk::DeviceMemory& indexBufferMemory)
{
    m_VertexBuffer = vertexBuffer;
    m_VertexBufferMemory = vertexBufferMemory;
    m_IndexBuffer = indexBuffer;
    m_IndexBufferMemory = indexBufferMemory;
}

void GeometryShader::setComputePipeline(
    const vk::CommandPool& commandPool,
    const vk::Queue& computeQueue,
    const vk::Pipeline& pipeline,
    const vk::PipelineLayout& pipelineLayout,
    const vk::DescriptorSetLayout& descriptorSetLayout,
    const vk::DescriptorSet& descriptorSet)
{
    m_CommandPool = commandPool;
    m_ComputeQueue = computeQueue;
    m_ComputePipeline = pipeline;
    m_PipelineLayout = pipelineLayout;
    m_DescriptorSetLayout = descriptorSetLayout;
    m_DescriptorSet = descriptorSet;
}

vk::CommandBuffer GeometryShader::beginSingleTimeCommands() const
{
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.commandPool = m_CommandPool;
    allocInfo.level = vk::CommandBufferLevel::ePrimary;
    allocInfo.commandBufferCount = 1;

    vk::CommandBuffer commandBuffer;
    if ( m_Device.allocateCommandBuffers(&allocInfo, &commandBuffer) != vk::Result::eSuccess )
        throw std::runtime_error("Failed to allocate single-time command buffer");

    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    auto _ = commandBuffer.begin(&beginInfo);

    return commandBuffer;
}

void GeometryShader::endSingleTimeCommands(vk::CommandBuffer commandBuffer) const
{
    commandBuffer.end();

    vk::SubmitInfo submitInfo{};
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    auto _ = m_ComputeQueue.submit(1, &submitInfo, nullptr);
    m_ComputeQueue.waitIdle(); // Sync for simplicity; use fences later

    m_Device.freeCommandBuffers(m_CommandPool, 1, &commandBuffer);
}

void GeometryShader::dispatchComputeShader(const size_t gridSize, float gridScale) const
{
    // Create a command buffer
    const vk::CommandBuffer commandBuffer = beginSingleTimeCommands();

    // Begin command buffer
    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.sType = vk::StructureType::eCommandBufferBeginInfo;
    beginInfo.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    auto _ = commandBuffer.begin(&beginInfo);

    // Bind the compute pipeline
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, m_ComputePipeline);

    // Bind descriptor set
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eCompute,
        m_PipelineLayout,
        0,
        1,
        &m_DescriptorSet,
        0,
        nullptr);

    // Set push constants
    commandBuffer.pushConstants(
        m_PipelineLayout,
        vk::ShaderStageFlagBits::eCompute,
        0,
        sizeof(int),
        &gridSize);
    commandBuffer.pushConstants(
        m_PipelineLayout,
        vk::ShaderStageFlagBits::eCompute,
        sizeof(int),
        sizeof(float),
        &gridScale);

    // Dispatch compute shader
    constexpr uint32_t groupSize = 32;
    const uint32_t groupsX = (gridSize + 1 + groupSize - 1) / groupSize; // 16
    const uint32_t groupsY = (gridSize + 1 + groupSize - 1) / groupSize; // 16
    commandBuffer.dispatch(groupsX, groupsY, 1);
    // commandBuffer.dispatch(gridSize + 1, gridSize  + 1, 1);

    // End command buffer
    commandBuffer.end();

    // Submit to queue
    vk::SubmitInfo submitInfo{};
    submitInfo.sType = vk::StructureType::eSubmitInfo;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    _ = m_ComputeQueue.submit(1, &submitInfo, nullptr);
    m_ComputeQueue.waitIdle(); // Wait for completion (for simplicity; use fences in production)

    // Free command buffer
    m_Device.freeCommandBuffers(m_CommandPool, 1, &commandBuffer);
}

void GeometryShader::copyBuffersToCPU(
    std::vector<Vertex>& vertices,
    std::vector<uint32_t>& indices,
    size_t vertexCount,
    size_t indexCount) const
{
    // Resize the output vectors
    vertices.resize(vertexCount);
    indices.resize(indexCount);

    std::cout << "CHECK SIZES: " << vertexCount << "/" << indexCount << "\n";
    const vk::DeviceSize vertexBufferSize = sizeof(Vertex) * vertexCount; // Now 48 * 251001
    vk::Buffer stagingBuffer = createBuffer(vertexBufferSize, vk::BufferUsageFlagBits::eTransferDst);
    const vk::DeviceMemory stagingMemory = allocateBufferMemory(
        stagingBuffer, vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    m_Device.bindBufferMemory(stagingBuffer, stagingMemory, 0);

    const vk::CommandBuffer cmd = beginSingleTimeCommands();
    vk::BufferCopy copyRegion{};
    copyRegion.setSize(vertexBufferSize);
    cmd.copyBuffer(m_VertexBuffer, stagingBuffer, 1, &copyRegion);
    endSingleTimeCommands(cmd);

    void* vertexData;
    auto _ = m_Device.mapMemory(stagingMemory, 0, vertexBufferSize, {}, &vertexData);
    memcpy(vertices.data(), vertexData, vertexBufferSize);
    m_Device.unmapMemory(stagingMemory);

    m_Device.destroyBuffer(stagingBuffer);
    m_Device.freeMemory(stagingMemory);

    // Copy index buffer back to CPU
    stagingBuffer = createBuffer(indexCount * sizeof(uint32_t), vk::BufferUsageFlagBits::eTransferDst);
    const vk::DeviceMemory stagingBufferMemory = allocateBufferMemory(
        stagingBuffer,
        vk::MemoryPropertyFlagBits::eHostVisible |
        vk::MemoryPropertyFlagBits::eHostCoherent);
    m_Device.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);

    copyBuffer(
        m_Device,
        m_CommandPool,
        m_ComputeQueue,
        m_IndexBuffer,
        stagingBuffer,
        indexCount * sizeof(uint32_t));

    void* indexData;
    _ = m_Device.mapMemory(stagingBufferMemory, 0, indexCount * sizeof(uint32_t), {}, &indexData);
    memcpy(indices.data(), indexData, indexCount * sizeof(uint32_t));
    m_Device.unmapMemory(stagingBufferMemory);

    m_Device.destroyBuffer(stagingBuffer);
    m_Device.freeMemory(stagingBufferMemory);
}

vk::Buffer GeometryShader::createBuffer(const vk::DeviceSize size, const vk::BufferUsageFlags usage) const
{
    vk::BufferCreateInfo bufferInfo{};
    bufferInfo.sType = vk::StructureType::eBufferCreateInfo;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = vk::SharingMode::eExclusive;
    const vk::Buffer buffer = m_Device.createBuffer(bufferInfo);
    if ( !buffer )
        throw std::runtime_error("Failed to create buffer");
    return buffer;
}

vk::DeviceMemory GeometryShader::allocateBufferMemory(
    const vk::Buffer buffer,
    const vk::MemoryPropertyFlags properties) const
{
    const vk::MemoryRequirements memRequirements = m_Device.getBufferMemoryRequirements(buffer);

    vk::MemoryAllocateInfo allocInfo{};
    allocInfo.sType = vk::StructureType::eMemoryAllocateInfo;
    allocInfo.allocationSize = memRequirements.size;

    // Find memory type
    const vk::PhysicalDeviceMemoryProperties memProperties = m_PhysicalDevice.getMemoryProperties();
    uint32_t memoryTypeIndex = -1;
    for ( uint32_t i = 0; i < memProperties.memoryTypeCount; ++i )
    {
        if ( memRequirements.memoryTypeBits & 1 << i &&
            (memProperties.memoryTypes[i].propertyFlags & properties) == properties )
        {
            memoryTypeIndex = i;
            break;
        }
    }

    if ( memoryTypeIndex == -1 )
        throw std::runtime_error("Failed to find suitable memory type");

    allocInfo.memoryTypeIndex = memoryTypeIndex;

    return m_Device.allocateMemory(allocInfo);
}

void GeometryShader::copyBuffer(
    const vk::Device& device,
    const vk::CommandPool& commandPool,
    const vk::Queue& queue,
    const vk::Buffer& srcBuffer,
    const vk::Buffer& dstBuffer,
    const vk::DeviceSize& size)
{
    // Create a command buffer
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = vk::StructureType::eCommandBufferAllocateInfo;
    allocInfo.commandPool = commandPool;
    allocInfo.level = vk::CommandBufferLevel::ePrimary;
    allocInfo.commandBufferCount = 1;

    vk::CommandBuffer commandBuffer;
    auto _ = device.allocateCommandBuffers(&allocInfo, &commandBuffer);

    // Begin command buffer
    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.sType = vk::StructureType::eCommandBufferBeginInfo;
    beginInfo.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    _ = commandBuffer.begin(&beginInfo);

    // Copy buffer
    vk::BufferCopy copyRegion{};
    copyRegion.size = size;
    commandBuffer.copyBuffer(srcBuffer, dstBuffer, copyRegion);

    // End command buffer
    commandBuffer.end();

    // Submit to queue
    vk::SubmitInfo submitInfo{};
    submitInfo.sType = vk::StructureType::eSubmitInfo;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    _ = queue.submit(1, &submitInfo, nullptr);
    queue.waitIdle();

    // Free command buffer
    device.freeCommandBuffers(commandPool, 1, &commandBuffer);
}

Geometry::Geometry(int grid_size, float grid_scale, float warp_strength)
    : m_GridSize(grid_size)
    , m_GridScale(grid_scale)
    , m_WarpStrength(warp_strength)
    , m_Shader(nullptr)
{
}

void Geometry::generateGrid(
    std::vector<Vertex>& vertices,
    std::vector<uint32_t>& indices,
    uint32_t gridSize,
    float scale) const
{
    // Copy data back to CPU for debugging (optional, remove in production)
    const size_t vertex_count = vertexCount();
    const size_t index_count = indexCount();

    vertices.resize(vertex_count);
    indices.resize(index_count);

    // Dispatch the compute shader to generate the grid on the GPU
    m_Shader->dispatchComputeShader(gridSize, scale);
    m_Shader->copyBuffersToCPU(vertices, indices, vertex_count, index_count);
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
