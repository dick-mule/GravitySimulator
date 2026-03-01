//
// Created by Richard Mule on 7/5/25.
//

#include "Geometry.hpp"

std::ostream& operator<<(std::ostream& os, const GeometryType& type)
{
    os << magic_enum::enum_name(type);
    return os;
}

GeometryShader::GeometryShader(std::shared_ptr<VkResourceManager> resourceManager)
    : m_ResourceManager(std::move(resourceManager))
    , m_ShaderPrograms({})
{
    initializeAll();
}

GeometryShader::~GeometryShader()
{
    const auto& device = m_ResourceManager->device();
    for ( const auto& [vertexShader, fragmentShader] : m_ShaderPrograms | std::views::values )
    {
        device.destroyShaderModule(vertexShader);
        device.destroyShaderModule(fragmentShader);
    }
}

void GeometryShader::initializeAll()
{
    loadShader(GeometryType::Flat, "flat_grid.comp.spv", "grid.frag.spv");
    loadShader(GeometryType::Spherical, "spherical_grid.comp.spv", "grid.frag.spv");
    loadShader(GeometryType::Hyperbolic, "hyperbolic_grid.comp.spv", "grid.frag.spv");
}

void GeometryShader::dispatchShader(const size_t grid_size, const float grid_scale)
{
    const vk::CommandPool& commandPool = m_ResourceManager->commandPools().at(ResourceTypes::COMPUTE);
    const WrappedDescriptorSet& descriptorSet = m_ResourceManager->wrappedDescriptorSet(ResourceTypes::COMPUTE);
    const vk::CommandBuffer commandBuffer = m_ResourceManager->startSingleTimeCommand(commandPool);
    // Bind the compute pipeline
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, descriptorSet.descPipeline);
    // Bind descriptor set
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eCompute,
        descriptorSet.descPipelineLayout,
        0,
        1,
        &descriptorSet.descSet,
        0,
        nullptr);
    // Set push constants
    commandBuffer.pushConstants(
        descriptorSet.descPipelineLayout,
        vk::ShaderStageFlagBits::eCompute,
        0,
        sizeof(int),
        &grid_size);
    commandBuffer.pushConstants(
        descriptorSet.descPipelineLayout,
        vk::ShaderStageFlagBits::eCompute,
        sizeof(int),
        sizeof(float),
        &grid_scale);

    // Dispatch compute shader
    constexpr uint32_t groupSize = 32;
    const uint32_t groupsX = (grid_size + 1 + groupSize - 1) / groupSize; // 16
    const uint32_t groupsY = (grid_size + 1 + groupSize - 1) / groupSize; // 16
    commandBuffer.dispatch(groupsX, groupsY, 1);

    // End command buffer
    commandBuffer.end();

    // Submit to queue
    const vk::Queue& computeQueue = m_ResourceManager->queues().at(ResourceTypes::COMPUTE);
    m_ResourceManager->terminateSingleTimeCommand(commandPool, commandBuffer, computeQueue);
}

const ShaderProgram& GeometryShader::getShaderProgram(const GeometryType type) const
{
    if ( m_ShaderPrograms.contains(type) )
        return m_ShaderPrograms.at(type);

    throw std::runtime_error(std::format(
        "Shader program not found for geometry type[{}]",
        magic_enum::enum_name(type)
    ));
}

void GeometryShader::loadShader(
    const GeometryType type,
    const std::string& vertexPath,
    const std::string& fragmentPath)
{
    const std::vector<char> vertexCode = utils::readFile(vertexPath);
    const std::vector<char> fragmentCode = utils::readFile(fragmentPath);
    const auto& device = m_ResourceManager->device();

    vk::ShaderModuleCreateInfo vertexInfo{};
    vertexInfo.sType = vk::StructureType::eShaderModuleCreateInfo;
    vertexInfo.codeSize = vertexCode.size();
    vertexInfo.pCode = reinterpret_cast<const uint32_t*>(vertexCode.data());
    const vk::ShaderModule vertexShader = device.createShaderModule(vertexInfo);

    vk::ShaderModuleCreateInfo fragmentInfo{};
    fragmentInfo.sType = vk::StructureType::eShaderModuleCreateInfo;
    fragmentInfo.codeSize = fragmentCode.size();
    fragmentInfo.pCode = reinterpret_cast<const uint32_t*>(fragmentCode.data());
    const vk::ShaderModule fragmentShader = device.createShaderModule(fragmentInfo);

    m_ShaderPrograms[type] = { vertexShader, fragmentShader };
}

Geometry::Geometry(
    std::shared_ptr<VkResourceManager> resourceManager,
    const GeometryType geometryType,
    const int grid_size,
    const float grid_scale,
    const float warp_strength)
    : m_Shader(std::make_shared<GeometryShader>(resourceManager))
    , m_GeometryType(geometryType)
    , m_GridSize(grid_size)
    , m_GridScale(grid_scale)
    , m_WarpStrength(warp_strength)
{
}

void Geometry::bindVertices()
{
    const size_t vertexBufferSize = vertexCount() * sizeof(Vertex);
    const vk::Buffer vertexBuffer = createBuffer(
        vertexBufferSize,
        vk::BufferUsageFlagBits::eStorageBuffer |
        vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eTransferSrc);
    const vk::DeviceMemory vertexBufferMemory = allocateBufferMemory(
        vertexBuffer,
        vk::MemoryPropertyFlagBits::eHostVisible |
        vk::MemoryPropertyFlagBits::eHostCoherent);
    (*m_Shader)->setBuffer(vertexBuffer, vertexBufferMemory, vertexBufferSize, ResourceTypes::VERTEX);
}

void Geometry::bindIndices()
{
    const size_t indexBufferSize = indexCount() * sizeof(uint32_t);
    const vk::Buffer indexBuffer = createBuffer(
        indexBufferSize,
        vk::BufferUsageFlagBits::eStorageBuffer |
        vk::BufferUsageFlagBits::eIndexBuffer | vk::BufferUsageFlagBits::eTransferSrc);
    const vk::DeviceMemory indexBufferMemory = allocateBufferMemory(
        indexBuffer,
        vk::MemoryPropertyFlagBits::eHostVisible |
        vk::MemoryPropertyFlagBits::eHostCoherent);
    (*m_Shader)->setBuffer(indexBuffer, indexBufferMemory, indexBufferSize, ResourceTypes::INDEX);
}

void Geometry::createGeometryPipeline()
{
    const auto& [computeShaderModule, fragModule] = m_Shader->getShaderProgram(m_GeometryType);
    (*m_Shader)->createResourcePipeline(computeShaderModule, ResourceTypes::COMPUTE);
    const auto& pipeline = (*m_Shader)->wrappedDescriptorSet(ResourceTypes::COMPUTE);
    /// Now initialize the compute pipeline to handle the size of the geometry
    bindVertices();
    bindIndices();
    (*m_Shader)->bindBufferDescriptorSets();
}

void Geometry::generateGrid() const
{
    m_Shader->dispatchShader(m_GridSize, m_GridScale);
    // m_Shader->copyBuffersToCPU(vertices, indices, vertex_count, index_count);
}

std::shared_ptr<Geometry> geometryFactory(
    std::shared_ptr<VkResourceManager> resource_manager,
    const GeometryType type,
    const int grid_size,
    const float grid_scale)
{
    switch ( type )
    {
    case GeometryType::Spherical:
        return std::make_shared<SphericalGeometry>(
            std::move(resource_manager),
            type,
            grid_size,
            grid_scale);
    case GeometryType::Hyperbolic:
        return std::make_shared<HyperbolicGeometry>(
            std::move(resource_manager),
            type,
            grid_size,
            grid_scale);
    default:
        return std::make_shared<FlatGeometry>(
            std::move(resource_manager),
            type,
            grid_size,
            grid_scale);
    }
}