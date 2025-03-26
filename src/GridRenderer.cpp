//
// Created by Richard Mule on 3/22/25.
//

#include "GridRenderer.hpp"
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <imgui.h>

static std::vector<char> readFile(const std::string& fileName)
{
    std::ifstream file(fileName, std::ios::ate | std::ios::binary);
    if ( !file.is_open() )
        throw std::runtime_error("Failed to open " + fileName);
    const size_t fileSize = file.tellg();
    std::vector<char> shaderCode(fileSize);
    file.seekg(0);
    file.read(shaderCode.data(), fileSize);
    file.close();
    return shaderCode;
}

static std::string vecToString(const glm::vec3& vec)
{
    std::stringstream ss;
    ss << vec.x << " " << vec.y << " " << vec.z;
    return ss.str();
}

static int firstFrame = 0;

GridRenderer::GridRenderer(
    const vk::Device& device,
    const vk::PhysicalDevice& physicalDevice,
    const vk::CommandPool& commandPool,
    const vk::Queue& graphicsQueue,
    const vk::RenderPass& renderPass,
    const std::vector<vk::Image>& swapchainImages,
    const vk::Extent2D& swapchainExtent,
    GeometryType geometryType)
    : m_Geometry(geometryFactory(geometryType, m_GridSize, m_GridScale))
    , m_CurrentGeometryType(geometryType)
    , m_Device(device)
    , m_PhysicalDevice(physicalDevice)
    , m_CommandPool(commandPool)
    , m_GraphicsQueue(graphicsQueue)
    , m_RenderPass(renderPass)
    , m_SwapchainImages(swapchainImages)
    , m_SwapchainExtent(swapchainExtent)
    , m_VertexBuffer(nullptr)
    , m_VertexBufferMemory(nullptr)
    , m_IndexBuffer(nullptr)
    , m_IndexBufferMemory(nullptr)
    , m_PipelineLayout(nullptr)
    , m_GraphicsPipeline(nullptr)
    , m_LinePipeline(nullptr)
    , m_TrianglePipeline(nullptr)
    , m_PushConstants()
    , m_DepthFormat()
    , m_GridObject()
{
    m_Geometry = geometryFactory(geometryType, m_GridSize, m_GridScale);
    Object centralObj{};
    centralObj.mass = 20000.0; // Very massive
    centralObj.position = glm::vec3(0.0, 0.0, 0.0);
    centralObj.modelMatrix = glm::translate(glm::mat4(1.0), centralObj.position); // Center of grid
    centralObj.velocity = glm::vec3(0.0); // Stationary
    auto centralShape = std::make_shared<Sphere>(centralObj);
    m_MassiveObjects.push_back(centralShape); // Using Sphere for simplicity
    centralShape->setSize(1.0 + std::log(centralShape->m_Object.mass / std::sqrt(m_GridScale))); // Larger orbiters

    // Add orbiting bodies
    constexpr int numOrbiters = 1; // Start with 5, adjust as needed
    const float G = m_Gravity; // 0.2f from your setup
    const float centralMass = centralObj.mass;
    float totalMass = centralMass;

    // Scale radii based on m_GridScale
    const float baseRadius = m_GridScale * 0.05; // 5% of grid scale as starting radius
    const float radiusStep = m_GridScale * 0.5 / numOrbiters; // Spread across 10% of grid

    for ( int i = 0; i < numOrbiters; ++i )
    {
        Object orbiter{};
        orbiter.mass = 10.0 + static_cast<float>(i) * 5.0;

        // Circular orbit parameters
        float radius = baseRadius + i * radiusStep;
        float angle = static_cast<float>(i) * 2.0 * glm::pi<float>() / numOrbiters;
        glm::vec3 rawPosition(radius * cos(angle), 0.0, radius * sin(angle));
        glm::vec3 position = convertCoordinates(
            rawPosition,
            GeometryType::Flat,
            geometryType,
            m_GridSize / 2,
            m_Geometry);
        orbiter.position = position;
        orbiter.modelMatrix = glm::translate(glm::mat4(1.0), position);

        float actualRadius = glm::length(position);
        float v = std::sqrt(G * centralMass / actualRadius);
        glm::vec3 radialDir = glm::normalize(position);
        glm::vec3 tangentDir(-radialDir.z, 0.0, radialDir.x);
        if ( glm::length(tangentDir) < 1e-6 )
            tangentDir = glm::vec3(0.0, 0.0, 1.0);
        orbiter.velocity = m_OrbitFactor * v * glm::normalize(tangentDir);

        m_MassiveObjects.push_back(std::make_unique<Sphere>(orbiter));
        auto& obj = m_MassiveObjects.back();
        obj->setSize(1.0 + std::log(obj->m_Object.mass / std::sqrt(m_GridScale))); // Larger orbiters
        totalMass += obj->m_Object.mass;
        std::cout << "Object " << obj->getName()
            << " Mass: " << obj->m_Object.mass << " w Radius: " << actualRadius
            << " G = " << G << " Orbit = " << centralMass
            << " Pos: " << vecToString(rawPosition) << " vs. " << vecToString(tangentDir)
            << " Vel: " << vecToString(obj->m_Object.velocity) << " / " << v << "\n";
    }

    m_WarpStrength = m_Gravity * m_GridScale / m_GridSize; // Scales with mass
    m_Geometry->setWarpStrength(m_WarpStrength);

    // Initialize trails
    m_Trails.resize(m_MassiveObjects.size());
    for ( auto& trail : m_Trails )
        trail.positions.clear();
}

GridRenderer::~GridRenderer()
{
    if ( m_GraphicsPipeline )
        m_Device.destroyPipeline(m_GraphicsPipeline);
    if ( m_TrianglePipeline )
        m_Device.destroyPipeline(m_TrianglePipeline);
    if ( m_PipelineLayout )
        m_Device.destroyPipelineLayout(m_PipelineLayout);
    if ( m_VertexBuffer )
        m_Device.destroyBuffer(m_VertexBuffer);
    if ( m_VertexBufferMemory )
        m_Device.freeMemory(m_VertexBufferMemory);
    if ( m_TrailVertexBuffer )
        m_Device.destroyBuffer(m_TrailVertexBuffer);
    if ( m_TrailVertexBufferMemory )
        m_Device.freeMemory(m_TrailVertexBufferMemory);
    if ( m_TrailIndexBuffer )
        m_Device.destroyBuffer(m_TrailIndexBuffer);
    if ( m_TrailIndexBufferMemory )
        m_Device.freeMemory(m_TrailIndexBufferMemory);
    if ( m_VertexBuffer )
        m_Device.destroyBuffer(m_IndexBuffer);
    if ( m_IndexBufferMemory )
        m_Device.freeMemory(m_IndexBufferMemory);
    for ( auto imageView : m_DepthImageViews )
        m_Device.destroyImageView(imageView);
    for ( auto image : m_DepthImages )
        m_Device.destroyImage(image);
    for ( auto memory : m_DepthImageMemory )
        m_Device.freeMemory(memory);
}

void GridRenderer::init()
{
    generateGrid();
    createVertexBuffer();
    createIndexBuffer();
    createGraphicsPipeline();
}

void GridRenderer::generateGrid()
{
    m_Vertices.clear();
    m_Indices.clear();

    /// Initialize Grid object with geometrized grid
    m_Geometry->generateGrid(m_Vertices, m_Indices, m_GridSize, m_GridScale);

    const uint32_t gridIndexCount = static_cast<uint32_t>(m_Indices.size());
    m_GridObject.indexOffset = 0;
    m_GridObject.indexCount = gridIndexCount;
    m_GridObject.modelMatrix = glm::mat4(1.0f);


    for ( const auto& shape : m_MassiveObjects )
        shape->addVertices(m_Vertices, m_Indices);

}

vk::Buffer GridRenderer::createBuffer(const vk::DeviceSize size, const vk::BufferUsageFlags usage) const
{
    vk::BufferCreateInfo bufferInfo{};
    bufferInfo.setSize(size)
              .setUsage(usage)
              .setSharingMode(vk::SharingMode::eExclusive);

    const auto result = m_Device.createBuffer(bufferInfo);
    if ( !result )
        throw std::runtime_error("Failed to create buffer");

    return result;
}

vk::DeviceMemory GridRenderer::allocateBufferMemory(
    const vk::Buffer buffer,
    const vk::MemoryPropertyFlags properties) const
{
    const auto memRequirements = m_Device.getBufferMemoryRequirements(buffer);

    vk::MemoryAllocateInfo allocInfo{};
    allocInfo.setAllocationSize(memRequirements.size);

    uint32_t memoryTypeIndex = -1;
    const auto memProperties = m_PhysicalDevice.getMemoryProperties();
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

    allocInfo.setMemoryTypeIndex(memoryTypeIndex);

    const auto result = m_Device.allocateMemory(allocInfo);
    if ( !result )
        throw std::runtime_error("Failed to allocate buffer memory");

    return result;
}

void GridRenderer::copyBuffer(const vk::Buffer srcBuffer, const vk::Buffer dstBuffer, const vk::DeviceSize size)
{
    const auto commandBuffer = beginSingleTimeCommands();

    vk::BufferCopy copyRegion{};
    copyRegion.setSize(size);
    commandBuffer.copyBuffer(srcBuffer, dstBuffer, copyRegion);

    endSingleTimeCommands(commandBuffer);
}

vk::CommandBuffer GridRenderer::beginSingleTimeCommands() const
{
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool(m_CommandPool)
             .setLevel(vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(1);

    const auto result = m_Device.allocateCommandBuffers(allocInfo);
    if ( result.empty() )
        throw std::runtime_error("Failed to allocate command buffer");

    const vk::CommandBuffer commandBuffer = result.front();

    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.setFlags(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
    commandBuffer.begin(beginInfo);

    return commandBuffer;
}

void GridRenderer::endSingleTimeCommands(const vk::CommandBuffer commandBuffer) const
{
    commandBuffer.end();

    vk::SubmitInfo submitInfo{};
    submitInfo.setCommandBufferCount(1)
              .setPCommandBuffers(&commandBuffer);

    m_GraphicsQueue.submit(submitInfo, nullptr);
    m_GraphicsQueue.waitIdle();

    m_Device.freeCommandBuffers(m_CommandPool, commandBuffer);
}

void GridRenderer::createVertexBuffer()
{
    const vk::DeviceSize bufferSize = sizeof(m_Vertices[0]) * m_Vertices.size();

    // Create staging buffer
    const auto stagingBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc);
    const auto stagingBufferMemory = allocateBufferMemory(
        stagingBuffer,
        vk::MemoryPropertyFlagBits::eHostVisible |
        vk::MemoryPropertyFlagBits::eHostCoherent);
    m_Device.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);

    // Copy vertex data to staging buffer
    void* data;
    auto _ = m_Device.mapMemory(stagingBufferMemory, 0, bufferSize, {}, &data);
    memcpy(data, m_Vertices.data(), bufferSize);
    m_Device.unmapMemory(stagingBufferMemory);

    // Create vertex buffer
    m_VertexBuffer = createBuffer(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferDst |
        vk::BufferUsageFlagBits::eVertexBuffer);
    m_VertexBufferMemory = allocateBufferMemory(m_VertexBuffer, vk::MemoryPropertyFlagBits::eDeviceLocal);
    m_Device.bindBufferMemory(m_VertexBuffer, m_VertexBufferMemory, 0);

    // Copy from staging to vertex buffer
    copyBuffer(stagingBuffer, m_VertexBuffer, bufferSize);

    // Clean up staging buffer
    m_Device.destroyBuffer(stagingBuffer);
    m_Device.freeMemory(stagingBufferMemory);
}

void GridRenderer::createIndexBuffer()
{
    const vk::DeviceSize bufferSize = sizeof(m_Indices[0]) * m_Indices.size();

    // Create staging buffer
    const auto stagingBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc);
    const auto stagingBufferMemory = allocateBufferMemory(
        stagingBuffer,
        vk::MemoryPropertyFlagBits::eHostVisible |
        vk::MemoryPropertyFlagBits::eHostCoherent);
    m_Device.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);

    // Copy index data to staging buffer
    void* data;
    auto _ = m_Device.mapMemory(stagingBufferMemory, 0, bufferSize, {}, &data);
    memcpy(data, m_Indices.data(), bufferSize);
    m_Device.unmapMemory(stagingBufferMemory);

    // Create index buffer
    m_IndexBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer);
    m_IndexBufferMemory = allocateBufferMemory(m_IndexBuffer, vk::MemoryPropertyFlagBits::eDeviceLocal);
    m_Device.bindBufferMemory(m_IndexBuffer, m_IndexBufferMemory, 0);

    // Copy from staging to index buffer
    copyBuffer(stagingBuffer, m_IndexBuffer, bufferSize);

    // Clean up staging buffer
    m_Device.destroyBuffer(stagingBuffer);
    m_Device.freeMemory(stagingBufferMemory);
}

void GridRenderer::createGraphicsPipeline()
{
    // Load shaders
    std::vector<char> vertShaderCode = readFile("grid.vert.spv");
    std::vector<char> fragShaderCode = readFile("grid.frag.spv");

    // Create shader modules
    vk::ShaderModuleCreateInfo vertShaderInfo{};
    vertShaderInfo.setCodeSize(vertShaderCode.size())
                  .setPCode(reinterpret_cast<const uint32_t*>(vertShaderCode.data()));
    auto vertShaderModule = m_Device.createShaderModule(vertShaderInfo);

    vk::ShaderModuleCreateInfo fragShaderInfo{};
    fragShaderInfo.setCodeSize(fragShaderCode.size())
                  .setPCode(reinterpret_cast<const uint32_t*>(fragShaderCode.data()));
    auto fragShaderModule = m_Device.createShaderModule(fragShaderInfo);

    // Shader stages (shared)
    vk::PipelineShaderStageCreateInfo vertStageInfo{};
    vertStageInfo.setStage(vk::ShaderStageFlagBits::eVertex)
                 .setModule(vertShaderModule)
                 .setPName("main");

    vk::PipelineShaderStageCreateInfo fragStageInfo{};
    fragStageInfo.setStage(vk::ShaderStageFlagBits::eFragment)
                 .setModule(fragShaderModule)
                 .setPName("main");

    std::array<vk::PipelineShaderStageCreateInfo, 2> shaderStages = { vertStageInfo, fragStageInfo };

    // Vertex input (shared)
    auto bindingDescription = Vertex::getBindingDescription();
    auto attributeDescriptions = Vertex::getAttributeDescriptions();
    vk::PipelineVertexInputStateCreateInfo vertexInputInfo{};
    vertexInputInfo.setVertexBindingDescriptionCount(1)
                   .setPVertexBindingDescriptions(&bindingDescription)
                   .setVertexAttributeDescriptionCount(static_cast<uint32_t>(attributeDescriptions.size()))
                   .setPVertexAttributeDescriptions(attributeDescriptions.data());

    // Dynamic state (shared)
    std::vector<vk::DynamicState> dynamicStates = { vk::DynamicState::eViewport, vk::DynamicState::eScissor };
    vk::PipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.setDynamicStateCount(static_cast<uint32_t>(dynamicStates.size()))
                .setPDynamicStates(dynamicStates.data());

    // Viewport and scissor (shared)
    vk::Viewport viewport{};
    viewport.setX(0.0f)
            .setY(0.0f)
            .setWidth(static_cast<float>(m_SwapchainExtent.width))
            .setHeight(static_cast<float>(m_SwapchainExtent.height))
            .setMinDepth(0.0f)
            .setMaxDepth(1.0f);

    vk::Rect2D scissor{};
    scissor.setOffset({0, 0})
           .setExtent(m_SwapchainExtent);

    vk::PipelineViewportStateCreateInfo viewportState{};
    viewportState.setViewportCount(1)
                 .setPViewports(&viewport)
                 .setScissorCount(1)
                 .setPScissors(&scissor);

    // Multisampling (shared)
    vk::PipelineMultisampleStateCreateInfo multisampling{};
    multisampling.setSampleShadingEnable(VK_FALSE)
                 .setRasterizationSamples(vk::SampleCountFlagBits::e1);

    // Color blending (shared)
    vk::PipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.setColorWriteMask(vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
                                            vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA)
                        .setBlendEnable(VK_FALSE);

    vk::PipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.setLogicOpEnable(VK_FALSE)
                 .setAttachmentCount(1)
                 .setPAttachments(&colorBlendAttachment);

    // Depth-stencil state (shared)
    vk::PipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.setDepthTestEnable(VK_TRUE)
                .setDepthWriteEnable(VK_TRUE)
                .setDepthCompareOp(vk::CompareOp::eLess)
                .setDepthBoundsTestEnable(VK_FALSE)
                .setStencilTestEnable(VK_FALSE);

    // Push constants (shared)
    vk::PushConstantRange pushConstantRange{};
    pushConstantRange.setStageFlags(vk::ShaderStageFlagBits::eVertex)
                     .setOffset(0)
                     .setSize(sizeof(PushConstants));

    // Pipeline layout (shared)
    vk::PipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.setPushConstantRangeCount(1)
                      .setPPushConstantRanges(&pushConstantRange);
    m_PipelineLayout = m_Device.createPipelineLayout(pipelineLayoutInfo);
    if (!m_PipelineLayout)
        throw std::runtime_error("Failed to create pipeline layout");

    // --- Grid Pipeline ---
    vk::PipelineInputAssemblyStateCreateInfo gridInputAssembly{};
    gridInputAssembly
        .setTopology(
            m_CurrentGeometryType == GeometryType::Flat ?
            vk::PrimitiveTopology::eLineList : vk::PrimitiveTopology::eTriangleList)
        .setPrimitiveRestartEnable(VK_FALSE);

    vk::PipelineRasterizationStateCreateInfo gridRasterizer{};
    gridRasterizer.setDepthClampEnable(VK_FALSE)
                  .setRasterizerDiscardEnable(VK_FALSE)
                  .setPolygonMode(vk::PolygonMode::eLine)
                  .setLineWidth(1.0f)
                  .setCullMode(vk::CullModeFlagBits::eNone) // Lines don’t need culling typically
                  .setFrontFace(vk::FrontFace::eCounterClockwise)
                  .setDepthBiasEnable(VK_FALSE);

    vk::GraphicsPipelineCreateInfo gridPipelineInfo{};
    gridPipelineInfo.setStageCount(2)
                    .setPStages(shaderStages.data())
                    .setPVertexInputState(&vertexInputInfo)
                    .setPInputAssemblyState(&gridInputAssembly)
                    .setPViewportState(&viewportState)
                    .setPRasterizationState(&gridRasterizer)
                    .setPMultisampleState(&multisampling)
                    .setPDepthStencilState(&depthStencil)
                    .setPColorBlendState(&colorBlending)
                    .setPDynamicState(&dynamicState)
                    .setLayout(m_PipelineLayout)
                    .setRenderPass(m_RenderPass)
                    .setSubpass(0);

    auto pipelineResult = m_Device.createGraphicsPipeline(nullptr, gridPipelineInfo);
    if (pipelineResult.result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create grid pipeline");
    m_GraphicsPipeline = pipelineResult.value;

    // --- Triangle Pipeline (for cube and sphere) ---
    vk::PipelineInputAssemblyStateCreateInfo triangleInputAssembly{};
    triangleInputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList)
                         .setPrimitiveRestartEnable(VK_FALSE);

    vk::PipelineRasterizationStateCreateInfo triangleRasterizer{};
    triangleRasterizer.setDepthClampEnable(VK_FALSE)
                      .setRasterizerDiscardEnable(VK_FALSE)
                      .setPolygonMode(vk::PolygonMode::eFill) // Use eFill for solid shapes; switch to eLine for wireframe
                      .setLineWidth(1.0f)
                      .setCullMode(vk::CullModeFlagBits::eNone) // Matches your working setup
                      .setFrontFace(vk::FrontFace::eCounterClockwise)
                      .setDepthBiasEnable(VK_FALSE);

    vk::GraphicsPipelineCreateInfo trianglePipelineInfo{};
    trianglePipelineInfo.setStageCount(2)
                        .setPStages(shaderStages.data())
                        .setPVertexInputState(&vertexInputInfo)
                        .setPInputAssemblyState(&triangleInputAssembly)
                        .setPViewportState(&viewportState)
                        .setPRasterizationState(&triangleRasterizer)
                        .setPMultisampleState(&multisampling)
                        .setPDepthStencilState(&depthStencil)
                        .setPColorBlendState(&colorBlending)
                        .setPDynamicState(&dynamicState)
                        .setLayout(m_PipelineLayout)
                        .setRenderPass(m_RenderPass)
                        .setSubpass(0);

    pipelineResult = m_Device.createGraphicsPipeline(nullptr, trianglePipelineInfo);
    if (pipelineResult.result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create triangle pipeline");
    m_TrianglePipeline = pipelineResult.value;

    // Clean up shader modules
    m_Device.destroyShaderModule(vertShaderModule);
    m_Device.destroyShaderModule(fragShaderModule);
}

void GridRenderer::updateGeometry(GeometryType type)
{
    firstFrame = 0;
    if ( m_CurrentGeometryType != type )
    {
        // Step 1: Compute orbital properties for reference (if at least 2 objects)
        constexpr float angularMomentum = 0.0f;
        constexpr float eccentricity = 0.0f;
        constexpr float mu = 0.0f;
        constexpr float dist = 0.0f;

        // Step 2: Update geometry factory
        m_Geometry = geometryFactory(type, m_GridSize, m_GridScale);
        const float R = m_GridScale / 2.0f;

        // Step 3: Convert positions, velocities, and reset accelerations
        for ( auto& shape : m_MassiveObjects )
        {
            Object& obj = shape->m_Object;
            glm::vec3 oldPos = obj.position; // Use position field directly
            glm::vec3 oldVel = obj.velocity;

            glm::vec3 newPos = convertCoordinates(oldPos, m_CurrentGeometryType, type, R, m_Geometry);
            glm::vec3 newVel = convertVelocity(oldPos, oldVel, m_CurrentGeometryType, type, R, dist, mu, m_Geometry);
            obj.position = newPos;
            obj.modelMatrix = glm::translate(glm::mat4(1.0f), newPos);
            obj.velocity = newVel;
            obj.acceleration = glm::vec3(0, 0, 0);
            std::cout << "Object at (" << oldPos.x << ", " << oldPos.y << ", " << oldPos.z
                      << ") -> (" << newPos.x << ", " << newPos.y << ", " << newPos.z
                      << "), Vel (" << oldVel.x << ", " << oldVel.y << ", " << oldVel.z
                      << ") -> (" << newVel.x << ", " << newVel.y << ", " << newVel.z << ")\n";
        }

        // Step 4: Update grid and buffers
        m_CurrentGeometryType = type;
        generateGrid();
        createVertexBuffer();
        createIndexBuffer();
    }

}

void GridRenderer::updateTrails()
{
    m_TrailVertices.clear();
    m_TrailIndices.clear();

    uint32_t vertexOffset = 0;
    for ( size_t i = 0; i < m_Trails.size(); ++i )
    {
        const auto& [positions] = m_Trails[i];
        // const auto& shape = m_MassiveObjects[i];
        const glm::vec3 color = i == 0 ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 1.0f);

        // Add vertices
        for ( const auto& pos : positions )
        {
            Vertex vertex{};
            vertex.position = pos;
            vertex.color = color;
            vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            m_TrailVertices.push_back(vertex);
        }

        // Add indices for a line strip
        for ( size_t j = 0; j < positions.size() - 1; ++j )
        {
            m_TrailIndices.push_back(vertexOffset + j);
            m_TrailIndices.push_back(vertexOffset + j + 1);
        }
        vertexOffset += static_cast<uint32_t>(positions.size());
    }

    // Update trail vertex buffer
    if ( !m_TrailVertices.empty() && !m_TrailIndices.empty() )
    {
        const vk::DeviceSize bufferSize = sizeof(m_TrailVertices[0]) * m_TrailVertices.size();
        const vk::Buffer stagingBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc);
        const vk::DeviceMemory stagingBufferMemory = allocateBufferMemory(
            stagingBuffer,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
        m_Device.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);

        void* data;
        auto _ = m_Device.mapMemory(stagingBufferMemory, 0, bufferSize, {}, &data);
        memcpy(data, m_TrailVertices.data(), bufferSize);
        m_Device.unmapMemory(stagingBufferMemory);

        static vk::DeviceSize lastVertexBufferSize = 0;
        if ( m_TrailVertexBuffer && bufferSize != lastVertexBufferSize )
        {
            m_Device.destroyBuffer(m_TrailVertexBuffer);
            m_Device.freeMemory(m_TrailVertexBufferMemory);
            m_TrailVertexBuffer = nullptr;
            m_TrailVertexBufferMemory = nullptr;
        }
        if ( !m_TrailVertexBuffer )
        {
            m_TrailVertexBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer);
            m_TrailVertexBufferMemory = allocateBufferMemory(m_TrailVertexBuffer, vk::MemoryPropertyFlagBits::eDeviceLocal);
            m_Device.bindBufferMemory(m_TrailVertexBuffer, m_TrailVertexBufferMemory, 0);
        }
        lastVertexBufferSize = bufferSize;
        copyBuffer(stagingBuffer, m_TrailVertexBuffer, bufferSize);

        m_Device.destroyBuffer(stagingBuffer);
        m_Device.freeMemory(stagingBufferMemory);

        const vk::DeviceSize indexBufferSize = sizeof(m_TrailIndices[0]) * m_TrailIndices.size();
        const vk::Buffer stagingIndexBuffer = createBuffer(indexBufferSize, vk::BufferUsageFlagBits::eTransferSrc);
        const vk::DeviceMemory stagingIndexBufferMemory = allocateBufferMemory(
            stagingIndexBuffer,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
        m_Device.bindBufferMemory(stagingIndexBuffer, stagingIndexBufferMemory, 0);

        _ = m_Device.mapMemory(stagingIndexBufferMemory, 0, indexBufferSize, {}, &data);
        memcpy(data, m_TrailIndices.data(), indexBufferSize);
        m_Device.unmapMemory(stagingIndexBufferMemory);

        static vk::DeviceSize lastIndexBufferSize = 0;
        if ( m_TrailIndexBuffer && indexBufferSize != lastIndexBufferSize )
        {
            m_Device.destroyBuffer(m_TrailIndexBuffer);
            m_Device.freeMemory(m_TrailIndexBufferMemory);
            m_TrailIndexBuffer = nullptr;
            m_TrailIndexBufferMemory = nullptr;
        }
        if ( !m_TrailIndexBuffer )
        {
            m_TrailIndexBuffer = createBuffer(indexBufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer);
            m_TrailIndexBufferMemory = allocateBufferMemory(m_TrailIndexBuffer, vk::MemoryPropertyFlagBits::eDeviceLocal);
            m_Device.bindBufferMemory(m_TrailIndexBuffer, m_TrailIndexBufferMemory, 0);
        }
        lastIndexBufferSize = indexBufferSize;
        copyBuffer(stagingIndexBuffer, m_TrailIndexBuffer, indexBufferSize);

        m_Device.destroyBuffer(stagingIndexBuffer);
        m_Device.freeMemory(stagingIndexBufferMemory);
    }
}


void GridRenderer::draw(vk::CommandBuffer commandBuffer) const
{
    vk::Viewport viewport{};
    viewport.setX(0.0f)
            .setY(0.0f)
            .setWidth(static_cast<float>(m_SwapchainExtent.width))
            .setHeight(static_cast<float>(m_SwapchainExtent.height))
            .setMinDepth(0.0f)
            .setMaxDepth(1.0f);
    commandBuffer.setViewport(0, viewport);

    vk::Rect2D scissor{};
    scissor.setOffset({0, 0})
           .setExtent(m_SwapchainExtent);
    commandBuffer.setScissor(0, scissor);

    constexpr vk::DeviceSize offsets[] = { 0 };
    commandBuffer.bindVertexBuffers(0, 1, &m_VertexBuffer, offsets);
    commandBuffer.bindIndexBuffer(m_IndexBuffer, 0, vk::IndexType::eUint32);

    PushConstants pc{};
    pc.view = m_Camera.getViewMatrix();
    pc.projection = glm::perspective(
        glm::radians(m_Camera.fov),
        static_cast<float>(m_SwapchainExtent.width) / static_cast<float>(m_SwapchainExtent.height),
        m_Camera.nearPlane, m_Camera.farPlane
    );
    pc.projection[1][1] *= -1; // Flip Y-axis for Vulkan

    // Draw grid (lines)
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_GraphicsPipeline);
    pc.model = m_GridObject.modelMatrix;
    commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &pc);
    commandBuffer.drawIndexed(m_GridObject.indexCount, 1, m_GridObject.indexOffset, 0, 0);

    // Draw all shapes (triangles)
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_TrianglePipeline);
    for ( const auto& shape : m_MassiveObjects )
    {
        const Object& obj = shape->m_Object;
        pc.model = obj.modelMatrix;
        commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &pc);
        commandBuffer.drawIndexed(obj.indexCount, 1, obj.indexOffset, 0, 0);
    }

    if ( !m_TrailVertices.empty() && !m_TrailIndices.empty() )
    {
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_GraphicsPipeline); // Use the line pipeline
        commandBuffer.bindVertexBuffers(0, 1, &m_TrailVertexBuffer, offsets);
        commandBuffer.bindIndexBuffer(m_TrailIndexBuffer, 0, vk::IndexType::eUint32);

        PushConstants trail_constants{};
        trail_constants.view = m_Camera.getViewMatrix();
        trail_constants.projection = glm::perspective(
            glm::radians(m_Camera.fov),
            static_cast<float>(m_SwapchainExtent.width) / static_cast<float>(m_SwapchainExtent.height),
            m_Camera.nearPlane, m_Camera.farPlane
        );
        trail_constants.projection[1][1] *= -1; // Flip Y-axis for Vulkan
        trail_constants.model = glm::mat4(1.0f); // Trails are in world space
        commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &trail_constants);

        commandBuffer.drawIndexed(static_cast<uint32_t>(m_TrailIndices.size()), 1, 0, 0, 0);
    }
}

void GridRenderer::updateCamera()
{
    const ImGuiIO& io = ImGui::GetIO();

    // Zoom with trackpad scroll
    if ( io.MouseWheel != 0.0f )
    {
        m_ZoomLevel -= io.MouseWheel * 0.1f;
        m_ZoomLevel = std::max(0.1f, std::min(m_ZoomLevel, 10.0f));
    }

    // Adjust radius with zoom level
    m_Camera.radius = 30.0f * m_ZoomLevel;
    m_Camera.radius = glm::clamp(m_Camera.radius, m_Camera.minRadius, m_Camera.maxRadius);

    // Orbiting: Rotate with left mouse button drag
    if ( ImGui::IsMouseDragging(ImGuiMouseButton_Left) )
    {
        const float deltaX = io.MouseDelta.x;
        const float deltaY = io.MouseDelta.y;

        m_Camera.azimuth -= deltaX * m_Camera.orbitSpeed;
        m_Camera.elevation -= deltaY * m_Camera.orbitSpeed;

        // Clamp elevation to avoid gimbal lock
        constexpr float minElevation = glm::radians(-89.9f);
        constexpr float maxElevation = glm::radians(89.9f);
        m_Camera.elevation = glm::clamp(m_Camera.elevation, minElevation, maxElevation);
    }

    // Panning: Move target with right mouse button drag or keyboard (WASD)
    if ( ImGui::IsMouseDragging(ImGuiMouseButton_Right) )
    {
        const float deltaX = io.MouseDelta.x;
        const float deltaY = io.MouseDelta.y;

        const glm::vec3 forward = glm::normalize(m_Camera.target - m_Camera.getPosition());
        const glm::vec3 right = glm::normalize(glm::cross(forward, m_Camera.up));
        const glm::vec3 up = glm::normalize(glm::cross(right, forward));

        m_Camera.target += right * (-deltaX * m_Camera.panSpeed);
        m_Camera.target += up * (deltaY * m_Camera.panSpeed);
    }

    // Keyboard panning (WASD)
    if ( !io.WantCaptureKeyboard )
    {
        const glm::vec3 forward = glm::normalize(m_Camera.target - m_Camera.getPosition());
        const glm::vec3 right = glm::normalize(glm::cross(forward, m_Camera.up));
        const glm::vec3 up = glm::normalize(glm::cross(right, forward));

        if ( ImGui::IsKeyDown(ImGuiKey_W) )
            m_Camera.target -= forward * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_S) )
            m_Camera.target += forward * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_A) )
            m_Camera.target -= right * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_D) )
            m_Camera.target += right * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_E) )
            m_Camera.target += up * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_Q) )
            m_Camera.target -= up * m_Camera.panSpeed;
    }

    static glm::vec3 lastPos = m_Camera.getPosition();
    if ( const glm::vec3 currentPos = m_Camera.getPosition(); glm::length(currentPos - lastPos) > 0.001f )
    {
        // std::cout << "Camera Position: (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.z << ")\n";
        lastPos = currentPos;
    }
}

void GridRenderer::updateGrid()
{
    // Grid starts at vertex 0, cube at 441, sphere follows
    std::vector<float> masses;
    std::vector<glm::vec3> massivePositions;
    for ( const auto& shape : m_MassiveObjects )
    {
        massivePositions.push_back(shape->m_Object.modelMatrix[3]);
        masses.push_back(shape->m_Object.mass);
    }

    constexpr float maxDisplacement = 100.0f;
    constexpr float minDistSquared = 0.01f;
    constexpr float softeningLength = 1.0f;

    m_Geometry->setGridParams(m_GridSize, m_GridScale);
    // m_Geometry->warpGrid(m_Vertices, m_MassiveObjects, m_Gravity, maxDisplacement, minDistSquared, softeningLength);

    /// Update vertex buffer
    const vk::DeviceSize bufferSize = sizeof(m_Vertices[0]) * m_Vertices.size();
    const vk::Buffer stagingBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc);
    const vk::DeviceMemory stagingBufferMemory = allocateBufferMemory(
        stagingBuffer,
        vk::MemoryPropertyFlagBits::eHostVisible |
        vk::MemoryPropertyFlagBits::eHostCoherent);
    m_Device.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);

    void* data;
    auto _ = m_Device.mapMemory(stagingBufferMemory, 0, bufferSize, {}, &data);
    memcpy(data, m_Vertices.data(), bufferSize);
    m_Device.unmapMemory(stagingBufferMemory);

    copyBuffer(stagingBuffer, m_VertexBuffer, bufferSize);

    m_Device.destroyBuffer(stagingBuffer);
    m_Device.freeMemory(stagingBufferMemory);
}

void GridRenderer::updateSimulation(float deltaTime)
{
    // return;
    if ( firstFrame > 100 )
        return;

    const float R = m_GridScale / 2.0f;
    deltaTime = std::min(deltaTime, m_TimeStep);
    // Step 1: Compute accelerations for all objects (Verlet Method)
    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        auto& obj = m_MassiveObjects[i]->m_Object;
        m_Geometry->updatePosition(obj, deltaTime, R, true);
        // Reset acceleration
        obj.acceleration = glm::vec3(0.0f);
    }

    // Step 2: Compute pairwise forces symmetrically
    constexpr float softeningLength = 1.0f;
    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        for ( size_t j = i + 1; j < m_MassiveObjects.size(); ++j )
        {   // Only iterate over j > i to avoid double-counting
            auto& shape1 = m_MassiveObjects[i];
            auto& shape2 = m_MassiveObjects[j];

            glm::vec3 pos1 = shape1->m_Object.position;
            glm::vec3 pos2 = shape2->m_Object.position;
            float dist = m_Geometry->computeDistance(pos1, pos2);
            if ( dist < 0.01f )
                dist = 0.01f;

            glm::vec3 direction;
            if ( m_CurrentGeometryType == GeometryType::Spherical )
            {
                const glm::vec3 pole(0.0f, R, 0.0f);
                const glm::vec3 pos1FromPole = pos1 - pole; // Central mass
                const glm::vec3 pos2FromPole = pos2 - pole; // Orbiter

                float distXZ2 = glm::length(glm::vec2(pos2FromPole.x, pos2FromPole.z));
                if (distXZ2 < 0.01f) distXZ2 = 0.01f;
                const float alpha2 = distXZ2 / R;
                const float theta2 = atan2(pos2FromPole.z, pos2FromPole.x);

                const glm::vec3 normal2 = glm::normalize(pos2FromPole); // At orbiter
                const glm::vec3 thetaTangent2(-sin(theta2), 0, cos(theta2));
                const glm::vec3 alphaTangent2(cos(alpha2) * cos(theta2), -sin(alpha2), cos(alpha2) * sin(theta2));

                // Centripetal direction toward pole
                glm::vec3 r = pole - pos2;
                float chordDist = glm::length(r);
                if (chordDist < 0.01f) continue;

                direction = glm::normalize(r);
                // Project onto tangent plane at pos2
                float radialComponent = glm::dot(direction, normal2);
                direction -= radialComponent * normal2;
                if (glm::length(direction) < 1e-6f)
                {
                    direction = thetaTangent2; // Fallback to tangential
                }
                direction = glm::normalize(direction);

                // Adjust direction based on orbiter's velocity
                glm::vec3 vel = shape2->m_Object.velocity;
                float vTheta = glm::dot(vel, thetaTangent2); // Angular velocity component
                if (vTheta < 0) // Counterclockwise (negative theta direction)
                {
                    direction = -direction; // Flip to align with velocity
                }
                // If vTheta > 0 (clockwise), keep direction as is

                // Geodesic distance between pos1 and pos2
                float distXZ1 = glm::length(glm::vec2(pos1FromPole.x, pos1FromPole.z));
                if (distXZ1 < 0.01f) distXZ1 = 0.01f;
                const float alpha1 = distXZ1 / R;
                const float theta1 = atan2(pos1FromPole.z, pos1FromPole.x);
                float centralAngle = acos(cos(alpha1) * cos(alpha2) + sin(alpha1) * sin(alpha2) * cos(theta2 - theta1));
                dist = R * centralAngle;


                // Debug
                if ( firstFrame < 10 )
                    std::cout << "Spherical: r = " << vecToString(r) << ", direction = " << vecToString(direction)
                 << ", normal2 = " << vecToString(normal2) << ", dist = " << dist << "\n";
            }
            else if ( m_CurrentGeometryType == GeometryType::Hyperbolic )
            {
                const float k = 2.0f * m_GridScale; // 250
                glm::vec3 diff(pos2.x - pos1.x,
                               (pos2.x * pos2.x - pos2.z * pos2.z) / k - (pos1.x * pos1.x - pos1.z * pos1.z) / k,
                               pos2.z - pos1.z);
                direction = glm::normalize(diff); // 3D direction on paraboloid

                // Project onto tangent plane (optional for precision)
                glm::vec3 normal(2.0f * pos1.x / k, -1.0f, -2.0f * pos1.z / k);
                normal = glm::normalize(normal);
                direction -= glm::dot(direction, normal) * normal; // Tangential component
                direction = glm::normalize(direction);
            }
            else
            {
                glm::vec3 r = pos2 - pos1;
                direction = glm::normalize(r);
            }

            float softenedDist = sqrt(dist * dist + softeningLength * softeningLength);

            // Compute force on shape1 due to shape2
            float force1 = m_Gravity * shape2->m_Object.mass / (softenedDist * softenedDist);
            shape1->m_Object.acceleration += force1 * direction;

            // Compute force on shape2 due to shape1 (equal and opposite)
            float force2 = m_Gravity * shape1->m_Object.mass / (softenedDist * softenedDist);
            shape2->m_Object.acceleration -= force2 * direction; // Opposite direction
        }
    }

    // Step 3: Update positions using the geometry's rules
    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        auto& shape = m_MassiveObjects[i];
        m_Geometry->updatePosition(shape->m_Object, deltaTime, R, false);

        if ( firstFrame < 10 )
        {
            std::cout << "Check Object[" << i << "] Values ...\n"
                      << "  Position: " << vecToString(shape->m_Object.position) << "\n"
                      << "  Velocity: " << vecToString(shape->m_Object.velocity) << "\n"
                      << "  Acceleration: " << vecToString(shape->m_Object.acceleration) << "\n";
        }
        // Add position to trail
        auto& [positions] = m_Trails[i];
        positions.push_back(shape->m_Object.modelMatrix[3]);
        if ( positions.size() > Trail::maxPoints )
            positions.pop_front();
    }

    // Compute energy
    m_KineticEnergy = 0.0f;
    m_PotentialEnergy = 0.0f;
    for ( const auto& shape : m_MassiveObjects )
    {
        // Kinetic energy: KE = 0.5 * m * v^2
        const float speedSquared = glm::length2(shape->m_Object.velocity);
        m_KineticEnergy += 0.5f * shape->m_Object.mass * speedSquared;
    }

    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        for ( size_t j = i + 1; j < m_MassiveObjects.size(); ++j )
        {
            const glm::vec3 pos1 = m_MassiveObjects[i]->m_Object.modelMatrix[3];
            const glm::vec3 pos2 = m_MassiveObjects[j]->m_Object.modelMatrix[3];
            float dist = m_Geometry->computeDistance(pos1, pos2);
            float softenedDist = std::sqrt(dist * dist + softeningLength * softeningLength);
            // Potential energy: PE = -G * m1 * m2 / r
            m_PotentialEnergy -= m_Gravity * m_MassiveObjects[i]->m_Object.mass * m_MassiveObjects[j]->m_Object.mass / softenedDist;
        }
    }
    m_TotalEnergy = m_KineticEnergy + m_PotentialEnergy;
    updateTrails();

    firstFrame++;
}

void GridRenderer::renderCameraControls()
{
    ImGui::Begin("Controls");

    // Geometry selection
    const char* geometryItems[] = {"Flat", "Spherical", "Hyperbolic"};
    static int currentGeometry = static_cast<int>(m_CurrentGeometryType);
    if ( ImGui::Combo("Geometry", &currentGeometry, geometryItems, IM_ARRAYSIZE(geometryItems)) )
    {
        std::cout << "CHECK INITIAL GEOMETRY: " << geometryItems[static_cast<int>(m_CurrentGeometryType)]
                  << " vs. " << geometryItems[currentGeometry] << std::endl;
        updateGeometry(static_cast<GeometryType>(currentGeometry));
    }

    // Camera controls
    glm::vec3 pos = m_Camera.getPosition();
    ImGui::InputFloat3("Camera Position", glm::value_ptr(pos), "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat3("Target", glm::value_ptr(m_Camera.target), "%.3f");

    float azimuthDegrees = glm::degrees(m_Camera.azimuth);
    float elevationDegrees = glm::degrees(m_Camera.elevation);
    ImGui::DragFloat("Azimuth", &azimuthDegrees, 1.0f, -180.0f, 180.0f, "%.1f degrees");
    ImGui::DragFloat("Elevation", &elevationDegrees, 1.0f, -89.0f, 89.0f, "%.1f degrees");
    ImGui::DragFloat("Radius", &m_Camera.radius, 1.0f, m_Camera.minRadius, m_Camera.maxRadius, "%.1f");
    m_Camera.azimuth = glm::radians(azimuthDegrees);
    m_Camera.elevation = glm::radians(elevationDegrees);

    ImGui::DragFloat("FOV", &m_Camera.fov, 1.0f, 10.0f, 120.0f, "%.1f degrees");
    ImGui::DragFloat("Near Plane", &m_Camera.nearPlane, 1.0f, 0.01f, 10.0f, "%.2f");
    ImGui::DragFloat("Far Plane", &m_Camera.farPlane, 1.0f, 10.0f, 1000.0f, "%.1f");

    ImGui::DragFloat("Orbit Speed", &m_Camera.orbitSpeed, 0.001f, 0.001f, 0.01f, "%.4f");
    ImGui::DragFloat("Zoom Speed", &m_Camera.zoomSpeed, 0.1f, 0.1f, 5.0f, "%.1f");
    ImGui::DragFloat("Pan Speed", &m_Camera.panSpeed, 0.01f, 0.01f, 1.0f, "%.2f");
    ImGui::DragFloat("Gravity Strength", &m_Gravity, 0.01f, 0.01f, 1.5f, "%.2f");
    ImGui::DragFloat("Time Step", &m_TimeStep, 0.001f, 0.001f, 0.1f, "%.3f");
    ImGui::SliderFloat("Velocity Angle", &m_VelocityAngle, 0.0f, 90.0f, "%.1f");

    ImGui::Text("Kinetic Energy: %.3f", m_KineticEnergy);
    ImGui::Text("Potential Energy: %.3f", m_PotentialEnergy);
    ImGui::Text("Total Energy: %.3f", m_TotalEnergy);

    if ( ImGui::Button("Reset Camera") )
    {
        switch ( m_CurrentGeometryType )
        {
            case GeometryType::Spherical:
            {
                m_Camera.position = glm::vec3(0.0f, 0.0f, m_GridScale * 1.5f); // Outside the sphere
                m_Camera.azimuth = 90.0f;
                m_Camera.elevation = 0.0f;
                m_Camera.radius = m_GridScale * 1.5f;
                break;
            }
            default:
            {
                m_Camera.position = glm::vec3(0.0f, 20.0f, 20.0f);
                m_Camera.azimuth = glm::radians(45.0f);
                m_Camera.elevation = glm::radians(45.0f);
                m_Camera.radius = 30.0f;
                    break;
            }
        }
        m_Camera.target = glm::vec3(0.0f, 0.0f, 0.0f);
        m_Camera.up = glm::vec3(0.0f, 1.0f, 0.0f);
        m_Camera.fov = 90.0f;
        m_Camera.nearPlane = 0.1f;
        m_Camera.farPlane = 100.0f;
        m_Camera.minRadius = 5.0f;
        m_Camera.maxRadius = 100.0f;
        m_Camera.panSpeed = 0.1f;
        m_Camera.orbitSpeed = 0.005f;
        m_Camera.zoomSpeed = 1.0f;
        m_ZoomLevel = 1.0f;
    }

    // Shape controls
    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        std::string label = m_MassiveObjects[i]->getName();
        ImGui::PushID(static_cast<int>(i));
        ImGui::Text("%s", label.c_str());
        ImGui::DragFloat("Mass", &m_MassiveObjects[i]->m_Object.mass, 0.1f, 0.0000001f, 50.0f);
        ImGui::DragFloat3("Velocity", &m_MassiveObjects[i]->m_Object.velocity[0], 0.01f, -1.0f, 1.0f);
        glm::vec3 obj_pos = m_MassiveObjects[i]->m_Object.modelMatrix[3];
        ImGui::InputFloat3("Position", glm::value_ptr(obj_pos), "%.3f");
        m_MassiveObjects[i]->m_Object.modelMatrix[3] = glm::vec4(obj_pos, 1.0f);
        ImGui::PopID();
    }

    // Reset simulation
    if ( ImGui::Button("Reset Simulation") )
    {
        m_MassiveObjects.clear();
        Object cubeObj{};
        cubeObj.mass = 10.0f;
        cubeObj.modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(-5.0f, 2.0f, 0.0f));
        cubeObj.velocity = glm::vec3(0.5f, 0.0f, 0.0f);
        m_MassiveObjects.push_back(std::make_unique<Cube>(cubeObj));

        Object sphereObj{};
        sphereObj.mass = 5.0f;
        sphereObj.modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(5.0f, 2.0f, 0.0f));
        sphereObj.velocity = glm::vec3(-0.5f, 0.0f, 0.0f);
        m_MassiveObjects.push_back(std::make_unique<Sphere>(sphereObj));

        // Regenerate grid to update indices
        generateGrid();
        createVertexBuffer();
        createIndexBuffer();
    }

    if ( ImGui::Button("Reset to Orbit") )
    {
        if ( m_MassiveObjects.size() >= 2 )
        {
            // Compute center of mass
            glm::vec3 com(0.0f);
            float totalMass = 0.0f;
            for ( const auto& shape : m_MassiveObjects )
            {
                com += shape->m_Object.mass * glm::vec3(shape->m_Object.modelMatrix[3]);
                totalMass += shape->m_Object.mass;
            }
            if ( totalMass > 0.0f )
                com /= totalMass;

            // Adjust positions
            for ( auto& shape : m_MassiveObjects )
            {
                shape->m_Object.modelMatrix[3] -= glm::vec4(com, 0.0f);

                if ( m_CurrentGeometryType == GeometryType::Spherical )
                {
                    const float R = static_cast<float>(m_GridSize) / 2.0f;
                    const glm::vec3 current_pos = shape->m_Object.modelMatrix[3];
                    const float len = glm::length(current_pos);
                    if ( len < 0.01f )
                        pos = glm::vec3(0.0f, 0.0f, R); // Default to north pole
                    else
                        pos = glm::normalize(pos) * R;
                    shape->m_Object.modelMatrix[3] = glm::vec4(pos, 1.0f);
                }
            }

            // Set velocities for first two shapes to orbit
            float mu = m_Gravity * (m_MassiveObjects[0]->m_Object.mass + m_MassiveObjects[1]->m_Object.mass);
            float r = m_Geometry->computeDistance(
                m_MassiveObjects[0]->m_Object.modelMatrix[3],
                m_MassiveObjects[1]->m_Object.modelMatrix[3]
            );
            if ( r < 0.01f )
                r = 0.01f;

            // Compute the radial vector r from object 0 to object 1
            glm::vec3 pos0 = m_MassiveObjects[0]->m_Object.modelMatrix[3];
            glm::vec3 pos1 = m_MassiveObjects[1]->m_Object.modelMatrix[3];
            glm::vec3 r_vec = pos1 - pos0;

            glm::vec3 tangent, bi_tangent;
            if ( m_CurrentGeometryType == GeometryType::Flat )
            {
                // In flat geometry, use the x-y plane tangent
                tangent = glm::normalize(glm::vec3(-r_vec.y, r_vec.x, 0.0f));
                bi_tangent = glm::normalize(glm::vec3(0.0f, -r_vec.z, r_vec.y));
            }
            else if ( m_CurrentGeometryType == GeometryType::Spherical )
            {
                // In spherical geometry, compute a tangent vector on the sphere
                // Cross product with the radial direction (toward the center) to get a tangent
                glm::vec3 radial0 = glm::normalize(pos0); // Direction toward the center
                glm::vec3 radial1 = glm::normalize(pos1);
                glm::vec3 avgRadial = glm::normalize(radial0 + radial1); // Approximate radial direction at midpoint
                tangent = glm::normalize(glm::cross(r_vec, avgRadial));
            } else {
                tangent = glm::normalize(glm::vec3(-r_vec.y, r_vec.x, 0.0f)); // Default for other geometries
            }

            float v = std::sqrt(mu / r);
            m_MassiveObjects[0]->m_Object.velocity = tangent * (v * m_MassiveObjects[1]->m_Object.mass / totalMass);
            m_MassiveObjects[1]->m_Object.velocity = -tangent * (v * m_MassiveObjects[0]->m_Object.mass / totalMass);

            for ( size_t i = 2; i < m_MassiveObjects.size(); ++i )
                m_MassiveObjects[i]->m_Object.velocity = glm::vec3(0.0f);
        }
    }

    // if ( ImGui::Button("Add Object") ) {
    //     auto newShape = std::make_shared<Shape>(m_GridScale, m_GridSize);
    //     m_MassiveObjects.push_back(newShape);
    //     generateGrid(); // Regenerate grid and velocities
    // }

    ImGui::End();
}

vk::Format GridRenderer::findSupportedFormat(
    const std::vector<vk::Format>& candidates,
    const vk::ImageTiling& tiling,
    const vk::FormatFeatureFlags& features)
{
    for ( const vk::Format& format : candidates )
    {
        const vk::FormatProperties props = m_PhysicalDevice.getFormatProperties(format);
        if ( tiling == vk::ImageTiling::eLinear && (props.linearTilingFeatures & features) == features )
            return format;
        if ( tiling == vk::ImageTiling::eOptimal && (props.optimalTilingFeatures & features) == features )
            return format;
    }
    return vk::Format::eD32Sfloat; // Fallback to default
}

vk::Format GridRenderer::findDepthFormat()
{
    return findSupportedFormat(
        { vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint, vk::Format::eD24UnormS8Uint },
        vk::ImageTiling::eOptimal,
        vk::FormatFeatureFlagBits::eDepthStencilAttachment
    );
}

uint32_t GridRenderer::findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties) const
{
    const vk::PhysicalDeviceMemoryProperties& memProperties = m_PhysicalDevice.getMemoryProperties();
    for ( uint32_t i = 0; i < memProperties.memoryTypeCount; i++ )
    {
        if ( typeFilter & 1 << i && (memProperties.memoryTypes[i].propertyFlags & properties) == properties )
            return i;
    }
    throw std::runtime_error("failed to find suitable memory type!");
}

void GridRenderer::transitionImageLayout(
        const vk::Image& image,
        const vk::Format& format,
        const vk::ImageLayout& oldLayout,
        const vk::ImageLayout& newLayout)
{
    const vk::CommandBuffer commandBuffer = beginSingleTimeCommands();

    vk::ImageMemoryBarrier barrier{};
    barrier.oldLayout = oldLayout;
    barrier.newLayout = newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    if ( !(oldLayout == vk::ImageLayout::eUndefined && newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal) )
        throw std::invalid_argument("unsupported layout transition!");

    barrier.srcAccessMask = vk::AccessFlagBits::eNone;
    barrier.dstAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentRead | vk::AccessFlagBits::eDepthStencilAttachmentWrite;
    constexpr vk::PipelineStageFlags sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
    constexpr vk::PipelineStageFlags destinationStage = vk::PipelineStageFlagBits::eEarlyFragmentTests;

    commandBuffer.pipelineBarrier(
        sourceStage, destinationStage,
        vk::DependencyFlagBits::eByRegion,
        0, nullptr,
        0, nullptr,
        1, &barrier
    );

    endSingleTimeCommands(commandBuffer);
}

void GridRenderer::createDepthResources()
{
    m_DepthFormat = findDepthFormat();
    m_DepthImages.resize(m_SwapchainImages.size());
    m_DepthImageMemory.resize(m_SwapchainImages.size());
    m_DepthImageViews.resize(m_SwapchainImages.size());

    for ( size_t i = 0; i < m_SwapchainImages.size(); i++ )
    {
        // Create depth image
        vk::ImageCreateInfo imageInfo{};
        imageInfo.imageType = vk::ImageType::e2D;
        imageInfo.extent.width = m_SwapchainExtent.width;
        imageInfo.extent.height = m_SwapchainExtent.height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = m_DepthFormat;
        imageInfo.tiling = vk::ImageTiling::eOptimal;
        imageInfo.initialLayout = vk::ImageLayout::eUndefined;
        imageInfo.usage = vk::ImageUsageFlagBits::eDepthStencilAttachment;
        imageInfo.sharingMode = vk::SharingMode::eExclusive;
        imageInfo.samples = vk::SampleCountFlagBits::e1;

        m_DepthImages[i] = m_Device.createImage(imageInfo);

        // Allocate memory for depth image
        vk::MemoryRequirements memRequirements = m_Device.getImageMemoryRequirements(m_DepthImages[i]);
        vk::MemoryAllocateInfo allocInfo{};
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal);
        m_DepthImageMemory[i] = m_Device.allocateMemory(allocInfo);
        m_Device.bindImageMemory(m_DepthImages[i], m_DepthImageMemory[i], 0);

        // Create image view
        vk::ImageViewCreateInfo viewInfo{};
        viewInfo.image = m_DepthImages[i];
        viewInfo.viewType = vk::ImageViewType::e2D;
        viewInfo.format = m_DepthFormat;
        viewInfo.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;
        m_DepthImageViews[i] = m_Device.createImageView(viewInfo);

        // Transition depth image layout
        transitionImageLayout(m_DepthImages[i], m_DepthFormat, vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthStencilAttachmentOptimal);
    }
}
