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
    Object cubeObj{};
    cubeObj.mass = 10.0f;
    constexpr auto og_coords = glm::vec3(-5.0f, 0.0f, 0.0f);
    const glm::vec3 converted_coords = convertCoordinates(
        og_coords,
        GeometryType::Flat,
        geometryType,
        m_GridSize / 2);
    cubeObj.modelMatrix = glm::translate(glm::mat4(1.0f), converted_coords);
    m_MassiveObjects.push_back(std::make_unique<Cube>(cubeObj));

    Object sphereObj{};
    sphereObj.mass = 5.0f;
    constexpr auto og_coords2 = glm::vec3(5.0f, 0.0f, 0.0f);
    const glm::vec3 converted_coords2 = convertCoordinates(
        og_coords2,
        GeometryType::Flat,
        geometryType,
        m_GridSize / 2);
    sphereObj.modelMatrix = glm::translate(glm::mat4(1.0f), converted_coords2);
    m_MassiveObjects.push_back(std::make_unique<Sphere>(sphereObj));
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
    m_GridObject.indexOffset = 0;
    m_GridObject.indexCount = static_cast<uint32_t>(m_Indices.size());
    m_GridObject.modelMatrix = glm::mat4(1.0f);

    for ( const auto& shape : m_MassiveObjects )
        shape->addVertices(m_Vertices, m_Indices);

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

        // Adjust positions relative to center of mass
        float dist = m_Geometry->computeDistance(
            m_MassiveObjects[0]->m_Object.modelMatrix[3],
            m_MassiveObjects[1]->m_Object.modelMatrix[3]
        );
        if ( dist < 0.01f )
            dist = 0.01f;
        for ( const auto& shape : m_MassiveObjects )
            shape->m_Object.modelMatrix[3] -= glm::vec4(com, 0.0f);

        // Ensure velocity is perpendicular to radius in the correct plane
        const float mu = m_Gravity * (m_MassiveObjects[0]->m_Object.mass + m_MassiveObjects[1]->m_Object.mass);
        const float v = std::sqrt(mu / dist);
        auto r = glm::vec3(m_MassiveObjects[1]->m_Object.modelMatrix[3]);
        r -= glm::vec3(m_MassiveObjects[0]->m_Object.modelMatrix[3]);
        glm::vec3 tangent;
        if ( m_CurrentGeometryType == GeometryType::Spherical )
        {
            // For spherical, use the tangent in the phi direction
            const float phi = atan2(r.y, r.x);
            tangent = glm::vec3(-sin(phi), cos(phi), 0.0f);
        } else
        {
            // For flat/hyperbolic, orbit in x-y plane
            tangent = glm::normalize(glm::vec3(-r.y, 0.0f, r.x));
        }
        m_MassiveObjects[0]->m_Object.velocity = v * tangent * (m_MassiveObjects[1]->m_Object.mass / totalMass);
        m_MassiveObjects[1]->m_Object.velocity = -v * tangent * (m_MassiveObjects[0]->m_Object.mass / totalMass);
        // Debug the tangent and velocity
        std::cout << "Distance: " << dist << ", mu: " << mu << ", v: " << v << "\n";
        std::cout << "Tangent: (" << tangent.x << ", " << tangent.y << ", " << tangent.z << ")\n";

        // Additional shapes start stationary
        for ( size_t i = 2; i < m_MassiveObjects.size(); ++i )
            m_MassiveObjects[i]->m_Object.velocity = glm::vec3(0.0f);
    }
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
    gridInputAssembly.setTopology(vk::PrimitiveTopology::eLineList)
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
    if ( m_CurrentGeometryType != type )
    {
        m_Geometry = geometryFactory(type, m_GridSize, m_GridScale);
        for ( auto& shape : m_MassiveObjects )
        {
            glm::vec3 currentPos = shape->m_Object.modelMatrix[3];

            // Convert the position from the old geometry type to the new one
            glm::vec3 newPos = convertCoordinates(currentPos, m_CurrentGeometryType, type, m_GridScale / 2.0f);

            // Update the modelMatrix with the new position
            shape->m_Object.modelMatrix = glm::translate(glm::mat4(1.0f), newPos);

            // Log the new position for debugging
            std::cout << "Object at (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.z
                      << ") converted to (" << newPos.x << ", " << newPos.y << ", " << newPos.z << ")\n";

        }

        m_CurrentGeometryType = type;
        generateGrid();
        createVertexBuffer();
        createIndexBuffer();
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
}

void GridRenderer::updateCamera()
{
    const ImGuiIO& io = ImGui::GetIO();

    // Zoom with trackpad scroll
    if ( io.MouseWheel != 0.0f )
    {
        m_ZoomLevel -= io.MouseWheel * 0.1f;
        m_ZoomLevel = std::max(0.1f, std::min(m_ZoomLevel, 5.0f));
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
        constexpr float minElevation = glm::radians(-89.0f);
        constexpr float maxElevation = glm::radians(89.0f);
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
    glm::vec3 currentPos = m_Camera.getPosition();
    if ( glm::length(currentPos - lastPos) > 0.001f )
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
    m_Geometry->warpGrid(m_Vertices, massivePositions, masses, m_Gravity, maxDisplacement, minDistSquared, softeningLength);

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

void GridRenderer::updateSimulation(float deltaTime) const
{
    deltaTime = std::min(deltaTime, 0.05f);

    // Reset accelerations
    for ( auto& shape : m_MassiveObjects )
        shape->m_Object.acceleration = glm::vec3(0.0f);

    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        for ( size_t j = i + 1; j < m_MassiveObjects.size(); ++j )
        {
            Object& obj1 = m_MassiveObjects[i]->m_Object;
            Object& obj2 = m_MassiveObjects[j]->m_Object;

            const glm::vec3 pos1 = obj1.modelMatrix[3];
            const glm::vec3 pos2 = obj2.modelMatrix[3];
            const glm::vec3 r = pos2 - pos1;
            constexpr float minDistance = 0.5f; // Approximate size of shapes
            if ( glm::length(r) < minDistance )
            {
                // Simple elastic collision
                const glm::vec3 avgVelocity = (
                        obj1.velocity * obj1.mass +
                        obj2.velocity * obj2.mass
                    ) / (obj1.mass + obj2.mass);
                obj1.velocity = avgVelocity;
                obj2.velocity = avgVelocity;
            }

            float dist = m_Geometry->computeDistance(pos1, pos2);
            if ( dist < 0.01f )
                dist = 0.01f;

            const float forceMagnitude = m_Gravity * obj1.mass * obj2.mass / (dist * dist);
            const glm::vec3 forceDir = glm::normalize(r);

            const glm::vec3 force = forceDir * forceMagnitude;
            obj1.acceleration += force / obj1.mass;
            obj2.acceleration -= force / obj2.mass;
        }
    }

    // Update positions using the geometry's rules
    for ( size_t i = 0; i < m_MassiveObjects.size(); ++i )
    {
        auto& shape = m_MassiveObjects[i];
        // Add damping to prevent runaway motion
        shape->m_Object.velocity *= 0.999f;
        m_Geometry->updatePosition(shape->m_Object, deltaTime, m_GridScale / 2.0f);

        // Debug position
        // glm::vec3 pos = shape->m_Object.modelMatrix[3];
        // std::cout << "Shape " << i << " Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    }
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

    if ( ImGui::Button("Reset Camera") )
    {
        m_Camera.position = glm::vec3(0.0f, 20.0f, 20.0f);
        m_Camera.target = glm::vec3(0.0f, 0.0f, 0.0f);
        m_Camera.up = glm::vec3(0.0f, 1.0f, 0.0f);
        m_Camera.fov = 90.0f;
        m_Camera.nearPlane = 0.1f;
        m_Camera.farPlane = 100.0f;
        m_Camera.azimuth = glm::radians(45.0f);
        m_Camera.elevation = glm::radians(45.0f);
        m_Camera.radius = 30.0f;
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

            glm::vec3 tangent;
            if ( m_CurrentGeometryType == GeometryType::Flat )
            {
                // In flat geometry, use the x-y plane tangent
                tangent = glm::normalize(glm::vec3(-r_vec.y, r_vec.x, 0.0f));
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
