//
// Created by Richard Mule on 3/22/25.
//

#include "GridRenderer.hpp"
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <imgui.h>
#include <sstream>
#include <filesystem>
#include <vector>
#include <random>
#include <cfloat>
#include <cstdio>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(_WIN32)
#include <windows.h>
#endif

namespace
{
    std::filesystem::path getExecutableDir()
    {
        namespace fs = std::filesystem;
        fs::path exePath;
#if defined(__APPLE__)
        char path[1024];
        uint32_t size = sizeof(path);
        if (_NSGetExecutablePath(path, &size) == 0)
        {
            std::error_code ec;
            exePath = fs::canonical(fs::path(path), ec);
            if (!ec)
                return exePath.parent_path();
        }
#elif defined(__linux__)
        char path[1024];
        ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
        if (len != -1)
        {
            path[len] = '\0';
            std::error_code ec;
            exePath = fs::canonical(fs::path(path), ec);
            if (!ec)
                return exePath.parent_path();
        }
#elif defined(_WIN32)
        wchar_t path[MAX_PATH];
        if (GetModuleFileNameW(NULL, path, MAX_PATH) > 0)
        {
            std::error_code ec;
            exePath = fs::canonical(fs::path(path), ec);
            if (!ec)
                return exePath.parent_path();
        }
#endif
        return fs::current_path(); // fallback
    }

    static std::vector<char> readFile(const std::string& fileName)
    {
        namespace fs = std::filesystem;
        static const fs::path exeDir = getExecutableDir();

        // Search in multiple likely locations so the app can be launched from IDE, terminal, etc.
        const std::vector<fs::path> candidates = {
            fs::path(fileName),                                   // CWD (original behavior)
            exeDir / fileName,
            exeDir / ".." / fileName,
            exeDir / "cmake-build-debug" / fileName,
            fs::current_path() / "cmake-build-debug" / fileName,
            fs::current_path() / fileName,
            exeDir / ".." / "cmake-build-debug" / fileName,
        };

        for (const auto& candidate : candidates)
        {
            std::error_code ec;
            fs::path p = candidate;
            if (fs::exists(p, ec) && !ec)
            {
                std::ifstream file(p, std::ios::ate | std::ios::binary);
                if (file.is_open())
                {
                    const size_t fileSize = static_cast<size_t>(file.tellg());
                    std::vector<char> shaderCode(fileSize);
                    file.seekg(0);
                    file.read(shaderCode.data(), fileSize);
                    return shaderCode;
                }
            }
            // Also try canonical in case of symlinks
            fs::path canon = fs::canonical(candidate, ec);
            if (!ec && fs::exists(canon))
            {
                std::ifstream file(canon, std::ios::ate | std::ios::binary);
                if (file.is_open())
                {
                    const size_t fileSize = static_cast<size_t>(file.tellg());
                    std::vector<char> shaderCode(fileSize);
                    file.seekg(0);
                    file.read(shaderCode.data(), fileSize);
                    return shaderCode;
                }
            }
        }

        throw std::runtime_error("Failed to open " + fileName +
                                 " (searched near CWD and executable dir: " + exeDir.string() + ")");
    }
} // anonymous namespace

GridRenderer::GridRenderer(
    const vk::Device& device,
    const vk::PhysicalDevice& physicalDevice,
    const vk::CommandPool& commandPool,
    const vk::Queue& graphicsQueue,
    const vk::RenderPass& renderPass,
    const std::vector<vk::Image>& swapchainImages,
    const vk::Extent2D& swapchainExtent,
    Simulation& simulation)
    : m_Simulation(simulation)
    , m_CurrentGeometryType(simulation.geometryType())
    , m_Device(device)
    , m_PhysicalDevice(physicalDevice)
    , m_CommandPool(commandPool)
    , m_GraphicsQueue(graphicsQueue)
    , m_RenderPass(renderPass)
    , m_SwapchainImages(swapchainImages)
    , m_SwapchainExtent(swapchainExtent)
    , m_PipelineLayout(nullptr)
    , m_GraphicsPipeline(nullptr)
    , m_TrianglePipeline(nullptr)
    , m_PushConstants()
    , m_DepthFormat()
    , m_GridObject()
{
    // Grid dimensions and the active geometry come from the simulation, which
    // is the single owner of that state (and survives swapchain recreation).
    m_GridSize  = m_Simulation.gridSize();
    m_GridScale = m_Simulation.gridScale();
    m_Geometry  = m_Simulation.geometry();
}

GridRenderer::~GridRenderer()
{
    // Buffers (m_VertexBuffer, m_IndexBuffer, m_BodiesBuffer, m_MembraneBuffer,
    // m_TrailVertexBuffer, m_TrailIndexBuffer) are VulkanBuffer members and
    // free themselves when this object is destroyed.

    if ( m_GraphicsPipeline )
        m_Device.destroyPipeline(m_GraphicsPipeline);
    if ( m_TrianglePipeline )
        m_Device.destroyPipeline(m_TrianglePipeline);
    if ( m_PipelineLayout )
        m_Device.destroyPipelineLayout(m_PipelineLayout);

    // Compute resources
    if ( m_ComputePipeline )
        m_Device.destroyPipeline(m_ComputePipeline);
    if ( m_ComputePipelineLayout )
        m_Device.destroyPipelineLayout(m_ComputePipelineLayout);
    if ( m_ComputeDescriptorSetLayout )
        m_Device.destroyDescriptorSetLayout(m_ComputeDescriptorSetLayout);
    if ( m_ComputeDescriptorPool )
        m_Device.destroyDescriptorPool(m_ComputeDescriptorPool);
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
    createComputePipeline();
    createMembraneBuffers();

    // Now that the compute descriptor set exists, write the current vertex buffer into it.
    ensureComputeDescriptors();
}

// Allocates the two ping-pong membrane buffers (one (height, velocity) pair per
// grid vertex) and zeroes them so the sheet starts flat and at rest. The grid
// resolution is fixed, so this runs once and the buffers are never recreated.
void GridRenderer::createMembraneBuffers()
{
    const vk::DeviceSize vertexCount =
        static_cast<vk::DeviceSize>(m_GridSize + 1) * static_cast<vk::DeviceSize>(m_GridSize + 1);
    const vk::DeviceSize bufferSize = vertexCount * 2u * sizeof(float);

    for ( auto& buffer : m_MembraneBuffer )
        buffer = makeBuffer(bufferSize,
                            vk::BufferUsageFlagBits::eStorageBuffer |
                            vk::BufferUsageFlagBits::eTransferDst,
                            vk::MemoryPropertyFlagBits::eDeviceLocal);

    const vk::CommandBuffer cmd = beginSingleTimeCommands();
    for ( auto& buffer : m_MembraneBuffer )
        cmd.fillBuffer(buffer.handle(), 0, bufferSize, 0u);
    endSingleTimeCommands(cmd);

    m_MembraneRead = 0;
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


    for ( const auto& shape : m_Simulation.bodies() )
        shape->addVertices(m_Vertices, m_Indices);

}

VulkanBuffer GridRenderer::makeBuffer(
    const vk::DeviceSize size,
    const vk::BufferUsageFlags usage,
    const vk::MemoryPropertyFlags properties) const
{
    return VulkanBuffer{ m_Device, m_PhysicalDevice, size, usage, properties };
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
    // Called on init and again on every geometry switch / reset. Wait for the
    // GPU before reallocating so the move-assignment below never frees a
    // buffer still referenced by an in-flight frame.
    if ( m_VertexBuffer.valid() )
        m_Device.waitIdle();

    const vk::DeviceSize bufferSize = sizeof(m_Vertices[0]) * m_Vertices.size();

    VulkanBuffer staging = makeBuffer(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferSrc,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    staging.writeFromHost(m_Vertices.data(), bufferSize);

    // Also usable as a storage buffer so the compute shader can write to it.
    // Move-assignment frees the previous vertex buffer (if any).
    m_VertexBuffer = makeBuffer(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferDst |
        vk::BufferUsageFlagBits::eVertexBuffer |
        vk::BufferUsageFlagBits::eStorageBuffer,
        vk::MemoryPropertyFlagBits::eDeviceLocal);

    copyBuffer(staging.handle(), m_VertexBuffer.handle(), bufferSize);
    // `staging` frees itself at end of scope.

    // NOTE: We no longer write the vertex descriptor here because m_ComputeDescriptorSet
    // may not exist yet (createVertexBuffer is called before createComputePipeline in init()).
    // The descriptor write now happens in ensureComputeDescriptors() after the set is created.
}

// === GPU Grid Toggle ===
// This function is the gate for compute work (now enabled by default).
void GridRenderer::setUseGPUGrid(bool enabled)
{
    if (m_UseGPUGrid == enabled)
        return;

    m_UseGPUGrid = enabled;

    if (enabled)
    {
        ensureComputeDescriptors();
    }
}

// Uploads the massive bodies (position, mass, visual size) to the SSBO that
// the warping compute shader reads. Called once per frame from drawFrame(),
// after the in-flight fence wait so the direct host write is race-free.
void GridRenderer::updateBodiesBuffer()
{
    if (!m_UseGPUGrid)
        return;

    const auto& massiveBodies = m_Simulation.bodies();
    const size_t numBodies = massiveBodies.size();

    // Center of mass (used by the GPU warping model).
    glm::vec3 com(0.0f);
    float totalMass = 0.0f;
    for (const auto& shape : massiveBodies)
    {
        const auto& obj = shape->m_Object;
        com += obj.position * obj.mass;
        totalMass += obj.mass;
    }
    if (totalMass > 0.0f)
        com /= totalMass;
    m_CenterOfMass = com;

    // Layout must match the BodyData struct in the compute shaders.
    struct GPUBody {
        glm::vec4 position_mass; // xyz = position, w = mass (Flat/Hyperbolic warping)
        float     size;          // visual size (Spherical warping + rendering)
        float     padding[3];
    };

    std::vector<GPUBody> bodies(numBodies);
    for (size_t i = 0; i < numBodies; ++i)
    {
        const auto& obj = massiveBodies[i]->m_Object;
        bodies[i].position_mass = glm::vec4(obj.position, obj.mass);
        bodies[i].size          = massiveBodies[i]->getSize();
    }

    // Persistent host-visible storage buffer, grown only when more bodies
    // appear and written directly each frame — no staging copy, no queue
    // stall. This is safe because VulkanApp::drawFrame() updates the bodies
    // buffer only *after* waiting on the in-flight fence, so the previous
    // frame's compute shader has finished reading it.
    const vk::DeviceSize requiredBytes = numBodies * sizeof(GPUBody);
    if (!m_BodiesBuffer.valid() || requiredBytes > m_BodiesBuffer.size())
    {
        const size_t capacity = std::max(numBodies, size_t(16));
        m_BodiesBuffer = makeBuffer(
            capacity * sizeof(GPUBody),
            vk::BufferUsageFlagBits::eStorageBuffer,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    }

    if (numBodies == 0)
        return;

    m_BodiesBuffer.writeFromHost(bodies.data(), requiredBytes);

    // Point the compute descriptor at the (possibly newly grown) bodies buffer.
    if (m_ComputeDescriptorSet)
    {
        vk::DescriptorBufferInfo bufInfo{};
        bufInfo.setBuffer(m_BodiesBuffer.handle())
               .setOffset(0)
               .setRange(VK_WHOLE_SIZE);

        vk::WriteDescriptorSet write{};
        write.setDstSet(m_ComputeDescriptorSet)
             .setDstBinding(1)                              // binding 1 = bodies
             .setDescriptorType(vk::DescriptorType::eStorageBuffer)
             .setDescriptorCount(1)
             .setPBufferInfo(&bufInfo);

        m_Device.updateDescriptorSets(write, {});
    }
}

// Ensures both compute descriptor bindings are valid.
// This fixes the "descriptor never updated" errors we were seeing on dispatch.
void GridRenderer::ensureComputeDescriptors()
{
    if (!m_ComputeDescriptorSet)
        return;

    // Binding 0 - Vertex buffer
    if (m_VertexBuffer.valid())
    {
        vk::DescriptorBufferInfo bufInfo{};
        bufInfo.setBuffer(m_VertexBuffer.handle())
               .setOffset(0)
               .setRange(VK_WHOLE_SIZE);

        vk::WriteDescriptorSet write{};
        write.setDstSet(m_ComputeDescriptorSet)
             .setDstBinding(0)
             .setDescriptorType(vk::DescriptorType::eStorageBuffer)
             .setDescriptorCount(1)
             .setPBufferInfo(&bufInfo);

        m_Device.updateDescriptorSets(write, {});
    }

    // Binding 1 - Bodies buffer
    if (m_BodiesBuffer.valid())
    {
        vk::DescriptorBufferInfo bufInfo{};
        bufInfo.setBuffer(m_BodiesBuffer.handle())
               .setOffset(0)
               .setRange(VK_WHOLE_SIZE);

        vk::WriteDescriptorSet write{};
        write.setDstSet(m_ComputeDescriptorSet)
             .setDstBinding(1)
             .setDescriptorType(vk::DescriptorType::eStorageBuffer)
             .setDescriptorCount(1)
             .setPBufferInfo(&bufInfo);

        m_Device.updateDescriptorSets(write, {});
    }

    // Bindings 2 / 3 - membrane state, ping-ponged. The compute shader reads
    // m_MembraneRead and writes the other; recordComputeWork() swaps the index
    // afterwards, so these are rewritten every frame.
    if (m_MembraneBuffer[0].valid() && m_MembraneBuffer[1].valid())
    {
        const int readIdx  = m_MembraneRead;
        const int writeIdx = 1 - m_MembraneRead;

        vk::DescriptorBufferInfo readInfo{};
        readInfo.setBuffer(m_MembraneBuffer[readIdx].handle())
                .setOffset(0).setRange(VK_WHOLE_SIZE);
        vk::DescriptorBufferInfo writeInfo{};
        writeInfo.setBuffer(m_MembraneBuffer[writeIdx].handle())
                 .setOffset(0).setRange(VK_WHOLE_SIZE);

        std::array<vk::WriteDescriptorSet, 2> writes{};
        writes[0].setDstSet(m_ComputeDescriptorSet).setDstBinding(2)
                 .setDescriptorType(vk::DescriptorType::eStorageBuffer)
                 .setDescriptorCount(1).setPBufferInfo(&readInfo);
        writes[1].setDstSet(m_ComputeDescriptorSet).setDstBinding(3)
                 .setDescriptorType(vk::DescriptorType::eStorageBuffer)
                 .setDescriptorCount(1).setPBufferInfo(&writeInfo);

        m_Device.updateDescriptorSets(writes, {});
    }
}

void GridRenderer::recordComputeWork(vk::CommandBuffer commandBuffer)
{
    // Records the per-geometry warping compute shader into `commandBuffer`.
    // The shader regenerates the base surface and warps it from the bodies
    // SSBO, writing the entire vertex storage buffer; updateGrid() is then a
    // no-op on the GPU path. A compute->vertex-input barrier (below) makes the
    // writes visible to the draw that follows.

    // GPU path gate (returns early when the CPU fallback is selected).
    if (!m_UseGPUGrid)
        return;

    if (!m_ComputePipeline || !m_ComputeDescriptorSet)
        return;

    // Make sure descriptors are written (handles cases where GPU mode is enabled later)
    ensureComputeDescriptors();

    // Bind compute pipeline and descriptor set
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, m_ComputePipeline);
    commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute,
                                     m_ComputePipelineLayout,
                                     0, 1, &m_ComputeDescriptorSet,
                                     0, nullptr);

    // Push constants — layout must match the PushConstants block in
    // flat_grid.comp exactly (order and types).
    struct PushConstants {
        int32_t gridSize;
        float   scale;
        float   warpGain;
        float   wellDepth;
        int32_t numBodies;
        float   softeningLength;
        float   radialInfluence;
        float   recenterOffset;
        int32_t geometryType;   // 0=Flat, 1=Spherical, 2=Hyperbolic
        int32_t membraneEnabled;
        float   membraneStiffness;
        float   membraneWaveSpeed;
        float   membraneDamping;
    };

    PushConstants pc{};
    pc.gridSize        = m_GridSize;
    pc.scale           = m_GridScale;
    pc.warpGain        = m_Warp.warpGain;
    pc.wellDepth       = m_Warp.wellDepth;
    pc.numBodies       = static_cast<int32_t>(m_Simulation.bodies().size());
    pc.softeningLength = m_Warp.softening;
    pc.radialInfluence = m_Warp.radialInfluence;
    pc.recenterOffset  = m_RecenterOffset;
    pc.membraneEnabled   = m_Warp.membraneEnabled ? 1 : 0;
    pc.membraneStiffness = m_Warp.membraneStiffness;
    pc.membraneWaveSpeed = m_Warp.membraneWaveSpeed;
    pc.membraneDamping   = m_Warp.membraneDamping;

    // Map our enum to the shader's expected values
    switch (m_CurrentGeometryType)
    {
        case GeometryType::Spherical:  pc.geometryType = 1; break;
        case GeometryType::Hyperbolic: pc.geometryType = 2; break;
        default:                       pc.geometryType = 0; break; // Flat
    }

    commandBuffer.pushConstants(m_ComputePipelineLayout,
                                vk::ShaderStageFlagBits::eCompute,
                                0, sizeof(pc), &pc);

    // Dispatch — 32x32 local workgroup size. The grid has (gridSize + 1)
    // vertices per row, so cover gridSize+1; the shader bounds-checks the rest.
    uint32_t groupsX = (m_GridSize + 1 + 31) / 32;
    uint32_t groupsY = (m_GridSize + 1 + 31) / 32;
    commandBuffer.dispatch(groupsX, groupsY, 1);

    // Barrier: the compute writes must be visible to (a) vertex input, for the
    // draw of the warped grid, and (b) next frame's compute, which reads the
    // membrane buffer it just wrote (ping-pong).
    vk::MemoryBarrier barrier{};
    barrier.setSrcAccessMask(vk::AccessFlagBits::eShaderWrite)
           .setDstAccessMask(vk::AccessFlagBits::eVertexAttributeRead |
                             vk::AccessFlagBits::eShaderRead);

    commandBuffer.pipelineBarrier(
        vk::PipelineStageFlagBits::eComputeShader,
        vk::PipelineStageFlagBits::eVertexInput | vk::PipelineStageFlagBits::eComputeShader,
        {},
        barrier, {}, {}
    );

    // Swap the membrane buffers: the state just written becomes next frame's
    // read source. The single in-flight frame + fence wait orders the access.
    m_MembraneRead = 1 - m_MembraneRead;
}

void GridRenderer::createIndexBuffer()
{
    // createVertexBuffer() is always called just before this and already
    // drained the GPU, so no extra waitIdle is needed. The previous index
    // buffer (if any) is freed by the move-assignment below.
    const vk::DeviceSize bufferSize = sizeof(m_Indices[0]) * m_Indices.size();

    VulkanBuffer staging = makeBuffer(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferSrc,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    staging.writeFromHost(m_Indices.data(), bufferSize);

    m_IndexBuffer = makeBuffer(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer,
        vk::MemoryPropertyFlagBits::eDeviceLocal);

    copyBuffer(staging.handle(), m_IndexBuffer.handle(), bufferSize);
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

void GridRenderer::createComputePipeline()
{
    // Load the flat grid compute shader (we will evolve this into full warping later)
    std::vector<char> compCode = readFile("flat_grid.comp.spv");

    vk::ShaderModuleCreateInfo compModuleInfo{};
    compModuleInfo.setCodeSize(compCode.size())
                  .setPCode(reinterpret_cast<const uint32_t*>(compCode.data()));
    vk::ShaderModule compShaderModule = m_Device.createShaderModule(compModuleInfo);

    // Descriptor set layout for compute shader (infrastructure phase)
    // Binding 0: Vertex buffer (positions written by compute)
    // Binding 1: Bodies buffer (position + mass of massive objects) - added for Option A
    vk::DescriptorSetLayoutBinding vertexBinding{};
    vertexBinding.setBinding(0)
               .setDescriptorType(vk::DescriptorType::eStorageBuffer)
               .setDescriptorCount(1)
               .setStageFlags(vk::ShaderStageFlagBits::eCompute);

    vk::DescriptorSetLayoutBinding bodiesBinding{};
    bodiesBinding.setBinding(1)
               .setDescriptorType(vk::DescriptorType::eStorageBuffer)
               .setDescriptorCount(1)
               .setStageFlags(vk::ShaderStageFlagBits::eCompute);

    // Bindings 2/3: dynamic-membrane state, ping-ponged each frame. Binding 2
    // is the previous frame's snapshot (read), binding 3 is this frame (write).
    vk::DescriptorSetLayoutBinding membraneReadBinding{};
    membraneReadBinding.setBinding(2)
                .setDescriptorType(vk::DescriptorType::eStorageBuffer)
                .setDescriptorCount(1)
                .setStageFlags(vk::ShaderStageFlagBits::eCompute);

    vk::DescriptorSetLayoutBinding membraneWriteBinding{};
    membraneWriteBinding.setBinding(3)
                .setDescriptorType(vk::DescriptorType::eStorageBuffer)
                .setDescriptorCount(1)
                .setStageFlags(vk::ShaderStageFlagBits::eCompute);

    std::array<vk::DescriptorSetLayoutBinding, 4> bindings =
        { vertexBinding, bodiesBinding, membraneReadBinding, membraneWriteBinding };

    vk::DescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.setBindingCount(static_cast<uint32_t>(bindings.size()))
            .setPBindings(bindings.data());

    m_ComputeDescriptorSetLayout = m_Device.createDescriptorSetLayout(layoutInfo);

    // Push constant range sized for the improved warping model
    vk::PushConstantRange pushConstantRange{};
    pushConstantRange.setStageFlags(vk::ShaderStageFlagBits::eCompute)
                     .setOffset(0)
                     .setSize(64);   // safe size for current + future parameters

    // Pipeline layout
    vk::PipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.setSetLayoutCount(1)
                      .setPSetLayouts(&m_ComputeDescriptorSetLayout)
                      .setPushConstantRangeCount(1)
                      .setPPushConstantRanges(&pushConstantRange);

    m_ComputePipelineLayout = m_Device.createPipelineLayout(pipelineLayoutInfo);

    // Compute pipeline
    vk::PipelineShaderStageCreateInfo compStage{};
    compStage.setStage(vk::ShaderStageFlagBits::eCompute)
             .setModule(compShaderModule)
             .setPName("main");

    vk::ComputePipelineCreateInfo computePipelineInfo{};
    computePipelineInfo.setStage(compStage)
                       .setLayout(m_ComputePipelineLayout);

    auto result = m_Device.createComputePipeline(nullptr, computePipelineInfo);
    if (result.result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create compute pipeline for grid");

    m_ComputePipeline = result.value;

    std::cout << "[GPU] Compute pipeline created successfully from flat_grid.comp.spv\n";

    // Clean up shader module (pipeline keeps its own reference)
    m_Device.destroyShaderModule(compShaderModule);

    // Create descriptor pool + allocate the compute descriptor set
    vk::DescriptorPoolSize poolSize{};
    poolSize.setType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(8);

    vk::DescriptorPoolCreateInfo poolCreate{};
    poolCreate.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet)
              .setMaxSets(4)
              .setPoolSizeCount(1)
              .setPPoolSizes(&poolSize);

    m_ComputeDescriptorPool = m_Device.createDescriptorPool(poolCreate);

    vk::DescriptorSetAllocateInfo allocInfo{};
    allocInfo.setDescriptorPool(m_ComputeDescriptorPool)
             .setDescriptorSetCount(1)
             .setPSetLayouts(&m_ComputeDescriptorSetLayout);

    auto allocated = m_Device.allocateDescriptorSets(allocInfo);
    m_ComputeDescriptorSet = allocated[0];
}

void GridRenderer::updateGeometry(GeometryType type)
{
    if ( m_CurrentGeometryType == type )
        return;

    // The simulation owns the geometry: it builds the new one and converts
    // all body + trail coordinates onto it.
    m_Simulation.switchGeometry(type);

    // Re-alias the (new) geometry.
    m_Geometry = m_Simulation.geometry();
    m_CurrentGeometryType = type;

    // Rebuild the visual grid for the new surface.
    generateGrid();
    createVertexBuffer();
    createIndexBuffer();
}

void GridRenderer::updateTrails()
{
    m_TrailVertices.clear();
    m_TrailIndices.clear();

    const auto& trails = m_Simulation.trails();
    uint32_t vertexOffset = 0;
    for ( size_t i = 0; i < trails.size(); ++i )
    {
        const auto& positions = trails[i].positions;
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

    // Append the null-geodesic (light-ray) paths in a bright light colour. The
    // trail already holds positions lifted onto the warped surface (done once
    // per point in Simulation::advanceRays), so this is just a copy.
    const glm::vec3 rayColor(1.0f, 0.92f, 0.35f);
    for ( const auto& ray : m_Simulation.rays() )
    {
        const auto& positions = ray.trail;
        if ( positions.size() < 2 )
            continue;

        for ( const auto& pos : positions )
        {
            Vertex vertex{};
            vertex.position = pos;
            vertex.color    = rayColor;
            vertex.normal   = glm::vec3(0.0f, 1.0f, 0.0f);
            m_TrailVertices.push_back(vertex);
        }
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
        // --- Trail vertex buffer ---
        const vk::DeviceSize bufferSize = sizeof(m_TrailVertices[0]) * m_TrailVertices.size();
        VulkanBuffer vertexStaging = makeBuffer(
            bufferSize,
            vk::BufferUsageFlagBits::eTransferSrc,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
        vertexStaging.writeFromHost(m_TrailVertices.data(), bufferSize);

        if ( !m_TrailVertexBuffer.valid() || m_TrailVertexBuffer.size() != bufferSize )
            m_TrailVertexBuffer = makeBuffer(
                bufferSize,
                vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer,
                vk::MemoryPropertyFlagBits::eDeviceLocal);
        copyBuffer(vertexStaging.handle(), m_TrailVertexBuffer.handle(), bufferSize);

        // --- Trail index buffer ---
        const vk::DeviceSize indexBufferSize = sizeof(m_TrailIndices[0]) * m_TrailIndices.size();
        VulkanBuffer indexStaging = makeBuffer(
            indexBufferSize,
            vk::BufferUsageFlagBits::eTransferSrc,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
        indexStaging.writeFromHost(m_TrailIndices.data(), indexBufferSize);

        if ( !m_TrailIndexBuffer.valid() || m_TrailIndexBuffer.size() != indexBufferSize )
            m_TrailIndexBuffer = makeBuffer(
                indexBufferSize,
                vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer,
                vk::MemoryPropertyFlagBits::eDeviceLocal);
        copyBuffer(indexStaging.handle(), m_TrailIndexBuffer.handle(), indexBufferSize);
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

    commandBuffer.bindVertexBuffers(0, m_VertexBuffer.handle(), { 0 });
    commandBuffer.bindIndexBuffer(m_IndexBuffer.handle(), 0, vk::IndexType::eUint32);

    // Auto-fit the clip planes to the scene each frame so the grid + bodies
    // can never fall outside the depth range as the camera moves. The scene is
    // centred on the origin with a radius of roughly 1.5 * gridScale.
    const float aspect   = static_cast<float>(m_SwapchainExtent.width)
                         / static_cast<float>(m_SwapchainExtent.height);
    const float sceneRadius = m_GridScale * 1.5f;
    const float camDist  = glm::length(m_CameraController.eyePosition());
    const float nearPlane = std::max(0.5f, camDist - sceneRadius);
    const float farPlane  = camDist + sceneRadius;

    PushConstants pc{};
    pc.view = m_CameraController.viewMatrix();
    pc.projection = glm::perspective(glm::radians(m_CameraController.fov()),
                                     aspect, nearPlane, farPlane);
    pc.projection[1][1] *= -1; // Flip Y-axis for Vulkan

    // Draw grid (lines) — the grid lives in world space (identity transform).
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_GraphicsPipeline);
    pc.model = glm::mat4(1.0f);
    commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &pc);
    commandBuffer.drawIndexed(m_GridObject.indexCount, 1, m_GridObject.indexOffset, 0, 0);

    // Draw all shapes (triangles) — model transform derived from position.
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_TrianglePipeline);
    for ( const auto& shape : m_Simulation.bodies() )
    {
        const Object& obj = shape->m_Object;
        pc.model = glm::translate(glm::mat4(1.0f), obj.position);
        commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &pc);
        commandBuffer.drawIndexed(obj.indexCount, 1, obj.indexOffset, 0, 0);
    }

    if ( !m_TrailVertices.empty() && !m_TrailIndices.empty() )
    {
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_GraphicsPipeline); // Use the line pipeline
        commandBuffer.bindVertexBuffers(0, m_TrailVertexBuffer.handle(), { 0 });
        commandBuffer.bindIndexBuffer(m_TrailIndexBuffer.handle(), 0, vk::IndexType::eUint32);

        PushConstants trail_constants{};
        trail_constants.view = m_CameraController.viewMatrix();
        trail_constants.projection = glm::perspective(glm::radians(m_CameraController.fov()),
                                                      aspect, nearPlane, farPlane);
        trail_constants.projection[1][1] *= -1; // Flip Y-axis for Vulkan
        trail_constants.model = glm::mat4(1.0f); // Trails are in world space
        commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &trail_constants);

        commandBuffer.drawIndexed(static_cast<uint32_t>(m_TrailIndices.size()), 1, 0, 0, 0);
    }
}

void GridRenderer::updateCamera()
{
    m_CameraController.update();
}

void GridRenderer::updateGrid()
{
    // Re-centering offset (mean warp depth) keeps the Spherical / Hyperbolic
    // manifolds from drifting under a global potential bias; Flat does not
    // re-center. Computed every frame here, before the GPU early-return, so
    // recordComputeWork() can read it as a push constant.
    m_RecenterOffset = (m_CurrentGeometryType == GeometryType::Flat)
        ? 0.0f
        : m_Geometry->computeRecenterOffset(m_Simulation.bodies(), m_Warp);

    // GPU path: recordComputeWork() warps and owns the vertex buffer directly.
    if (m_UseGPUGrid)
        return;

    // ===================== CPU FALLBACK PATH =====================
    m_Geometry->setGridParams(m_GridSize, m_GridScale);
    m_Geometry->warpGrid(m_Vertices, m_Simulation.bodies(), m_Warp, m_RecenterOffset);

    /// Upload the CPU-warped vertices to the GPU vertex buffer.
    const vk::DeviceSize bufferSize = sizeof(m_Vertices[0]) * m_Vertices.size();
    VulkanBuffer staging = makeBuffer(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferSrc,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    staging.writeFromHost(m_Vertices.data(), bufferSize);

    copyBuffer(staging.handle(), m_VertexBuffer.handle(), bufferSize);
}

void GridRenderer::updateSimulation(float deltaTime)
{
    // Hand the simulation the current warp so freshly traced ray points get
    // lifted onto the warped surface as they are added (cheap), rather than
    // re-lifting the whole trail history every frame (which tanked the FPS).
    m_Simulation.setRayWarp(m_Warp, m_RecenterOffset);

    // The N-body step runs in the Simulation; the renderer then rebuilds the
    // trail GPU buffers from the updated trail history.
    m_Simulation.step(deltaTime);

    // A merge changed the body set — rebuild the grid + body vertex/index
    // buffers so the (now fewer, resized) bodies render correctly.
    if ( m_Simulation.consumeBodiesChanged() )
    {
        generateGrid();
        createVertexBuffer();
        createIndexBuffer();
    }

    updateTrails();
}

void GridRenderer::renderCameraControls()
{
    // Position the detailed controls on the right side so it doesn't fight the status panel
    ImGui::SetNextWindowPos(ImVec2(1920 - 380, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
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

    // GPU toggle - moved here so it's grouped with the geometry selector
    ImGui::Checkbox("Use GPU Grid", &m_UseGPUGrid);
    ImGui::SameLine();
    ImGui::Checkbox("Body Merging", &m_Simulation.mergeEnabled());

    // Camera controls
    m_CameraController.renderControls();

    ImGui::DragFloat("Gravity Strength", &m_Simulation.gravity(), 0.01f, 0.01f, 1.5f, "%.2f");
    ImGui::DragFloat("Time Step", &m_Simulation.timeStep(), 0.001f, 0.001f, 0.1f, "%.3f");
    ImGui::SliderFloat("Velocity Angle", &m_VelocityAngle, 0.0f, 90.0f, "%.1f");

    // Grid-warp (gravitational-well visualisation) tuning.
    ImGui::DragFloat("Well Depth", &m_Warp.wellDepth, 1.0f, 0.0f, 200.0f, "%.0f");
    ImGui::DragFloat("Warp Gain", &m_Warp.warpGain, 0.005f, 0.01f, 2.0f, "%.3f");
    ImGui::DragFloat("Radial Influence", &m_Warp.radialInfluence, 0.005f, 0.0f, 1.0f, "%.3f");

    // Dynamic-membrane (rippling grid) tuning — GPU path only.
    ImGui::Checkbox("Membrane Ripples", &m_Warp.membraneEnabled);
    if ( m_Warp.membraneEnabled )
    {
        ImGui::DragFloat("Membrane Stiffness", &m_Warp.membraneStiffness, 0.005f, 0.0f, 0.5f, "%.3f");
        ImGui::DragFloat("Wave Speed", &m_Warp.membraneWaveSpeed, 0.005f, 0.0f, 0.49f, "%.3f");
        ImGui::DragFloat("Membrane Damping", &m_Warp.membraneDamping, 0.005f, 0.0f, 0.3f, "%.3f");
    }

    // Null-geodesic (light-ray) demo. Emit a fan of rays, then toggle Lensing:
    // off = pure manifold geodesics (curvature deviation), on = light bending.
    ImGui::Separator();

    // Ray Mode — Dynamic: rays advance against the live, moving field.
    // Static: the bodies (and the curvature they produce) are frozen, but rays
    // still travel one step per frame — animating through a fixed snapshot.
    int rayMode = m_Simulation.raysStatic() ? 1 : 0;
    if ( ImGui::Combo("Ray Mode", &rayMode, "Dynamic\0Static (frozen)\0") )
    {
        m_Simulation.raysStatic() = (rayMode == 1);
        m_Simulation.clearRays();   // rays from the other mode no longer apply
    }

    if ( ImGui::Button("Emit Rays") )
        m_Simulation.emitRays();
    ImGui::SameLine();
    if ( ImGui::Button("Clear Rays") )
        m_Simulation.clearRays();
    ImGui::SameLine();
    ImGui::Checkbox("Lensing", &m_Simulation.lensingEnabled());
    ImGui::SliderInt("Ray Count", &m_Simulation.rayCount(), 3, 400);
    ImGui::DragFloat("Ray Speed", &m_Simulation.raySpeed(), 1.0f, 1.0f, 200.0f, "%.0f");
    if ( m_Simulation.lensingEnabled() )
        ImGui::DragFloat("Lens Strength", &m_Simulation.rayLensStrength(), 0.005f, 0.0f, 1.0f, "%.3f");
    ImGui::Separator();

    // Conserved-quantity diagnostics: rolling plots of total energy and total
    // angular momentum. A good (symplectic) integrator keeps these ~flat.
    m_EnergyHistory[m_HistoryHead] = m_Simulation.totalEnergy();
    m_AngMomHistory[m_HistoryHead] = m_Simulation.angularMomentum();
    m_HistoryHead = (m_HistoryHead + 1) % kHistorySize;

    ImGui::Text("KE %.0f   PE %.0f", m_Simulation.kineticEnergy(),
                                     m_Simulation.potentialEnergy());

    char energyLabel[32];
    std::snprintf(energyLabel, sizeof(energyLabel), "%.0f", m_Simulation.totalEnergy());
    ImGui::PlotLines("Total Energy", m_EnergyHistory.data(), kHistorySize, m_HistoryHead,
                     energyLabel, FLT_MAX, FLT_MAX, ImVec2(0.0f, 48.0f));

    char angLabel[32];
    std::snprintf(angLabel, sizeof(angLabel), "%.0f", m_Simulation.angularMomentum());
    ImGui::PlotLines("Ang. Momentum", m_AngMomHistory.data(), kHistorySize, m_HistoryHead,
                     angLabel, FLT_MAX, FLT_MAX, ImVec2(0.0f, 48.0f));

    if ( ImGui::Button("Reset Camera") )
        m_CameraController.reset(m_CurrentGeometryType == GeometryType::Spherical, m_GridScale);

    // Per-body controls (mass / velocity / position).
    const auto& bodies = m_Simulation.bodies();
    for ( size_t i = 0; i < bodies.size(); ++i )
    {
        ImGui::PushID(static_cast<int>(i));
        ImGui::Text("%s", bodies[i]->getName().c_str());
        ImGui::DragFloat("Mass", &bodies[i]->m_Object.mass, 0.1f, 0.0000001f, 50.0f);
        ImGui::DragFloat3("Velocity", &bodies[i]->m_Object.velocity[0], 0.01f, -1.0f, 1.0f);
        ImGui::InputFloat3("Position", glm::value_ptr(bodies[i]->m_Object.position), "%.3f");
        ImGui::PopID();
    }

    // Reset simulation: rebuild the body set, then rebuild the grid buffers.
    if ( ImGui::Button("Reset Simulation") )
    {
        m_Simulation.resetTwoBody();
        generateGrid();
        createVertexBuffer();
        createIndexBuffer();
    }

    if ( ImGui::Button("Reset to Orbit") )
        m_Simulation.resetToOrbit();

    ImGui::End();
    ImGui::PopStyleVar();
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
