//
// Created by Richard Mule on 7/5/25.
//

#include "ResourceManager.hpp"


std::ostream& operator<<(std::ostream& os, const ResourceTypes& type)
{
    os << magic_enum::enum_name(type);
    return os;
}


VulkanResourceManager::VulkanResourceManager(GLFWwindow* window)
    : m_Window(window)
    , m_Instance(nullptr)
    , m_PhysicalDevice(nullptr)
    , m_Device(nullptr)
    , m_Surface(nullptr)
    , m_SwapchainExtent({})
    , m_SwapchainImages({})
    , m_SwapchainImageViews({})
    , m_RenderPass(nullptr)
    , m_SwapchainFramebuffers({})
    , m_QueueFamilyIndices({})
    , m_Queues(std::unordered_map<ResourceTypes, vk::Queue>())
    , m_CommandPools(std::unordered_map<ResourceTypes, vk::CommandPool>())
    , m_CommandBuffers(std::unordered_map<ResourceTypes, std::vector<vk::CommandBuffer>>())
    , m_Buffers(std::unordered_map<ResourceTypes, vk::Buffer>())
    , m_DeviceMemory(std::unordered_map<ResourceTypes, vk::DeviceMemory>())
    , m_Semaphores(std::unordered_map<ResourceTypes, vk::Semaphore>())
    , m_Fences(std::unordered_map<ResourceTypes, vk::Fence>())
    , m_Formats(std::unordered_map<ResourceTypes, vk::Format>())
    , m_Images(std::unordered_map<ResourceTypes, std::vector<vk::Image>>())
    , m_ImageMemory(std::unordered_map<ResourceTypes, std::vector<vk::DeviceMemory>>())
    , m_ImageViews(std::unordered_map<ResourceTypes, std::vector<vk::ImageView>>())
    , m_Pipelines(std::unordered_map<ResourceTypes, vk::Pipeline>())
    , m_PipelineLayouts(std::unordered_map<ResourceTypes, vk::PipelineLayout>())
    , m_DescriptorSet(std::unordered_map<ResourceTypes, vk::DescriptorSet>())
    , m_DescriptorSetLayouts(std::unordered_map<ResourceTypes, vk::DescriptorSetLayout>())
    , m_DescriptorPools(std::unordered_map<ResourceTypes, vk::DescriptorPool>())
{
}

VulkanResourceManager::~VulkanResourceManager()
{
    for ( const auto& pool : m_CommandPools | std::views::values )
        m_Device.destroyCommandPool(pool);
    for ( const auto& buffer : m_Buffers| std::views::values )
        m_Device.destroyBuffer(buffer);
    for ( const auto& memory : m_DeviceMemory | std::views::values )
        m_Device.freeMemory(memory);
    for ( const auto& semaphore : m_Semaphores | std::views::values )
        m_Device.destroySemaphore(semaphore);
    for ( const auto& fence : m_Fences | std::views::values )
        m_Device.destroyFence(fence);
    for ( const auto& framebuffer : m_SwapchainFramebuffers )
        m_Device.destroyFramebuffer(framebuffer);
    if ( m_RenderPass )
        m_Device.destroyRenderPass(m_RenderPass);
    for ( const auto& imageView : m_SwapchainImageViews )
        m_Device.destroyImageView(imageView);
    if ( m_Surface )
        m_Instance.destroySurfaceKHR(m_Surface);
    m_Device.destroy();
    m_Instance.destroy();
}

void VulkanResourceManager::initialize()
{
    createInstance();
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapchain();
    createImageViews();
    createRenderPass();
    createFramebuffers();
    createCommandPool();
    createCommandBuffers();
    createSyncObjects();
}

void VulkanResourceManager::resetSwapchainFrameBuffers() const
{
    for ( const auto& framebuffer : m_SwapchainFramebuffers )
        m_Device.destroyFramebuffer(framebuffer);
}

void VulkanResourceManager::resetSwapchainImageViews() const
{
    for ( const auto& imageView : m_SwapchainImageViews )
        m_Device.destroyImageView(imageView);
}

vk::ShaderModule VulkanResourceManager::makeShader(const std::string& shader_file_path) const
{
    const std::vector<char> shader_code = utils::readFile(shader_file_path);
    vk::ShaderModuleCreateInfo shaderInfo{};
    shaderInfo.sType = vk::StructureType::eShaderModuleCreateInfo;
    shaderInfo.codeSize = shader_code.size();
    shaderInfo.pCode = reinterpret_cast<const uint32_t*>(shader_code.data());
    return m_Device.createShaderModule(shaderInfo);
}

const vk::Queue& VulkanResourceManager::getResourceQueue(const ResourceTypes type) const
{
    return m_Queues.at(type);
}

size_t VulkanResourceManager::resourceImageCount(const ResourceTypes type) const
{
    return m_Images.at(type).size();
}

void VulkanResourceManager::makeDescriptorSetLayout(
    const std::vector<vk::DescriptorSetLayoutBinding>& bindings,
    const ResourceTypes type)
{
    vk::DescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.bindingCount = bindings.size();
    layoutInfo.pBindings = bindings.data();
    m_DescriptorSetLayouts[type] = m_Device.createDescriptorSetLayout(layoutInfo);
}
void VulkanResourceManager::makeDescriptorPool(
    const vk::DescriptorPoolCreateInfo& info,
    const ResourceTypes type)
{
    m_DescriptorPools[type] = m_Device.createDescriptorPool(info);
    if ( !m_DescriptorPools[type] )
        throw std::runtime_error(std::format(
            "Failed to create descriptor pool for ImGui (resource = {})", magic_enum::enum_name(type)
        ));
}

void VulkanResourceManager::makeDescriptorSet(const vk::DescriptorSetAllocateInfo& info, const ResourceTypes type)
{
    const auto result = m_Device.allocateDescriptorSets(&info, &m_DescriptorSet[type]);
    if ( result != vk::Result::eSuccess )
        throw std::runtime_error("Failed to allocate object descriptor set: " + vk::to_string(result));
}

void VulkanResourceManager::makePipelineLayout(
    const size_t layout_count,
    const vk::ShaderStageFlagBits bit_flags,
    const size_t push_constant_size,
    const ResourceTypes type)
{
    vk::PipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.setLayoutCount = layout_count;
    pipelineLayoutInfo.pSetLayouts = &getResourceDescriptorPoolLayout(type);
    vk::PushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = bit_flags;
    pushConstantRange.offset = 0;
    pushConstantRange.size = push_constant_size;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
    m_PipelineLayouts[type] = m_Device.createPipelineLayout(pipelineLayoutInfo);
}

void VulkanResourceManager::makeComputePipeline(
    const vk::StructureType s_type,
    const vk::StructureType stage_type,
    const vk::ShaderModule& shader_module,
    const ResourceTypes type)
{
    vk::ComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = s_type;
    pipelineInfo.stage.sType = stage_type;
    pipelineInfo.stage.stage = vk::ShaderStageFlagBits::eCompute;
    pipelineInfo.stage.module = shader_module;
    pipelineInfo.stage.pName = "main";
    pipelineInfo.layout = getResourcePipelineLayout(type);
    const auto pipelineResult = m_Device.createComputePipeline(nullptr, pipelineInfo);
    if ( pipelineResult.result != vk::Result::eSuccess )
        throw std::runtime_error(std::format(
            "Failed to create {} compute pipeline: {}",
            magic_enum::enum_name(type),
            vk::to_string(pipelineResult.result)
        ));
    m_Pipelines[type] = pipelineResult.value;
}

const vk::Fence& VulkanResourceManager::getResourceFence(ResourceTypes type) const
{
    return m_Fences.at(type);
}

const vk::CommandBuffer& VulkanResourceManager::getResourceCommandBuffer(const ResourceTypes type) const
{
    return m_CommandBuffers.at(type).front();
}

const vk::DescriptorSet& VulkanResourceManager::getResourceDescriptorSet(const ResourceTypes type) const
{
    return m_DescriptorSet.at(type);
}

const vk::DescriptorSetLayout& VulkanResourceManager::getResourceDescriptorPoolLayout(const ResourceTypes type) const
{
    return m_DescriptorSetLayouts.at(type);
}

const vk::DescriptorPool& VulkanResourceManager::getResourceDescriptorPool(const ResourceTypes type) const
{
    return m_DescriptorPools.at(type);
}

const vk::CommandPool& VulkanResourceManager::getResourceCommandPool(const ResourceTypes type) const
{
    return m_CommandPools.at(type);
}

const vk::PipelineLayout& VulkanResourceManager::getResourcePipelineLayout(ResourceTypes type) const
{
    return m_PipelineLayouts.at(type);
}

const vk::Pipeline& VulkanResourceManager::getResourcePipeline(ResourceTypes type) const
{
    return m_Pipelines.at(type);
}

vk::CommandBuffer VulkanResourceManager::beginSingleTimeCommands(
    const vk::CommandPool& commandPool,
    vk::CommandBufferAllocateInfo& allocInfo,
    const vk::CommandBufferBeginInfo& beginInfo) const
{
    allocInfo.setCommandPool(commandPool);const auto result = m_Device.allocateCommandBuffers(allocInfo);
    if ( result.empty() )
        throw std::runtime_error("Failed to allocate command buffer");

    const vk::CommandBuffer commandBuffer = result.front();
    commandBuffer.begin(beginInfo);
    return commandBuffer;
}

vk::CommandBuffer VulkanResourceManager::beginSingleTimeCommands(const vk::CommandPool& commandPool) const
{
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setLevel(vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(1);
    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.setFlags(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
    return beginSingleTimeCommands(commandPool, allocInfo, beginInfo);
}

void VulkanResourceManager::endSingleTimeCommands(
    const vk::CommandBuffer& commandBuffer,
    const vk::Queue& queue,
    const vk::CommandPool& pool,
    const vk::Fence& fence) const
{
    commandBuffer.end();vk::SubmitInfo submitInfo{};
    submitInfo.setCommandBufferCount(1)
              .setPCommandBuffers(&commandBuffer);

    auto result = queue.submit(1, &submitInfo, fence);
    if ( result != vk::Result::eSuccess )
        throw std::runtime_error("Failed to submit command buffer: " + vk::to_string(result));
    if ( !fence )
        queue.waitIdle();
    else
    {
        result = m_Device.waitForFences(1, &fence, VK_TRUE, UINT64_MAX);
        if ( result != vk::Result::eSuccess )
            throw std::runtime_error("Failed to wait for fence: " + vk::to_string(result));
    }

    m_Device.freeCommandBuffers(pool, commandBuffer);
}

vk::Buffer VulkanResourceManager::createBuffer(
    const vk::DeviceSize size,
    const vk::BufferUsageFlags usage) const
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

vk::DeviceMemory VulkanResourceManager::allocateBufferMemory(
    const vk::Buffer& buffer,
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

void VulkanResourceManager::copyBuffer(
    const vk::CommandPool& commandPool,
    const vk::Queue& queue,
    const vk::Buffer& srcBuffer,
    const vk::Buffer& dstBuffer,
    const vk::DeviceSize size) const
{
    const auto commandBuffer = beginSingleTimeCommands(commandPool);
    vk::BufferCopy copyRegion{};
    copyRegion.setSize(size);
    commandBuffer.copyBuffer(srcBuffer, dstBuffer, copyRegion);

    endSingleTimeCommands(commandBuffer, queue, commandPool);
}

QueueFamilyIndices VulkanResourceManager::findQueueFamilies(const vk::PhysicalDevice& device) const
{
    uint32_t i = 0;
    QueueFamilyIndices indices;
    const auto queueFamilies = device.getQueueFamilyProperties();
    for ( const auto& queueFamily : queueFamilies )
    {
        if ( queueFamily.queueFlags & vk::QueueFlagBits::eGraphics )
            indices.graphicsFamily = i;
        if ( device.getSurfaceSupportKHR(i, m_Surface) )
            indices.presentFamily = i;
        if ( queueFamily.queueFlags & vk::QueueFlagBits::eCompute )
            indices.computeFamily = i;
        if ( indices.isComplete() )
            break;
        ++i;
    }
    return indices;
}

bool VulkanResourceManager::isDeviceSuitable(const vk::PhysicalDevice& device) const
{
    const QueueFamilyIndices indices = findQueueFamilies(device);
    return indices.isComplete();
}

vk::Format VulkanResourceManager::findSupportedFormat(
    const std::vector<vk::Format>& candidates,
    const vk::ImageTiling& tiling,
    const vk::FormatFeatureFlags features) const
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

vk::Format VulkanResourceManager::findDepthFormat() const
{
    return findSupportedFormat(
        { vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint, vk::Format::eD24UnormS8Uint },
        vk::ImageTiling::eOptimal,
        vk::FormatFeatureFlagBits::eDepthStencilAttachment
    );
}

uint32_t VulkanResourceManager::findMemoryType(
    const uint32_t typeFilter,
    const vk::MemoryPropertyFlags& properties) const
{
    const vk::PhysicalDeviceMemoryProperties& memProperties = m_PhysicalDevice.getMemoryProperties();
    for ( uint32_t i = 0; i < memProperties.memoryTypeCount; i++ )
    {
        if ( typeFilter & 1 << i && (memProperties.memoryTypes[i].propertyFlags & properties) == properties )
            return i;
    }
    throw std::runtime_error("failed to find suitable memory type!");
}

void VulkanResourceManager::transitionImageLayout(
    const vk::Image& image,
    const vk::Format& format,
    const vk::ImageLayout& oldLayout,
    const vk::ImageLayout& newLayout)
{
    const vk::CommandBuffer commandBuffer = beginSingleTimeCommands(m_CommandPools[ResourceTypes::GRAPHICS]);
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

    endSingleTimeCommands(commandBuffer, m_Queues[ResourceTypes::GRAPHICS], m_CommandPools[ResourceTypes::GRAPHICS]);
}

void VulkanResourceManager::createInstance()
{
    vk::ApplicationInfo appInfo{};
    appInfo.setPApplicationName("Gravity Simulator")
           .setApplicationVersion(VK_MAKE_VERSION(1, 0, 0))
           .setPEngineName("No Engine")
           .setEngineVersion(VK_MAKE_VERSION(1, 0, 0))
           .setApiVersion(VK_API_VERSION_1_2);uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
    const std::vector<const char*> add_extensions = {
        VK_KHR_SURFACE_EXTENSION_NAME,
        VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME,
        VK_MVK_MACOS_SURFACE_EXTENSION_NAME2
    };
    for ( const auto& extension : add_extensions )
        extensions.push_back(extension);

    vk::InstanceCreateInfo createInfo{};
    createInfo.setPApplicationInfo(&appInfo)
              .setEnabledExtensionCount(static_cast<uint32_t>(extensions.size()))
              .setPEnabledExtensionNames(extensions)
              .setEnabledLayerCount(0);
    createInfo.setFlags(vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR);

    m_Instance = vk::createInstance(createInfo);
    if ( !m_Instance )
        throw std::runtime_error("Failed to create Vulkan instance");
}

void VulkanResourceManager::createSurface()
{
    VkSurfaceKHR surface;
    if ( glfwCreateWindowSurface(m_Instance, m_Window, nullptr, &surface) != VK_SUCCESS )
        throw std::runtime_error("Failed to create window surface");
    m_Surface = surface;
}

void VulkanResourceManager::pickPhysicalDevice()
{
    const auto devices = m_Instance.enumeratePhysicalDevices();
    if ( devices.empty() )
        throw std::runtime_error("Failed to find GPUs with Vulkan support");

    for ( const auto& device : devices )
    {
        if ( isDeviceSuitable(device) )
        {
            m_PhysicalDevice = device;
            break;
        }
    }

    if ( !m_PhysicalDevice )
        throw std::runtime_error("Failed to find a suitable GPU");}void VulkanResourceManager::createLogicalDevice()
    {
        const auto [
            graphicsFamily,
            presentFamily,
            computeFamily
        ] = findQueueFamilies(m_PhysicalDevice);// Verify queue capabilities
    const auto queueFamilies = m_PhysicalDevice.getQueueFamilyProperties();
    if ( graphicsFamily >= queueFamilies.size() )
        throw std::runtime_error("Invalid graphics queue family index");
    if ( computeFamily >= queueFamilies.size() )
        throw std::runtime_error("Invalid compute queue family index");

    // Check graphics queue capabilities
    if ( !(queueFamilies[graphicsFamily].queueFlags & vk::QueueFlagBits::eGraphics) )
        throw std::runtime_error("Graphics queue family does not support graphics operations");
    if ( !(queueFamilies[graphicsFamily].queueFlags & vk::QueueFlagBits::eTransfer) )
        throw std::runtime_error("Graphics queue family does not support transfer operations");

    // Check compute queue capabilities
    if ( !(queueFamilies[computeFamily].queueFlags & vk::QueueFlagBits::eCompute) )
        throw std::runtime_error("Compute queue family does not support compute operations");

    std::set<uint32_t> uniqueQueueFamilies = { graphicsFamily, presentFamily, computeFamily };
    std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
    constexpr float queuePriority = 1.0f;
    for ( uint32_t queueFamily : uniqueQueueFamilies )
    {
        vk::DeviceQueueCreateInfo queueCreateInfo{};
        queueCreateInfo.setQueueFamilyIndex(queueFamily)
                       .setQueueCount(1)
                       .setPQueuePriorities(&queuePriority);
        queueCreateInfos.push_back(queueCreateInfo);
    }

    vk::PhysicalDeviceFeatures deviceFeatures{};

    vk::DeviceCreateInfo createInfo{};
    const std::vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
    };
    createInfo.setQueueCreateInfoCount(static_cast<uint32_t>(queueCreateInfos.size()))
              .setPQueueCreateInfos(queueCreateInfos.data())
              .setPEnabledFeatures(&deviceFeatures)
              .setEnabledExtensionCount(1)
              .setPEnabledExtensionNames(deviceExtensions)
              .setEnabledLayerCount(0);

    m_Device = m_PhysicalDevice.createDevice(createInfo);
    if ( !m_Device )
        throw std::runtime_error("Failed to create logical device");

    m_Queues[ResourceTypes::GRAPHICS] = m_Device.getQueue(graphicsFamily, 0);
    m_Queues[ResourceTypes::PRESENT] = m_Device.getQueue(presentFamily, 0);
    m_Queues[ResourceTypes::COMPUTE] = m_Device.getQueue(computeFamily, 0);}void VulkanResourceManager::createSwapchain()
    {
        const auto capabilities = m_PhysicalDevice.getSurfaceCapabilitiesKHR(m_Surface);
        const auto formats = m_PhysicalDevice.getSurfaceFormatsKHR(m_Surface);
        if ( formats.empty() )
            throw std::runtime_error("No surface formats available");vk::SurfaceFormatKHR surfaceFormat = formats[0];
    for ( const auto& format : formats )
    {
        if ( format.format == vk::Format::eB8G8R8A8Srgb && format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear )
        {
            surfaceFormat = format;
            break;
        }
    }

    const auto presentModes = m_PhysicalDevice.getSurfacePresentModesKHR(m_Surface);
    if ( presentModes.empty() )
        throw std::runtime_error("No present modes available");

    auto presentMode = vk::PresentModeKHR::eFifo;
    for ( const auto& mode : presentModes )
    {
        if ( mode == vk::PresentModeKHR::eMailbox )
        {
            presentMode = mode;
            break;
        }
    }

    int width, height;
    glfwGetFramebufferSize(m_Window, &width, &height);
    while ( width == 0 || height == 0 )  // Handle minimization
    {
        glfwGetFramebufferSize(m_Window, &width, &height);
        glfwWaitEvents();
    }

    m_SwapchainExtent = vk::Extent2D{
        static_cast<uint32_t>(width),
        static_cast<uint32_t>(height)};
    m_SwapchainExtent.width = std::max(
        capabilities.minImageExtent.width, std::min(
            capabilities.maxImageExtent.width,
            m_SwapchainExtent.width));
    m_SwapchainExtent.height = std::max(
        capabilities.minImageExtent.height, std::min(
            capabilities.maxImageExtent.height,
            m_SwapchainExtent.height));

    uint32_t imageCount = capabilities.minImageCount + 1;
    if ( capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount )
        imageCount = capabilities.maxImageCount;

    vk::SwapchainCreateInfoKHR createInfo{};
    createInfo.setSurface(m_Surface)
              .setMinImageCount(imageCount)
              .setImageFormat(surfaceFormat.format)
              .setImageColorSpace(surfaceFormat.colorSpace)
              .setImageExtent(m_SwapchainExtent)
              .setImageArrayLayers(1)
              .setImageUsage(vk::ImageUsageFlagBits::eColorAttachment);

    const auto [
        graphicsFamily,
        presentFamily,
        computeFamily
    ] = findQueueFamilies(m_PhysicalDevice);
    const uint32_t queueFamilyIndices[] = { graphicsFamily, presentFamily, computeFamily };
    if ( graphicsFamily != presentFamily )
        createInfo.setImageSharingMode(vk::SharingMode::eConcurrent)
                  .setQueueFamilyIndexCount(2)
                  .setPQueueFamilyIndices(queueFamilyIndices);
    else
        createInfo.setImageSharingMode(vk::SharingMode::eExclusive);

    createInfo.setPreTransform(capabilities.currentTransform)
              .setCompositeAlpha(vk::CompositeAlphaFlagBitsKHR::eOpaque)
              .setPresentMode(presentMode)
              .setClipped(VK_TRUE)
              .setOldSwapchain(m_Swapchain);

    // Store the old swapchain
    const vk::SwapchainKHR oldSwapchain = m_Swapchain;
    createInfo.setOldSwapchain(oldSwapchain);

    m_Swapchain = m_Device.createSwapchainKHR(createInfo);
    if ( !m_Swapchain )
        throw std::runtime_error("Failed to create swapchain");

    // Destroy the old swapchain after the new one is created
    if ( oldSwapchain )
        m_Device.destroySwapchainKHR(oldSwapchain);

    m_SwapchainImages = m_Device.getSwapchainImagesKHR(m_Swapchain);}void VulkanResourceManager::createImageViews()
    {
        m_SwapchainImageViews.resize(m_SwapchainImages.size());
        for ( size_t i = 0; i < m_SwapchainImages.size(); ++i )
        {
            vk::ImageViewCreateInfo createInfo{};
            createInfo.setImage(m_SwapchainImages[i])
                      .setViewType(vk::ImageViewType::e2D)
                      .setFormat(vk::Format::eB8G8R8A8Srgb)
                      .setComponents(vk::ComponentMapping())
                      .setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));    m_SwapchainImageViews[i] = m_Device.createImageView(createInfo);
        if ( !m_SwapchainImageViews[i] )
            throw std::runtime_error("Failed to create image views");
    }

}

void VulkanResourceManager::createRenderPass()
{
    vk::AttachmentDescription colorAttachment{};
    colorAttachment.setFormat(vk::Format::eB8G8R8A8Srgb)
                   .setSamples(vk::SampleCountFlagBits::e1)
                   .setLoadOp(vk::AttachmentLoadOp::eClear)
                   .setStoreOp(vk::AttachmentStoreOp::eStore)
                   .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
                   .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
                   .setInitialLayout(vk::ImageLayout::eUndefined)
                   .setFinalLayout(vk::ImageLayout::ePresentSrcKHR);vk::AttachmentReference colorAttachmentRef{};
    colorAttachmentRef.setAttachment(0)
                      .setLayout(vk::ImageLayout::eColorAttachmentOptimal);

    // Depth attachment
    vk::AttachmentDescription depthAttachment{};
    depthAttachment.format = findDepthFormat(); // vk::Format::eD32Sfloat; // Use default if nullptr
    depthAttachment.samples = vk::SampleCountFlagBits::e1;
    depthAttachment.loadOp = vk::AttachmentLoadOp::eClear;
    depthAttachment.storeOp = vk::AttachmentStoreOp::eDontCare;
    depthAttachment.stencilLoadOp = vk::AttachmentLoadOp::eDontCare;
    depthAttachment.stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
    depthAttachment.initialLayout = vk::ImageLayout::eUndefined;
    depthAttachment.finalLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;

    vk::AttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 1;
    depthAttachmentRef.layout = vk::ImageLayout::eDepthStencilAttachmentOptimal;

    vk::SubpassDescription subpass{};
    subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
           .setColorAttachmentCount(1)
           .setPColorAttachments(&colorAttachmentRef);

    // Subpass dependency
    vk::SubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests;
    dependency.srcAccessMask = vk::AccessFlagBits::eNone;
    dependency.dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests;
    dependency.dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentWrite;

    // Render Pass
    std::array<vk::AttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};
    vk::RenderPassCreateInfo renderPassInfo{};
    renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    m_RenderPass = m_Device.createRenderPass(renderPassInfo);
    if ( !m_RenderPass )
        throw std::runtime_error("Failed to create render pass");}void VulkanResourceManager::createDepthResources()
    {
        m_Formats[ResourceTypes::DEPTH] = findDepthFormat();
        m_Images[ResourceTypes::DEPTH].resize(m_SwapchainImages.size());
        m_ImageMemory[ResourceTypes::DEPTH].resize(m_SwapchainImages.size());
        m_ImageViews[ResourceTypes::DEPTH].resize(m_SwapchainImages.size());for ( size_t i = 0; i < m_SwapchainImages.size(); i++ )
    {
        // Create depth image
        vk::ImageCreateInfo imageInfo{};
        imageInfo.imageType = vk::ImageType::e2D;
        imageInfo.extent.width = m_SwapchainExtent.width;
        imageInfo.extent.height = m_SwapchainExtent.height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = m_Formats[ResourceTypes::DEPTH];
        imageInfo.tiling = vk::ImageTiling::eOptimal;
        imageInfo.initialLayout = vk::ImageLayout::eUndefined;
        imageInfo.usage = vk::ImageUsageFlagBits::eDepthStencilAttachment;
        imageInfo.sharingMode = vk::SharingMode::eExclusive;
        imageInfo.samples = vk::SampleCountFlagBits::e1;

        m_Images[ResourceTypes::DEPTH][i] = m_Device.createImage(imageInfo);

        // Allocate memory for depth image
        vk::MemoryRequirements memRequirements = m_Device.getImageMemoryRequirements(m_Images[ResourceTypes::DEPTH][i]);
        vk::MemoryAllocateInfo allocInfo{};
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(
            memRequirements.memoryTypeBits,
            vk::MemoryPropertyFlagBits::eDeviceLocal);
        m_ImageMemory[ResourceTypes::DEPTH][i] = m_Device.allocateMemory(allocInfo);
        m_Device.bindImageMemory(
            m_Images[ResourceTypes::DEPTH][i],
            m_ImageMemory[ResourceTypes::DEPTH][i],
            0);

        // Create image view
        vk::ImageViewCreateInfo viewInfo{};
        viewInfo.image = m_Images[ResourceTypes::DEPTH][i];
        viewInfo.viewType = vk::ImageViewType::e2D;
        viewInfo.format = m_Formats[ResourceTypes::DEPTH];
        viewInfo.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;
        m_ImageViews[ResourceTypes::DEPTH][i] = m_Device.createImageView(viewInfo);

        // Transition depth image layout
        transitionImageLayout(
            m_Images[ResourceTypes::DEPTH][i],
            m_Formats[ResourceTypes::DEPTH],
            vk::ImageLayout::eUndefined,
            vk::ImageLayout::eDepthStencilAttachmentOptimal);
    }
}

void VulkanResourceManager::createFramebuffers()
{
    m_SwapchainFramebuffers.resize(m_SwapchainImageViews.size());
    std::vector<vk::ImageView> imageViews;
    if ( m_ImageViews.contains(ResourceTypes::DEPTH) )
        imageViews = m_ImageViews[ResourceTypes::DEPTH];for ( size_t i = 0; i < m_SwapchainImageViews.size(); ++i )
    {
        std::array<vk::ImageView, 2> attachments = {
            m_SwapchainImageViews[i],
            !imageViews.empty() ? imageViews[i] : m_SwapchainImageViews[i]
        };

        vk::FramebufferCreateInfo framebufferInfo{};
        framebufferInfo.setRenderPass(m_RenderPass)
                       .setAttachmentCount(2)
                       .setPAttachments(attachments.data())
                       .setWidth(m_SwapchainExtent.width)
                       .setHeight(m_SwapchainExtent.height)
                       .setLayers(1);

        m_SwapchainFramebuffers[i] = m_Device.createFramebuffer(framebufferInfo);
        if ( !m_SwapchainFramebuffers[i] )
            throw std::runtime_error("Failed to create framebuffer");
    }
}

void VulkanResourceManager::createCommandPool()
{
    const auto [
        graphicsFamily,
        presentFamily,
        computeFamily
    ] = findQueueFamilies(m_PhysicalDevice);vk::CommandPoolCreateInfo poolInfo{};
    poolInfo.setQueueFamilyIndex(graphicsFamily)
            .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);

    m_CommandPools[ResourceTypes::GRAPHICS] = m_Device.createCommandPool(poolInfo);
    if ( !m_CommandPools[ResourceTypes::GRAPHICS] )
        throw std::runtime_error("Failed to create compute command pool");

    // Compute command pool
    vk::CommandPoolCreateInfo computePoolInfo{};
    computePoolInfo.setQueueFamilyIndex(computeFamily)
                   .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
    m_CommandPools[ResourceTypes::COMPUTE] = m_Device.createCommandPool(computePoolInfo);
    if ( !m_CommandPools[ResourceTypes::COMPUTE] )
        throw std::runtime_error("Failed to create compute command pool");
}

void VulkanResourceManager::createCommandBuffers()
{
    m_CommandBuffers[ResourceTypes::GRAPHICS].resize(m_SwapchainFramebuffers.size());
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool(m_CommandPools[ResourceTypes::GRAPHICS])
             .setLevel(vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(static_cast<uint32_t>(m_CommandBuffers.size()));

    m_CommandBuffers[ResourceTypes::GRAPHICS] = m_Device.allocateCommandBuffers(allocInfo);
    if ( m_CommandBuffers[ResourceTypes::GRAPHICS].empty() )
        throw std::runtime_error("Failed to allocate command buffers");
}

void VulkanResourceManager::createSyncObjects()
{
    vk::FenceCreateInfo fenceInfo{};
    fenceInfo.setFlags(vk::FenceCreateFlagBits::eSignaled);
    constexpr vk::SemaphoreCreateInfo semaphoreInfo{};
    m_Semaphores[ResourceTypes::IMAGE] = m_Device.createSemaphore(semaphoreInfo);
    m_Semaphores[ResourceTypes::RENDER] = m_Device.createSemaphore(semaphoreInfo);
    m_Fences[ResourceTypes::INFLIGHT] = m_Device.createFence(fenceInfo);

    if (
        !m_Semaphores[ResourceTypes::IMAGE] ||
        !m_Semaphores[ResourceTypes::RENDER] ||
        !m_Fences[ResourceTypes::INFLIGHT]
    )
        throw std::runtime_error("Failed to create synchronization objects");
}

GLFWwindow* VulkanResourceManager::window() const
{
    return m_Window;
}

void VulkanResourceManager::setWindow(GLFWwindow* window)
{
    m_Window = window;
}

const vk::Instance& VulkanResourceManager::instance() const
{
    return m_Instance;
}

void VulkanResourceManager::setInstance(const vk::Instance& instance)
{
    m_Instance = instance;
}

const vk::PhysicalDevice& VulkanResourceManager::physicalDevice() const
{
    return m_PhysicalDevice;
}

void VulkanResourceManager::setPhysicalDevice(const vk::PhysicalDevice& physical_device)
{
    m_PhysicalDevice = physical_device;
}

const vk::Device& VulkanResourceManager::device() const
{
    return m_Device;
}

void VulkanResourceManager::setDevice(const vk::Device& device)
{
    m_Device = device;
}

const vk::SurfaceKHR& VulkanResourceManager::surface() const
{
    return m_Surface;
}

void VulkanResourceManager::setSurface(const vk::SurfaceKHR& surface)
{
    m_Surface = surface;
}

const vk::SwapchainKHR& VulkanResourceManager::swapchain() const
{
    return m_Swapchain;
}

void VulkanResourceManager::setSwapchain(const vk::SwapchainKHR& swapchain)
{
    m_Swapchain = swapchain;
}

const vk::Extent2D& VulkanResourceManager::swapchainExtent() const
{
    return m_SwapchainExtent;
}

void VulkanResourceManager::setSwapchainExtent(const vk::Extent2D& swapchain_extent)
{
    m_SwapchainExtent = swapchain_extent;
}

const std::vector<vk::Image>& VulkanResourceManager::swapchainImages() const
{
    return m_SwapchainImages;
}

void VulkanResourceManager::setSwapchainImages(const std::vector<vk::Image>& swapchain_images)
{
    m_SwapchainImages = swapchain_images;
}

const std::vector<vk::ImageView>& VulkanResourceManager::swapchainImageViews() const
{
    return m_SwapchainImageViews;
}

void VulkanResourceManager::setSwapchainImageViews(const std::vector<vk::ImageView>& swapchain_image_views)
{
    m_SwapchainImageViews = swapchain_image_views;
}

const vk::RenderPass& VulkanResourceManager::renderPass() const
{
    return m_RenderPass;
}

void VulkanResourceManager::setRenderPass(const vk::RenderPass& render_pass)
{
    m_RenderPass = render_pass;
}

const std::vector<vk::Framebuffer>& VulkanResourceManager::swapchainFramebuffers() const
{
    return m_SwapchainFramebuffers;
}

void VulkanResourceManager::setSwapchainFramebuffers(const std::vector<vk::Framebuffer>& swapchain_framebuffers)
{
    m_SwapchainFramebuffers = swapchain_framebuffers;
}

const QueueFamilyIndices& VulkanResourceManager::queueFamilyIndices() const
{
    return m_QueueFamilyIndices;
}

void VulkanResourceManager::setQueueFamilyIndices(const QueueFamilyIndices& queue_family_indices)
{
    m_QueueFamilyIndices = queue_family_indices;
}

const std::unordered_map<ResourceTypes, vk::Queue>& VulkanResourceManager::queues() const
{
    return m_Queues;
}

void VulkanResourceManager::setQueues(const std::unordered_map<ResourceTypes, vk::Queue>& queues)
{
    m_Queues = queues;
}

const std::unordered_map<ResourceTypes, vk::CommandPool>& VulkanResourceManager::commandPools() const
{
    return m_CommandPools;
}

void VulkanResourceManager::setCommandPools(const std::unordered_map<ResourceTypes, vk::CommandPool>& command_pools)
{
    m_CommandPools = command_pools;
}

const std::unordered_map<ResourceTypes, std::vector<vk::CommandBuffer>>& VulkanResourceManager::commandBuffers() const
{
    return m_CommandBuffers;
}

void VulkanResourceManager::setCommandBuffers(
    const std::unordered_map<ResourceTypes, std::vector<vk::CommandBuffer>>& command_buffers)
{
    m_CommandBuffers = command_buffers;
}

const std::unordered_map<ResourceTypes, vk::Buffer>& VulkanResourceManager::buffers() const
{
    return m_Buffers;
}

void VulkanResourceManager::setBuffers(const std::unordered_map<ResourceTypes, vk::Buffer>& buffers)
{
    m_Buffers = buffers;
}

const std::unordered_map<ResourceTypes, vk::DeviceMemory>& VulkanResourceManager::deviceMemory() const
{
    return m_DeviceMemory;
}

void VulkanResourceManager::setDeviceMemory(const std::unordered_map<ResourceTypes, vk::DeviceMemory>& device_memory)
{
    m_DeviceMemory = device_memory;
}

const std::unordered_map<ResourceTypes, vk::Semaphore>& VulkanResourceManager::semaphores() const
{
    return m_Semaphores;
}

void VulkanResourceManager::setSemaphores(const std::unordered_map<ResourceTypes, vk::Semaphore>& semaphores)
{
    m_Semaphores = semaphores;
}

const std::unordered_map<ResourceTypes, vk::Fence>& VulkanResourceManager::fences() const
{
    return m_Fences;
}

void VulkanResourceManager::setFences(const std::unordered_map<ResourceTypes, vk::Fence>& fences)
{
    m_Fences = fences;
}

const std::unordered_map<ResourceTypes, vk::Format>& VulkanResourceManager::formats() const
{
    return m_Formats;
}

void VulkanResourceManager::setFormats(const std::unordered_map<ResourceTypes, vk::Format>& formats)
{
    m_Formats = formats;
}

const std::unordered_map<ResourceTypes, std::vector<vk::Image>>& VulkanResourceManager::images() const
{
    return m_Images;
}

void VulkanResourceManager::setImages(const std::unordered_map<ResourceTypes, std::vector<vk::Image>>& images)
{
    m_Images = images;
}

const std::unordered_map<ResourceTypes, std::vector<vk::DeviceMemory>>& VulkanResourceManager::imageMemory() const
{
    return m_ImageMemory;
}

void VulkanResourceManager::setImageMemory(
    const std::unordered_map<ResourceTypes, std::vector<vk::DeviceMemory>>& image_memory)
{
    m_ImageMemory = image_memory;
}

const std::unordered_map<ResourceTypes, std::vector<vk::ImageView>>& VulkanResourceManager::imageViews() const
{
    return m_ImageViews;
}

void VulkanResourceManager::setImageViews(
    const std::unordered_map<ResourceTypes, std::vector<vk::ImageView>>& image_views)
{
    m_ImageViews = image_views;
}

const std::unordered_map<ResourceTypes, vk::Pipeline>& VulkanResourceManager::pipelines() const
{
    return m_Pipelines;
}

void VulkanResourceManager::setPipelines(const std::unordered_map<ResourceTypes, vk::Pipeline>& pipelines)
{
    m_Pipelines = pipelines;
}

const std::unordered_map<ResourceTypes, vk::PipelineLayout>& VulkanResourceManager::pipelineLayouts() const
{
    return m_PipelineLayouts;
}

void VulkanResourceManager::setPipelineLayouts(
    const std::unordered_map<ResourceTypes, vk::PipelineLayout>& pipeline_layouts)
{
    m_PipelineLayouts = pipeline_layouts;
}

const std::unordered_map<ResourceTypes, vk::DescriptorSet>& VulkanResourceManager::descriptorSet() const
{
    return m_DescriptorSet;
}

void VulkanResourceManager::setDescriptorSet(const std::unordered_map<ResourceTypes, vk::DescriptorSet>& descriptor_set)
{
    m_DescriptorSet = descriptor_set;
}

const std::unordered_map<ResourceTypes, vk::DescriptorSetLayout>& VulkanResourceManager::descriptorSetLayouts() const
{
    return m_DescriptorSetLayouts;
}

void VulkanResourceManager::setDescriptorSetLayouts(
    const std::unordered_map<ResourceTypes, vk::DescriptorSetLayout>& descriptor_set_layouts)
{
    m_DescriptorSetLayouts = descriptor_set_layouts;
}

const std::unordered_map<ResourceTypes, vk::DescriptorPool>& VulkanResourceManager::descriptorPools() const
{
    return m_DescriptorPools;
}

void VulkanResourceManager::setDescriptorPools(
    const std::unordered_map<ResourceTypes, vk::DescriptorPool>& descriptor_pools)
{
    m_DescriptorPools = descriptor_pools;
}
