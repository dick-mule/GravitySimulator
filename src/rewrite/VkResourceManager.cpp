//
// Created by Richard Mule on 7/5/25.
//

#include "VkResourceManager.hpp"


std::ostream& operator<<(std::ostream& os, const ResourceTypes& type)
{
    os << magic_enum::enum_name(type);
    return os;
}

QueueFamilyIndices QueueFamilyIndices::fromDevice(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    QueueFamilyIndices indices;
    const auto queueFamilies = device.getQueueFamilyProperties();
    uint32_t i = 0;
    for ( const auto& queueFamily : queueFamilies )
    {
        if ( queueFamily.queueFlags & vk::QueueFlagBits::eGraphics )
            indices.graphicsFamily = i;
        if ( device.getSurfaceSupportKHR(i, surface) )
            indices.presentFamily = i;
        if ( queueFamily.queueFlags & vk::QueueFlagBits::eCompute )
            indices.computeFamily = i;
        if ( indices.isComplete() )
            break;
        ++i;
    }

    return indices;
}

GlfwWindowHandler::GlfwWindowHandler(
    std::string title,
    VkResourceManager* resourceManager,
    const size_t width,
    const size_t height)
    : m_Window(nullptr)
    , m_ResourceManager(resourceManager)
    , m_WindowTitle(std::move(title))
    , m_Width(width)
    , m_Height(height)
    , m_FrameBufferCallback(nullptr)
{
    initialize();
}


GlfwWindowHandler::~GlfwWindowHandler()
{
    destroy();
}

void GlfwWindowHandler::initialize()
{
    if ( !glfwInit() )
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API); // No OpenGL context
    m_Window = glfwCreateWindow(m_Width, m_Height, "Gravity Simulator (Vulkan)", nullptr, nullptr);
    if ( !m_Window )
        throw std::runtime_error("Failed to create GLFW window");
    glfwSetWindowUserPointer(m_Window, m_ResourceManager); // Pass VulkanApp instance
    glfwSetFramebufferSizeCallback(m_Window, framebufferResizeCallback); // Set callback
    glfwSetScrollCallback(m_Window, scrollCallback); // Add this
}

void GlfwWindowHandler::destroy()
{
    glfwDestroyWindow(m_Window);
}

void GlfwWindowHandler::framebufferResizeCallback(GLFWwindow* window, int width, int height)
{
    auto app = reinterpret_cast<VkResourceManager*>(glfwGetWindowUserPointer(window));
    app->setFrameBufferResized(true);
}

void GlfwWindowHandler::scrollCallback(GLFWwindow* window, double xOffset, double yOffset)
{
    // Pass the scroll delta to GridRenderer via ImGui IO or directly
    ImGuiIO& io = ImGui::GetIO();
    io.MouseWheel += static_cast<float>(yOffset); // Vertical scroll (zoom)
    io.MouseWheelH += static_cast<float>(xOffset); // Horizontal scroll (optional)
}

std::vector<const char*> GlfwWindowHandler::getGlfwExtensions()
{
    uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
    std::vector<const char*> add_extensions = {
        VK_KHR_SURFACE_EXTENSION_NAME,
        VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME,
        VK_MVK_MACOS_SURFACE_EXTENSION_NAME2
    };
    for ( const auto& extension : add_extensions )
        extensions.push_back(extension);

    return extensions;
}

vk::Extent2D GlfwWindowHandler::currentExtent() const
{
    int width, height;
    glfwGetFramebufferSize(m_Window, &width, &height);
    while ( width == 0 || height == 0 )  // Handle minimization
    {
        glfwGetFramebufferSize(m_Window, &width, &height);
        glfwWaitEvents();
    }

    return vk::Extent2D{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
}

VkResourceManager::VkResourceManager(std::string title)
    : m_WindowHandler(nullptr)
    , m_Instance(nullptr)
    , m_Surface(nullptr)
    , m_PhysicalDevice(nullptr)
    , m_Device(nullptr)
    , m_QueueFamilyIndices({})
    , m_Queues({})
    , m_Swapchain(nullptr)
    , m_SwapchainImages({})
    , m_SwapchainImageViews({})
    , m_RenderPass(nullptr)
    , m_SwapchainFramebuffers({})
    , m_CommandPools({})
    , m_CommandBuffers({})
    , m_ImageAvailableSemaphore(nullptr)
    , m_RenderFinishedSemaphore(nullptr)
    , m_InFlightFences({})
    , m_WrappedDescriptors({})
    , m_WrappedBuffers({})
    , m_FrameBufferResized(false)
    , m_SwapchainCleanupCallback(nullptr)
    , m_DrawCallback(nullptr)
{
    if ( !glfwInit() )
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }
    m_WindowHandler = std::make_unique<GlfwWindowHandler>(std::move(title), this);
    auto callback = [this](GLFWwindow* window, int width, int height)
    {
        const auto app = static_cast<VkResourceManager*>(glfwGetWindowUserPointer(window));
        app->m_FrameBufferResized = true;
    };
    m_WindowHandler->setFrameBufferCallback(callback);
}

VkResourceManager::~VkResourceManager()
{
    for ( const auto fence : m_InFlightFences | std::views::values )
        m_Device.destroyFence(fence);
    if ( m_RenderFinishedSemaphore )
        m_Device.destroySemaphore(m_RenderFinishedSemaphore);
    if ( m_ImageAvailableSemaphore )
        m_Device.destroySemaphore(m_ImageAvailableSemaphore);

    for ( const auto& commandPool : m_CommandPools | std::views::values )
        m_Device.destroyCommandPool(commandPool);

    for ( const auto framebuffer : m_SwapchainFramebuffers )
        m_Device.destroyFramebuffer(framebuffer);

    if ( m_RenderPass )
        m_Device.destroyRenderPass(m_RenderPass);

    for ( const auto imageView : m_SwapchainImageViews )
        m_Device.destroyImageView(imageView);

    if ( m_Swapchain )
        m_Device.destroySwapchainKHR(m_Swapchain);

    m_Device.destroy();
    if ( m_Surface )
        m_Instance.destroySurfaceKHR(m_Surface);

    m_Instance.destroy();

    m_WindowHandler.reset();
}

vk::ApplicationInfo VkResourceManager::makeApplicationInfo(const std::string& appName)
{
    vk::ApplicationInfo appInfo{};
    appInfo.setPApplicationName(appName.c_str())
           .setApplicationVersion(VK_MAKE_VERSION(1, 0, 0))
           .setPEngineName("No Engine")
           .setEngineVersion(VK_MAKE_VERSION(1, 0, 0))
           .setApiVersion(VK_API_VERSION_1_2);
    return appInfo;
}

vk::InstanceCreateInfo VkResourceManager::makeInstanceCreateInfo(
    const vk::ApplicationInfo& appInfo,
    const std::vector<const char*>& enabledExtensions)
{
    vk::InstanceCreateInfo createInfo{};
    createInfo.setPApplicationInfo(&appInfo)
              .setEnabledExtensionCount(static_cast<uint32_t>(enabledExtensions.size()))
              .setPEnabledExtensionNames(enabledExtensions)
              .setEnabledLayerCount(0);
    createInfo.setFlags(vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR);
    return createInfo;
}

vk::SurfaceFormatKHR VkResourceManager::getSurfaceFormat() const
{
    const auto formats = m_PhysicalDevice.getSurfaceFormatsKHR(m_Surface);
    if ( formats.empty() )
        throw std::runtime_error("VkResourceManager::getSurfaceFormat => No surface formats available");
    vk::SurfaceFormatKHR surfaceFormat = formats.front();
    for ( const auto& format : formats )
    {
        if ( format.format == vk::Format::eB8G8R8A8Srgb && format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear )
        {
            surfaceFormat = format;
            break;
        }
    }

    return surfaceFormat;
}

vk::PresentModeKHR VkResourceManager::getPresentModes() const
{
    if ( !m_Surface )
        throw std::runtime_error("VkResourceManager::getPresentModes => Surface is not available");
    if ( !!m_Device )
        throw std::runtime_error("VkResourceManager::getPresentModes => Device is not available");
    const std::vector<vk::PresentModeKHR> presentModes = m_PhysicalDevice.getSurfacePresentModesKHR(m_Surface);
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
    return presentMode;
}

void VkResourceManager::initializeApplication()
{
    const vk::InstanceCreateInfo createInfo = makeInstanceCreateInfo(
        makeApplicationInfo(m_WindowHandler->title()),
        m_WindowHandler->getGlfwExtensions());
    initializeAppInstance(createInfo);
    initializeSurface();
    initializeDevices();
    initializeQueues();
    createSwapChain();
    createRenderPass();
    createFramebuffers();
    createCommandPools();
    initializeSyncObjects();
}


void VkResourceManager::initializeAppInstance(const vk::InstanceCreateInfo& createInfo)
{
    m_Instance = vk::createInstance(createInfo);
    if ( !m_Instance )
        throw std::runtime_error("VkResourceManager::initializeAppInstance => Failed to create Vulkan instance");
}

void VkResourceManager::initializeSurface()
{
    if ( m_Instance == nullptr )
        throw std::runtime_error("VkResourceManager::initializeSurface => Application not initialized");
    VkSurfaceKHR surface;
    if ( glfwCreateWindowSurface(m_Instance, m_WindowHandler->window(), nullptr, &surface) != VK_SUCCESS )
        throw std::runtime_error("VkResourceManager::initializeSurface => Failed to create window surface");
    m_Surface = surface;
}

void VkResourceManager::initializeDevices()
{
    const std::vector<vk::PhysicalDevice> devices = m_Instance.enumeratePhysicalDevices();
    if ( devices.empty() )
        throw std::runtime_error("VkResourceManager::initializeDevices => Failed to find GPUs with Vulkan support");

    for ( const auto& device : devices )
    {
        m_QueueFamilyIndices = QueueFamilyIndices::fromDevice(device, m_Surface);
        if ( m_QueueFamilyIndices.isComplete() )
        {
            m_PhysicalDevice = device;
            break;
        }
    }

    if ( !m_PhysicalDevice )
        throw std::runtime_error("VkResourceManager::initializeDevices => Failed to find a suitable GPU");

    constexpr float queuePriority = 1.0f;
    std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
    for ( const uint32_t queueFamily : m_QueueFamilyIndices.toVect() )
    {
        vk::DeviceQueueCreateInfo queueCreateInfo{};
        queueCreateInfo.setQueueFamilyIndex(queueFamily)
                       .setQueueCount(1)
                       .setPQueuePriorities(&queuePriority);
        queueCreateInfos.push_back(queueCreateInfo);
    }

    vk::DeviceCreateInfo createInfo{};
    vk::PhysicalDeviceFeatures deviceFeatures{};
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
        throw std::runtime_error("VkResourceManager::initializeDevices => Failed to create logical device");
}

void VkResourceManager::initializeQueues()
{
    if ( !m_Device )
        throw std::runtime_error("Failed to initialize queues - no logical device");
    m_Queues[ResourceTypes::GRAPHICS] = m_Device.getQueue(m_QueueFamilyIndices.graphicsFamily, 0);
    m_Queues[ResourceTypes::PRESENT] = m_Device.getQueue(m_QueueFamilyIndices.presentFamily, 0);
    m_Queues[ResourceTypes::COMPUTE] = m_Device.getQueue(m_QueueFamilyIndices.computeFamily, 0);
}

void VkResourceManager::createSwapChain()
{
    vk::SwapchainCreateInfoKHR createInfo = initializeCreateSwapchainInfo();
    // Store the old swapchain
    const vk::SwapchainKHR oldSwapchain = m_Swapchain;
    createInfo.setOldSwapchain(oldSwapchain);
    m_Swapchain = m_Device.createSwapchainKHR(createInfo);
    if ( !m_Swapchain )
        throw std::runtime_error("VkResourceManager::createSwapChain => Failed to create swapchain");
    // Destroy the old swapchain after the new one is created
    if ( oldSwapchain )
        m_Device.destroySwapchainKHR(oldSwapchain);
    m_SwapchainImages = m_Device.getSwapchainImagesKHR(m_Swapchain);
    initializeImageViews();
}

void VkResourceManager::initializeImageViews()
{
    m_SwapchainImageViews.resize(m_SwapchainImages.size());
    for ( size_t i = 0; i < m_SwapchainImages.size(); ++i )
    {
        vk::ImageViewCreateInfo createInfo{};
        createInfo.setImage(m_SwapchainImages[i])
                  .setViewType(vk::ImageViewType::e2D)
                  .setFormat(vk::Format::eB8G8R8A8Srgb)
                  .setComponents(vk::ComponentMapping())
                  .setSubresourceRange(
                      vk::ImageSubresourceRange(
                        vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1
                      )
                  );

        m_SwapchainImageViews[i] = m_Device.createImageView(createInfo);
        if ( !m_SwapchainImageViews[i] )
            throw std::runtime_error(std::format(
                "VkResourceManager::initializeImageViews => Failed to create image views[{}]", i
            ));
    }
}

void VkResourceManager::cleanupSwapChain()
{
    for ( const auto framebuffer : m_SwapchainFramebuffers )
        m_Device.destroyFramebuffer(framebuffer);

    for ( const auto imageView : m_SwapchainImageViews )
        m_Device.destroyImageView(imageView);

    m_Device.destroySwapchainKHR(m_Swapchain);
    if ( m_SwapchainCleanupCallback )
        m_SwapchainCleanupCallback();
}

vk::SwapchainCreateInfoKHR VkResourceManager::initializeCreateSwapchainInfo()
{
    auto extent = m_WindowHandler->currentExtent();
    const auto capabilities = m_PhysicalDevice.getSurfaceCapabilitiesKHR(m_Surface);
    extent.width = std::max(
        capabilities.minImageExtent.width,
        std::min(capabilities.maxImageExtent.width, extent.width));
    extent.height = std::max(
        capabilities.minImageExtent.height,
        std::min(capabilities.maxImageExtent.height, extent.height));
    m_SwapchainExtent = extent;

    uint32_t imageCount = capabilities.minImageCount + 1;
    if ( capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount )
        imageCount = capabilities.maxImageCount;

    vk::SurfaceFormatKHR surfaceFormat = getSurfaceFormat();
    vk::SwapchainCreateInfoKHR createInfo{};
    createInfo.setSurface(m_Surface)
              .setMinImageCount(imageCount)
              .setImageFormat(surfaceFormat.format)
              .setImageColorSpace(surfaceFormat.colorSpace)
              .setImageExtent(extent)
              .setImageArrayLayers(1)
              .setImageUsage(vk::ImageUsageFlagBits::eColorAttachment);

    const auto [
        graphicsFamily,
        presentFamily,
        computeFamily
    ] = m_QueueFamilyIndices;
    const uint32_t queueFamilyIndices[] = { graphicsFamily, presentFamily, computeFamily };
    if ( graphicsFamily != presentFamily )
        createInfo.setImageSharingMode(vk::SharingMode::eConcurrent)
                  .setQueueFamilyIndexCount(2)
                  .setPQueueFamilyIndices(queueFamilyIndices);
    else
        createInfo.setImageSharingMode(vk::SharingMode::eExclusive);
    vk::PresentModeKHR presentMode = getPresentModes();
    createInfo.setPreTransform(capabilities.currentTransform)
              .setCompositeAlpha(vk::CompositeAlphaFlagBitsKHR::eOpaque)
              .setPresentMode(presentMode)
              .setClipped(VK_TRUE)
              .setOldSwapchain(m_Swapchain);
    return createInfo;
}

void VkResourceManager::createRenderPass(const vk::Format depthFormat)
{
    static vk::AttachmentDescription colorAttachment{};
    colorAttachment.setFormat(vk::Format::eB8G8R8A8Srgb)
                   .setSamples(vk::SampleCountFlagBits::e1)
                   .setLoadOp(vk::AttachmentLoadOp::eClear)
                   .setStoreOp(vk::AttachmentStoreOp::eStore)
                   .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
                   .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
                   .setInitialLayout(vk::ImageLayout::eUndefined)
                   .setFinalLayout(vk::ImageLayout::ePresentSrcKHR);

    static vk::AttachmentReference colorAttachmentRef{};
    colorAttachmentRef.setAttachment(0)
                      .setLayout(vk::ImageLayout::eColorAttachmentOptimal);

    vk::AttachmentDescription depthAttachment{};
    depthAttachment.format = depthFormat;
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
        throw std::runtime_error("VkResourceManager::createRenderPass => Failed to create render pass");
}

void VkResourceManager::createFramebuffers(const std::vector<vk::ImageView>& imageViews)
{
    if ( !m_RenderPass )
        throw std::runtime_error("VkResourceManager::createFramebuffers => No Render Pass initialized");
    m_SwapchainFramebuffers.resize(m_SwapchainImageViews.size());
    for ( size_t i = 0; i < m_SwapchainImageViews.size(); ++i )
    {
        std::array<vk::ImageView, 2> attachments = {
            m_SwapchainImageViews[i],
            i >= imageViews.size() ? m_SwapchainImageViews[i] : imageViews[i]
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

void VkResourceManager::createCommandPools()
{
    for ( const auto& [k, v] : m_QueueFamilyIndices.mappedIndices() )
    {
        vk::CommandPoolCreateInfo poolInfo{};
        poolInfo.setQueueFamilyIndex(v)
                .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);

        m_CommandPools[k] = m_Device.createCommandPool(poolInfo);
        if ( !m_CommandPools[k] )
            throw std::runtime_error(std::format(
                "VkResourceManager::createCommandPools => Failed to create command pool[{}]",
                magic_enum::enum_name(k)
            ));
    }

    m_CommandBuffers.resize(m_SwapchainFramebuffers.size());
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool(m_CommandPools[ResourceTypes::GRAPHICS])
             .setLevel(vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(static_cast<uint32_t>(m_CommandBuffers.size()));
    m_CommandBuffers = m_Device.allocateCommandBuffers(allocInfo);
    if ( m_CommandBuffers.empty() )
        throw std::runtime_error("Failed to allocate command buffers");
}

void VkResourceManager::createNewFence(const ResourceTypes resourceType)
{
    vk::FenceCreateInfo fenceInfo{};
    fenceInfo.setFlags(vk::FenceCreateFlagBits::eSignaled);
    m_InFlightFences[resourceType] = m_Device.createFence(fenceInfo);
}

void VkResourceManager::initializeSyncObjects()
{
    vk::SemaphoreCreateInfo semaphoreInfo{};

    m_ImageAvailableSemaphore = m_Device.createSemaphore(semaphoreInfo);
    m_RenderFinishedSemaphore = m_Device.createSemaphore(semaphoreInfo);
    createNewFence(ResourceTypes::GRAPHICS);

    if ( !m_ImageAvailableSemaphore || !m_RenderFinishedSemaphore || !m_InFlightFences[ResourceTypes::GRAPHICS] )
        throw std::runtime_error("Failed to create synchronization objects");
}

void VkResourceManager::createResourcePipeline(vk::ShaderModule computeShaderModule, const ResourceTypes resourceType)
{
    WrappedDescriptorSet wrappedSet;
    std::vector<vk::DescriptorSetLayoutBinding> bindings(2);
    bindings[0].binding = 0;
    bindings[0].descriptorType = vk::DescriptorType::eStorageBuffer;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = vk::ShaderStageFlagBits::eCompute;
    bindings[1].binding = 1;
    bindings[1].descriptorType = vk::DescriptorType::eStorageBuffer;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = vk::ShaderStageFlagBits::eCompute;
    vk::DescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = vk::StructureType::eDescriptorSetLayoutCreateInfo;
    layoutInfo.bindingCount = bindings.size();
    layoutInfo.pBindings = bindings.data();
    /// Create layout first
    wrappedSet.descLayout = m_Device.createDescriptorSetLayout(layoutInfo);
    /// Now pools
    std::vector<vk::DescriptorPoolSize> poolSizes(1);
    poolSizes[0].type = vk::DescriptorType::eStorageBuffer;
    poolSizes[0].descriptorCount = 2;
    vk::DescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = vk::StructureType::eDescriptorPoolCreateInfo;
    poolInfo.poolSizeCount = poolSizes.size();
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = 1;
    wrappedSet.descPool = m_Device.createDescriptorPool(poolInfo);
    // Allocate descriptor set
    vk::DescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = vk::StructureType::eDescriptorSetAllocateInfo;
    allocInfo.descriptorPool = wrappedSet.descPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &wrappedSet.descLayout;
    const auto result = m_Device.allocateDescriptorSets(&allocInfo, &wrappedSet.descSet);
    if ( result != vk::Result::eSuccess )
        throw std::runtime_error("Failed to allocate descriptor set");

    /// Create pipeline layout with push constants
    vk::PushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = vk::ShaderStageFlagBits::eCompute;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(int) + sizeof(float); // uGridSize, uScale
    wrappedSet.descLayoutInfo.sType = vk::StructureType::ePipelineLayoutCreateInfo;
    wrappedSet.descLayoutInfo.setLayoutCount = 1;
    wrappedSet.descLayoutInfo.pSetLayouts = &wrappedSet.descLayout;
    wrappedSet.descLayoutInfo.pushConstantRangeCount = 1;
    wrappedSet.descLayoutInfo.pPushConstantRanges = &pushConstantRange;
    wrappedSet.descPipelineLayout = m_Device.createPipelineLayout(wrappedSet.descLayoutInfo);
    // Create compute pipeline
    wrappedSet.descPipelineInfo.sType = vk::StructureType::eComputePipelineCreateInfo;
    wrappedSet.descPipelineInfo.stage.sType = vk::StructureType::ePipelineShaderStageCreateInfo;
    wrappedSet.descPipelineInfo.stage.stage = vk::ShaderStageFlagBits::eCompute;
    wrappedSet.descPipelineInfo.stage.module = computeShaderModule;
    wrappedSet.descPipelineInfo.stage.pName = "main";
    wrappedSet.descPipelineInfo.layout = wrappedSet.descPipelineLayout;
    wrappedSet.descPipeline = m_Device.createComputePipeline(nullptr, wrappedSet.descPipelineInfo).value;
    m_WrappedDescriptors[resourceType] = wrappedSet;
}

vk::CommandBuffer VkResourceManager::startSingleTimeCommand(const vk::CommandPool& commandPool) const
{
    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool(commandPool)
             .setLevel(vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(1);
    const auto result = m_Device.allocateCommandBuffers(allocInfo);
    if ( result.empty() )
        throw std::runtime_error("Failed to allocate command buffer");

    const vk::CommandBuffer commandBuffer = result.front();
    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.sType = vk::StructureType::eCommandBufferBeginInfo;
    beginInfo.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    commandBuffer.begin(beginInfo);
    return commandBuffer;
}

void VkResourceManager::terminateSingleTimeCommand(
    const vk::CommandPool& commandPool,
    const vk::CommandBuffer& commandBuffer,
    const vk::Queue& queue) const
{
    commandBuffer.end();
    vk::SubmitInfo submitInfo{};
    submitInfo.setCommandBufferCount(1)
              .setPCommandBuffers(&commandBuffer);
    const auto _ = queue.submit(1, &submitInfo, nullptr);
    queue.waitIdle();
    m_Device.freeCommandBuffers(commandPool, commandBuffer);
}

void VkResourceManager::recreateSwapchain()
{
    m_Device.waitIdle();
    cleanupSwapChain();
    createSwapChain();
    createRenderPass();
    createFramebuffers();
    if ( m_SwapchainReinitCallback )
        m_SwapchainReinitCallback();
}


void VkResourceManager::handleWindowResize()
{
    if ( !m_FrameBufferResized )
        return;
    m_FrameBufferResized = false;
    recreateSwapChain();
}

void VkResourceManager::drawFrame()
{
    auto _ = m_Device.waitForFences(m_InFlightFences[ResourceTypes::GRAPHICS], VK_TRUE, UINT64_MAX);
    m_Device.resetFences(m_InFlightFences[ResourceTypes::GRAPHICS]);

    uint32_t imageIndex;
    auto result = m_Device.acquireNextImageKHR(
        m_Swapchain,
        UINT64_MAX,
        m_ImageAvailableSemaphore,
        nullptr,
        &imageIndex);
    if ( result == vk::Result::eErrorOutOfDateKHR )
    {
        recreateSwapchain();
        return; // Skip this frame
    }

    if ( result != vk::Result::eSuccess && result != vk::Result::eSuboptimalKHR )
        throw std::runtime_error("Failed to acquire swapchain image");

    m_CommandBuffers[imageIndex].reset();
    vk::CommandBufferBeginInfo beginInfo{};
    m_CommandBuffers[imageIndex].begin(beginInfo);

    vk::RenderPassBeginInfo renderPassInfo{};
    renderPassInfo.setRenderPass(m_RenderPass)
                  .setFramebuffer(m_SwapchainFramebuffers[imageIndex])
                  .setRenderArea(vk::Rect2D({0, 0}, m_SwapchainExtent));

    std::array<vk::ClearValue, 2> clearValues{};
    clearValues[0].color = vk::ClearColorValue{std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f}};
    clearValues[1].depthStencil = vk::ClearDepthStencilValue{1.0f, 0}; // Clear depth to 1.0 (farthest)
    renderPassInfo.setClearValueCount(clearValues.size())
                  .setPClearValues(clearValues.data());

    m_CommandBuffers[imageIndex].beginRenderPass(renderPassInfo, vk::SubpassContents::eInline);

    /**
     * Calls the following in the callback
     * m_GridRenderer->draw(m_CommandBuffers[imageIndex]);
     * m_ImGuiHandler->renderDrawData(m_CommandBuffers[imageIndex]);
     */
    m_DrawCallback(m_CommandBuffers[imageIndex]);

    m_CommandBuffers[imageIndex].endRenderPass();

    m_CommandBuffers[imageIndex].end();

    vk::SubmitInfo submitInfo{};
    const vk::Semaphore waitSemaphores[] = { m_ImageAvailableSemaphore };
    constexpr vk::PipelineStageFlags waitStages[] = { vk::PipelineStageFlagBits::eColorAttachmentOutput };
    submitInfo.setWaitSemaphoreCount(1)
              .setPWaitSemaphores(waitSemaphores)
              .setPWaitDstStageMask(waitStages)
              .setCommandBufferCount(1)
              .setPCommandBuffers(&m_CommandBuffers[imageIndex]);

    const vk::Semaphore signalSemaphores[] = { m_RenderFinishedSemaphore };
    submitInfo.setSignalSemaphoreCount(1)
              .setPSignalSemaphores(signalSemaphores);

    m_Queues.at(ResourceTypes::GRAPHICS).submit(submitInfo, m_InFlightFences[ResourceTypes::GRAPHICS]);

    vk::PresentInfoKHR presentInfo{};
    presentInfo.setWaitSemaphoreCount(1)
               .setPWaitSemaphores(signalSemaphores)
               .setSwapchainCount(1)
               .setPSwapchains(&m_Swapchain)
               .setPImageIndices(&imageIndex);

    result = m_Queues.at(ResourceTypes::PRESENT).presentKHR(presentInfo);
    if ( result == vk::Result::eErrorOutOfDateKHR || result == vk::Result::eSuboptimalKHR )
        return recreateSwapchain();

    if ( result != vk::Result::eSuccess )
        throw std::runtime_error("Failed to present: " + vk::to_string(result));
}

void VkResourceManager::setBuffer(
    const vk::Buffer& buffer,
    const vk::DeviceMemory& memory,
    const size_t buffer_size,
    const ResourceTypes resource_type,
    const uint32_t memOffset)
{
    m_Device.bindBufferMemory(buffer, memory, memOffset);
    m_WrappedBuffers[resource_type] = { buffer, memory, memOffset, buffer_size };
}

void VkResourceManager::bindBufferDescriptorSets() const
{
    std::vector<vk::WriteDescriptorSet> descriptorWrites(2);

    vk::DescriptorBufferInfo vertexBufferInfo{};
    vertexBufferInfo.buffer = m_WrappedBuffers.at(ResourceTypes::VERTEX).buffer;
    vertexBufferInfo.offset = 0;
    vertexBufferInfo.range = m_WrappedBuffers.at(ResourceTypes::VERTEX).size;

    descriptorWrites[0].sType = vk::StructureType::eWriteDescriptorSet;
    descriptorWrites[0].dstSet = m_WrappedDescriptors.at(ResourceTypes::COMPUTE).descSet;
    descriptorWrites[0].dstBinding = 0;
    descriptorWrites[0].dstArrayElement = 0;
    descriptorWrites[0].descriptorType = vk::DescriptorType::eStorageBuffer;
    descriptorWrites[0].descriptorCount = 1;
    descriptorWrites[0].pBufferInfo = &vertexBufferInfo;

    vk::DescriptorBufferInfo indexBufferInfo{};
    indexBufferInfo.buffer = m_WrappedBuffers.at(ResourceTypes::INDEX).buffer;
    indexBufferInfo.offset = 0;
    indexBufferInfo.range = m_WrappedBuffers.at(ResourceTypes::INDEX).size;

    descriptorWrites[1].sType = vk::StructureType::eWriteDescriptorSet;
    descriptorWrites[1].dstSet = m_WrappedDescriptors.at(ResourceTypes::COMPUTE).descSet;
    descriptorWrites[1].dstBinding = 1;
    descriptorWrites[1].dstArrayElement = 0;
    descriptorWrites[1].descriptorType = vk::DescriptorType::eStorageBuffer;
    descriptorWrites[1].descriptorCount = 1;
    descriptorWrites[1].pBufferInfo = &indexBufferInfo;

    m_Device.updateDescriptorSets(descriptorWrites, {});
}
