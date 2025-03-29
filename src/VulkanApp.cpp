//
// Created by Richard Mule on 3/21/25.
//

#include "VulkanApp.hpp"
#include "ImGuiHandler.hpp"
#include "GridRenderer.hpp"
#include <iostream>
#include <stdexcept>

VulkanApp::VulkanApp()
    : m_Window(nullptr)
    , m_ImGuiHandler(nullptr)
    , m_GridRenderer(nullptr)
{
}

VulkanApp::~VulkanApp()
{
    cleanup();
}

void VulkanApp::run()
{
    initWindow();
    initVulkan();
    initGridRenderer();
    initImGui();
    mainLoop();
}

void VulkanApp::framebufferResizeCallback(GLFWwindow* window, int width, int height)
{
    auto app = reinterpret_cast<VulkanApp*>(glfwGetWindowUserPointer(window));
    app->m_FramebufferResized = true;
}

void VulkanApp::recreateSwapchain()
{
    m_Device.waitIdle();

    // Clean up old swapchain resources
    cleanupSwapchain();
    // Recreate swapchain
    createSwapchain();
    createImageViews();
    createRenderPass(); // Optional: only if format might change
    initGridRenderer(); // Recreate depth resources and pipelines
    createFramebuffers();

    // Update ImGui (optional, if it needs swapchain info)
    m_ImGuiHandler.reset();
    initImGui();
}

void VulkanApp::cleanupSwapchain()
{
    for ( auto framebuffer : m_SwapchainFramebuffers )
        m_Device.destroyFramebuffer(framebuffer);

    m_GridRenderer.reset(); // Reset GridRenderer to recreate depth resources
    for ( auto imageView : m_SwapchainImageViews )
        m_Device.destroyImageView(imageView);

    // m_Device.destroySwapchainKHR(m_Swapchain);
}

void VulkanApp::scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto app = reinterpret_cast<VulkanApp*>(glfwGetWindowUserPointer(window));
    // Pass the scroll delta to GridRenderer via ImGui IO or directly
    ImGuiIO& io = ImGui::GetIO();
    io.MouseWheel += static_cast<float>(yoffset); // Vertical scroll (zoom)
    io.MouseWheelH += static_cast<float>(xoffset); // Horizontal scroll (optional)
}


void VulkanApp::initWindow()
{
    if ( !glfwInit() )
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API); // No OpenGL context
    m_Window = glfwCreateWindow(m_WIDTH, m_HEIGHT, "Gravity Simulator (Vulkan)", nullptr, nullptr);
    if ( !m_Window )
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwSetWindowUserPointer(m_Window, this); // Pass VulkanApp instance
    glfwSetFramebufferSizeCallback(m_Window, framebufferResizeCallback); // Set callback
    glfwSetScrollCallback(m_Window, scrollCallback); // Add this
}

void VulkanApp::initVulkan()
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

void VulkanApp::createInstance()
{
    vk::ApplicationInfo appInfo{};
    appInfo.setPApplicationName("Gravity Simulator")
           .setApplicationVersion(VK_MAKE_VERSION(1, 0, 0))
           .setPEngineName("No Engine")
           .setEngineVersion(VK_MAKE_VERSION(1, 0, 0))
           .setApiVersion(VK_API_VERSION_1_2);

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

void VulkanApp::createSurface()
{
    VkSurfaceKHR surface;
    if ( glfwCreateWindowSurface(m_Instance, m_Window, nullptr, &surface) != VK_SUCCESS )
        throw std::runtime_error("Failed to create window surface");
    m_Surface = surface;
}

void VulkanApp::pickPhysicalDevice()
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
        throw std::runtime_error("Failed to find a suitable GPU");
}

bool VulkanApp::isDeviceSuitable(const vk::PhysicalDevice& device) const
{
    const QueueFamilyIndices indices = findQueueFamilies(device);
    return indices.isComplete();
}

VulkanApp::QueueFamilyIndices VulkanApp::findQueueFamilies(const vk::PhysicalDevice& device) const
{
    QueueFamilyIndices indices;
    const auto queueFamilies = device.getQueueFamilyProperties();
    uint32_t i = 0;
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

void VulkanApp::createLogicalDevice()
{
    const auto [
        graphicsFamily,
        presentFamily,
        computeFamily
    ] = findQueueFamilies(m_PhysicalDevice);
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

    m_GraphicsQueue = m_Device.getQueue(graphicsFamily, 0);
    m_PresentQueue = m_Device.getQueue(presentFamily, 0);
    m_ComputeQueue = m_Device.getQueue(computeFamily, 0);  // New member
}

void VulkanApp::createSwapchain()
{
    const auto capabilities = m_PhysicalDevice.getSurfaceCapabilitiesKHR(m_Surface);

    const auto formats = m_PhysicalDevice.getSurfaceFormatsKHR(m_Surface);
    if ( formats.empty() )
        throw std::runtime_error("No surface formats available");

    vk::SurfaceFormatKHR surfaceFormat = formats[0];
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

    m_SwapchainExtent = vk::Extent2D{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
    m_SwapchainExtent.width = std::max(capabilities.minImageExtent.width,
                                       std::min(capabilities.maxImageExtent.width, m_SwapchainExtent.width));
    m_SwapchainExtent.height = std::max(capabilities.minImageExtent.height,
                                        std::min(capabilities.maxImageExtent.height, m_SwapchainExtent.height));

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
    {
        createInfo.setImageSharingMode(vk::SharingMode::eConcurrent)
                  .setQueueFamilyIndexCount(2)
                  .setPQueueFamilyIndices(queueFamilyIndices);
    }
    else
    {
        createInfo.setImageSharingMode(vk::SharingMode::eExclusive);
    }

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

    m_SwapchainImages = m_Device.getSwapchainImagesKHR(m_Swapchain);
}

void VulkanApp::createImageViews()
{
    m_SwapchainImageViews.resize(m_SwapchainImages.size());
    for ( size_t i = 0; i < m_SwapchainImages.size(); ++i )
    {
        vk::ImageViewCreateInfo createInfo{};
        createInfo.setImage(m_SwapchainImages[i])
                  .setViewType(vk::ImageViewType::e2D)
                  .setFormat(vk::Format::eB8G8R8A8Srgb)
                  .setComponents(vk::ComponentMapping())
                  .setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));

        m_SwapchainImageViews[i] = m_Device.createImageView(createInfo);
        if ( !m_SwapchainImageViews[i] )
            throw std::runtime_error("Failed to create image views");
    }
}

void VulkanApp::createRenderPass()
{
    vk::AttachmentDescription colorAttachment{};
    colorAttachment.setFormat(vk::Format::eB8G8R8A8Srgb)
                   .setSamples(vk::SampleCountFlagBits::e1)
                   .setLoadOp(vk::AttachmentLoadOp::eClear)
                   .setStoreOp(vk::AttachmentStoreOp::eStore)
                   .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
                   .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
                   .setInitialLayout(vk::ImageLayout::eUndefined)
                   .setFinalLayout(vk::ImageLayout::ePresentSrcKHR);

    vk::AttachmentReference colorAttachmentRef{};
    colorAttachmentRef.setAttachment(0)
                      .setLayout(vk::ImageLayout::eColorAttachmentOptimal);

    // Depth attachment
    vk::AttachmentDescription depthAttachment{};
    depthAttachment.format = m_GridRenderer ? m_GridRenderer->getDepthFormat() : vk::Format::eD32Sfloat; // Use default if nullptr
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
        throw std::runtime_error("Failed to create render pass");
}

void VulkanApp::createFramebuffers()
{
    m_SwapchainFramebuffers.resize(m_SwapchainImageViews.size());
    for ( size_t i = 0; i < m_SwapchainImageViews.size(); ++i )
    {
        std::array<vk::ImageView, 2> attachments = {
            m_SwapchainImageViews[i],
            m_GridRenderer ? m_GridRenderer->getDepthImageViews()[i] : m_SwapchainImageViews[i]
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

void VulkanApp::createCommandPool()
{
    const auto [
        graphicsFamily,
        presentFamily,
        computeFamily
    ] = findQueueFamilies(m_PhysicalDevice);

    vk::CommandPoolCreateInfo poolInfo{};
    poolInfo.setQueueFamilyIndex(graphicsFamily)
            .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);

    m_CommandPool = m_Device.createCommandPool(poolInfo);
    if ( !m_CommandPool )
        throw std::runtime_error("Failed to create command pool");
}

void VulkanApp::createCommandBuffers()
{
    m_CommandBuffers.resize(m_SwapchainFramebuffers.size());

    vk::CommandBufferAllocateInfo allocInfo{};
    allocInfo.setCommandPool(m_CommandPool)
             .setLevel(vk::CommandBufferLevel::ePrimary)
             .setCommandBufferCount(static_cast<uint32_t>(m_CommandBuffers.size()));

    m_CommandBuffers = m_Device.allocateCommandBuffers(allocInfo);
    if ( m_CommandBuffers.empty() )
        throw std::runtime_error("Failed to allocate command buffers");
}

void VulkanApp::createSyncObjects()
{
    vk::SemaphoreCreateInfo semaphoreInfo{};
    vk::FenceCreateInfo fenceInfo{};
    fenceInfo.setFlags(vk::FenceCreateFlagBits::eSignaled);

    m_ImageAvailableSemaphore = m_Device.createSemaphore(semaphoreInfo);
    m_RenderFinishedSemaphore = m_Device.createSemaphore(semaphoreInfo);
    m_InFlightFence = m_Device.createFence(fenceInfo);

    if ( !m_ImageAvailableSemaphore || !m_RenderFinishedSemaphore || !m_InFlightFence )
        throw std::runtime_error("Failed to create synchronization objects");
}

void VulkanApp::initImGui()
{
    m_ImGuiHandler = std::make_shared<ImGuiHandler>(
        m_Window,
        m_Instance,
        m_PhysicalDevice,
        m_Device,
        findQueueFamilies(m_PhysicalDevice).graphicsFamily,
        m_GraphicsQueue,
        m_CommandPool,
        m_RenderPass,
        m_SwapchainImages.size());
    m_ImGuiHandler->init();
}

void VulkanApp::initGridRenderer()
{
    if ( m_GridRenderer )
        m_GridRenderer.reset();
    m_GridRenderer = std::make_shared<GridRenderer>(
        m_Device,
        m_PhysicalDevice,
        m_CommandPool,
        m_GraphicsQueue,
        m_ComputeQueue,
        m_RenderPass,
        m_SwapchainImages,
        m_SwapchainExtent,
        GeometryType::Flat);
    m_GridRenderer->init();
    m_GridRenderer->createDepthResources();
}

vk::CommandBuffer VulkanApp::beginSingleTimeCommands() const
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

void VulkanApp::endSingleTimeCommands(const vk::CommandBuffer& commandBuffer) const
{
    commandBuffer.end();

    vk::SubmitInfo submitInfo{};
    submitInfo.setCommandBufferCount(1)
              .setPCommandBuffers(&commandBuffer);

    m_GraphicsQueue.submit(submitInfo, nullptr);
    m_GraphicsQueue.waitIdle();

    m_Device.freeCommandBuffers(m_CommandPool, commandBuffer);
}

void VulkanApp::mainLoop()
{
    double lastTime = glfwGetTime();
    while ( !glfwWindowShouldClose(m_Window) )
    {
        const double currentTime = glfwGetTime();
        const float deltaTime = static_cast<float>(currentTime - lastTime);
        lastTime = currentTime;
        glfwPollEvents();

        if ( m_FramebufferResized )
        {
            m_FramebufferResized = false;
            recreateSwapchain();
        }

        m_ImGuiHandler->newFrame();
        m_GridRenderer->updateCamera();
        // m_GridRenderer->updateSimulation(deltaTime); // Simulate motion
        m_GridRenderer->updateGrid();
        m_GridRenderer->renderCameraControls();
        m_ImGuiHandler->renderUI();
        drawFrame();
    }

    m_Device.waitIdle();
}

void VulkanApp::drawFrame()
{
    auto _ = m_Device.waitForFences(m_InFlightFence, VK_TRUE, UINT64_MAX);
    m_Device.resetFences(m_InFlightFence);

    uint32_t imageIndex;
    auto result = m_Device.acquireNextImageKHR(m_Swapchain, UINT64_MAX, m_ImageAvailableSemaphore, nullptr, &imageIndex);
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

    m_GridRenderer->draw(m_CommandBuffers[imageIndex]);
    m_ImGuiHandler->renderDrawData(m_CommandBuffers[imageIndex]);

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

    m_GraphicsQueue.submit(submitInfo, m_InFlightFence);

    vk::PresentInfoKHR presentInfo{};
    presentInfo.setWaitSemaphoreCount(1)
               .setPWaitSemaphores(signalSemaphores)
               .setSwapchainCount(1)
               .setPSwapchains(&m_Swapchain)
               .setPImageIndices(&imageIndex);

    result = m_PresentQueue.presentKHR(presentInfo);
    if ( result == vk::Result::eErrorOutOfDateKHR || result == vk::Result::eSuboptimalKHR )
        return recreateSwapchain();
    if ( result != vk::Result::eSuccess )
        throw std::runtime_error("Failed to present: " + vk::to_string(result));
}

void VulkanApp::cleanup()
{
    m_ImGuiHandler.reset();
    m_GridRenderer.reset();

    if ( m_RenderFinishedSemaphore )
        m_Device.destroySemaphore(m_RenderFinishedSemaphore);
    if ( m_ImageAvailableSemaphore )
        m_Device.destroySemaphore(m_ImageAvailableSemaphore);
    if ( m_InFlightFence )
        m_Device.destroyFence(m_InFlightFence);

    if ( m_CommandPool )
        m_Device.destroyCommandPool(m_CommandPool);

    for ( auto framebuffer : m_SwapchainFramebuffers )
        m_Device.destroyFramebuffer(framebuffer);

    if ( m_RenderPass )
        m_Device.destroyRenderPass(m_RenderPass);

    for ( auto imageView : m_SwapchainImageViews )
        m_Device.destroyImageView(imageView);

    if ( m_Swapchain )
        m_Device.destroySwapchainKHR(m_Swapchain);

    m_Device.destroy();
    if ( m_Surface )
        m_Instance.destroySurfaceKHR(m_Surface);

    m_Instance.destroy();

    glfwDestroyWindow(m_Window);
    glfwTerminate();
}
