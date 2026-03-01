//
// Created by Richard Mule on 7/5/25.
//

#include "VulkanApp.hpp"

VulkanApp::VulkanApp(std::string title)
    : m_ResourceManager(nullptr)
    , m_ImGuiHandler(nullptr)
    , m_GridRenderer(nullptr)
{
    if ( !glfwInit() )
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }
    m_ResourceManager = std::make_shared<VkResourceManager>(std::move(title));
}

VulkanApp::~VulkanApp()
{
    glfwTerminate();
}

void VulkanApp::run()
{
    m_ResourceManager->initializeApplication();
    m_ResourceManager->setSwapchainCleanupCallback([this]() -> void
        {
            m_ImGuiHandler.reset();
            m_GridRenderer.reset();
        });
    m_ResourceManager->setSwapchainReinitCallback([this]() -> void
        {
            initImGui();
            initGridRenderer();
        });
    m_ResourceManager->setDrawCallback([this](const vk::CommandBuffer& commandBuffer) -> void
        {
            m_GridRenderer->draw(commandBuffer);
            m_ImGuiHandler->renderDrawData(commandBuffer);
        });
    initImGui();
    initGridRenderer();
    mainLoop();
}

void VulkanApp::initGridRenderer()
{
    m_GridRenderer.reset();
    m_GridRenderer = std::make_shared<GridRenderer>(m_ResourceManager);
    m_GridRenderer->initialize();
}

void VulkanApp::initImGui()
{
    m_ImGuiHandler.reset();
    m_ImGuiHandler = std::make_shared<ImGuiHandler>(m_ResourceManager);
}

void VulkanApp::mainLoop()
{
    double lastTime = glfwGetTime();
    while ( !glfwWindowShouldClose(m_ResourceManager->windowHandler()->window()) )
    {
        const double currentTime = glfwGetTime();
        const float deltaTime = static_cast<float>(currentTime - lastTime);
        lastTime = currentTime;
        glfwPollEvents();
        /// Check if we need to update swapchain to handle new window dimensions
        m_ResourceManager->handleWindowResize();
        m_ImGuiHandler->newFrame();
        m_GridRenderer->updateCamera();
        // m_GridRenderer->updateSimulation(deltaTime); // Simulate motion
        m_GridRenderer->updateGrid();
        m_GridRenderer->renderCameraControls();
        m_ImGuiHandler->renderUI();
        drawFrame();
    }

    m_ResourceManager->device().waitIdle();
}

void VulkanApp::drawFrame()
{
    m_ResourceManager->drawFrame();
}
