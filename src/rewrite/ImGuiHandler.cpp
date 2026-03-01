//
// Created by Richard Mule on 7/5/25.
//

#include "ImGuiHandler.hpp"

ImGuiHandler::ImGuiHandler(std::shared_ptr<VkResourceManager> resourceManager)
    : m_ResourceManager(std::move(resourceManager))
{
}

ImGuiHandler::~ImGuiHandler()
{
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void ImGuiHandler::initialize()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForVulkan(m_ResourceManager->windowHandler()->window(), true);

    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance = m_ResourceManager->appInstance();
    init_info.PhysicalDevice = m_ResourceManager->physicalDevice();
    init_info.Device = m_ResourceManager->device();
    init_info.QueueFamily = m_ResourceManager->queueFamilyIndices().graphicsFamily;
    init_info.Queue = m_ResourceManager->queues().at(ResourceTypes::GRAPHICS);
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.DescriptorPool = VK_NULL_HANDLE;
    init_info.Allocator = nullptr;
    init_info.MinImageCount = 2;
    init_info.ImageCount = m_ResourceManager->swapchainImages().size();
    init_info.CheckVkResultFn = nullptr;
    init_info.RenderPass = m_ResourceManager->renderPass();

    const auto descriptorPool = m_ResourceManager->defaultDescriptorPool();
    if ( !descriptorPool )
        throw std::runtime_error("Failed to create descriptor pool for ImGui");

    init_info.DescriptorPool = static_cast<VkDescriptorPool>(descriptorPool);
    if ( !ImGui_ImplVulkan_Init(&init_info) )
        throw std::runtime_error("Failed to initialize ImGui Vulkan backend");

    const vk::CommandPool& guiPool = m_ResourceManager->commandPools().at(ResourceTypes::GRAPHICS);
    const vk::CommandBuffer commandBuffer = m_ResourceManager->singleTimeCommand(guiPool);
    const vk::Queue& guiQueue = m_ResourceManager->queues().at(ResourceTypes::GRAPHICS);
    ImGui_ImplVulkan_CreateFontsTexture();
    m_ResourceManager->terminateSingleTimeCommand(guiPool, commandBuffer, guiQueue);
    ImGui_ImplVulkan_DestroyFontsTexture();
}

void ImGuiHandler::newFrame()
{
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiHandler::render()
{
    ImGui::SetNextWindowSize(ImVec2(400.0f, 300.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Gravity Simulator Controls");
    ImGui::Text("Hello, Vulkan!");
    ImGui::End();

    ImGui::Render();
}

void ImGuiHandler::renderDrawData(const vk::CommandBuffer& commandBuffer)
{
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
}
