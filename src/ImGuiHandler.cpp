//
// Created by Richard Mule on 3/21/25.
//

#include "ImGuiHandler.hpp"
#include "GridRenderer.hpp"      // For GPU grid toggle (getUseGPUGrid)
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_vulkan.h"
#include <stdexcept>

ImGuiHandler::ImGuiHandler(
    GLFWwindow* window,
    vk::Instance instance,
    vk::PhysicalDevice physicalDevice,
    vk::Device device,
    uint32_t queueFamily,
    vk::Queue queue,
    vk::CommandPool commandPool,
    vk::RenderPass renderPass,
    uint32_t imageCount)
    : m_Window(window)
    , m_Instance(instance)
    , m_PhysicalDevice(physicalDevice)
    , m_Device(device)
    , m_QueueFamily(queueFamily)
    , m_Queue(queue)
    , m_CommandPool(commandPool)
    , m_RenderPass(renderPass)
    , m_ImageCount(imageCount)
    , m_DescriptorPool(nullptr)
{
}

ImGuiHandler::~ImGuiHandler()
{
    // ImGui_ImplVulkan_Shutdown() frees the descriptor sets it allocated from
    // m_DescriptorPool, so it must run BEFORE the pool itself is destroyed.
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    m_Device.destroyDescriptorPool(m_DescriptorPool);
}

void ImGuiHandler::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForVulkan(m_Window, true);

    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance = getVkInstance();
    init_info.PhysicalDevice = getVkPhysicalDevice();
    init_info.Device = getVkDevice();
    init_info.QueueFamily = m_QueueFamily;
    init_info.Queue = getVkQueue();
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.DescriptorPool = VK_NULL_HANDLE;
    init_info.Allocator = nullptr;
    init_info.MinImageCount = 2;
    init_info.ImageCount = m_ImageCount;
    init_info.CheckVkResultFn = nullptr;
    init_info.RenderPass = getVkRenderPass();

    const vk::DescriptorPoolSize pool_sizes[] = {
        {vk::DescriptorType::eSampler, 1000},
        {vk::DescriptorType::eCombinedImageSampler, 1000},
        {vk::DescriptorType::eSampledImage, 1000},
        {vk::DescriptorType::eStorageImage, 1000},
        {vk::DescriptorType::eUniformTexelBuffer, 1000},
        {vk::DescriptorType::eStorageTexelBuffer, 1000},
        {vk::DescriptorType::eUniformBuffer, 1000},
        {vk::DescriptorType::eStorageBuffer, 1000},
        {vk::DescriptorType::eUniformBufferDynamic, 1000},
        {vk::DescriptorType::eStorageBufferDynamic, 1000},
        {vk::DescriptorType::eInputAttachment, 1000}
    };

    vk::DescriptorPoolCreateInfo pool_info{};
    pool_info.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet)
              .setMaxSets(1000 * IM_ARRAYSIZE(pool_sizes))
              .setPoolSizeCount(static_cast<uint32_t>(IM_ARRAYSIZE(pool_sizes)))
              .setPPoolSizes(pool_sizes);

    m_DescriptorPool = m_Device.createDescriptorPool(pool_info);
    if ( !m_DescriptorPool )
        throw std::runtime_error("Failed to create descriptor pool for ImGui");

    init_info.DescriptorPool = static_cast<VkDescriptorPool>(m_DescriptorPool);

    if ( !ImGui_ImplVulkan_Init(&init_info) )
        throw std::runtime_error("Failed to initialize ImGui Vulkan backend");

    const vk::CommandBuffer commandBuffer = beginSingleTimeCommands();
    ImGui_ImplVulkan_CreateFontsTexture();
    endSingleTimeCommands(commandBuffer);
    ImGui_ImplVulkan_DestroyFontsTexture();
}

void ImGuiHandler::newFrame()
{
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiHandler::renderUI(GridRenderer* gridRenderer)
{
    // === Left-side Simulation Status Panel (sleek scientific viz aesthetic) ===
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(310, 0), ImGuiCond_FirstUseEver);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.07f, 0.09f, 0.96f));           // Deep charcoal
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.15f, 0.18f, 0.22f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.08f, 0.10f, 0.13f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.10f, 0.13f, 0.17f, 1.0f));

    if (ImGui::Begin("Simulation", nullptr,
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_AlwaysAutoResize))
    {
        // Header with accent
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.85f, 0.78f, 1.0f)); // Teal accent
        ImGui::SeparatorText("GRAVITY SIM");
        ImGui::PopStyleColor();

        ImGui::Spacing();

        // Quick status row
        ImGui::Text("Geometry:");
        ImGui::SameLine(110);
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Flat"); // TODO: query from renderer

        bool gpuEnabled = gridRenderer ? gridRenderer->getUseGPUGrid() : false;
        ImGui::Text("Mode:");
        ImGui::SameLine(110);
        ImGui::TextColored(gpuEnabled ? ImVec4(0.0f, 0.9f, 0.7f, 1.0f) : ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                           gpuEnabled ? "GPU Grid" : "CPU Fallback");

        ImGui::Separator();

        // Live metrics - clean two-column layout
        float fps = ImGui::GetIO().Framerate;
        ImGui::Text("FPS");
        ImGui::SameLine(110);
        ImGui::TextColored(fps > 55 ? ImVec4(0.4f, 1.0f, 0.6f, 1.0f) : ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "%.1f", fps);

        ImGui::Text("Frame");
        ImGui::SameLine(110);
        ImGui::Text("%.2f ms", 1000.0f / fps);

        ImGui::Text("Vertices");
        ImGui::SameLine(110);
        ImGui::Text("~250k"); // TODO: query real count

        ImGui::Text("Bodies");
        ImGui::SameLine(110);
        ImGui::Text("%d", 10); // TODO: query real count

        ImGui::Separator();

        // GPU status (read-only display now that the toggle lives in the main Controls window)
        if (gpuEnabled)
        {
            ImGui::TextColored(ImVec4(0.0f, 0.85f, 0.78f, 1.0f), "GPU Grid: ON");
        }
        else
        {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "GPU Grid: OFF (CPU fallback)");
        }

        ImGui::Spacing();
        ImGui::TextDisabled("xAI Gravity Simulator  •  2026");
    }
    ImGui::End();

    ImGui::PopStyleColor(4);
    ImGui::PopStyleVar(2);

    ImGui::Render();
}

void ImGuiHandler::renderDrawData(const vk::CommandBuffer commandBuffer)
{
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
}

vk::CommandBuffer ImGuiHandler::beginSingleTimeCommands() const
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

void ImGuiHandler::endSingleTimeCommands(vk::CommandBuffer commandBuffer) const
{
    commandBuffer.end();

    vk::SubmitInfo submitInfo{};
    submitInfo.setCommandBufferCount(1)
              .setPCommandBuffers(&commandBuffer);

    m_Queue.submit(submitInfo, nullptr);
    m_Queue.waitIdle();

    m_Device.freeCommandBuffers(m_CommandPool, commandBuffer);
}
