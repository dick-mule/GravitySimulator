//
// Created by Richard Mule on 3/21/25.
//

#include "ImGuiHandler.hpp"
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
    m_Device.destroyDescriptorPool(m_DescriptorPool);
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
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

void ImGuiHandler::renderUI()
{
    ImGui::SetNextWindowSize(ImVec2(400.0f, 300.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Gravity Simulator Controls");
    ImGui::Text("Hello, Vulkan!");
    ImGui::End();

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
