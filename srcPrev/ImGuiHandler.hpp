//
// Created by Richard Mule on 3/21/25.
//

#pragma once

#include <GLFW/glfw3.h>
#include <vulkan/vulkan.hpp>
#include <imgui/imgui.h>

class ImGuiHandler
{
public:
    ImGuiHandler(
        GLFWwindow* window,
        vk::Instance instance,
        vk::PhysicalDevice physicalDevice,
        vk::Device device,
        uint32_t queueFamily,
        vk::Queue queue,
        vk::CommandPool commandPool,
        vk::RenderPass renderPass,
        uint32_t imageCount);
    ~ImGuiHandler();

    void init();
    static void newFrame();
    void renderUI();
    static void renderDrawData(vk::CommandBuffer commandBuffer);

private:
    GLFWwindow* m_Window;
    vk::Instance m_Instance;
    vk::PhysicalDevice m_PhysicalDevice;
    vk::Device m_Device;
    uint32_t m_QueueFamily;
    vk::Queue m_Queue;
    vk::CommandPool m_CommandPool;
    vk::RenderPass m_RenderPass;
    uint32_t m_ImageCount;
    vk::DescriptorPool m_DescriptorPool;

    vk::CommandBuffer beginSingleTimeCommands() const;
    void endSingleTimeCommands(vk::CommandBuffer commandBuffer) const;

    // Helper functions to convert between vk:: and Vk types for ImGui
    VkInstance getVkInstance() const { return static_cast<VkInstance>(m_Instance); }
    VkPhysicalDevice getVkPhysicalDevice() const { return static_cast<VkPhysicalDevice>(m_PhysicalDevice); }
    VkDevice getVkDevice() const { return static_cast<VkDevice>(m_Device); }
    VkQueue getVkQueue() const { return static_cast<VkQueue>(m_Queue); }
    VkRenderPass getVkRenderPass() const { return static_cast<VkRenderPass>(m_RenderPass); }
    VkCommandPool getVkCommandPool() const { return static_cast<VkCommandPool>(m_CommandPool); }
};