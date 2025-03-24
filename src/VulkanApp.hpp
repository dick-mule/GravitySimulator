//
// Created by Richard Mule on 3/21/25.
//

#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vulkan/vulkan.hpp>
#include <vector>
#include <set>

class ImGuiHandler;
class GridRenderer;

#ifndef VK_MVK_MACOS_SURFACE_EXTENSION_NAME2
#define VK_MVK_MACOS_SURFACE_EXTENSION_NAME2 "VK_EXT_metal_surface"
#endif


class VulkanApp
{
public:
    VulkanApp();
    ~VulkanApp();

    void run();

private:
    GLFWwindow* m_Window;
    vk::Instance m_Instance;
    vk::PhysicalDevice m_PhysicalDevice;
    vk::Device m_Device;
    vk::Queue m_GraphicsQueue;
    vk::Queue m_PresentQueue;
    vk::SurfaceKHR m_Surface;
    vk::SwapchainKHR m_Swapchain;
    vk::Extent2D m_SwapchainExtent;
    std::vector<vk::Image> m_SwapchainImages;
    std::vector<vk::ImageView> m_SwapchainImageViews;
    vk::RenderPass m_RenderPass;
    std::vector<vk::Framebuffer> m_SwapchainFramebuffers;
    vk::CommandPool m_CommandPool;
    std::vector<vk::CommandBuffer> m_CommandBuffers;
    vk::Semaphore m_ImageAvailableSemaphore;
    vk::Semaphore m_RenderFinishedSemaphore;
    vk::Fence m_InFlightFence;

    const int m_WIDTH = 1280;
    const int m_HEIGHT = 720;

    std::shared_ptr<ImGuiHandler> m_ImGuiHandler;
    std::shared_ptr<GridRenderer> m_GridRenderer;

    bool m_FramebufferResized = false; // Flag to signal resize
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
    void recreateSwapchain(); // New method to handle resize
    void cleanupSwapchain(); // Helper to clean up old resources
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    void initWindow();
    void initVulkan();
    void createInstance();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSwapchain();
    void createImageViews();
    void createRenderPass();
    void createFramebuffers();
    void createCommandPool();
    void createCommandBuffers();
    void createSyncObjects();

    struct QueueFamilyIndices
    {
        uint32_t graphicsFamily = UINT32_MAX;
        uint32_t presentFamily = UINT32_MAX;

        [[nodiscard]] bool isComplete() const
        {
            return graphicsFamily != UINT32_MAX && presentFamily != UINT32_MAX;
        }
    };
    QueueFamilyIndices findQueueFamilies(const vk::PhysicalDevice& device) const;
    bool isDeviceSuitable(const vk::PhysicalDevice& device) const;

    void initImGui();
    void initGridRenderer();
    vk::CommandBuffer beginSingleTimeCommands() const;
    void endSingleTimeCommands(const vk::CommandBuffer& commandBuffer) const;

    void mainLoop();
    void drawFrame();

    void cleanup();

    // Helper functions to convert between vk:: and Vk types for ImGui
    [[nodiscard]] VkInstance getVkInstance() const { return static_cast<VkInstance>(m_Instance); }
    [[nodiscard]] VkPhysicalDevice getVkPhysicalDevice() const { return static_cast<VkPhysicalDevice>(m_PhysicalDevice); }
    [[nodiscard]] VkDevice getVkDevice() const { return static_cast<VkDevice>(m_Device); }
    [[nodiscard]] VkQueue getVkGraphicsQueue() const { return static_cast<VkQueue>(m_GraphicsQueue); }
    [[nodiscard]] VkRenderPass getVkRenderPass() const { return static_cast<VkRenderPass>(m_RenderPass); }
    [[nodiscard]] VkCommandPool getVkCommandPool() const { return static_cast<VkCommandPool>(m_CommandPool); }
};