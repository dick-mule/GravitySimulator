//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#define GLFW_INCLUDE_VULKAN

#include <GLFW/glfw3.h>
#include <vulkan/vulkan.hpp>

#include <set>
#include <unordered_map>
#include <ranges>
#include <vector>

#ifndef VK_MVK_MACOS_SURFACE_EXTENSION_NAME2
#define VK_MVK_MACOS_SURFACE_EXTENSION_NAME2 "VK_EXT_metal_surface"
#endif

#include "../cpp_dependencies/magic_enum/include/magic_enum/magic_enum.hpp"
#include "../imgui/backends/imgui_impl_glfw.h"
#include "../imgui/backends/imgui_impl_vulkan.h"

static const std::vector<vk::DescriptorPoolSize> s_DescriptorPoolSizes = {
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

struct ShaderProgram
{
    vk::ShaderModule vertexShader;
    vk::ShaderModule fragmentShader;
};

enum class ResourceTypes : int
{
    GRAPHICS,
    PRESENT,
    COMPUTE,
    IMAGE,
    RENDER,
    DEPTH,
    INFLIGHT,
    VERTEX,
    INDEX,
    OBJECT,
    TRAIL
};

std::ostream& operator<<(std::ostream& os, const ResourceTypes& type);

struct QueueFamilyIndices
{
    uint32_t graphicsFamily = UINT32_MAX;
    uint32_t presentFamily = UINT32_MAX;
    uint32_t computeFamily = UINT32_MAX;

    [[nodiscard]] bool isComplete() const
    {
        return graphicsFamily != UINT32_MAX && presentFamily != UINT32_MAX && computeFamily != UINT32_MAX;
    }
    static QueueFamilyIndices fromDevice(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface);
    [[nodiscard]] std::vector<uint32_t> toVect() const { return { graphicsFamily, presentFamily, computeFamily }; }
    [[nodiscard]] std::unordered_map<ResourceTypes, uint32_t> mappedIndices() const
    {
        return {
            { ResourceTypes::GRAPHICS, graphicsFamily },
            { ResourceTypes::PRESENT, presentFamily },
            { ResourceTypes::COMPUTE, computeFamily }
        };
    }
};

struct WrappedDescriptorSet
{
    vk::DescriptorSetLayout descLayout;
    vk::DescriptorPool descPool;
    vk::DescriptorSet descSet;
    vk::PipelineLayoutCreateInfo descLayoutInfo;
    vk::PipelineLayout descPipelineLayout;
    vk::ComputePipelineCreateInfo descPipelineInfo;
    vk::Pipeline descPipeline;
};

struct WrappedBuffers
{
    vk::Buffer buffer;
    vk::DeviceMemory memory;
    uint32_t memOffset;
    size_t size;
};

class VkResourceManager;

class GlfwWindowHandler
{
private:
    GLFWwindow* m_Window;
    std::string m_WindowTitle;
    size_t m_Width, m_Height;
    VkResourceManager* m_ResourceManager;

public:
    explicit GlfwWindowHandler(
        std::string title,
        VkResourceManager* resourceManager,
        size_t width = 2000,
        size_t height = 1500);
    ~GlfwWindowHandler();

    void initialize();
    void destroy();
    static std::vector<const char*> getGlfwExtensions();
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);

    [[nodiscard]] GLFWwindow* window() const { return m_Window; }
    [[nodiscard]] const std::string& title() const { return m_WindowTitle; }
    [[nodiscard]] size_t width() const { return m_Width; }
    void setWidth(size_t update_width) { m_Width = update_width; }
    [[nodiscard]] size_t height() const { return m_Height; }
    void setHeight(size_t update_height) { m_Height = update_height; }

    vk::Extent2D currentExtent() const;
};

class VkResourceManager 
{
private:
    std::unique_ptr<GlfwWindowHandler> m_WindowHandler;
    vk::Instance m_Instance;
    vk::SurfaceKHR m_Surface;
    vk::PhysicalDevice m_PhysicalDevice;
    vk::Device m_Device;
    QueueFamilyIndices m_QueueFamilyIndices;
    std::unordered_map<ResourceTypes, vk::Queue> m_Queues;
    vk::Extent2D m_SwapchainExtent;
    vk::SwapchainKHR m_Swapchain;
    std::vector<vk::Image> m_SwapchainImages;
    std::vector<vk::ImageView> m_SwapchainImageViews;
    vk::RenderPass m_RenderPass;
    std::vector<vk::Framebuffer> m_SwapchainFramebuffers;
    std::unordered_map<ResourceTypes, vk::CommandPool> m_CommandPools;
    std::vector<vk::CommandBuffer> m_CommandBuffers;
    vk::Semaphore m_ImageAvailableSemaphore, m_RenderFinishedSemaphore;
    std::unordered_map<ResourceTypes, vk::Fence> m_InFlightFences;
    std::unordered_map<ResourceTypes, WrappedDescriptorSet> m_WrappedDescriptors;
    std::unordered_map<ResourceTypes, WrappedBuffers> m_WrappedBuffers;
    bool m_FrameBufferResized;
    std::function<void()> m_SwapchainCleanupCallback, m_SwapchainReinitCallback;
    std::function<void(const vk::CommandBuffer& buffer)> m_DrawCallback;

public:
    explicit VkResourceManager(std::string title);
    ~VkResourceManager();

    static vk::ApplicationInfo makeApplicationInfo(const std::string& appName);
    static vk::InstanceCreateInfo makeInstanceCreateInfo(
        const vk::ApplicationInfo& appInfo,
        const std::vector<const char*>& enabledExtensions);
    vk::SurfaceFormatKHR getSurfaceFormat() const;
    vk::PresentModeKHR getPresentModes() const;

    void initializeApplication();

protected:
    void initializeAppInstance(const vk::InstanceCreateInfo& createInfo);
    void initializeSurface();
    void initializeDevices();
    void initializeQueues();
    void createSwapChain();
    void initializeImageViews();
    void cleanupSwapChain();

private:
    vk::SwapchainCreateInfoKHR initializeCreateSwapchainInfo();


public:
    void createRenderPass(vk::Format depthFormat = vk::Format::eD32Sfloat);
    void createFramebuffers(const std::vector<vk::ImageView>& imageViews = {});
    void createCommandPools();
    void createNewFence(ResourceTypes resourceType);
    void initializeSyncObjects();
    void createResourcePipeline(vk::ShaderModule computeShaderModule, ResourceTypes resourceType);

    static vk::DescriptorPoolCreateInfo defaultDescriptorPoolInfo()
    {
        vk::DescriptorPoolCreateInfo pool_info{};
        pool_info.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet)
                  .setMaxSets(1000 * s_DescriptorPoolSizes.size())
                  .setPoolSizeCount(s_DescriptorPoolSizes.size())
                  .setPPoolSizes(s_DescriptorPoolSizes.data());
        return pool_info;
    }

    vk::DescriptorPool createDescriptorPool(const vk::DescriptorPoolCreateInfo& info) const
    {
        return m_Device.createDescriptorPool(info);
    }

    vk::DescriptorPool defaultDescriptorPool() const
    {
        return m_Device.createDescriptorPool(defaultDescriptorPoolInfo());
    }

    vk::CommandBuffer startSingleTimeCommand(const vk::CommandPool& commandPool) const;
    void terminateSingleTimeCommand(
        const vk::CommandPool& commandPool,
        const vk::CommandBuffer& commandBuffer,
        const vk::Queue& queue) const;

    void recreateSwapchain();
    void handleWindowResize();
    void drawFrame();

    /// Getters/setters
    [[nodiscard]] vk::Instance appInstance() const { return m_Instance; }
    [[nodiscard]] const std::unique_ptr<GlfwWindowHandler>& windowHandler() const { return m_WindowHandler; }
    [[nodiscard]] const vk::SurfaceKHR& surface() const { return m_Surface; }
    [[nodiscard]] const vk::PhysicalDevice& physicalDevice() const { return m_PhysicalDevice; }
    [[nodiscard]] const vk::Device& device() const { return m_Device; }
    [[nodiscard]] const QueueFamilyIndices& queueFamilyIndices() const { return m_QueueFamilyIndices; }
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Queue>& queues() const { return m_Queues; }
    [[nodiscard]] const vk::SwapchainKHR& swapchain() const { return m_Swapchain; }
    [[nodiscard]] const vk::Extent2D& extent() const { return m_SwapchainExtent; }
    [[nodiscard]] const std::vector<vk::Image>& swapchainImages() const { return m_SwapchainImages; }
    [[nodiscard]] const std::vector<vk::ImageView>& imageViews() const { return m_SwapchainImageViews; }
    [[nodiscard]] const vk::RenderPass& renderPass() const { return m_RenderPass; }
    [[nodiscard]] const std::vector<vk::Framebuffer>& swapchainFramebuffers() const { return m_SwapchainFramebuffers; }
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::CommandPool>& commandPools() const { return m_CommandPools; }
    [[nodiscard]] const std::vector<vk::CommandBuffer>& commandBuffers() const { return m_CommandBuffers; }
    [[nodiscard]] const vk::Semaphore& imageAvailableSemaphore() const { return m_ImageAvailableSemaphore; }
    [[nodiscard]] const vk::Semaphore& renderFinishedSemaphore() const { return m_RenderFinishedSemaphore; }
    [[nodiscard]] const vk::Fence& fence(const ResourceTypes type) const { return m_InFlightFences.at(type); }
    [[nodiscard]] bool framebufferResized() const { return m_FrameBufferResized; }
    [[nodiscard]] const WrappedDescriptorSet& wrappedDescriptorSet(const ResourceTypes type) const
    {
        if ( m_WrappedDescriptors.contains(type) )
            return m_WrappedDescriptors.at(type);
        throw std::out_of_range("Wrapped descriptor set does not exist");
    }
    [[nodiscard]] const WrappedBuffers& wrappedBuffers(const ResourceTypes type) const
    {
        if ( m_WrappedBuffers.contains(type) )
            return m_WrappedBuffers.at(type);
        throw std::out_of_range("Wrapped buffers does not exist");
    }
    void setFrameBufferResized(const bool resized) { m_FrameBufferResized = resized; }
    void setSwapchainCleanupCallback(const std::function<void()>& callback) { m_SwapchainCleanupCallback = callback; }
    void setSwapchainReinitCallback(const std::function<void()>& callback) { m_SwapchainReinitCallback = callback; }
    void setDrawCallback(const std::function<void(const vk::CommandBuffer& commandBuffer)>& callback)
    {
        m_DrawCallback = callback;
    }
    void setBuffer(
        const vk::Buffer& buffer,
        const vk::DeviceMemory& memory,
        size_t buffer_size,
        ResourceTypes resource_type,
        uint32_t memOffset = 0);
    void bindBufferDescriptorSets() const;
};
