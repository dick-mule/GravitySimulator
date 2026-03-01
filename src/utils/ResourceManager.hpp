//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#include "utils/ShaderLoader.hpp"
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vulkan/vulkan.hpp>
#include <ranges>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#ifndef VK_MVK_MACOS_SURFACE_EXTENSION_NAME2
#define VK_MVK_MACOS_SURFACE_EXTENSION_NAME2 "VK_EXT_metal_surface"
#endif

#include "enum_utils.hpp"


struct QueueFamilyIndices
{
    uint32_t graphicsFamily = UINT32_MAX;
    uint32_t presentFamily = UINT32_MAX;
    uint32_t computeFamily = UINT32_MAX;

    [[nodiscard]] bool isComplete() const
    {
        return graphicsFamily != UINT32_MAX && presentFamily != UINT32_MAX && computeFamily != UINT32_MAX;
    }
};


static const std::vector<vk::DescriptorPoolSize> DescriptorPoolSizes = {
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
    COMPUTE,
    PRESENT,
    IMAGE,
    RENDER,
    DEPTH,
    INFLIGHT,
    Vertex,
    Index,
    Object,
    Trail
};

std::ostream& operator<<(std::ostream& os, const ResourceTypes& type);

class VulkanResourceManager
{
protected:
    GLFWwindow *m_Window;
    vk::Instance m_Instance;
    vk::PhysicalDevice m_PhysicalDevice;
    vk::Device m_Device;
    vk::SurfaceKHR m_Surface;
    vk::SwapchainKHR m_Swapchain;
    vk::Extent2D m_SwapchainExtent;
    std::vector<vk::Image> m_SwapchainImages;
    std::vector<vk::ImageView> m_SwapchainImageViews;
    vk::RenderPass m_RenderPass;
    std::vector<vk::Framebuffer> m_SwapchainFramebuffers;
    QueueFamilyIndices m_QueueFamilyIndices;
    std::unordered_map<ResourceTypes, vk::Queue> m_Queues;
    std::unordered_map<ResourceTypes, vk::CommandPool> m_CommandPools;
    std::unordered_map<ResourceTypes, std::vector<vk::CommandBuffer>> m_CommandBuffers;
    std::unordered_map<ResourceTypes, vk::Buffer> m_Buffers;
    std::unordered_map<ResourceTypes, vk::DeviceMemory> m_DeviceMemory;
    std::unordered_map<ResourceTypes, vk::Semaphore> m_Semaphores;
    std::unordered_map<ResourceTypes, vk::Fence> m_Fences;
    std::unordered_map<ResourceTypes, vk::Format> m_Formats;
    std::unordered_map<ResourceTypes, std::vector<vk::Image>> m_Images;
    std::unordered_map<ResourceTypes, std::vector<vk::DeviceMemory>> m_ImageMemory;
    std::unordered_map<ResourceTypes, std::vector<vk::ImageView>> m_ImageViews;
    std::unordered_map<ResourceTypes, vk::Pipeline> m_Pipelines;
    std::unordered_map<ResourceTypes, vk::PipelineLayout> m_PipelineLayouts;
    std::unordered_map<ResourceTypes, vk::DescriptorSet> m_DescriptorSet;
    std::unordered_map<ResourceTypes, vk::DescriptorSetLayout> m_DescriptorSetLayouts;
    std::unordered_map<ResourceTypes, vk::DescriptorPool> m_DescriptorPools;
public:
    explicit VulkanResourceManager(GLFWwindow* window);
    ~VulkanResourceManager();

    void initialize();
    void resetSwapchainFrameBuffers() const;
    void resetSwapchainImageViews() const;
    [[nodiscard]] vk::ShaderModule makeShader(const std::string& shader_file_path) const;
    [[nodiscard]] const vk::Queue& getResourceQueue(ResourceTypes type) const;
    [[nodiscard]] size_t resourceImageCount(ResourceTypes type) const;
    void makeDescriptorSetLayout(
        const std::vector<vk::DescriptorSetLayoutBinding>& bindings,
        ResourceTypes type);
    void makeDescriptorPool(
        const vk::DescriptorPoolCreateInfo& info,
        ResourceTypes type);
    void makeDescriptorSet(
        const vk::DescriptorSetAllocateInfo& info,
        ResourceTypes type);
    void makePipelineLayout(
        size_t layout_count,
        vk::ShaderStageFlagBits bit_flags,
        size_t push_constant_size,
        ResourceTypes type);
    void makeComputePipeline(
        vk::StructureType s_type,
        vk::StructureType stage_type,
        const vk::ShaderModule& shader_module,
        ResourceTypes type);
    [[nodiscard]] const vk::Fence& getResourceFence(ResourceTypes type) const;
    [[nodiscard]] const vk::CommandBuffer& getResourceCommandBuffer(ResourceTypes type) const;
    [[nodiscard]] const vk::DescriptorSet& getResourceDescriptorSet(ResourceTypes type) const;
    [[nodiscard]] const vk::DescriptorSetLayout& getResourceDescriptorPoolLayout(ResourceTypes type) const;
    [[nodiscard]] const vk::DescriptorPool& getResourceDescriptorPool(ResourceTypes type) const;
    [[nodiscard]] const vk::CommandPool& getResourceCommandPool(ResourceTypes type) const;
    [[nodiscard]] const vk::PipelineLayout& getResourcePipelineLayout(ResourceTypes type) const;
    [[nodiscard]] const vk::Pipeline& getResourcePipeline(ResourceTypes type) const;
    vk::CommandBuffer beginSingleTimeCommands(
        const vk::CommandPool& commandPool,
        vk::CommandBufferAllocateInfo& allocInfo,
        const vk::CommandBufferBeginInfo& beginInfo) const;
    vk::CommandBuffer beginSingleTimeCommands(const vk::CommandPool& commandPool) const;
    void endSingleTimeCommands(
        const vk::CommandBuffer& commandBuffer,
        const vk::Queue& queue,
        const vk::CommandPool& pool,
        const vk::Fence& fence = nullptr) const;
    vk::Buffer createBuffer(
        vk::DeviceSize size,
        vk::BufferUsageFlags usage) const;
    vk::DeviceMemory allocateBufferMemory(
        const vk::Buffer& buffer,
        vk::MemoryPropertyFlags properties) const;
    void copyBuffer(
        const vk::CommandPool& commandPool,
        const vk::Queue& queue,
        const vk::Buffer& srcBuffer,
        const vk::Buffer& dstBuffer,
        vk::DeviceSize size) const;

    template<typename T>
    void updateBufferMemory(
        const ResourceTypes type,
        const std::vector<T>& update_data)
    {
        void* data;
        const vk::DeviceSize objectSize = update_data.size() * sizeof(T);
        const auto result = m_Device.mapMemory(m_DeviceMemory[type], 0, objectSize, {}, &data);
        if ( result != vk::Result::eSuccess )
            throw std::runtime_error("Failed to map object buffer memory: " + vk::to_string(result));

        memcpy(data, update_data.data(), objectSize);
        m_Device.unmapMemory(m_DeviceMemory[type]);
    }

    template<typename T>
    void makeNewBuffer(
        const ResourceTypes type,
        const std::vector<T>& data_to_share,
        const vk::MemoryPropertyFlags properties,
        const vk::BufferUsageFlags usage,
        const vk::MemoryPropertyFlagBits memory_bits)
    {
        const vk::DeviceSize bufferSize = sizeof(T) * data_to_share.size();
        const auto stagingBuffer = createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc);
        const auto stagingBufferMemory = allocateBufferMemory(stagingBuffer, properties);
        m_Device.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);

        /// COPY DATA TO BUFFER
        void* data;
        auto _ = m_Device.mapMemory(stagingBufferMemory, 0, bufferSize, {}, &data);
        memcpy(data, data_to_share.data(), bufferSize);
        m_Device.unmapMemory(stagingBufferMemory);

        // Create vertex buffer
        m_Buffers[type] = createBuffer(bufferSize, usage);
        m_DeviceMemory[type] = allocateBufferMemory(m_Buffers[type], memory_bits);
        m_Device.bindBufferMemory(m_Buffers[type], m_DeviceMemory[type], 0);

        // Copy from staging to vertex buffer
        const auto& commandPool = getResourceCommandPool(type);
        const auto& queue = getResourceQueue(type);
        copyBuffer(commandPool, queue, stagingBuffer, m_Buffers[type], bufferSize);

        // Clean up staging buffer
        m_Device.destroyBuffer(stagingBuffer);
        m_Device.freeMemory(stagingBufferMemory);
    }

private:
    // utility functions
    [[nodiscard]] QueueFamilyIndices findQueueFamilies(const vk::PhysicalDevice& device) const;
    [[nodiscard]] bool isDeviceSuitable(const vk::PhysicalDevice& device) const;
    [[nodiscard]] vk::Format findSupportedFormat(
        const std::vector<vk::Format>& candidates,
        const vk::ImageTiling& tiling,
        vk::FormatFeatureFlags features) const;
    [[nodiscard]] vk::Format findDepthFormat() const;
    [[nodiscard]] uint32_t findMemoryType(
        uint32_t typeFilter,
        const vk::MemoryPropertyFlags& properties) const;
    void transitionImageLayout(
        const vk::Image& image,
        const vk::Format& format,
        const vk::ImageLayout& oldLayout,
        const vk::ImageLayout& newLayout);

    // initialization functions
    void createInstance();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSwapchain();
    void createImageViews();
    void createRenderPass();
    void createDepthResources();
    void createFramebuffers();
    void createCommandPool();
    void createCommandBuffers();
    void createSyncObjects();

public:
    // Getters and setters
    [[nodiscard]] GLFWwindow* window() const;
    void setWindow(GLFWwindow* window);
    [[nodiscard]] const vk::Instance& instance() const;
    void setInstance(const vk::Instance& instance);
    [[nodiscard]] const vk::PhysicalDevice& physicalDevice() const;
    void setPhysicalDevice(const vk::PhysicalDevice& physical_device);
    [[nodiscard]] const vk::Device& device() const;
    void setDevice(const vk::Device& device);
    [[nodiscard]] const vk::SurfaceKHR& surface() const;
    void setSurface(const vk::SurfaceKHR& surface);
    [[nodiscard]] const vk::SwapchainKHR& swapchain() const;
    void setSwapchain(const vk::SwapchainKHR& swapchain);
    [[nodiscard]] const vk::Extent2D& swapchainExtent() const;
    void setSwapchainExtent(const vk::Extent2D& swapchain_extent);
    [[nodiscard]] const std::vector<vk::Image>& swapchainImages() const;
    void setSwapchainImages(const std::vector<vk::Image>& swapchain_images);
    [[nodiscard]] const std::vector<vk::ImageView>& swapchainImageViews() const;
    void setSwapchainImageViews(const std::vector<vk::ImageView>& swapchain_image_views);
    [[nodiscard]] const vk::RenderPass& renderPass() const;
    void setRenderPass(const vk::RenderPass& render_pass);
    [[nodiscard]] const std::vector<vk::Framebuffer>& swapchainFramebuffers() const;
    void setSwapchainFramebuffers(const std::vector<vk::Framebuffer>& swapchain_framebuffers);
    [[nodiscard]] const QueueFamilyIndices& queueFamilyIndices() const;
    void setQueueFamilyIndices(const QueueFamilyIndices& queue_family_indices);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Queue>& queues() const;
    void setQueues(const std::unordered_map<ResourceTypes, vk::Queue>& queues);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::CommandPool>& commandPools() const;
    void setCommandPools(const std::unordered_map<ResourceTypes, vk::CommandPool>& command_pools);
    [[nodiscard]] const std::unordered_map<ResourceTypes, std::vector<vk::CommandBuffer>>& commandBuffers() const;
    void setCommandBuffers(const std::unordered_map<ResourceTypes, std::vector<vk::CommandBuffer>>& command_buffers);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Buffer>& buffers() const;
    void setBuffers(const std::unordered_map<ResourceTypes, vk::Buffer>& buffers);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::DeviceMemory>& deviceMemory() const;
    void setDeviceMemory(const std::unordered_map<ResourceTypes, vk::DeviceMemory>& device_memory);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Semaphore>& semaphores() const;
    void setSemaphores(const std::unordered_map<ResourceTypes, vk::Semaphore>& semaphores);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Fence>& fences() const;
    void setFences(const std::unordered_map<ResourceTypes, vk::Fence>& fences);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Format>& formats() const;
    void setFormats(const std::unordered_map<ResourceTypes, vk::Format>& formats);
    [[nodiscard]] const std::unordered_map<ResourceTypes, std::vector<vk::Image>>& images() const;
    void setImages(const std::unordered_map<ResourceTypes, std::vector<vk::Image>>& images);
    [[nodiscard]] const std::unordered_map<ResourceTypes, std::vector<vk::DeviceMemory>>& imageMemory() const;
    void setImageMemory(const std::unordered_map<ResourceTypes, std::vector<vk::DeviceMemory>>& image_memory);
    [[nodiscard]] const std::unordered_map<ResourceTypes, std::vector<vk::ImageView>>& imageViews() const;
    void setImageViews(const std::unordered_map<ResourceTypes, std::vector<vk::ImageView>>& image_views);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::Pipeline>& pipelines() const;
    void setPipelines(const std::unordered_map<ResourceTypes, vk::Pipeline>& pipelines);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::PipelineLayout>& pipelineLayouts() const;
    void setPipelineLayouts(const std::unordered_map<ResourceTypes, vk::PipelineLayout>& pipeline_layouts);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::DescriptorSet>& descriptorSet() const;
    void setDescriptorSet(const std::unordered_map<ResourceTypes, vk::DescriptorSet>& descriptor_set);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::DescriptorSetLayout>& descriptorSetLayouts() const;
    void setDescriptorSetLayouts(
        const std::unordered_map<ResourceTypes, vk::DescriptorSetLayout>& descriptor_set_layouts);
    [[nodiscard]] const std::unordered_map<ResourceTypes, vk::DescriptorPool>& descriptorPools() const;
    void setDescriptorPools(const std::unordered_map<ResourceTypes, vk::DescriptorPool>& descriptor_pools);
};


