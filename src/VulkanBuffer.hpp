//
// RAII wrapper for a Vulkan buffer and its backing device memory.
//

#pragma once

#include <vulkan/vulkan.hpp>

// Owns a vk::Buffer together with its vk::DeviceMemory. Move-only: assigning
// to (or destroying) a VulkanBuffer frees whatever it previously held, so the
// buffer leaks that used to occur on geometry switch / reset are now
// structurally impossible. It also replaces the createBuffer / allocate /
// bind / destroy boilerplate that was repeated throughout GridRenderer.
class VulkanBuffer
{
public:
    VulkanBuffer() = default;
    VulkanBuffer(vk::Device device,
                 vk::PhysicalDevice physicalDevice,
                 vk::DeviceSize size,
                 vk::BufferUsageFlags usage,
                 vk::MemoryPropertyFlags properties);
    ~VulkanBuffer();

    VulkanBuffer(const VulkanBuffer&) = delete;
    VulkanBuffer& operator=(const VulkanBuffer&) = delete;
    VulkanBuffer(VulkanBuffer&& other) noexcept;
    VulkanBuffer& operator=(VulkanBuffer&& other) noexcept;

    // Frees the buffer + memory immediately (no-op if empty).
    void reset();

    [[nodiscard]] vk::Buffer handle() const { return m_Buffer; }
    [[nodiscard]] vk::DeviceMemory memory() const { return m_Memory; }
    [[nodiscard]] vk::DeviceSize size() const { return m_Size; }
    [[nodiscard]] bool valid() const { return static_cast<bool>(m_Buffer); }

    // memcpy `bytes` of `src` into the buffer. Only valid when the buffer was
    // allocated with host-visible memory (e.g. a staging buffer).
    void writeFromHost(const void* src, vk::DeviceSize bytes) const;

private:
    static uint32_t findMemoryType(vk::PhysicalDevice physicalDevice,
                                   uint32_t typeFilter,
                                   vk::MemoryPropertyFlags properties);

    vk::Device       m_Device{};
    vk::Buffer       m_Buffer{};
    vk::DeviceMemory m_Memory{};
    vk::DeviceSize   m_Size = 0;
};
