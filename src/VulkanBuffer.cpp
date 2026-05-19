//
// RAII wrapper for a Vulkan buffer and its backing device memory.
//

#include "VulkanBuffer.hpp"

#include <cstring>
#include <stdexcept>

VulkanBuffer::VulkanBuffer(
    vk::Device device,
    vk::PhysicalDevice physicalDevice,
    vk::DeviceSize size,
    vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags properties)
    : m_Device(device)
    , m_Size(size)
{
    vk::BufferCreateInfo bufferInfo{};
    bufferInfo.setSize(size)
              .setUsage(usage)
              .setSharingMode(vk::SharingMode::eExclusive);

    m_Buffer = m_Device.createBuffer(bufferInfo);
    if ( !m_Buffer )
        throw std::runtime_error("VulkanBuffer: failed to create buffer");

    const vk::MemoryRequirements memReq = m_Device.getBufferMemoryRequirements(m_Buffer);

    vk::MemoryAllocateInfo allocInfo{};
    allocInfo.setAllocationSize(memReq.size)
             .setMemoryTypeIndex(
                 findMemoryType(physicalDevice, memReq.memoryTypeBits, properties));

    m_Memory = m_Device.allocateMemory(allocInfo);
    if ( !m_Memory )
        throw std::runtime_error("VulkanBuffer: failed to allocate buffer memory");

    m_Device.bindBufferMemory(m_Buffer, m_Memory, 0);
}

VulkanBuffer::~VulkanBuffer()
{
    reset();
}

VulkanBuffer::VulkanBuffer(VulkanBuffer&& other) noexcept
    : m_Device(other.m_Device)
    , m_Buffer(other.m_Buffer)
    , m_Memory(other.m_Memory)
    , m_Size(other.m_Size)
{
    other.m_Buffer = nullptr;
    other.m_Memory = nullptr;
    other.m_Size = 0;
}

VulkanBuffer& VulkanBuffer::operator=(VulkanBuffer&& other) noexcept
{
    if ( this != &other )
    {
        reset();   // free whatever we currently hold first
        m_Device = other.m_Device;
        m_Buffer = other.m_Buffer;
        m_Memory = other.m_Memory;
        m_Size   = other.m_Size;
        other.m_Buffer = nullptr;
        other.m_Memory = nullptr;
        other.m_Size = 0;
    }
    return *this;
}

void VulkanBuffer::reset()
{
    if ( m_Buffer )
        m_Device.destroyBuffer(m_Buffer);
    if ( m_Memory )
        m_Device.freeMemory(m_Memory);
    m_Buffer = nullptr;
    m_Memory = nullptr;
    m_Size = 0;
}

void VulkanBuffer::writeFromHost(const void* src, vk::DeviceSize bytes) const
{
    void* mapped = m_Device.mapMemory(m_Memory, 0, bytes, {});
    std::memcpy(mapped, src, static_cast<size_t>(bytes));
    m_Device.unmapMemory(m_Memory);
}

uint32_t VulkanBuffer::findMemoryType(
    vk::PhysicalDevice physicalDevice,
    uint32_t typeFilter,
    vk::MemoryPropertyFlags properties)
{
    const vk::PhysicalDeviceMemoryProperties memProps = physicalDevice.getMemoryProperties();
    for ( uint32_t i = 0; i < memProps.memoryTypeCount; ++i )
    {
        if ( (typeFilter & (1u << i)) &&
             (memProps.memoryTypes[i].propertyFlags & properties) == properties )
            return i;
    }
    throw std::runtime_error("VulkanBuffer: no suitable memory type");
}
