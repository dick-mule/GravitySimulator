//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#include "VkResourceManager.hpp"

class ImGuiHandler 
{
public:
    explicit ImGuiHandler(std::shared_ptr<VkResourceManager> resourceManager);
    ~ImGuiHandler();

    void initialize();
    static void newFrame();
    static void render();
    static void renderDrawData(const vk::CommandBuffer& commandBuffer);

private:
    std::shared_ptr<VkResourceManager> m_ResourceManager;

};
