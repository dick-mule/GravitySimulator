//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#include "VkResourceManager.hpp"
#include "ImGuiHandler.hpp"
#include "GridRenderer.hpp"

class VulkanApp 
{
public:
    explicit VulkanApp(std::string title);
    ~VulkanApp();

protected:
    void initGridRenderer();
    void initImGui();
    void mainLoop();
    void drawFrame();

public:
    void run();

private:
    std::shared_ptr<VkResourceManager> m_ResourceManager;
    std::shared_ptr<ImGuiHandler> m_ImGuiHandler;
    std::shared_ptr<GridRenderer> m_GridRenderer;

public: /// Utilities
    [[nodiscard]] std::shared_ptr<VkResourceManager> operator->() { return m_ResourceManager; }
};
