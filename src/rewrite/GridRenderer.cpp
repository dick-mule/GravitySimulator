//
// Created by Richard Mule on 7/5/25.
//

#include "GridRenderer.hpp"

GridRenderer::GridRenderer(
    std::shared_ptr<VkResourceManager> resourceManager,
    const GeometryType geometryType,
    const SimulationConfig& simulationConfig)
    : m_ResourceManager(resourceManager)
    , m_Geometry(
        geometryFactory(
            resourceManager,
            geometryType,
            simulationConfig.grid_size,
            simulationConfig.grid_scale))
    , m_GeometryType(geometryType)
    , m_SimulationConfig(simulationConfig)
    , m_MassiveObjects({})
    , m_Trails({})
{
    initializeObjects();
}

GridRenderer::~GridRenderer()
{

}

void GridRenderer::makeCentralObject()
{
    Object centralObj{};
    centralObj.mass = 20000.0; // Very massive
    centralObj.color = glm::vec4(1.0, 0.0, 0.0, 0.0);
    alignPosition(centralObj);
    const auto centralShape = std::make_shared<Sphere>(centralObj);
    centralShape->setSize(1.0 + std::log(centralObj.mass / std::sqrt(m_SimulationConfig.grid_scale))); // Larger orbiters
    m_MassiveObjects.insert(m_MassiveObjects.begin(), centralShape); // Using Sphere for simplicity
}

void GridRenderer::makeOrbiters()
{
    if ( m_SimulationConfig.n_orbiters < 1 || m_MassiveObjects.empty() )
        return;

    const float centralMass = (*m_MassiveObjects.front())->mass;

    // Scale radii based on m_GridScale
    float totalMass = centralMass;
    for ( size_t i = 0; i < m_SimulationConfig.n_orbiters; ++i )
    {
        Object orbiter{};
        orbiter.mass = 10.0 + static_cast<float>(i) * 5.0;

        // Circular orbit parameters
        float angle = m_SimulationConfig.angle(i);
        float radius = m_SimulationConfig.radius(i);
        glm::vec3 rawPosition(radius * cos(angle), 0.0, radius * sin(angle));
        glm::vec3 position = convertCoordinates(
            rawPosition,
            GeometryType::Flat,
            m_CurrentGeometryType,
            m_SimulationConfig.grid_size / 2,
            m_Geometry);
        orbiter.position = position;
        alignPosition(orbiter);
        float actualRadius = glm::length(position);
        float v = std::sqrt(m_SimulationConfig.g_factor * centralMass / actualRadius);
        glm::vec3 radialDir = glm::normalize(position);
        glm::vec3 tangentDir(-radialDir.z, 0.0, radialDir.x);
        if ( glm::length(tangentDir) < 1e-6 )
            tangentDir = glm::vec3(0.0, 0.0, 1.0);
        orbiter.velocity = v * glm::normalize(tangentDir);
        m_MassiveObjects.push_back(std::make_unique<Sphere>(orbiter));
        auto& obj = m_MassiveObjects.back();
        totalMass += obj->m_Object.mass;
        // Use a cleaner scaling for the sizes of the masses
        obj->setSize(1.0 + std::log(obj->m_Object.mass / std::sqrt(m_SimulationConfig.grid_scale)));
    }
}

void GridRenderer::initializeObjects()
{
    makeCentralObject();
    makeOrbiters();

    m_Trails.resize(m_MassiveObjects.size());
    m_ResourceManager->createNewFence(ResourceTypes::TRAIL);
    for ( auto& [positions] : m_Trails )
        positions.clear();
}


void GridRenderer::initialize() const
{
    m_Geometry->createGeometryPipeline();
}

void GridRenderer::draw()
{
    const vk::Extent2D& extent = m_ResourceManager->extent();
    const std::vector<vk::CommandBuffer>& commandBuffers = m_ResourceManager->commandBuffers();
    if ( commandBuffers.empty() )
        throw std::runtime_error("No command buffers available");
    const vk::CommandBuffer& commandBuffer = commandBuffers[0];
    vk::Viewport viewport{};
    viewport.setX(0.0f)
            .setY(0.0f)
            .setWidth(static_cast<float>(extent.width))
            .setHeight(static_cast<float>(extent.height))
            .setMinDepth(0.0f)
            .setMaxDepth(1.0f);
    commandBuffer.setViewport(0, viewport);

    vk::Rect2D scissor{};
    scissor.setOffset({0, 0})
           .setExtent(extent);
    commandBuffer.setScissor(0, scissor);

    constexpr vk::DeviceSize offsets[] = { 0 };
    const auto& shaderManager = m_Geometry->shader();
    commandBuffer.bindVertexBuffers(0, 1, &m_Geometry->vertexBuffer(), offsets);
    commandBuffer.bindIndexBuffer(m_Geometry->indexBuffer(), 0, vk::IndexType::eUint32);

    PushConstants pc{};
    pc.view = m_Camera.getViewMatrix();
    pc.projection = glm::perspective(
        glm::radians(m_Camera.fov),
        static_cast<float>(extent.width) / static_cast<float>(extent.height),
        m_Camera.nearPlane, m_Camera.farPlane
    );
    pc.projection[1][1] *= -1; // Flip Y-axis for Vulkan
    pc.model = glm::mat4(1.0f);

    // Draw grid (lines)
    const auto& wrappedSet = m_ResourceManager->wrappedDescriptorSet(ResourceTypes::GRAPHICS);
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, wrappedSet.descPipeline);
    commandBuffer.pushConstants(wrappedSet.descPipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &pc);
    if ( !m_MassiveObjects.empty() )
    {
        const auto& firstObject = m_MassiveObjects.front();
        commandBuffer.drawIndexed((*firstObject)->indexCount, 1, (*firstObject)->indexOffset, 0, 0);
    }

    // Draw all shapes (triangles)
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_TrianglePipeline);
    for ( const auto& shape : m_MassiveObjects )
    {
        const Object& obj = shape->m_Object;
        pc.model = obj.modelMatrix;
        commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &pc);
        commandBuffer.drawIndexed(obj.indexCount, 1, obj.indexOffset, 0, 0);
    }

    if ( !m_TrailVertices.empty() && !m_TrailIndices.empty() )
    {
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, m_GraphicsPipeline); // Use the line pipeline
        commandBuffer.bindVertexBuffers(0, 1, &m_TrailVertexBuffer, offsets);
        commandBuffer.bindIndexBuffer(m_TrailIndexBuffer, 0, vk::IndexType::eUint32);

        PushConstants trail_constants{};
        trail_constants.view = m_Camera.getViewMatrix();
        trail_constants.projection = glm::perspective(
            glm::radians(m_Camera.fov),
            static_cast<float>(m_SwapchainExtent.width) / static_cast<float>(m_SwapchainExtent.height),
            m_Camera.nearPlane, m_Camera.farPlane
        );
        trail_constants.projection[1][1] *= -1; // Flip Y-axis for Vulkan
        trail_constants.model = glm::mat4(1.0f); // Trails are in world space
        commandBuffer.pushConstants(m_PipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(PushConstants), &trail_constants);

        commandBuffer.drawIndexed(static_cast<uint32_t>(m_TrailIndices.size()), 1, 0, 0, 0);
    }
}

void GridRenderer::updateCamera()
{
    const ImGuiIO& io = ImGui::GetIO();

    // Zoom with trackpad scroll
    if ( io.MouseWheel != 0.0f )
    {
        m_Camera.zoomLevel -= io.MouseWheel * 0.1f;
        m_Camera.zoomLevel = std::max(0.1f, std::min(m_Camera.zoomLevel, 30.0f));
    }

    // Adjust radius with zoom level
    m_Camera.radius = 30.0f * m_Camera.zoomLevel;
    m_Camera.radius = glm::clamp(m_Camera.radius, m_Camera.minRadius, m_Camera.maxRadius);

    // Orbiting: Rotate with left mouse button drag
    if ( ImGui::IsMouseDragging(ImGuiMouseButton_Left) )
    {
        const float deltaX = io.MouseDelta.x;
        const float deltaY = io.MouseDelta.y;

        m_Camera.azimuth -= deltaX * m_Camera.orbitSpeed;
        m_Camera.elevation -= deltaY * m_Camera.orbitSpeed;

        // Clamp elevation to avoid gimbal lock
        constexpr float minElevation = glm::radians(-89.9f);
        constexpr float maxElevation = glm::radians(89.9f);
        m_Camera.elevation = glm::clamp(m_Camera.elevation, minElevation, maxElevation);
    }

    // Panning: Move target with right mouse button drag or keyboard (WASD)
    if ( ImGui::IsMouseDragging(ImGuiMouseButton_Right) )
    {
        const float deltaX = io.MouseDelta.x;
        const float deltaY = io.MouseDelta.y;

        const glm::vec3 forward = glm::normalize(m_Camera.target - m_Camera.getPosition());
        const glm::vec3 right = glm::normalize(glm::cross(forward, m_Camera.up));
        const glm::vec3 up = glm::normalize(glm::cross(right, forward));

        m_Camera.target += right * (-deltaX * m_Camera.panSpeed);
        m_Camera.target += up * (deltaY * m_Camera.panSpeed);
    }

    // Keyboard panning (WASD)
    if ( !io.WantCaptureKeyboard )
    {
        const glm::vec3 forward = glm::normalize(m_Camera.target - m_Camera.getPosition());
        const glm::vec3 right = glm::normalize(glm::cross(forward, m_Camera.up));
        const glm::vec3 up = glm::normalize(glm::cross(right, forward));

        if ( ImGui::IsKeyDown(ImGuiKey_W) )
            m_Camera.target -= forward * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_S) )
            m_Camera.target += forward * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_A) )
            m_Camera.target -= right * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_D) )
            m_Camera.target += right * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_E) )
            m_Camera.target += up * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_Q) )
            m_Camera.target -= up * m_Camera.panSpeed;
    }

    static glm::vec3 lastPos = m_Camera.getPosition();
    if ( const glm::vec3 currentPos = m_Camera.getPosition(); glm::length(currentPos - lastPos) > 0.001f )
    {
        // std::cout << "Camera Position: (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.z << ")\n";
        lastPos = currentPos;
    }
}


