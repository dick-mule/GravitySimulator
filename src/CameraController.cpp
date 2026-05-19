//
// Orbit-camera input handling, controls UI, and view parameters.
//

#include "CameraController.hpp"

#include <imgui.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

void CameraController::update()
{
    const ImGuiIO& io = ImGui::GetIO();

    // Zoom with trackpad / wheel scroll.
    if ( io.MouseWheel != 0.0f )
    {
        m_ZoomLevel -= io.MouseWheel * 0.1f;
        m_ZoomLevel = std::max(0.1f, std::min(m_ZoomLevel, 30.0f));
    }
    m_Camera.radius = glm::clamp(30.0f * m_ZoomLevel, m_Camera.minRadius, m_Camera.maxRadius);

    // Orbit with left-button drag.
    if ( ImGui::IsMouseDragging(ImGuiMouseButton_Left) )
    {
        m_Camera.azimuth   -= io.MouseDelta.x * m_Camera.orbitSpeed;
        m_Camera.elevation -= io.MouseDelta.y * m_Camera.orbitSpeed;

        // Clamp elevation to avoid gimbal lock.
        constexpr float minElevation = glm::radians(-89.9f);
        constexpr float maxElevation = glm::radians(89.9f);
        m_Camera.elevation = glm::clamp(m_Camera.elevation, minElevation, maxElevation);
    }

    // Pan the target with right-button drag.
    if ( ImGui::IsMouseDragging(ImGuiMouseButton_Right) )
    {
        const glm::vec3 forward = glm::normalize(m_Camera.target - m_Camera.getPosition());
        const glm::vec3 right   = glm::normalize(glm::cross(forward, m_Camera.up));
        const glm::vec3 up      = glm::normalize(glm::cross(right, forward));

        m_Camera.target += right * (-io.MouseDelta.x * m_Camera.panSpeed);
        m_Camera.target += up    * ( io.MouseDelta.y * m_Camera.panSpeed);
    }

    // Pan the target with the keyboard (WASD + QE).
    if ( !io.WantCaptureKeyboard )
    {
        const glm::vec3 forward = glm::normalize(m_Camera.target - m_Camera.getPosition());
        const glm::vec3 right   = glm::normalize(glm::cross(forward, m_Camera.up));
        const glm::vec3 up      = glm::normalize(glm::cross(right, forward));

        if ( ImGui::IsKeyDown(ImGuiKey_W) ) m_Camera.target -= forward * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_S) ) m_Camera.target += forward * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_A) ) m_Camera.target -= right   * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_D) ) m_Camera.target += right   * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_E) ) m_Camera.target += up      * m_Camera.panSpeed;
        if ( ImGui::IsKeyDown(ImGuiKey_Q) ) m_Camera.target -= up      * m_Camera.panSpeed;
    }
}

void CameraController::renderControls()
{
    glm::vec3 pos = m_Camera.getPosition();
    ImGui::InputFloat3("Camera Position", glm::value_ptr(pos), "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat3("Target", glm::value_ptr(m_Camera.target), "%.3f");

    float azimuthDegrees   = glm::degrees(m_Camera.azimuth);
    float elevationDegrees = glm::degrees(m_Camera.elevation);
    ImGui::DragFloat("Azimuth", &azimuthDegrees, 1.0f, -180.0f, 180.0f, "%.1f degrees");
    ImGui::DragFloat("Elevation", &elevationDegrees, 1.0f, -89.0f, 89.0f, "%.1f degrees");
    ImGui::DragFloat("Radius", &m_Camera.radius, 1.0f, m_Camera.minRadius, m_Camera.maxRadius, "%.1f");
    m_Camera.azimuth   = glm::radians(azimuthDegrees);
    m_Camera.elevation = glm::radians(elevationDegrees);

    ImGui::DragFloat("FOV", &m_Camera.fov, 1.0f, 10.0f, 120.0f, "%.1f degrees");
    // Near/far clip planes are auto-fitted to the scene each frame (see
    // GridRenderer::draw) — no manual sliders, so they can't clip the grid.

    ImGui::DragFloat("Orbit Speed", &m_Camera.orbitSpeed, 0.001f, 0.001f, 0.01f, "%.4f");
    ImGui::DragFloat("Zoom Speed", &m_Camera.zoomSpeed, 0.1f, 0.1f, 5.0f, "%.1f");
    ImGui::DragFloat("Pan Speed", &m_Camera.panSpeed, 0.01f, 0.01f, 1.0f, "%.2f");
}

void CameraController::reset(bool sphericalView, float gridScale)
{
    m_Camera.target     = glm::vec3(0.0f, 0.0f, 0.0f);
    m_Camera.up         = glm::vec3(0.0f, 1.0f, 0.0f);
    m_Camera.fov        = 90.0f;
    m_Camera.azimuth    = glm::radians(45.0f);
    m_Camera.elevation  = glm::radians(30.0f);
    m_Camera.panSpeed   = 0.1f;
    m_Camera.orbitSpeed = 0.005f;
    m_Camera.zoomSpeed  = 1.0f;

    // Zoom limits scale with the grid so every geometry frames correctly. The
    // camera radius is derived as 30 * zoomLevel each frame (see update()), so
    // we pick a zoom level rather than setting the radius directly.
    m_Camera.minRadius = gridScale * 0.05f;
    m_Camera.maxRadius = gridScale * 4.0f;
    const float viewDist = (sphericalView ? 1.6f : 0.6f) * gridScale;
    m_ZoomLevel = viewDist / 30.0f;
}
