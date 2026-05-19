//
// Orbit-camera input handling, controls UI, and view parameters.
//

#pragma once

#include "Types.hpp" // Camera

// Owns the orbit camera that was previously embedded in GridRenderer:
// per-frame mouse/keyboard input, the camera section of the controls panel,
// and the view/projection parameters used when recording draw commands.
class CameraController
{
public:
    // Applies this frame's mouse / keyboard input (read through ImGui IO).
    void update();

    // Renders the camera widgets into the current ImGui window
    // (the caller owns Begin/End).
    void renderControls();

    // Restores a default view. `sphericalView` pulls the camera far enough
    // out to frame the whole sphere.
    void reset(bool sphericalView, float gridScale);

    [[nodiscard]] glm::mat4 viewMatrix()  const { return m_Camera.getViewMatrix(); }
    [[nodiscard]] glm::vec3 eyePosition() const { return m_Camera.getPosition(); }
    [[nodiscard]] float     fov()         const { return m_Camera.fov; }

private:
    Camera m_Camera;
    float  m_ZoomLevel = 0.0f;
};
