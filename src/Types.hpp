//
// Created by Richard Mule on 3/23/25.
//

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //translate, rotate, scale, perspective
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp> // Include the extension header
#include <vulkan/vulkan.hpp>
#include <string>

struct Vertex
{
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 normal;

    static vk::VertexInputBindingDescription getBindingDescription();
    static std::array<vk::VertexInputAttributeDescription, 2> getAttributeDescriptions();
};

struct Object
{
    uint32_t indexOffset;
    uint32_t indexCount;
    glm::mat4 modelMatrix;
    float mass = 0.0f;
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 velocity = glm::vec3(0.0f);
    glm::vec3 acceleration = glm::vec3(0.0f);
};


struct PushConstants
{
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;
};

struct Camera
{
    glm::vec3 position = glm::vec3(0.0f, 20.0f, 20.0f); // Camera at (0, 20, 20)
    glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);    // Look at (0, 0, 0)
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);        // Up vector (0, 1, 0)
    float fov = 90.0f;                                  // FOV in degrees
    float nearPlane = 0.1f;
    float farPlane = 500.0f;

    // Orbit control parameters
    float azimuth = glm::radians(45.0f);                // Angle in XZ-plane (radians)
    float elevation = glm::radians(45.0f);              // Angle above XZ-plane (radians)
    float radius = 30.0f;                               // Distance from target
    float minRadius = 0.01f;                             // Minimum zoom distance
    float maxRadius = 1000.0f;                           // Maximum zoom distance
    float panSpeed = 0.1f;                              // Panning speed
    float orbitSpeed = 0.005f;                          // Orbiting speed
    float zoomSpeed = 1.0f;                             // Zooming speed

    glm::vec3 getPosition() const;
    glm::mat4 getViewMatrix() const;
};

class Shape
{
public:
    Object m_Object;

protected:
    int m_MyId;
    static std::atomic<int> s_NumShapes;
    float m_Size;

public:
    explicit Shape(const Object& object);
    virtual ~Shape();
    virtual std::string getName() const = 0;
    virtual void addVertices(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices) = 0;
    void setSize(float size) { m_Size = size; }
};

class Cube final : public Shape
{
protected:
    std::string m_Name = "Cube";

public:
    using Shape::Shape;
    ~Cube() override = default;
    std::string getName() const override { return "Cube: " + std::to_string(m_MyId - 1); }
    void addVertices(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices) override;
};

class Sphere final : public Shape
{
protected:
    int m_Stacks = 20;
    int m_Slices = 20;
    std::string m_Name = "Sphere";

public:
    using Shape::Shape;
    ~Sphere() override = default;
    std::string getName() const override { return "Sphere: " + std::to_string(m_MyId - 1); }
    void addVertices(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices) override;
};
