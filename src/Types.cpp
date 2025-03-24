//
// Created by Richard Mule on 3/23/25.
//

#include "Types.hpp"


vk::VertexInputBindingDescription Vertex::getBindingDescription()
{
    vk::VertexInputBindingDescription bindingDescription{};
    bindingDescription.setBinding(0)
                      .setStride(sizeof(Vertex))
                      .setInputRate(vk::VertexInputRate::eVertex);
    return bindingDescription;
}

std::array<vk::VertexInputAttributeDescription, 2> Vertex::getAttributeDescriptions()
{
    std::array<vk::VertexInputAttributeDescription, 2> attributeDescriptions{};
    attributeDescriptions[0].setBinding(0)
                            .setLocation(0)
                            .setFormat(vk::Format::eR32G32B32Sfloat)
                            .setOffset(offsetof(Vertex, position));
    attributeDescriptions[1].setBinding(0)
                            .setLocation(1)
                            .setFormat(vk::Format::eR32G32B32Sfloat)
                            .setOffset(offsetof(Vertex, color));
    return attributeDescriptions;
}

glm::vec3 Camera::getPosition() const
{
    // Convert spherical coordinates to Cartesian coordinates
    const float x = radius * cos(elevation) * cos(azimuth);
    const float z = radius * cos(elevation) * sin(azimuth);
    const float y = radius * sin(elevation);
    return target + glm::vec3(x, y, z);
}

glm::mat4 Camera::getViewMatrix() const
{
    return glm::lookAt(getPosition(), target, up);
}

std::atomic<int> Shape::s_NumShapes = 0;

Shape::Shape(const Object& object)
    : m_Object(object)
    , m_MyId(-1)
{
    m_MyId = ++s_NumShapes;
}

Shape::~Shape()
{
    --s_NumShapes;
}

void Cube::addVertices(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices)
{
    uint32_t cubeVertexOffset = static_cast<uint32_t>(vertices.size());
    std::array<glm::vec3, 8> cubePositions = {
        glm::vec3(-m_CubeSize / 2, -m_CubeSize / 2, -m_CubeSize / 2), // 0: Bottom-left-front
        glm::vec3(m_CubeSize / 2, -m_CubeSize / 2, -m_CubeSize / 2),  // 1: Bottom-right-front
        glm::vec3(m_CubeSize / 2, m_CubeSize / 2, -m_CubeSize / 2),   // 2: Top-right-front
        glm::vec3(-m_CubeSize / 2, m_CubeSize / 2, -m_CubeSize / 2),  // 3: Top-left-front
        glm::vec3(-m_CubeSize / 2, -m_CubeSize / 2, m_CubeSize / 2),  // 4: Bottom-left-back
        glm::vec3(m_CubeSize / 2, -m_CubeSize / 2, m_CubeSize / 2),   // 5: Bottom-right-back
        glm::vec3(m_CubeSize / 2, m_CubeSize / 2, m_CubeSize / 2),    // 6: Top-right-back
        glm::vec3(-m_CubeSize / 2, m_CubeSize / 2, m_CubeSize / 2)    // 7: Top-left-back
    };

    // Add cube indices (2 triangles per face, 6 faces)
    uint32_t cubeIndexOffset = static_cast<uint32_t>(indices.size());
    std::array<glm::vec3, 6> faceColors = {
        glm::vec3{1.0f, 0.0f, 0.0f}, // Front: Red
        glm::vec3{0.0f, 1.0f, 0.0f}, // Back: Green
        glm::vec3{0.0f, 0.0f, 1.0f}, // Left: Blue
        glm::vec3{1.0f, 1.0f, 0.0f}, // Right: Yellow
        glm::vec3{1.0f, 0.0f, 1.0f}, // Bottom: Magenta
        glm::vec3{0.0f, 1.0f, 1.0f}  // Top: Cyan
    };
    std::array<std::array<uint32_t, 4>, 6> faceVerts = {{
        {0, 1, 2, 3}, // Front
        {5, 4, 7, 6}, // Back
        {0, 4, 7, 3}, // Left
        {5, 1, 2, 6}, // Right
        {4, 5, 1, 0}, // Bottom
        {3, 2, 6, 7}  // Top
    }};
    for ( int face = 0; face < 6; ++face )
    {
        for ( int i = 0; i < 4; ++i )
        {
            Vertex vertex{};
            vertex.position = cubePositions[faceVerts[face][i]];
            vertex.color = faceColors[0];
            vertex.normal = glm::vec3(0.0f, 0.0f, 0.0f);
            vertices.push_back(vertex);
        }
        const uint32_t base = cubeVertexOffset + face * 4;
        indices.push_back(base + 0);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
        indices.push_back(base + 0);
        indices.push_back(base + 2);
        indices.push_back(base + 3);
    }
    m_Object.indexOffset = cubeIndexOffset;
    m_Object.indexCount = 36;
    // m_Object.modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)); // Center of simulation
    // m_Object.velocity = glm::vec3(0.0f, 0.0f, 0.0f); // Stationary for simplicity
}

void Sphere::addVertices(std::vector<Vertex>& vertices, std::vector<uint32_t>& indices)
{
    const uint32_t sphereVertexOffset = static_cast<uint32_t>(vertices.size());
    // Generate sphere vertices
    for ( int i = 0; i <= m_Stacks; ++i )
    {
        const float phi = glm::pi<float>() * static_cast<float>(i) / m_Stacks;
        const float sinPhi = sin(phi);
        const float cosPhi = cos(phi);

        for ( int j = 0; j <= m_Slices; ++j )
        {
            const float theta = 2.0f * glm::pi<float>() * static_cast<float>(j) / m_Slices;
            const float sinTheta = sin(theta);
            const float cosTheta = cos(theta);

            Vertex vertex{};
            vertex.position = glm::vec3(
                m_Radius * sinPhi * cosTheta,
                m_Radius * cosPhi,
                m_Radius * sinPhi * sinTheta
            );
            vertex.color = glm::vec3(1.0f, 0.0f, 0.0f); // Red sphere
            vertex.normal = glm::normalize(vertex.position); // Normal is the position normalized
            vertices.push_back(vertex);
        }
    }

    // Generate sphere indices (triangles)
    const uint32_t sphereIndexOffset = static_cast<uint32_t>(indices.size());
    for ( int i = 0; i < m_Stacks; ++i )
    {
        for ( int j = 0; j < m_Slices; ++j )
        {
            uint32_t idx0 = sphereVertexOffset + i * (m_Slices + 1) + j;
            uint32_t idx1 = idx0 + 1;
            uint32_t idx2 = idx0 + (m_Slices + 1);
            uint32_t idx3 = idx2 + 1;

            indices.push_back(idx0);
            indices.push_back(idx2);
            indices.push_back(idx1);
            indices.push_back(idx1);
            indices.push_back(idx2);
            indices.push_back(idx3);
        }
    }
    m_Object.indexOffset = sphereIndexOffset;
    m_Object.indexCount = static_cast<uint32_t>(indices.size()) - sphereIndexOffset;
    // m_Object.modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 0.0f, 0.0f)); // Start 10 units right
    // m_Object.velocity = glm::vec3(0.0f, 0.0f, -0.5f); // Moving left
}
