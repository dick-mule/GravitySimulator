cmake_minimum_required(VERSION 3.30)
project(GravitySimulator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Vulkan
find_package(Vulkan REQUIRED)

# Find GLFW
find_package(glfw3 3.3 REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src/imgui)
include_directories(${Vulkan_INCLUDE_DIRS})

set(SHADER_DIR ${CMAKE_SOURCE_DIR}/src/shaders)
set(SHADER_FILES
        ${SHADER_DIR}/grid.frag
        ${SHADER_DIR}/grid.vert
)

foreach(SHADER ${SHADER_FILES})
    get_filename_component(SHADER_NAME ${SHADER} NAME)
    set(SHADER_OUTPUT ${CMAKE_BINARY_DIR}/${SHADER_NAME}.spv)
    add_custom_command(
            OUTPUT ${SHADER_OUTPUT}
            COMMAND ${Vulkan_GLSLC_EXECUTABLE} ${SHADER} -o ${SHADER_OUTPUT}
            DEPENDS ${SHADER}
            COMMENT "Compiling shader ${SHADER_NAME}"
    )
    list(APPEND SHADER_OUTPUTS ${SHADER_OUTPUT})
endforeach()
add_custom_target(Shaders DEPENDS ${SHADER_OUTPUTS})


# Source files
set(SOURCES
        src/main.cpp
        src/imgui/imgui.cpp
        src/imgui/imgui_draw.cpp
        src/imgui/imgui_tables.cpp
        src/imgui/imgui_widgets.cpp
        src/imgui/imgui_demo.cpp
        src/imgui/backends/imgui_impl_glfw.cpp
        src/imgui/backends/imgui_impl_vulkan.cpp
        src/VulkanApp.cpp
        src/VulkanApp.hpp
        src/ImGuiHandler.cpp
        src/ImGuiHandler.hpp
        src/GridRenderer.cpp
        src/GridRenderer.hpp
        src/Types.cpp
        src/Types.hpp
        src/Geometry.cpp
        src/Geometry.hpp
)

# Create executable
add_executable(GravitySimulator ${SOURCES})
add_dependencies(GravitySimulator Shaders)
# Link libraries
target_link_libraries(GravitySimulator glfw Vulkan::Vulkan)
