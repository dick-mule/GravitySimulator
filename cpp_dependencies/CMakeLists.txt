cmake_minimum_required(VERSION 3.30)
project(cpp_dependencies LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

include_directories(.)

add_subdirectory(magic_enum)

set(DEPENDENCY_SOURCES
    cpp_dependencies.cpp
)

set(DEPENDENCY_HEADERS
    cpp_dependencies.hpp
)

add_library(cpp_dependencies
        SHARED
        ${DEPENDENCY_SOURCES}
        ${DEPENDENCY_HEADERS})
set_target_properties(cpp_dependencies PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(cpp_dependencies magic_enum::magic_enum)

install(TARGETS cpp_dependencies)
