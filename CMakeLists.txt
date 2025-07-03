cmake_minimum_required(VERSION 3.10)
project(gravity_sim)

set(CMAKE_GENERATOR "Ninja")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_policy(SET CMP0072 NEW)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

# GLM headers
include_directories("/usr/include/glm")

set(SRC_FILES
    src/main.cpp
    src/simulation.cpp
    src/celestialBody.cpp
    src/octreeNode.cpp
)

set(INCLUDE_DIRS
    src/include
)

add_executable(gravity_sim ${SRC_FILES})

target_include_directories(gravity_sim PRIVATE
    ${INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

target_link_libraries(gravity_sim PRIVATE
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    GLEW::GLEW
)

target_compile_options(gravity_sim PRIVATE ${GLFW_CFLAGS_OTHER})

set_target_properties(gravity_sim PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
