cmake_minimum_required(VERSION 3.1)
project(gp21-project)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)

# use this if you want to keep assert statements 
# for improved debugging while still having performance
#
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"     ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"    ON)

find_package(LIBIGL REQUIRED QUIET)

# Add your project files
FILE(GLOB SRCFILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui)

# Note This option is implemented only by Makefile Generators and the Ninja. It is ignored on other generators.
# If this still doesnt work, try adding it manually to the cmake command as an option like 
#
# cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ../
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# message("CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
# message("CMAKE_CXX_FLAGS_RELEASE = ${CMAKE_CXX_FLAGS_RELEASE}")