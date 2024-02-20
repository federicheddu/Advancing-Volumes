cmake_minimum_required(VERSION 3.10)

project(Advancing-Volumes)

set(cinolib_DIR "/Users/federicomeloni/Documents/GitHub/cinolib")
set(CINOLIB_USES_OPENGL_GLFW_IMGUI ON)
set(CINOLIB_USES_SHEWCHUK_PREDICATES ON)

find_package(cinolib REQUIRED)
find_package(CGAL REQUIRED)

add_executable(
        ${PROJECT_NAME} main.cpp
        advancing_io.h advancing_io.cpp
        advancing_data.h advancing_data.cpp
        advancing_volumes.h advancing_volumes.cpp
        advancing_rationals.cpp advancing_rationals.h
        ANSI_color_codes.h
        advancing_commands.cpp
        advancing_commands.h
)

target_link_libraries(${CMAKE_PROJECT_NAME} cinolib CGAL::CGAL)
