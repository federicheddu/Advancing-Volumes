cmake_minimum_required(VERSION 3.10)
project(Advancing-Volumes)
set(cinolib_DIR "/Users/federicomeloni/Documents/GitHub/cinolib")
find_package(CGAL REQUIRED)
set(CINOLIB_USES_OPENGL_GLFW_IMGUI ON)
set(CINOLIB_USES_SHEWCHUK_PREDICATES ON)
find_package(cinolib REQUIRED)
add_executable(${PROJECT_NAME} main.cpp
                advancing_data.h
                advancing_volumes.cpp advancing_volumes.h
                advancing_topological.cpp advancing_topological.h
                advancing_front.cpp advancing_front.h
                advancing_utility.cpp advancing_utility.h
                advancing_io.cpp advancing_io.h
                visualization.cpp visualization.h
                sphere.h sphere.cpp
                rationals.h rationals.cpp
                text_format.h
)
target_link_libraries(${CMAKE_PROJECT_NAME} cinolib CGAL::CGAL)