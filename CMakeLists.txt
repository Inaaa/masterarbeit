cmake_minimum_required(VERSION 3.15)

project(masterarbeit)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(main  main.cpp filter.h filter.cpp surface.h surface.cpp visualization.h visualization.cpp range_image.cpp range_image.h preprocessing.cpp preprocessing.h CustomVoxelGrid.cpp CustomVoxelGrid.h)

target_link_libraries ( main ${PCL_LIBRARIES})



