cmake_minimum_required(VERSION 3.0)


project(SFM)

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})




add_definitions(${OpenCV_DEFINITIONS} )
add_definitions(${PCL_DEFINITIONS} )

add_executable (SFM main.cpp)
target_link_libraries(SFM ${OpenCV_LIBS})
target_link_libraries (SFM ${PCL_LIBRARIES})