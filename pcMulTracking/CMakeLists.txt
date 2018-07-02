cmake_minimum_required (VERSION 2.6)
project (pcMulTracking)

set (CMAKE_CXX_STANDARD 14)
find_package(PCL 1.3  REQUIRED)
find_package(PythonLibs 2.7)
include_directories(src) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")
add_executable(run  ${sourcefiles})

include_directories(${PCL_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(run  ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})





