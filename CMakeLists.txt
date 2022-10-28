cmake_minimum_required(VERSION 3.1.0)

project(ground_segmentation)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE "Release")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(PCL 1.12 REQUIRED)
find_package(OpenCV REQUIRED )


include_directories(
    ${PCL_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/iros/include
)
aux_source_directory(src SOURCE_LIST)
aux_source_directory(${PROJECT_SOURCE_DIR}/iros/src SOURCE_LIST)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ground_segmentation main.cpp ${SOURCE_LIST})
target_link_libraries (ground_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} -lpthread)