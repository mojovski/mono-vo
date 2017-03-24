cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME "mono-vo")
PROJECT(${PROJECTNAME} CXX C)
project( ${PROJECTNAME} )
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(EIGEN REQUIRED)
find_package(GTSAM REQUIRED)
MESSAGE(STATUS "Found GTSAM on GTSAM_INCLUDE_DIR ${GTSAM_INCLUDE_DIR}")
find_package( OpenCV REQUIRED )

include_directories( ${CMAKE_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

file(GLOB viso
    "src/*.h"
    "src/visodo.cpp"
)
add_executable( vo ${viso} )
target_link_libraries( vo ${OpenCV_LIBS} )



#EuRoC data reader
file(GLOB viso
    "src/*.h"
    "src/mono-vo-euroc.cpp"
)
add_executable( euroc-vo ${viso} src/GravityBiasSeparator.cpp src/GravityBiasSeparator.h)
target_link_libraries( euroc-vo ${OpenCV_LIBS} )


#----------------------------
#	Unit tests
#----------------------------
add_subdirectory(./external/googletest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_subdirectory(tests)