MESSAGE(STATUS "\n-----------INCLUDING FLANN-------------------")
SET(PC_FLANN_INCLUDEDIR ${PROJECT_SOURCE_DIR}/external/flann/src/cpp)
MESSAGE(STATUS "Setting PC_FLANN_INCLUDEDIR to ${PC_FLANN_INCLUDEDIR}")

INCLUDE("${PROJECT_SOURCE_DIR}/external/flann/cmake/FindFlann.cmake")