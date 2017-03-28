ADD_DEFINITIONS(-DPERF_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
enable_testing()
include(Perf)


include_directories( ${FLANN_INCLUDE_DIRS})
SetupPerformanceTest(imupreintegration "${OpenCV_LIBS};${GTSAM_LIBRARIES};${FLANN_FLANN_LIBRARIES}")

