ADD_DEFINITIONS(-DTEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

include(Testing) #load from the CMakeModules directory

SetupTest(imuPreIntegration "${OpenCV_LIBS};${GTSAM_LIBRARIES}")