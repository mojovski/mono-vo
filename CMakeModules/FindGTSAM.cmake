include("${CMAKE_SOURCE_DIR}/external/gtsam/_build/GTSAMConfig.cmake")
SET(GTSAM_INCLUDE_DIR "${GTSAM_INCLUDE_DIR};${CMAKE_SOURCE_DIR}/external/gtsam/_build")
SET(GTSAM_INCLUDE_DIR "${GTSAM_INCLUDE_DIR};${CMAKE_SOURCE_DIR}/external/gtsam/gtsam/3rdparty/Eigen")
SET(GTSAM_LIBRARIES "gtsam")
