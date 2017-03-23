include(Testing) #load from the CMakeModules directory

SetupTest(imuPreIntegration "${OpenCV_LIBS};${GTSAM_LIBRARIES}")