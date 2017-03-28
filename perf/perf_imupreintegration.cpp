/*
this performance script evaluates the quality of the
IMU pre integration with respect
to the ground truth data in the EuReC dataset.
*/

#include "gtest/gtest.h"
#include <glog/logging.h>
#include <CSVReader.h>

TEST(ManifoldPreintegration, evaluateDrift)
{
	CSVReader imureader;
    std::string imufile=std::string(PERF_DIR)+std::string("/../datasets/eurec/MH1/imu0.csv");
    imureader.read(imufile.c_str());
    CSVReader::LinesType lines=imureader.lines;

    CSVReader leica;
    std::string leicafile=std::string(PERF_DIR)+std::string("/../datasets/eurec/MH1/leica0.csv");
    leica.read(leicafile.c_str());


    
	    



}