//
// Created by eugen on 23.03.17.
//
#include "gtest/gtest.h"
#include <glog/logging.h>

#include <CSVReader.h>
#include <NNSearch.h>


//for a test about how to use the preintegration, see the test
// https://bitbucket.org/gtborg/gtsam/src/fbb9d3bdda8b88df51896bc401bfd170573e66f5/gtsam/navigation/tests/testTangentPreintegration.cpp?at=develop&fileviewer=file-view-default

TEST(Search, IndexAndSearchByNNTime)
{


    CSVReader imureader;
    //imureader.read("/home/eugen/datasets/EuRec/mav0/imu0/data.csv");
    std::string imufile=std::string(TEST_DIR)+std::string("/../datasets/eurec/MH1/imu0.csv");
    imureader.read(imufile.c_str());
    CSVReader::LinesType lines=imureader.lines;


    NNSearch<double> nnsearch;
    std::vector<double> times;
    for (int i=1; i<lines.size(); i++)
    {
        uint64_t t=std::atoll(lines[i].elements[0].c_str());
        double td=(double)t/(double)1e9;
        times.push_back(td);
    }
    nnsearch.buildDatasetFromTimeStamps(times);
    std::vector<int> res=nnsearch.search_by_time(times[3], 1);
    std::cout << "time searched: " << times[3] << " (idx:3), res index: " << res[0] << " with corresponding time: " << times[res[0]]
    << " total items: " << times.size()<< "\n";
    ASSERT_EQ(res[0],3);


}


