//
// Created by eugen on 23.03.17.
//
#include "gtest/gtest.h"
#include <glog/logging.h>

#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <boost/bind.hpp>

#include "ImuFactorTesting.h"
#include "../src/csv.h"
#include <csv.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace testing {
// Create default parameters with Z-down and above noise parameters
    static boost::shared_ptr<PreintegrationParams> Params() {
        auto p = PreintegrationParams::MakeSharedD(kGravity);
        p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
        p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
        p->integrationCovariance = 0.002 * I_3x3;
        return p;
    }
}


//for a test about how to use the preintegration, see the test
// https://bitbucket.org/gtborg/gtsam/src/fbb9d3bdda8b88df51896bc401bfd170573e66f5/gtsam/navigation/tests/testTangentPreintegration.cpp?at=develop&fileviewer=file-view-default

TEST(ManifoldPreintegration, computePose)
{
    ManifoldPreintegration pim(testing::Params());
    NavState x1, x2;
    imuBias::ConstantBias bias;
    Matrix9 aH1, aH2;
    Matrix96 aH3;

    uint64_t t;
    Vector3 acc;
    Vector3 omega;

    CSVReader csvreader;
    csvreader.read("/home/eugen/datasets/EuRec/mav0/imu0/data.csv");
    CSVReader::LinesType lines=csvreader.lines;

    Pose3 prior_pose; //identity, t=0
    Vector3 prior_velocity;
    prior_velocity.setZero();
    NavState prev_state(prior_pose, prior_velocity);
    int off=8803;
    uint64_t last_t=std::atoll(lines[off].elements[0].c_str());
    for (int i=off; i<9203; i++) //until time 1403636590008555520
    {
        t=std::atoll(lines[i].elements[0].c_str());
        for (int ai=4; ai<7; ai++)
        {
            acc(ai-4)=std::atof(lines[i].elements[ai].c_str());
        }
        for (int wi=1; wi<4; wi++)
        {
            omega(wi-1)=std::atof(lines[i].elements[wi].c_str());
        }

        double dt=(t-last_t)/1e9;
        pim.integrateMeasurement(acc, omega, dt);
        last_t=t;
        //pim.update()

    }
    //the gt pose from the leica ds:
    CSVReader leica;
    leica.read("/home/eugen/datasets/EuRec/mav0/leica0/data.csv");
    Vector3 p0(std::atof(leica.lines[702].elements[1].c_str()),
               std::atof(leica.lines[702].elements[2].c_str()),
               std::atof(leica.lines[702].elements[3].c_str())
    );
    Vector3 p1(std::atof(leica.lines[735].elements[1].c_str()),
               std::atof(leica.lines[735].elements[2].c_str()),
               std::atof(leica.lines[735].elements[3].c_str())
    );

    std::cout << t <<  "ns --  pose: " <<  pim.predict(prev_state, bias) << std::endl << std::flush;
    std::cout << std::atoll(leica.lines[135].elements[1].c_str()) << "ns - GT position: " << (p1-p0).transpose();






}