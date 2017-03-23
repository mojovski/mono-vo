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
        //see noise param estimation http://www.nxp.com/assets/documents/data/en/application-notes/AN5087.pdf
        p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
        p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
        p->integrationCovariance = 0.002 * I_3x3;
        return p;
    }
}

Vector3 estimateGravityVector(CSVReader& reader, uint64_t t0, uint64_t t1, Matrix33& g_cov)
{
    //collect the acc and omega data
    std::vector<Vector3> accs, omegas;
    CSVReader::LinesType lines=reader.lines;
    for (int i=1; i<lines.size(); i++)
    {
        uint64_t t=std::atoll(lines[i].elements[0].c_str());
        if ((t>=t0) && (t<=t1))
        {
            Vector3 acc, omega;
            for (int ai=4; ai<7; ai++)
            {
                acc(ai-4)=std::atof(lines[i].elements[ai].c_str());
            }
            for (int wi=1; wi<4; wi++)
            {
                omega(wi-1)=std::atof(lines[i].elements[wi].c_str());
            }
            accs.push_back(acc);
            omegas.push_back(omega);
        }
        if (t>t1)
            break;
    }
    //extract the mean vector
    Vector3 g; g.setZero();
    double n=(double)accs.size();
    for (int i=0; i<accs.size(); i++)
    {
        g=g+accs[i]/n;
    }
    //estimate also the cov
    g_cov.setZero();
    for (int i=0; i<accs.size(); i++)
    {
        g_cov=(g-accs[i])*((g-accs[i]).transpose())/n;
    }
    /*for (int i=0; i<3; i++)
    {
        g_cov(i,i)=std::sqrt(g_cov(i,i));
    }*/
    std::cout << "\ncov: " << g_cov<< "\n";

    //remove all accs, which are outside of 1*sigma
    std::vector<Vector3> filtered_accs;
    for (int i=0; i<accs.size(); i++)
    {
        Vector3 diff=accs[i]-g;
        bool ok=true;
        for (int j=0; j<3; j++) {
            ok = ok && std::abs(diff(j)) < std::sqrt(g_cov(j, j))*3.0;
        }
        if (ok)
        {
            filtered_accs.push_back(accs[i]);
        }
    }
    //re estimate the mean

    n=(double)filtered_accs.size();
    if (n<10)
    {
        return g;
    }

    Vector3 g_filtered; g_filtered.setZero();
    for (int i=0; i<filtered_accs.size(); i++)
    {
        g_filtered=g_filtered+filtered_accs[i]/n;
    }
    return g_filtered;

}


//for a test about how to use the preintegration, see the test
// https://bitbucket.org/gtborg/gtsam/src/fbb9d3bdda8b88df51896bc401bfd170573e66f5/gtsam/navigation/tests/testTangentPreintegration.cpp?at=develop&fileviewer=file-view-default

TEST(ManifoldPreintegration, computePose)
{


    uint64_t t;
    Vector3 acc;
    Vector3 omega;

    CSVReader imureader;
    imureader.read("/home/eugen/datasets/EuRec/mav0/imu0/data.csv");
    CSVReader::LinesType lines=imureader.lines;

    //estimate the noise and  gravity vector during the zero-movement
    Matrix33 cov;

    Vector3 g=estimateGravityVector(imureader, 1403636601363555584, 1403636610863555584, cov);
    //Matrix33 acc_cov; acc_cov.setZero();
    //for (int i=0; i<3; i++)
    //    acc_cov(i,i)=std::sqrt(cov(i,i));

    boost::shared_ptr<PreintegrationParams> params=testing::Params();
    params->accelerometerCovariance=cov;

    ManifoldPreintegration pim(params);
    NavState x1, x2;
    imuBias::ConstantBias bias;

    Pose3 prior_pose; //identity, t=0
    Vector3 prior_velocity;
    prior_velocity.setZero();
    NavState prev_state(prior_pose, prior_velocity);
    int off=8803;
    uint64_t last_t=std::atoll(lines[off-1].elements[0].c_str());
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
        //rotate the gravity vector by the current pose rotation
        NavState pose=pim.predict(prev_state, bias);
        //std::cout << "Integrated pose: " << pose;
        Vector3 current_g=pose.R().transpose()*g;
        Vector3 acc_without_g=acc-current_g;
        pim.integrateMeasurement(acc_without_g, omega, dt);
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
    std::cout << std::atoll(leica.lines[735].elements[0].c_str()) << "ns - GT position: " << (p1-p0).transpose();






}