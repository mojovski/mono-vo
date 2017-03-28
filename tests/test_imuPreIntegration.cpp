//
// Created by eugen on 23.03.17.
//
#include "gtest/gtest.h"
#include <glog/logging.h>

#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <boost/bind.hpp>

#include "ImuFactorTesting.h"
#include <CSVReader.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <GravityBiasSeparator.h>


namespace testing {
// Create default parameters with Z-down and above noise parameters
    static boost::shared_ptr<PreintegrationParams> Params() {
        auto p = PreintegrationParams::MakeSharedD(kGravity);
        //see noise param estimation http://www.nxp.com/assets/documents/data/en/application-notes/AN5087.pdf
        p->gyroscopeCovariance = kGyroSigma *  I_3x3;
        p->accelerometerCovariance = kAccelSigma *  I_3x3;
        p->integrationCovariance = 0.0002 * I_3x3;
        return p;
    }
}

void estimateConstBiases(CSVReader& reader, uint64_t t0, uint64_t t1, Vector3& g, Vector3& g_bias, Matrix33& g_cov,
Vector3& w, Matrix33& w_cov, bool do_refinement=false);


//for a test about how to use the preintegration, see the test
// https://bitbucket.org/gtborg/gtsam/src/fbb9d3bdda8b88df51896bc401bfd170573e66f5/gtsam/navigation/tests/testTangentPreintegration.cpp?at=develop&fileviewer=file-view-default

TEST(ManifoldPreintegration, computePose)
{


    uint64_t t;
    Vector3 acc;
    Vector3 omega;

    CSVReader imureader;
    //imureader.read("/home/eugen/datasets/EuRec/mav0/imu0/data.csv");
    std::string imufile=std::string(TEST_DIR)+std::string("/../datasets/eurec/MH1/imu0.csv");
    imureader.read(imufile.c_str());
    CSVReader::LinesType lines=imureader.lines;

    //estimate the noise and  gravity vector during the zero-movement
    Matrix33 g_cov, w_cov;
    Vector3 g, acc_bias, w_bias;

    bool do_refinement=true;
    estimateConstBiases(imureader, 1403636601363555584, 1403636610863555584, g, acc_bias, g_cov, w_bias, w_cov, do_refinement);
    std::cout << "\ng: "<< g <<
              "\nacc_bias: " << acc_bias <<
        "\nw: " << w_bias << "\ng_cov:\n" << g_cov << "\nw_cov: " << w_cov << "\n\n" << std::endl;
    //g=g+acc_bias;
    //Matrix33 acc_cov; acc_cov.setZero();
    //for (int i=0; i<3; i++)
    //    acc_cov(i,i)=std::sqrt(cov(i,i));

    /*
     * boost::shared_ptr<PreintegrationParams> params=new PreintegrationParams(g);
    params->gyroscopeCovariance = kGyroSigma *  I_3x3;
    params->accelerometerCovariance = kAccelSigma *  I_3x3;
    params->integrationCovariance = 0.0002 * I_3x3;
     */

    boost::shared_ptr<PreintegrationParams> params=testing::Params();
    //params->n_gravity=g;
    //params->accelerometerCovariance=g_cov;
    //params->gyroscopeCovariance=w_cov;

    ManifoldPreintegration pim(params);
    //TangentPreintegration pim(params);
    NavState x1, x2;
    Vector3 nullbias(0,0,0);
    imuBias::ConstantBias bias(acc_bias, w_bias);

    Pose3 prior_pose; //identity, t=0
    Vector3 prior_velocity;
    prior_velocity.setZero();
    NavState prev_state(prior_pose, prior_velocity);
    int off=8804-1; //t: 1403636623763555584
    uint64_t last_t=std::atoll(lines[off-1].elements[0].c_str());
    //9031 / 1403636624903555584
    //9204 / 1403636625768555520
    for (int i=off; i<9204-1; i++) //until time 1403636625763555584
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
        //acc=acc-acc_bias; use the constBiasClass for this: ConstantBias(acc_bias, w_bias)
        //rotate the gravity vector by the current pose rotation
        NavState pose=pim.predict(prev_state, bias);
        //std::cout << "Integrated pose: " << pose;
        Vector3 current_g=pose.R().transpose()*g;
        //TODO: The estimated g vector is a mixture of g and noise. Estimate both separately....
        Vector3 acc_without_g=acc-current_g;
        //omega=omega-w_bias;
        //std::cout << "\n " << t << " - \nacc: \n" << acc << "\nomega:\n" << omega << "\n";
        std::cout << acc_without_g(2) << "\t";
        pim.integrateMeasurement(acc_without_g, omega, dt);
        //pim.integrateMeasurement(acc, omega, dt);
        last_t=t;
        //pim.update()

    }
    std::cout <<"\n";
    //the gt pose from the leica ds:
    CSVReader leica;
    //leica.read("/home/eugen/datasets/EuRec/mav0/leica0/data.csv");
    std::string leicafile=std::string(TEST_DIR)+std::string("/../datasets/eurec/MH1/leica0.csv");
    leica.read(leicafile.c_str());
    Vector3 p0(std::atof(leica.lines[702].elements[1].c_str()),
               std::atof(leica.lines[702].elements[2].c_str()),
               std::atof(leica.lines[702].elements[3].c_str())
    );
    //722 / 1403636624970881280
    //735 / 1403636625769881344
    int end_id=735-1;
    Vector3 p1(std::atof(leica.lines[end_id].elements[1].c_str()),
               std::atof(leica.lines[end_id].elements[2].c_str()),
               std::atof(leica.lines[end_id].elements[3].c_str())
    );


    std::cout << t <<  "ns --  pose: " <<  pim.predict(prev_state, bias) << std::endl << "\nbias: " << bias << "\n" <<std::flush;
    std::cout << std::atoll(leica.lines[end_id].elements[0].c_str()) << "ns - GT position: " << (p1-p0).transpose()
            << "\n";

}



void estimateConstBiases(CSVReader& reader, uint64_t t0, uint64_t t1,
                         Vector3& g, Vector3& g_bias, Matrix33& g_cov,
                            Vector3& w, Matrix33& w_cov, bool do_refinement)
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

    //separate the acc to g and bias

    gtsam::GravityBiasSeparator::separate(accs, g, g_bias, g_cov);



    //extract mean and cov for omega-----------------
    double n=(double)omegas.size();
    w.setZero();
    w_cov.setZero();
    for (int i=0; i<omegas.size(); i++)
    {
        w=w+omegas[i]/n;
    }
    //estimate also the cov
    w_cov.setZero();
    for (int i=0; i<accs.size(); i++)
    {
        w_cov=w_cov+(w-omegas[i])*((w-omegas[i]).transpose())/n;
    }



}

