//
// Created by eugen on 24.03.17.
//

#ifndef MONO_VO_GRAVITYBIASSEPARATOR_H
#define MONO_VO_GRAVITYBIASSEPARATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <Eigen/SVD>

namespace gtsam{
    namespace GravityBiasSeparator
    {
        //skew-symmetric cross-product matrix of v
        Matrix33 rodrigues(Vector3& v)
        {
            Matrix33 res; res.setZero();
            res(0,1)=-v(2);
            res(0,2)=v(1);
            res(1,0)=v(2);
            res(2,0)=-v(1);
            res(2,1)=v(0);
            res(1,2)=-v(0);
            return res;
        }
        void separate(std::vector<Vector3>& accs, Vector3& g,
                      Vector3& g_bias,
                      Matrix33& g_cov)
        {
            Vector3 g_gt(9.81, 0,0);

            Matrix33  C; C.setIdentity();

            //compute mean values
            Vector3 mean1(0,0,0);
            double n1=(double)accs.size();
            for (int i=0; i<accs.size(); i++)
            {
                mean1=mean1+accs[i]/n1;
            }

            //experiment on http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
            Vector3 a=g_gt;
            a.normalize();
            Vector3 b=mean1;
            b.normalize();
            Vector3 v=a.cross(b);
            double s=v.norm();
            double c=a.dot(b);
            Matrix33 R=Matrix33::Identity()+rodrigues(v)+rodrigues(v)*rodrigues(v)*(1.0-c)/(s*s);
            g_bias=mean1-R*g_gt;
            double det=R.determinant();
            std::cout << "acc translation bias: " << g_bias.transpose() << "\ndet(R):" << det;

            g=R*g_gt;

            //estimate the bias
            std::vector<Vector3> accs_no_g;
            Vector3 acc_mean(0,0,0);
            double na=(double)accs.size();
            for (int i=0; i<accs.size(); i++)
            {
                Vector3 acc=accs[i]-g_bias;
                Vector3 acc_no_g= acc - g;
                accs_no_g.push_back(acc_no_g);
                acc_mean=acc_mean+acc_no_g/na;
            }
            //estimate the cov
            g_cov.setZero();
            for (int i=0; i<accs_no_g.size(); i++)
            {
                g_cov=g_cov+(acc_mean-accs_no_g[i])*((acc_mean-accs_no_g[i]).transpose())/n1;
            }

        }
    };
}


#endif //MONO_VO_GRAVITYBIASSEPARATOR_H
