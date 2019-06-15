//
// Created by liwenqiang on 6/13/19.
//

#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>


int main(int argc, char** argv)
{
    using namespace std;

    // initial value
    Eigen::Matrix3d baseR = Eigen::Matrix3d::Identity();
    cerr << "base Rotation maxtrix : " <<endl<<baseR << endl ;
    Sophus::Quaterniond baseQ = Sophus::Quaterniond(baseR);
    cerr << "base quaterniod construct from base rotation maxtrix: " <<endl<<baseQ.coeffs().transpose() << endl << endl;
    cerr<<"-----------------------------"<<endl;

    // R multiply exp(w^)
    Eigen::Vector3d w(0.01, 0.02, 0.03);
    Sophus::SO3 updateSo3 = Sophus::SO3::exp(w);
    Sophus::SO3 multipliedSO3(baseR * updateSo3.matrix());
    cerr << "SO3 right multiplied corresponding matrix: " <<endl<<multipliedSO3.matrix()<< endl;
    cerr << "SO3 right multiplied corresponding so3: " <<endl<<multipliedSO3 << endl << endl;
    cerr<<"-----------------------------"<<endl;

    // quaterniond multiply quaterniod[1, w/2]
    Eigen::Vector4d udate(1,w[0] * .5,w[1] * .5,w[2] * .5);
    udate.normalize();
    Sophus::Quaterniond udateQ(udate[0],udate[1],udate[2],udate[3]);
    Sophus::Quaterniond multipliedQ = baseQ * udateQ;
    cerr << "quaterniod right multiplied corresponding result matrix: " <<endl<<multipliedQ.matrix() << endl;
    cerr << "quaterniod right multiplied corresponding so3: " <<endl<<Sophus::SO3(multipliedQ) << endl << endl;
    cerr<<"-----------------------------"<<endl;

    // computer norm of difference between two updated matrix
    double diff = (multipliedQ.matrix() - multipliedSO3.matrix()).norm();
    cerr<<"difference between two updated matrix: "<<diff << endl << endl;
    // computer norm of difference between two rotation vector
    Eigen::Vector3d diffVector = multipliedSO3.log()-(Sophus::SO3(multipliedQ)).log();
    cerr<<"difference between two updated rotation vector:"<<diffVector.norm()<<endl;
    return 0;
}

