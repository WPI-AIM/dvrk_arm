//
// Created by adnan on 5/11/18.
//
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;

int main()
{
    MatrixXd m = MatrixXd::Random(3,3);
    cout << "m before was " << endl << MatrixXd::Constant(3,3,1.2) << endl;
    m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
    cout << "m =" << endl << m.inverse() << endl;
    VectorXd v(3);
    v << 1, 2, 3;
    cout << "m * v =" << endl << m * v << endl;
}
