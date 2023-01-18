/*
	FILE: kalman_filter.h
	--------------------------------------
	header of kalman_filter velocity estimator
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

class kalman_filter
{
    private:
    // members
    bool is_initialized;
    MatrixXd states;
    MatrixXd A; // state matrix
    MatrixXd B; // input matrix
    MatrixXd H; // observation matrix
    MatrixXd P; // uncertianty
    MatrixXd Q; // process noise
    MatrixXd R; // obsevation noise

    public:
    // constructor
    kalman_filter();

    // set up the filter
    void setup( MatrixXd states,
                MatrixXd A,
                MatrixXd B,
                MatrixXd H,
                MatrixXd P,
                MatrixXd Q,
                MatrixXd R);

    // set A (sometimes sampling time will differ)
    void setA(MatrixXd A);

    // state estimate
    void estimate(MatrixXd z, MatrixXd u);

    // read output from the state
    double output(int state_index);
};

#endif