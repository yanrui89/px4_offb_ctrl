#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <bspline_utils.hpp>

using namespace trajectory;

/**
 * @brief To test whether the M matrix provides a proper matrix 
 * 
 */

int main() {

    bspline_trajectory bt;

    Eigen::MatrixXd m3(4, 4);
    m3 <<   0.1667,    0.6667,    0.1667,         0,
            -0.5000,         0,    0.5000,         0,
            0.5000,   -1.0000,    0.5000,         0,
            -0.1667,    0.5000,   -0.5000,    0.1667;

    Eigen::MatrixXd m4(5, 5);
    m4 <<   0.0417,    0.4583,    0.4583,    0.0417,         0,
            -0.1667,   -0.5000,    0.5000,    0.1667,         0,
            0.2500,   -0.2500,   -0.2500,    0.2500,         0,
            -0.1667,    0.5000,   -0.5000,    0.1667,         0,
            0.0417,   -0.1667,    0.2500,   -0.1667,    0.0417;

    Eigen::MatrixXd m5(6, 6);
    m5 <<   0.0083,    0.2167,    0.5500,    0.2167,    0.0083,         0,
            -0.0417,   -0.4167,         0,    0.4167,    0.0417,         0,
            0.0833,    0.1667,   -0.5000,    0.1667,    0.0833,         0,
            -0.0833,    0.1667,         0,   -0.1667,    0.0833,         0,
            0.0417,   -0.1667,    0.2500,   -0.1667,    0.0417,         0,
            -0.0083,    0.0417,   -0.0833,    0.0833,   -0.0417,    0.0083;

    // eigen matrix index (row, col)
    // precision is up to 4 dp
    Eigen::MatrixXd m3_check = bt.create_m(3);
    for (int i = 0; i < m3.rows(); i++)
    {
        for (int j = 0; j < m3.cols(); j++)
        {
            if (m3_check(i,j) - m3(i,j) > 0.0001)
                return -1;
        }
    }
    MatrixXd m4_check = bt.create_m(4);
    for (int i = 0; i < m4.rows(); i++)
    {
        for (int j = 0; j < m4.cols(); j++)
        {
            if (m4_check(i,j) - m4(i,j) > 0.0001)
                return -1;
        }
    }
    MatrixXd m5_check = bt.create_m(5);
    for (int i = 0; i < m5.rows(); i++)
    {
        for (int j = 0; j < m5.cols(); j++)
        {
            if (m5_check(i,j) - m5(i,j) > 0.0001)
                return -1;
        }
    }


    return 0;
}