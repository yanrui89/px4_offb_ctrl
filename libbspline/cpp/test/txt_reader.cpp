#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "txt_reader.hpp"

/**
 * @brief txt format reader
 * 
 * In MATLAB
 * Use dlmwrite('XX.txt', variable_vector, 'delimiter','\t','newline','pc')
 * This will give a txt file that is able to be read by this script
 */

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("Input txt file required!\n");
        return -1;
    }

    // Read vector in the file
    vector<double> v = read_vector(argv[1]);

    Eigen::VectorXd m(16);
    m <<    9.1862,	9.1862,	9.1862,	9.1862,	
            9.1862,	9.1862,	9.5424,	2.2425,	
            5.8772,	5.347,	7.7558,	7.7558,	
            7.7558,	7.7558,	7.7558,	7.7558;

    for (int i = 0; i < (int)v.size(); i++)
    {
        if (m(i) - v[i] > 0.0001)
            return -1;
    }

    return 0;
}