#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "txt_reader.hpp"
#include <bspline_utils.hpp>

using namespace trajectory;

/**
 * @brief 1d bspline test case
 * 
 * In MATLAB
 * Use dlmwrite('XX.txt', variable_vector, 'delimiter','\t','newline','pc')
 * This will give a txt file that is able to be read by this script
 */

int main(int argc, char **argv) 
{
    bspline_trajectory bt;
    bspline_trajectory::bs_pva_state_1d state;

    if (argc != 5) {
        printf("Input txt file required!\n");
        return -1;
    }

    // Read vector in the file
    vector<double> ctrlpt = read_vector(argv[1]);
    vector<double> pos = read_vector(argv[2]);
    vector<double> vel = read_vector(argv[3]);
    vector<double> time = read_vector(argv[4]);

    vector<double> timespan;
    timespan.push_back(0.0);
    timespan.push_back(1.0);

    int knotdiv = 5;
    int order = 5;

    state = bt.get_uni_bspline_1d(order, timespan, ctrlpt, knotdiv);

    // The output from matlab to txt is inconsistent so we will just use 3dp as the standard
    for (int i = 0; i < (int)state.pos.size(); i++)
    {
        if (abs(state.pos[i] - pos[i]) > 0.001)
            return -1;
    }
    for (int i = 0; i < (int)state.vel.size(); i++)
    {
        if (abs(state.vel[i] - vel[i]) > 0.001)
            return -1;
    }
    for (int i = 0; i < (int)state.rts.size(); i++)
    {
        if (abs(state.rts[i] - time[i]) > 0.001)
            return -1;
    }

    return 0;
}