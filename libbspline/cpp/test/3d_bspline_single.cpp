#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "txt_reader.hpp"
#include <bspline_utils.hpp>

using namespace trajectory;

/**
 * @brief 3d single bspline test case
 * 
 * In MATLAB
 * Use dlmwrite('XX.txt', variable_vector, 'delimiter','\t','newline','pc')
 * This will give a txt file that is able to be read by this script
 */

int main(int argc, char **argv) 
{
    bspline_trajectory bt;
    bspline_trajectory::bs_pva_state_3d state;

    if (argc != 8) {
        printf("Input txt file required!\n");
        return -1;
    }

    // Read vector in the file
    vector<double> ctrlpt_x = read_vector(argv[1]);
    vector<double> ctrlpt_y = read_vector(argv[2]);
    vector<double> ctrlpt_z = read_vector(argv[3]);

    int cp_array_size = ctrlpt_x.size();
    vector<Vector3d> ctrlpt;
    for (int i = 0; i < cp_array_size; i++)
    {
        Eigen::Vector3d single_ctrlpt = Vector3d(ctrlpt_x[i], ctrlpt_y[i], ctrlpt_z[i]);
        ctrlpt.push_back(single_ctrlpt);
    }

    vector<double> pos_x = read_vector(argv[4]);
    vector<double> pos_y = read_vector(argv[5]);
    vector<double> pos_z = read_vector(argv[6]);

    int pos_check_array_size = pos_x.size();
    vector<Vector3d> check_pos;
    for (int i = 0; i < pos_check_array_size; i++)
    {
        Eigen::Vector3d single_pos = Vector3d(pos_x[i], pos_y[i], pos_z[i]);
        check_pos.push_back(single_pos);
    }

    vector<double> time = read_vector(argv[7]);

    vector<double> timespan;
    timespan.push_back(0.0);
    timespan.push_back(1.0);

    int random_index = 20;
    double query_time = time[random_index-1];

    std::cout << "[3d_bspline_single]" <<
            " query_time: " << std::endl <<
            KBLU << query_time << KNRM << std::endl;

    int order = 5;

    time_point<std::chrono::system_clock> start = system_clock::now();

    state = bt.get_single_bspline_3d(order, timespan, ctrlpt, query_time);

    auto test_time_diff = duration<double>(system_clock::now() - start).count();
    std::cout << "[3d_bspline_single]" << 
        " time for 3d_bspline_single: " << 
        KGRN << test_time_diff*1000 << KNRM << "ms" << std::endl;

    std::cout << "[3d_bspline_single]" <<
            " state.pos[0]: " << std::endl <<
            KBLU << state.pos[0] << KNRM << std::endl;
    
    std::cout << "[3d_bspline_single]" <<
            " check_pos[random_index-1]: " << std::endl <<
            KBLU << check_pos[random_index-1] << KNRM << std::endl;

    if (state.pos.empty())
        return -1;

    if (abs(state.pos[0](0) - check_pos[random_index-1](0)) > 0.001)
        return -1;
    if (abs(state.pos[0](1) - check_pos[random_index-1](1)) > 0.001)
        return -1;
    if (abs(state.pos[0](2) - check_pos[random_index-1](2)) > 0.001)
        return -1;

    std::cout << KGRN << "Success" << KNRM << std::endl;

    return 0;
}