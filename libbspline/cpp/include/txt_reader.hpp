#include <iterator>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

vector<double> read_vector(std::string file_name) 
{
    std::vector<double> values;
    std::ifstream myfile(file_name);
    if (!myfile) {
        std::cout << "Unable to open file: " << file_name << std::endl;
        throw std::runtime_error("[txt reader] Cannot open file");
    }

    // read the input
    std::copy(std::istream_iterator<double>(myfile), 
        std::istream_iterator<double>(), std::back_inserter(values));

    return values;
}