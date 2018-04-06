#include "plot_weights.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

void split(const std::string line,
           std::vector<double> &values)
{
    std::istringstream ss(line);
    std::string token;
    while(std::getline(ss, token, ',')) {
        std::stringstream str(token);
        double value;
        str >> value;
        values.emplace_back(value);
    }

}



int main(int argc, char *argv[])
{
    if(argc != 2) {
        std::cout << "muse_mcl_2d_statistics_plotter <path-to-csv-file>" << std::endl;
        return 0;
    }


    std::vector<std::vector<double>> weights;
    std::vector<double> means;
    std::vector<double> std_devs;
    std::ifstream in(argv[1]);
    if(!in.is_open()) {
        std::cerr << "Could not open file!" << std::endl;
        return -1;
    }

    std::string line;
    while(std::getline(in, line)) {
        std::vector<double> line_values;
        split(line,  line_values);
        double mean = line_values[line_values.size() - 2];
        double std_dev = line_values[line_values.size() - 1];
        line_values.erase(line_values.begin() + line_values.size() - 2 , line_values.end());
        weights.emplace_back(line_values);
        means.emplace_back(mean);
        std_devs.emplace_back(std_dev);
    }

    muse_mcl_2d::PlotWeights pw(800, 1200, "Weights");
    int key;
    int pos = 0;
    while(true) {
        const std::vector<double> &w = weights.at(pos);

        pw.plot(w);

        std::cout << "------ " << pos << " ------" << std::endl;

        key = cv::waitKey(0) & 0xFF;
        if(key == 81) {
            // right
            ++pos;
            if(pos >= static_cast<int>(means.size())) {
                pos = 0;
            }
        }
        if(key == 83) {
            // left
            --pos;
            if(pos < 0) {
                pos = means.size() - 1;
            }
        }
        if(key == 27) {
            break;
        }
    }

    return 0;
}
