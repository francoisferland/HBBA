// A parser of simulation data from M. Doyon's new solver model.
// Converts simultation scenario description into strategies / matrices for the
// old solver.

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>

namespace {
    struct DesireDescriptionSim {
        double  r_d;
        double  x_b_bar;
        double  s_d;
        double  p_d;
        double  i_d;
        double  m;
        bool    a_d;
        double  x_d;
        bool    indiv;
        bool    discr;
        
    };
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " sim_data" << std::endl;
        return -1;
    }

    const char* fname = argv[1];
    std::ifstream input_file(fname);

    std::string line;
    std::getline(input_file, line); // Skipping first line (header).
    while (std::getline(input_file, line)) {
        // Splitting the line into tokens (words separated by whitespace,
        // expecting 11). Exit if it's not the case.
        std::istringstream iss(line);
        std::vector<std::string> tokens((std::istream_iterator<std::string>(iss)),
                                        std::istream_iterator<std::string>());
        if (tokens.size() != 11) {
            std::cerr << "Error : malformed simulation data. " 
                      << "Expecting 11 tokens per line." << std::endl
                      << "Line: " << line << std::endl;
            return -1;
        }

        const std::string& dname = tokens[0];    // Desire name
        std::cout << "Parsing desire '" << dname << "'..." << std::endl; 
        try {
            DesireDescriptionSim dds;
            dds.r_d     = atof_l(tokens[1].c_str(), nullptr);   // Reservation
            dds.x_b_bar = atof_l(tokens[2].c_str(), nullptr);   // Rationing
            dds.s_d     = atof_l(tokens[3].c_str(), nullptr);   // Fixed cost
            dds.p_d     = atof_l(tokens[4].c_str(), nullptr);   // Unit cost
            dds.i_d     = atof_l(tokens[5].c_str(), nullptr);   // Intensity
            dds.m       = atof_l(tokens[6].c_str(), nullptr);   // CPU budget
            dds.a_d     = bool(atoi(tokens[7].c_str()));        // activated
            dds.x_d     = atof_l(tokens[8].c_str(), nullptr);   // Optimal flow
            dds.indiv   = bool(atoi(tokens[9].c_str()));        // Indivisible
            dds.discr   = bool(atoi(tokens[10].c_str()));       // Discrete

        } catch (std::exception e) {
            std::cerr << "Parsing failure: " << e.what() << std::endl; 
            return -1;
        }
        
    }

}

