#ifndef PARSER_H
#define PARSER_H

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <fstream>
#include <experimental/filesystem>
#include <Eigen/Core>
#include <memory>
#include <third_party/Vectormap.h>
#include <geometry_msgs/Point.h>

using namespace std;

namespace Planner
{

class parser
{
    private:
        int number = 5; //Number of Points in one LaneNode
        // vector<shared_ptr<LaneNode>> LanePath_;
        LanePath LanePath_;
        string filename_="keti_pangyo_path3.csv";

    public:
        parser();
        parser(int n, LanePath path) : number(n), LanePath_(path){};
        void get_Coorddata(string Xfilename);

        vector<string> split(string str, char delimiter); // Parse one line of string into vector of strings
        vector<double> split_double(string str, char delimiter);

        void display_result();
        //void converstion_UTM2NED(); 

        LanePath get_lanepath(){
            return LanePath_;
        }
};
} // namespace parser


#endif