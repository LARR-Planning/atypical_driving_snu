#include <third_party/parser.h>
#include <typeinfo>

namespace Planner{

parser::parser()
{
    number = 10;
    LanePath_.lanes.clear();
}

void parser::get_Coorddata(string Xfilename)
{   
    //string cur_path = experimental::filesystem::current_path();
//    Xfilename = cur_path+"/../"+Xfilename;
    // Yfilename = cur_path+"/../src/"+Yfilename;
    //X coordinate
    cout << Xfilename << endl;
    // ROS_INFO("%s", Xfilename);
    try
    {
        ifstream infile(Xfilename);      
        // infile >> setprecision(10);
        std::string linex;
        int iter = 0;
        if(infile.fail())
        {
            ROS_ERROR("%s", "No csv map file! Please check your csv file path");
            return;
        }

        vector< vector<double>> xdata; 

        while(std::getline(infile, linex))
        {   
            std::istringstream iss(linex);
            vector<double> vecX = parser::parser::split_double(linex, ',');
            xdata.push_back(vecX);
        }

        infile.close();
        vector<geometry_msgs::Point> coord_vec;
        shared_ptr<LaneNode> node (new LaneNode);

        for(int i=0; i<xdata.size(); i++)
        {   
            
            //Make i-th Node's coordinate
            double X = xdata.at(i).at(0);
            double Y = xdata.at(i).at(1);
            geometry_msgs::Point coord1;
            coord1.x = X;
            coord1.y = Y;
            coord1.z = 0.0;
            coord_vec.push_back(coord1);
            
            // node = LanePath_.at(i);
            //Update
            if((i+1)%number ==0 || (i+1)==xdata.size())
            {   
                node->laneCenters = coord_vec;
                LanePath_.lanes.push_back(*node);
                coord_vec.clear();
            }
        }
    }

    catch(const std::exception& e)
    {   
        cout << "File is not open" << endl;        
        std::cerr << e.what() << '\n';
    }
}

void parser::display_result()
{
    cout << LanePath_.lanes.size() << endl;
    for(int i=0; i<10; ++i)
    {
        if(i < LanePath_.lanes.size())
        {   cout << "i " << i << endl;
            cout << LanePath_.lanes.at(i).laneCenters.size() << endl;
            cout << LanePath_.lanes.at(i).laneCenters.at(0).x;
            cout << ", ";
            cout << LanePath_.lanes.at(i).laneCenters.at(0).y << endl;
        }
    }
}


vector<string> parser::split(string str, char delimiter) {
    vector<string> internal;
    stringstream ss(str);
    string temp; 
    while (getline(ss, temp, delimiter)) {
        internal.push_back(temp);
    }
    return internal;
}

vector<double> parser::split_double(string str, char delimiter) {
    vector<double> internal;
    stringstream ss(str);
    string temp;
    while (getline(ss, temp, delimiter)) {

        stringstream geek(temp);
        // geek.precision(10);
        // stringstream out;
        // out << fixed << setprecision(10) << temp;
        double d=0.0, garbage; 
        // geek >>  d;
        cout.precision(11);
        // cout << setprecision(10) << temp << endl;
        d = stod(temp);

        internal.push_back(d);
    }
    return internal;
}



}