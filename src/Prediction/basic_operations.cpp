
#include <third_party/Prediction/basic_operations.hpp>
using namespace  DAP;
using namespace std;

string current_working_directory()
{
    char* cwd  ; // **** microsoft specific ****
    std::string working_directory(cwd) ;
    std::free(cwd) ;
    return working_directory ;
}


void DAP::mean_var_update(int * count , float *mean ,float * var,float new_val){
    float m2 = (*var)*(*count);
    *count +=1 ;
    float delta = new_val - *mean;
    *mean  += (delta/float(*count));
    float delta2 = new_val - *mean;
    m2 += delta*delta2;    
    *var = m2/(*count);
}

vector<TransformMatrix> DAP::read_SE3_seq(string file_name){

    std::ifstream infile; TransformMatrix tf; tf.setIdentity(); int r=0 ,c=0;
    vector<TransformMatrix> tf_seq;
    
    infile.open(file_name);
    if(infile.is_open()){
        while (! infile.eof()){ 
            std::string line;            
            getline(infile, line); // if no delimiter given, new line is that
            // if empty line was read = new pose started 
            if (line.size() == 0){ 
                tf_seq.push_back(tf);  // save previous one 
                tf.setIdentity(); // init
                r = 0;
            }
            // keeps reading through row 
            else{
                std::stringstream stream(line);
                std::string val;
                int c = 0;
                while(! stream.eof() and c != 4) { // reading through column 
                    getline(stream, val, ' ');
                    tf(r,c) = atof(val.c_str());
                    c++;
                }
                r++;
            }
        }
        tf_seq.push_back(tf);  // save previous one 
    }else
        cout << "Warning: no file to read for SE3 sequence" <<endl;
    return tf_seq;
}


TXYZQuatTraj DAP::read_TXYZQuat_seq(string file_name){
    TXYZQuatTraj enough_pose_mat(8,10000);
    std::ifstream infile; int r=0 ,c=0;
    infile.open(file_name);
    if(infile.is_open()){
        while (! infile.eof()){ 
            std::string line;            
            getline(infile, line); // if no delimiter given, new line is that            
            std::stringstream stream(line);
            std::string val;
            int r = 0;
            while(! stream.eof() and r != 8) { // reading through column 
                getline(stream, val, ' ');
                enough_pose_mat(r,c) = atof(val.c_str());
                r++;
            }
            c++;
        }
    }else
        cout << "Warning: no file to read for SE3 sequence" <<endl;
    return enough_pose_mat.block(0,0,8,c-1);    
} 

Eigen::VectorXf DAP::polyfit(Eigen::VectorXf xvals, Eigen::VectorXf yvals,int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXf W(xvals.size(),xvals.size()); W.setZero();

    for (int i = 0; i < xvals.size() ; i++)
        W(i,i) = pow(i+1,5);
    
    
    Eigen::MatrixXf A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

  auto Q = (A).householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial.
float DAP::polyeval(Eigen::VectorXf coeffs,float x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

float DAP::polyeval_derivative(Eigen::VectorXf coeffs,float x) {
    double result = 0.0;
    if(coeffs.size() == 1 )
        return result;
    else{
        for (int i = 1; i < coeffs.size(); i++) {
            result += i*coeffs[i] * pow(x, i-1);
        }
        return result;
    }
}



