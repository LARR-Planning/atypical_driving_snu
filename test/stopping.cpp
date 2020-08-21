#include <third_party/stopping.h>


int main(){

    double x0 = 2.0;
    double y0 = 0.0;
    const double v0 = 1.0;
    const double a0 = 0.0;
    const double del0 = 0.0;
    const double th0 = pi/4.0; 

    const double xf = 10.0;
    const double yf = 5.0;
    const double vf = 0.0;
    const double af = 0.0;

    auto t1 = std::chrono::high_resolution_clock::now();

    Planner::Stopping stop()
    

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "NLP took "
               << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
               << " milliseconds\n";
}