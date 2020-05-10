//
// Created by lyw on 20. 5. 10..
//
#include <optimization_module/utils/math.hpp>

VectorXd power(const double a, const VectorXd b)
{
    int Nb = b.size();
    VectorXd apowb(Nb);
    for(int i=0; i<Nb; i++)
        apowb(i) = std::pow(a, b(i));
    return apowb;
}

MatrixXd nan(const int n, const int m)
{
    MatrixXd res = MatrixXd::Constant(n,m,std::nan("1"));
    return res;
}
int factorial(int n)
{
    return (n == 1 || n == 0) ? 1 : factorial(n-1)*n;
}
