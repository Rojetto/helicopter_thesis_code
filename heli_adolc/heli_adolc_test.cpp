#include "heli_adolc.h"
#include "adolc/adutils.h"
#include <iostream>

int main()
{
    buildTapes();

    int n = 8;
    int m = 2;

    double *X = myalloc(n);
    double *gamma = myalloc(m);
    double **lambda = myalloc2(m, m);

    X[0] = 1.0;
    X[1] = 2.0;
    X[2] = 3.0;
    X[3] = 4.0;
    X[4] = 5.0;
    X[5] = 6.0;
    X[6] = 7.0;
    X[7] = 8.0;

    calcGamma(X, gamma);
    calcLambda(X, lambda);

    cout << "Gamma: " << gamma[0] << "\t" << gamma[1] << "\n";
    cout << "Lambda:\n" << lambda[0][0] << "\t" << lambda[0][1] << "\n" << lambda[1][0] << "\t" << lambda[1][1] << "\n";

    return 0;
}