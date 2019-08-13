#include "heli_adolc.h"
#include "adolc/adutils.h"
#include <iostream>
#include <Windows.h>

int main()
{
    buildTapes();

    int n = 8;
    int m = 2;

    double *X = myalloc(n);
    double *gamma = myalloc(m);
    double *lambda_result = myalloc(m*m);
    double **lambda = myalloc2(m, m);
    double *phi = myalloc(n);
    double *phi_jac_result = myalloc(n*n);
    double **phi_jac = myalloc2(n, n);
    double *all_derivs = myalloc(14);

    X[0] = 1.0;
    X[1] = 2.0;
    X[2] = 3.0;
    X[3] = 4.0;
    X[4] = 5.0;
    X[5] = 6.0;
    X[6] = 7.0;
    X[7] = 8.0;

    calcGamma(X, gamma);
    calcLambda(X, lambda_result);
    lambda[0][0] = lambda_result[0];
    lambda[1][0] = lambda_result[1];
    lambda[0][1] = lambda_result[2];
    lambda[1][1] = lambda_result[3];
    calcPhi(X, phi);
    calcPhiJacobian(X, phi_jac_result);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            phi_jac[i][j] = phi_jac_result[i + j*n];
        }
    }

    cout << "Gamma: " << gamma[0] << "\t" << gamma[1] << "\n";
    cout << "Lambda:\n" << lambda[0][0] << "\t" << lambda[0][1] << "\n" << lambda[1][0] << "\t" << lambda[1][1] << "\n";

    cout << "Phi:\n";
    for (int i= 0; i < n; i++)
    {
        cout << phi[i] << "\t";
    }
    cout << "\n";

    cout << "Phi jacobian:\n";
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cout << phi_jac[i][j] << "\t";
        }
        cout << "\n";
    }

    LARGE_INTEGER start_time, end_time, elapsed_microseconds;
    LARGE_INTEGER frequency;

    int n_calls = 1000;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start_time);

    for (int i = 0; i < n_calls; i++) {
        calcLambda(X, lambda_result);
    }

    QueryPerformanceCounter(&end_time);
    elapsed_microseconds.QuadPart = end_time.QuadPart - start_time.QuadPart;
    elapsed_microseconds.QuadPart *= 1000000;
    elapsed_microseconds.QuadPart /= frequency.QuadPart;

    cout << "Time for " << n_calls << " calls to calcLambda() in us: " << elapsed_microseconds.QuadPart << "\n";

    calcRequiredDerivatives(X, all_derivs);
    for (int i = 0; i < 14; i++)
    {
        cout << all_derivs[i] << "\t";
    }
    cout << "\n";

    return 0;
}