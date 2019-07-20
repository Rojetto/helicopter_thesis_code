#include "heli_adolc.h"
#include "adolc/adutils.h"
#include <iostream>

int main()
{
    buildTapes();

    int n = 8;
    int m = 8;
    int d = 1;

    double** X = myalloc2(n, d + 1);
    double** Y = myalloc2(m, d + 1);

    X[0][0] = 1.0;
    X[1][0] = 2.0;
    X[2][0] = 3.0;
    X[3][0] = 4.0;
    X[4][0] = 5.0;
    X[5][0] = 6.0;
    X[6][0] = 7.0;
    X[7][0] = 8.0;

    X[0][1] = 1.0;

    fForward(d, X, Y);

    for (int j = 0; j <= d; j++)
    {
        cout << j;
        for (int i = 0; i < m; i++) {
            cout << "\t" << Y[i][j];
        }
        cout << "\n";
    }

    return 0;
}