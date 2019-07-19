#include <math.h>
#include "adolc/adouble.h"
#include "adolc/adutils.h"
#include "adolc_test.h"

void Duffing(short int tape)
{
    adouble x1, x2, z1, z2;
    double z;
    trace_on(tape);
    x1 <<= 0.0;
    x2 <<= 0.0;
    z1 = x2;
    z2 = x1 - pow(x1, 3);
    z1 >>= z;
    z2 >>= z;
    trace_off();
}

void generateTape()
{
    short int tape = 0;
    Duffing(tape);
}

void evaluateTape()
{
    short int tape = 0;
    int n = 2;
    int d = 10;
    double **X;
    X = myalloc2(n, d + 1);
    X[0][0] = 1.412;
    X[1][0] = 0.0;
    forode(tape, n, d, X);
    for (int j = 0; j <= d; j++)
    {
        cout << j;
        for (int i = 0; i < n; i++) {
            cout << "\t" << X[i][j];
        }
        cout << "\n";
    }
    myfree2(X);
}