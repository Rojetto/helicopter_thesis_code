#include <math.h>
#include "adolc/adouble.h"
#include "adolc/adutils.h"
#include "adolc/lie/drivers.h"
#include "heli_adolc.h"

const int tapeF = 0;
int tapeH1 = 1;
int tapeH2 = 2;
int tapeFG1 = 3;
int tapeFG2 = 4;
int tapeH = 5;

const double g = 9.81;

const double l_h = 0.67;
const double l_p = 0.178;
const double l_c = 0.4069;

const double d_h = 0.0027;
const double d_c = 0.0639;

const double m_c = 1.7638;
const double m_h = 1.2006;

const double mup = 0.0334;
const double mue = 0.0755;
const double mul = 0.2569;

void functionF(adouble *X, adouble *Y)
{
    adouble phi = X[0];
    adouble eps = X[1];
    adouble lamb = X[2];
    adouble dphi = X[3];
    adouble deps = X[4];
    adouble dlamb = X[5];
    adouble Fs = X[6];
    adouble dFs = X[7];

    adouble p_phi_1 = m_h*(pow(l_p,2)+pow(d_h,2));
    adouble p_eps_1 = m_c*(pow(l_c,2)+pow(d_c,2)) + m_h*(pow(l_h,2)+pow(d_h,2)) + m_h*pow(sin(phi),2)*(pow(l_p,2)-pow(d_h,2));
    adouble p_lamb_1 = -pow(d_c,2)*m_c*pow(cos(eps),2)-d_c*l_c*m_c*sin(2*eps)-pow(d_h,2)*m_h*pow(cos(eps),2)*pow(cos(phi),2)+pow(d_h,2)*m_h - d_h*l_h*m_h/2*(sin(2*eps-phi)+sin(2*eps+phi))+pow(l_c,2)*m_c*pow(cos(eps),2)+pow(l_h,2)*m_h*pow(cos(eps),2)+pow(l_p,2)*m_h*pow(cos(eps),2)*pow(cos(phi),2)-pow(l_p,2)*m_h*pow(cos(eps),2)+pow(l_p,2)*m_h;

    adouble p_phi_2 = - g*d_h*m_h*cos(eps);
    adouble p_eps_2 = g*(d_c*m_c - d_h*m_h*cos(phi));
    adouble p_eps_3 = g*(l_h*m_h - m_c*l_c);

    Y[0] = dphi;
    Y[1] = deps;
    Y[2] = dlamb;

    Y[3] = -1/p_phi_1*(mup*dphi+p_phi_2*sin(phi));
    Y[4] = -1/p_eps_1*(mue*deps+p_eps_2*sin(eps)+p_eps_3*cos(eps) - l_h*cos(phi)*Fs);
    Y[5] = -1/p_lamb_1*(mul*dlamb - l_h*sin(phi)*cos(eps)*Fs);

    Y[6] = dFs;
    Y[7] = 0.0;
}

void functionG1(adouble *X, adouble *Y)
{
    Y[0] = 0.0;
    Y[1] = 0.0;
    Y[2] = 0.0;

    Y[3] = 0.0;
    Y[4] = 0.0;
    Y[5] = 0.0;

    Y[6] = 0.0;
    Y[7] = 1.0;
}

void functionG2(adouble *X, adouble *Y)
{
    adouble p_phi_1 = m_h*(pow(l_p,2)+pow(d_h,2));

    Y[0] = 0.0;
    Y[1] = 0.0;
    Y[2] = 0.0;

    Y[3] = l_p / p_phi_1;
    Y[4] = 0.0;
    Y[5] = 0.0;

    Y[6] = 0.0;
    Y[7] = 0.0;
}

void functionH(adouble *X, adouble *Y)
{
    Y[0] = X[1];  // epsilon
    Y[1] = X[2];  // lambda
}

void functionH1(adouble *X, adouble *Y)
{
    adouble eps = X[1];
    *Y = eps;
}

void functionH2(adouble *X, adouble *Y)
{
    adouble lamb = X[2];
    *Y = lamb;
}

void buildTapes()
{
    int n = 8;
    int m = 2;

    adouble *X = new adouble[n];
    adouble *Y = new adouble[n];
    adouble *Y1 = new adouble[n];
    adouble *Y2 = new adouble[n];
    adouble *H = new adouble[m];
    adouble Z;
    double *Yp = myalloc(n);
    double *Hp = myalloc(m);
    double Zp = 0.0;

    // Tape for f
    trace_on(tapeF);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    functionF(X, Y);

    for (int i = 0; i < n; i++)
    {
        Y[i] >>= Yp[i];
    }

    trace_off();

    // Tape for h
    trace_on(tapeH);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    functionH(X, H);

    H[0] >>= Hp[0];
    H[1] >>= Hp[1];

    trace_off();

    // Tape for h1
    trace_on(tapeH1);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    functionH1(X, &Z);

    Z >>= Zp;

    trace_off();

    // Tape for h2
    trace_on(tapeH2);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    functionH2(X, &Z);

    Z >>= Zp;

    trace_off();

    // Tape for f+g1
    trace_on(tapeFG1);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    functionF(X, Y1);
    functionG1(X, Y2);

    for (int i = 0; i < n; i++)
    {
        Y[i] = Y1[i] + Y2[i];
    }

    for (int i = 0; i < n; i++)
    {
        Y[i] >>= Yp[i];
    }

    trace_off();

    // Tape for f+g2
    trace_on(tapeFG2);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    functionF(X, Y1);
    functionG2(X, Y2);

    for (int i = 0; i < n; i++)
    {
        Y[i] = Y1[i] + Y2[i];
    }

    for (int i = 0; i < n; i++)
    {
        Y[i] >>= Yp[i];
    }

    trace_off();


    myfree(Yp);
    delete[] Y2;
    delete[] Y1;
    delete[] Y;
    delete[] X;
}

void calcGamma(double *X, double *Y)
{
    int d = 4;
    int n = 8;
    int m = 2;

    double **result = myalloc(m, d + 1);

    lie_scalar(tapeF, tapeH, n, m, X, d, result);
    Y[0] = result[0][d];
    Y[1] = result[1][d];

    myfree(result);
}

void calcLambda(double *X, double *Y)
{
    int r = 4;
    int n = 8;
    int m = 2;

    double **result = myalloc(m, r+1);
    double L3fg1h[2];
    double L3fg2h[2];
    double L3fh[2];

    lie_scalar(tapeF, tapeH, n, m, X, r, result);
    L3fh[0] = result[0][r];
    L3fh[1] = result[1][r];

    lie_scalar(tapeFG1, tapeH, n, m, X, r, result);
    L3fg1h[0] = result[0][r];
    L3fg1h[1] = result[1][r];

    lie_scalar(tapeFG2, tapeH, n, m, X, r, result);
    L3fg2h[0] = result[0][r];
    L3fg2h[1] = result[1][r];

    Y[0] = L3fg1h[0] - L3fh[0];
    Y[1] = L3fg1h[1] - L3fh[1];
    Y[2] = L3fg2h[0] - L3fh[0];
    Y[3] = L3fg2h[1] - L3fh[1];

    myfree(result);
}

void calcPhi(double *X, double *Y)
{
    int r = 4;
    int n = 8;
    int m = 2;

    double **result = myalloc(m, r);

    lie_scalar(tapeF, tapeH, n, m, X, r - 1, result);
    for (int i = 0; i < r; i++) {
        Y[i] = result[0][i];
        Y[i+r] = result[1][i];
    }

    myfree(result);
}

void calcPhiJacobian(double *X, double*Y)
{
    int r = 4;
    int n = 8;
    int m = 2;

    double ***result = myalloc(m, n, r);

    lie_gradient(tapeF, tapeH, n, m, X, r - 1, result);
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < n; j++)
        {
            Y[i + j*n] = result[0][j][i];
            Y[i + r + j*n] = result[1][j][i];
        }
    }

    myfree(result);
}
