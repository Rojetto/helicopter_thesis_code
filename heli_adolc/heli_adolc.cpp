#include <math.h>
#include "adolc/adouble.h"
#include "adolc/adutils.h"
#include "heli_adolc.h"


void f(adouble *X, adouble *Y)
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

void buildTapes()
{
    short tape = 0;
    int n = 8;

    adouble *X = new adouble[n];
    adouble *Y = new adouble[n];
    double *Yp = myalloc(n);

    trace_on(tape);

    for (int i = 0; i < n; i++)
    {
        X[i] <<= 0;
    }

    f(X, Y);

    for (int i = 0; i < n; i++)
    {
        Y[i] >>= Yp[i];
    }

    trace_off();

    myfree(Yp);
    delete[] Y;
    delete[] X;
}


void fForward(int d, double **X, double **Y)
{
    short tape = 0;
    int n = 8;
    int m = 8;

    forward(tape, m, n, d, 1, X, Y);
}