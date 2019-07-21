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

#ifdef __cplusplus
extern "C" {
#endif
void buildTapes();

void calcGamma(double *, double *);

void calcLambda(double *, double *);
#ifdef __cplusplus
}
#endif
