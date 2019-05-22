from scipy.interpolate import interp1d
from numpy import concatenate as con


class TrajectoryInstance:
    def __init__(self, t, phi, eps, lamb, vf, vb):
        # phi, eps, lamb, vf, vb contain vectors, index is derivative order
        self.t = t
        self.phi = phi
        self.eps = eps
        self.lamb = lamb
        self.vf = vf
        self.vb = vb


class Trajectory:
    def __init__(self, t, phi, eps, lamb, vf, vb):
        # phi, eps, lamb, vf, vb contain matrices, first index is time index, second index is derivative order
        self.t = t
        self.phi = phi
        self.eps = eps
        self.lamb = lamb
        self.vf = vf
        self.vb = vb

        # We can only set up interpolators, if we have more than one value to interpolate
        if len(t) > 1:
            self.phi_interp = interp1d(t, phi, axis=0, copy=False, bounds_error=False, fill_value=(phi[0], phi[-1]))
            self.eps_interp = interp1d(t, eps, axis=0, copy=False, bounds_error=False, fill_value=(eps[0], eps[-1]))
            self.lamb_interp = interp1d(t, lamb, axis=0, copy=False, bounds_error=False, fill_value=(lamb[0], lamb[-1]))
            self.vf_interp = interp1d(t, vf, axis=0, copy=False, bounds_error=False, fill_value=(vf[0], vf[-1]))
            self.vb_interp = interp1d(t, vb, axis=0, copy=False, bounds_error=False, fill_value=(vb[0], vb[-1]))
        else:
            def fake_interp(value):
                return lambda t0: value

            self.phi_interp = fake_interp(phi[0])
            self.eps_interp = fake_interp(eps[0])
            self.lamb_interp = fake_interp(lamb[0])
            self.vf_interp = fake_interp(vf[0])
            self.vb_interp = fake_interp(vb[0])

    def eval(self, t) -> TrajectoryInstance:
        phi = self.phi_interp(t)
        eps = self.eps_interp(t)
        lamb = self.lamb_interp(t)
        vf = self.vf_interp(t)
        vb = self.vb_interp(t)

        return TrajectoryInstance(t, phi, eps, lamb, vf, vb)

    def to_dict(self):
        return dict(t=self.t, phi=self.phi, eps=self.eps, lamb=self.lamb, vf=self.vf, vb=self.vb)
