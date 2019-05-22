class Trajectory:
    def __init__(self, t, phi, eps, lamb, vf, vb):
        # phi, eps, lamb, vf, vb contain matrices, first index is time index, second index is derivative order
        self.t = t
        self.phi = phi
        self.eps = eps
        self.lamb = lamb
        self.vf = vf
        self.vb = vb

    def to_dict(self):
        return dict(t=self.t, phi=self.phi, eps=self.eps, lamb=self.lamb, vf=self.vf, vb=self.vb)
