function ls = stab_long(xCG,pos,wing,hstab,assumptions)

CLalpha_w     = wing.CLalpha;
c_w           = wing.MAC;
V_h           = assumptions.Vh;
eta_h         = assumptions.eta_h;
CLalpha_h     = hstab.CLalpha;
eps_dalpha    = hstab.eps_dalpha;

xCG_norm      = xCG/c_w;
xAC_norm      = pos.xAC/c_w;


Cmalpha = CLalpha_w*(xCG_norm - xAC_norm) - CLalpha_h*eta_h*V_h*(1-eps_dalpha);
NP      = CLalpha_h/CLalpha_w*eta_h*V_h*(1-eps_dalpha) + xAC_norm;
SM      = NP - xCG_norm;

ls.Cmalpha = Cmalpha;
ls.NP      = NP;
ls.SM      = SM;

end

