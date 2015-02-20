function ls = stab_long(wing,hstab,assumptions,battery_m,gimbal1,gimbal2)

m2cm    = 100;
rad2deg = 180/pi;

cg = aero_balance(assumptions,battery_m,wing,wing.xAC,0,0,gimbal1,gimbal2);

xCG           = cg.x;
CLalpha_w     = wing.CLalpha;
c_w           = wing.MAC;
V_h           = assumptions.Vh;
eta_h         = assumptions.eta_h;
CLalpha_h     = hstab.CLalpha;
eps_dalpha    = wing.eps_dalpha;

xCG_norm      = xCG/c_w;
xAC_norm      = wing.xAC/c_w;


Cmalpha = CLalpha_w*(xCG_norm - xAC_norm) - CLalpha_h*eta_h*V_h*(1-eps_dalpha);
NP      = CLalpha_h/CLalpha_w*eta_h*V_h*(1-eps_dalpha) + xAC_norm;
SM      = NP - xCG_norm;

ls.mass    = cg.mass;
ls.Cmalpha = Cmalpha;
ls.XCG     = xCG;
ls.NP      = NP;
ls.SM      = SM;

display('LONG STABILITY (DESIGN)')
display('--------')
fprintf('%15s%15.6f\n','Mass: ',cg.mass);
fprintf('%15s%15.6f\n','Cm_alpha: ',Cmalpha);
fprintf('%15s%15.6f\n','x_CG: ',cg.x*m2cm);
fprintf('%15s%15.6f\n','NP: ',NP*wing.MAC*m2cm);
fprintf('%15s%15.6f\n\n','SM: ',SM*wing.MAC*m2cm);

end

