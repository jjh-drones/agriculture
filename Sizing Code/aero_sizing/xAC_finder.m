function F = xAC_finder(x,assumptions, wing)

xAC        = x;
xAC_norm   = x/wing.MAC;

cg         = aero_balance(assumptions,assumptions.battery_m,wing,xAC,0,0,1,1);

xCG        = cg.x;
xCG_norm   = xCG/wing.MAC;

CLalpha_w  = wing.CLalpha;
CLalpha_h  = assumptions.CLalpha_h; 
SM         = assumptions.StaticMargin;
eta_h      = assumptions.eta_h;
V_h        = assumptions.Vh;
eps_dalpha = wing.eps_dalpha;

F =  - CLalpha_h/CLalpha_w*eta_h*V_h*(1-eps_dalpha) + xCG_norm + SM - xAC_norm;

end

