function pos = aero_pos2(assumptions,wing)

%Parameters
CLalpha_w = wing.CLalpha;
c_w       = wing.MAC;
A_w       = assumptions.Aw;

Clalpha_h  = assumptions.tail_Clalpha;
eta_h      = assumptions.eta_h;
V_h        = assumptions.Vh;
Ah_mul     = assumptions.Ah_mul;
A_h        = Ah_mul*A_w;
CLalpha_h  = Clalpha_h/(1+(Clalpha_h/(pi*A_h)));
eps_dalpha = 2*CLalpha_w/(pi*A_w);

Df        = assumptions.Df;
Lf_nose   = assumptions.Lf_nose;
Lf_body   = assumptions.Lf_body;
Lf_rear   = assumptions.Lf_rear;
battery_m = assumptions.battery_m;
cg        = aero_balance(Df,Lf_nose,Lf_body,Lf_rear,battery_m,wing);
xCG       = cg.x;
xCG_norm  = xCG/c_w;
SM        = assumptions.StaticMargin;

% Computation xAC
xAC_norm    = - CLalpha_h/CLalpha_w*eta_h*V_h*(1-eps_dalpha) + xCG_norm + SM;
pos.xAC     = xAC_norm*c_w;
pos.xCG     = cg.x;
pos.zCG     = cg.z;
pos.Lf_body = Lf_body;
pos.Ixx     = cg.Ixx;
pos.Iyy     = cg.Iyy;
pos.Izz     = cg.Izz;


end

