function y = syst(x,wing,assumptions)

%Parameters
CLalpha_w = wing.CLalpha;
S_w       = wing.S;
c_w       = wing.MAC;

A_w       = assumptions.Aw;
Clalpha_h = assumptions.tail_Clalpha;
eta_h     = assumptions.eta_h;
V_h       = assumptions.Vh;
Ah_mul    = assumptions.Ah_mul;
SM        = assumptions.StaticMargin;

Df        = assumptions.Df;
Lf_nose   = assumptions.Lf_nose;
Lf_rear   = assumptions.Lf_rear;
battery_m = assumptions.battery_m;


% Derived Parameters
A_h         = Ah_mul*A_w;
CLalpha_h   = Clalpha_h/(1+(Clalpha_h/(pi*A_h)));
eps_dalpha  = 2*CLalpha_w/(pi*A_w);

xAC         = x(1);
xAC_norm    = x(1)/c_w;

Lf_body     = x(2);
cg          = aero_balance(Df,Lf_nose,Lf_body,Lf_rear,battery_m);
xCG         = cg.x;
xCG_norm    = xCG/c_w;

Lf_body_aft = x(3);

% System of equations
y(1) = CLalpha_h/CLalpha_w*eta_h*V_h*(1-eps_dalpha) + xAC_norm - xCG_norm - SM;

y(2) = 4*Df - 2*c_w*S_w*V_h/(Lf_body_aft + Lf_rear - 0.75*c_w)^2;                 %Assumed ch = cw, xMACh = xMACw, xTEh = Lf

y(3) = Lf_body_aft - (Lf_nose + Lf_body - xCG);


end

