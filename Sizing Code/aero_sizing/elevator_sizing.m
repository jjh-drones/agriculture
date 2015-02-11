function elevator = elevator_sizing(assumptions,mission,wing,cg,hstab)
deg2rad = pi/180; 
rad2deg = 180/pi;

%% PARAMETERS
deltaE_max = assumptions.deltaE_max*deg2rad;
bratio_E   = assumptions.bratio_E;
T_excess   = assumptions.T_excess;
Iyy        = cg.Iyy;
xCG        = cg.x;
theta_ddot = mission.pitch_rate*deg2rad;
xAC_w      = wing.xAC;
zAC_w      = assumptions.Df/2;
z_prop     = assumptions.Df/2;
Lw         = wing.L_launch;       
Dw_launch  = wing.D_launch;
Dw_cruise  = wing.D_cruise;
Macw       = wing.M_launch;
T_launch   = Dw_launch + T_excess;
T_cruise   = Dw_cruise + fuselage.D + hstab.D;
eps        = wing.eps_launch;
ih         = hstab.incidence;
iw         = wing.incidence;
alpha_w    = wing.alpha_cruise;

%% LAUNCH REQUIREMENT

% Required Cl_h for rotation maneuver after hand launch
Mw   =  Macw + Lw*(xCG - xAC_w) - Dw*(zCG - zAC_w);      % is this moment nose up or nose down? 
MT   = T_launch*(zCG - z_prop);
Mh   = -Mw -MT + Iyy*theta_ddot;
Lh   = Mh/(xCG-xAC_h);
CL_h = Lh/(mission.q_second*S_w);

% Derivation of elevator efficiency (assuming that Cl_h required is achieved with deltaE_max (fix))
alpha_w     = 0;
CLw_launch  = wing.CL_launch;
eps0        = 2*CLw_launch/(pi*A_w);
eps_dalpha  = 2*CLalpha_w/(pi*A_w);
eps         = eps0 + eps_dalpha*alpha_w;
alpha_h     = alpha_w + incidence_h - eps;
tauE        = (alpha_h + (CL_h/CLalpha_h))/deltaE_max;

% Derivation of other elevator parameters
cE2ch        = 0.4;                                      % this value comes from a curve
cE           = cE2ch*croot_h;
delta_alpha0 = -1.15*cE2ch*deltaE_max;
Cm_deltaE = - CLalpha_h*eta_h*V_h*bE2b*tauE;
CL_deltaE = CLalpha_h*eta_h*S_h/S_w*tauE;

% Elevator deflection for various flight conditions
rho        = mission.rho;
V          = linspace(10,30,100);
q          = 0.5*rho*V.^2;
CL0        = 0;                                                 % this probably is not zero
CLalpha    = CLalpha_w;                                         % review this
CLw_cruise = wing.CL_cruise;
ls         = stab_long(xCG,pos,wing,hstab,assumptions);
Cmalpha    = ls.Cmalpha;
deltaE     = ((T_cruise*z_prop./(q*S_w*c_w) + (CLw_cruise - CL0)*Cmalpha)/(CLalpha*Cm_deltaE - Cmalpha*CL_deltaE));

plot(V,deltaE*rad2deg)


end

