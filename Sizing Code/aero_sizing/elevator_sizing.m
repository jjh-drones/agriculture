function elevator = elevator_sizing(assumptions,mission,wing,fuselage,cg,hstab)
deg2rad = pi/180; 
rad2deg = 180/pi;

%% PARAMETERS
theta_ddot = mission.pitch_rate*deg2rad;
deltaE_max = assumptions.deltaE_max*deg2rad;
bratio_E   = assumptions.bratio_E;
T_excess   = assumptions.T_excess;
Iyy        = cg.Iyy;
xCG        = cg.x;
zCG        = cg.z;
xAC_w      = wing.xAC;
xAC_h      = hstab.xAC;
zAC_w      = assumptions.Df/2;
z_prop     = assumptions.Df/2;
Lw         = wing.L_launch;       
Dw_launch  = wing.D_launch;
Dw_cruise  = wing.D_cruise;
Macw       = wing.M_launch;
T_launch   = Dw_launch + T_excess;
T_cruise   = Dw_cruise + fuselage.D + hstab.D;
ih         = hstab.incidence;
iw         = wing.incidence;
S_w        = wing.S;
alpha_w    = wing.alpha_launch;
eps        = wing.eps_launch;
CLalpha_h  = assumptions.CLalpha_h;
croot_h    = hstab.croot;
S_h        = hstab.S;
V_h        = assumptions.Vh;
eta_h      = assumptions.eta_h;
CLalpha_w  = wing.CLalpha;
c_w        = wing.MAC;

%% LAUNCH REQUIREMENT

% Required Cl_h for rotation maneuver after hand launch
Mw   =  Macw + Lw*(xCG - xAC_w) - Dw_launch*(zCG - zAC_w);      % is this moment nose up or nose down? 
MT   = T_launch*(zCG - z_prop);
Mh   = -Mw -MT + Iyy*theta_ddot;
Lh   = Mh/(xCG-xAC_h);
CL_h = Lh/(mission.q_second*S_h);

% Derivation of elevator efficiency (assuming that Cl_h required is achieved with deltaE_max (fix))
alpha_h   = ih + alpha_w -iw  - eps;
tauE      = (alpha_h + (CL_h/CLalpha_h))/deltaE_max;
G         = csvread('control_eff.csv');
cratio_E  = interp1(G(:,2),G(:,1),tauE)

% Derivation of other elevator parameters
cE           = cratio_E*croot_h;
delta_alpha0 = -1.15*cratio_E*deltaE_max;
Cm_deltaE    = - CLalpha_h*eta_h*V_h*bratio_E*tauE;
CL_deltaE    = CLalpha_h*eta_h*S_h/S_w*tauE;

%% ELEVATOR DEFLECTION AT VARIOUS FLIGHT CONDITIONS
rho        = mission.rho;
V          = linspace(10,30,100);
q          = 0.5*rho*V.^2;
CL0        = 0;                                                 % this probably is not zero
CLalpha    = CLalpha_w;                                         % review this
CLw_cruise = wing.CL_cruise;
ls         = stab_long(cg.x,wing,hstab,assumptions);
Cmalpha    = ls.Cmalpha;
deltaE     = ((T_cruise*z_prop./(q*S_w*c_w) + (CLw_cruise - CL0)*Cmalpha)/(CLalpha*Cm_deltaE - Cmalpha*CL_deltaE));

% plot(V,deltaE*rad2deg)

elevator.cE = cE;
end

