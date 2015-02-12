function elevator = elevator_sizing(assumptions,mission,wing,fuselage,cg,hstab)
deg2rad = pi/180; 
rad2deg = 180/pi;

%% PARAMETERS
deltaE_max = assumptions.deltaE_max*deg2rad;
bratio_E   = assumptions.bratio_E;
Iyy        = cg.Iyy;
xCG        = cg.x;
zCG        = cg.z;
xAC_w      = wing.xAC;
xAC_h      = hstab.xAC;
z_prop     = assumptions.Df/2;     
Dw_cruise  = wing.D_cruise;
ih         = hstab.incidence;
iw         = wing.incidence;
CLalpha_h  = assumptions.CLalpha_h;
croot_h    = hstab.croot;
S_h        = hstab.S;
V_h        = assumptions.Vh;
eta_h      = assumptions.eta_h;
CL0        = 0.1;

%% LAUNCH REQUIREMENT

% Required Cl_h for rotation maneuver after hand launch
rot_launch_sec = mission.rot_launch_sec;
pitch_ang_rate = wing.alpha_launch/rot_launch_sec^2;
q              = mission.q_second;
S_w            = wing.S;
c_w            = wing.MAC; 
A_w            = wing.A;
CLalpha_w      = wing.CLalpha;
alpha_w        = 0;                                   
CL             = CL0;                       
CM             = wing.CM;
Lw             = q*S_w*CL; 
Macw           = q*S_w*c_w*CM;
T              = mission.thrust_launch;

Mw   =  Macw + Lw*(xCG - xAC_w);
MT   = T*(zCG - z_prop);
Mh   = -Mw -MT + Iyy*pitch_ang_rate;
Lh   = Mh/(xCG-xAC_h);
CL_h = Lh/(mission.q_second*S_h);

% Derivation of elevator efficiency (assuming that Cl_h required is achieved with deltaE_max (fix))

eps0       = 2*CL/(pi*A_w);
eps_dalpha = 2*wing.CLalpha/(pi*wing.A);
eps        = wing.eps0_launch + wing.eps_dalpha*wing.alpha_launch;
alpha_h    = ih + alpha_w - iw  - eps;
tauE       = (alpha_h + (CL_h/CLalpha_h))/deltaE_max;
G          = csvread('control_eff.csv');
cratio_E   = interp1(G(:,2),G(:,1),tauE)

% Derivation of other elevator parameters
cE           = cratio_E*croot_h;
delta_alpha0 = -1.15*cratio_E*deltaE_max;
Cm_deltaE    = - CLalpha_h*eta_h*V_h*bratio_E*tauE;
CL_deltaE    = CLalpha_h*eta_h*S_h/S_w*tauE;

%% ELEVATOR DEFLECTION AT VARIOUS FLIGHT CONDITIONS
T_cruise   = Dw_cruise + fuselage.D + hstab.D;
rho        = mission.rho;
V          = linspace(5,30,100);
q          = 0.5*rho*V.^2;                                                 % this probably is not zero
CLalpha    = CLalpha_w;                                         % review this
CLw_cruise = wing.CL_cruise;
ls         = stab_long(cg.x,wing,hstab,assumptions);
Cmalpha    = ls.Cmalpha;
deltaE     = ((T_cruise*z_prop./(q*S_w*c_w) + (CLw_cruise - CL0)*Cmalpha)/(CLalpha*Cm_deltaE - Cmalpha*CL_deltaE));

plot(V,deltaE*rad2deg)

elevator.cE = cE;
end

