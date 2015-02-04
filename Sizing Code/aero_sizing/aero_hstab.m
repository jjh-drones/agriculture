function hstab    = aero_hstab(assumptions,mission,wing,pos,fuselage)

deg2rad = pi/180; 
rad2deg = 180/pi;

%% PARAMETERS
CL_w          = wing.CL_cruise;
CLalpha_w     = wing.CLalpha;
Cmo_wf        = wing.CM;
S_w           = wing.S;
A_w           = wing.A;
c_w           = wing.MAC;
taper_w       = wing.taper;
alpha_w       = wing.alpha_cruise;
incidence_w   = wing.incidence;
Df            = fuselage.Df;
Lf_nose       = assumptions.Lf_nose;
Lf_body       = assumptions.Lf_body;
V_h           = assumptions.Vh;
eta_h         = assumptions.eta_h;
Clalpha_h     = assumptions.tail_Clalpha;
CD0_h         = assumptions.tail_Cd0;
q             = mission.q;
L_hinge       = assumptions.tail_Lhinge;
xCG           = pos.xCG;
zCG           = pos.zCG;

Ah_mul = assumptions.Ah_mul; 
ch_mul = assumptions.ch_mul;

%% LONGITUDINAL TRIM

% Locate wing aerodynamic center
xAC_norm      = pos.xAC/c_w;
xCG_norm      = pos.xCG/c_w;

% Calculate tail moment arm:
l_h  = Lf_nose + Lf_body + L_hinge - 0.5*(ch_mul*c_w) - pos.xCG;

% Calculate horizontal tail planform area (based on the definition of hstab volume coefficient)
S_h = c_w*S_w*V_h/l_h;

% Calculate horizontal tail desired lift coefficient at cruise from trim (based on longitudinal trim equation)
CL_h = (Cmo_wf + CL_w*(xCG_norm - xAC_norm))/(V_h*eta_h);

% Horizontal tail geometry calculations
A_h      = Ah_mul*A_w;
MAC_h    = sqrt(S_h/A_h);
b_h      = A_h*MAC_h;
taper_h  = taper_w;
croot_h  = 3/2*MAC_h*((1+taper_h)/(1+taper_h+taper_h^2));

%Airfoil selection (fixed)
CLalpha_h = Clalpha_h/(1+(Clalpha_h/(pi*A_h)));

%Angle of attack of the tail during cruise
alpha_h_initial = CL_h/Clalpha_h;
estFcn          = @(alpha_h) lifting_line(alpha_h,S_h,A_h,taper_h,CLalpha_h,CL_h);
alpha_h         = fzero(estFcn,alpha_h_initial);

%Incidence angle
eps0        = 2*CL_w/(pi*A_w);
eps_dalpha  = 2*CLalpha_w/(pi*A_w);
eps         = eps0 + eps_dalpha*alpha_w;
incidence_h = alpha_h - alpha_w + incidence_w + eps;

% Positioning
xLE_h       = pos.xCG + l_h - wing.xMAC*croot_h;
xAC_h       = xLE_h + wing.xMAC*croot_h;
zAC_h       = assumptions.tail_height;

%% DRAG COMPUTATION
s         = 1-2*(Df/b_h)^2;
u         = 0.99;                                     
e_invicid = 1/(u*s);
K         = 0.38;                                      
e_viscous = K*CD0_h;
e_total   = (e_invicid + e_viscous*pi*A_h)^(-1);
K_surf    = (pi*A_h*e_total)^(-1);
CD_h      = CD0_h + (K_surf)*(CL_h)^2;
D_h       = q*S_h*CD_h;

%% ELEVATOR SIZING (TAKE-OFF)
T_excess   = 5;
Iyy        = pos.Iyy;
theta_ddot = 10*deg2rad;
xAC_w      = pos.xAC;
zAC_w      = Df/2;
z_prop     = Df/2;
Lw         = wing.L_launch;       
Dw         = wing.D_launch; 
Macw       = wing.M_launch;
T          = Dw + T_excess;

Mw   =  Macw + Lw*(xCG - xAC_w) - Dw*(zCG - zAC_w);
MT   = T*(zCG - z_prop);
Mh   = -Mw -MT + Iyy*theta_ddot;
Lh   = Mh/(xCG-xAC_h);
CL_h = Lh/(mission.q_second*S_w);

alpha_w     = 0;
CL_w        = wing.CL_launch;
eps0        = 2*CL_w/(pi*A_w);
eps_dalpha  = 2*CLalpha_w/(pi*A_w);
eps         = eps0 + eps_dalpha*alpha_w;
alpha_h     = alpha_w + incidence_h - eps;

deltaE_max  = -15*deg2rad;
tauE        = (alpha_h + (CL_h/CLalpha_h))/deltaE_max;

%% OUTPUT STRUCTURE
hstab.S          = S_h;
hstab.b          = b_h;
hstab.lh         = l_h;
hstab.croot      = croot_h;
hstab.D          = D_h;
hstab.ih         = incidence_h*180/pi;
hstab.CLalpha    = CLalpha_h;
hstab.eps_dalpha = eps_dalpha;
hstab.xLE        = xLE_h;
hstab.xAC        = xAC_h;
hstab.zAC        = zAC_h;


