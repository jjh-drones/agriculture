function hstab    = aero_hstab(assumptions,mission,wing,fuselage,cg)

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
iw            = wing.incidence;
Df            = fuselage.Df;
Lf_nose       = assumptions.Lf_nose;
Lf_body       = assumptions.Lf_body;
V_h           = assumptions.Vh;
eta_h         = assumptions.eta_h;
CD0_h         = assumptions.Cd0_h;
q             = mission.q;
L_hinge       = assumptions.tail_Lhinge;
xCG           = cg.x;
zCG           = cg.z;
xAC_w         = wing.xAC;
xHinge_h      = assumptions.xHinge_h;
xMAC_h        = assumptions.xMAC_h;
ch_mul        = assumptions.ch_mul;
A_h           = assumptions.Ah;
CLalpha_h     = assumptions.CLalpha_h;
eps           = wing.eps_cruise;


%% LONGITUDINAL TRIM

% Locate wing aerodynamic center
xAC_norm      = xAC_w/c_w;
xCG_norm      = xCG/c_w;

% Calculate tail moment arm:
l_h  = Lf_nose + Lf_body + L_hinge - (xHinge_h-xMAC_h)*(ch_mul*c_w) - xCG;

% Calculate horizontal tail planform area (based on the definition of hstab volume coefficient)
S_h = c_w*S_w*V_h/l_h;

% Calculate horizontal tail desired lift coefficient at cruise from trim (based on longitudinal trim equation)
CL_h = (Cmo_wf + CL_w*(xCG_norm - xAC_norm))/(V_h*eta_h);

% Horizontal tail geometry calculations
MAC_h    = sqrt(S_h/A_h);
b_h      = A_h*MAC_h;
taper_h  = taper_w;
croot_h  = 3/2*MAC_h*((1+taper_h)/(1+taper_h+taper_h^2));

%Angle of attack of the tail during cruise
alpha_h_initial = CL_h/CLalpha_h;
estFcn          = @(alpha_h) lifting_line(alpha_h,S_h,A_h,taper_h,CLalpha_h,CL_h);
alpha_h         = fzero(estFcn,alpha_h_initial);

%Incidence angle
ih = alpha_h - alpha_w + iw + eps;

% Positioning
xLE_h       = xCG + l_h - xMAC_h*croot_h;
xAC_h       = xLE_h + xMAC_h*croot_h;

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

%% OUTPUT STRUCTURE
hstab.S             = S_h;
hstab.b             = b_h;
hstab.lh            = l_h;
hstab.croot         = croot_h;
hstab.D             = D_h;
hstab.incidence     = ih;
hstab.CLalpha       = CLalpha_h;
hstab.xLE           = xLE_h;
hstab.xAC           = xAC_h;
hstab.airfoil_name  = assumptions.airfoil_h;

%% LIFT DISTRIBUTION
alpha_h     = 8*deg2rad;
CL_h        = CLalpha_h*alpha_h;
L_h         = mission.q*S_h*CL_h;
hstab.L_max = L_h;


