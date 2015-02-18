function vstab = aero_vstab(assumptions,mission,wing,hstab,cg)

deg2rad = pi/180; 
rad2deg = 180/pi;

%% PARAMETERS
b_w         = wing.b;
S_w         = wing.S;
taper_w     = wing.taper;
Vv          = assumptions.Vv;
A_v         = assumptions.Av;
lv          = hstab.lh;
xCG         = cg.x;
CLalpha_v   = assumptions.CLalpha_h;


%% TRIM

% Calculate vertical tail planform area (based on the definition of hstab volume coefficient)
S_v = b_w*S_w*Vv/lv;

% Vertical tail geometry calculations
MAC_v   = sqrt(S_v/A_v);
b_v     = A_v*MAC_v;
taper_v = taper_w;
root_v  = 3/2*MAC_v*((1+taper_v)/(1+taper_v+taper_v^2));
tip_v   = taper_v*root_v;

%% OUTPUT STRUCTURE
vstab.croot         = root_v;
vstab.b             = b_v;
vstab.S             = S_v;
vstab.xLE           = xCG + lv - wing.xMAC*root_v;
vstab.xAC           = vstab.xLE + wing.xMAC*vstab.croot;
vstab.airfoil_name  = assumptions.airfoil_v;
vstab.taper         = assumptions.taper_v;

%% LIFT DISTRIBUTION
beta_v      = 8*deg2rad;
CL_v        = CLalpha_v*beta_v;
L_v         = mission.q*S_v*CL_v;
vstab.L_max = L_v;

end

