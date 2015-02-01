function vstab = aero_vstab(wing,hstab,pos,assumptions)

deg2rad = pi/180; 
rad2deg = 180/pi;

%% PARAMETERS
b_w     = wing.b;
S_w     = wing.S;
taper_w = wing.taper;
Vv      = assumptions.Vv;
A_v     = assumptions.Av;
lv      = hstab.lh;

%% TRIM

% Calculate vertical tail planform area (based on the definition of hstab volume coefficient)
S_v = b_w*S_w*Vv/lv;

%Airfoil selection (fixed)
Clalpha_v = 2*pi;
CLalpha_v = Clalpha_v/(1+(Clalpha_v/(pi*A_v)));

% Vertical tail geometry calculations
MAC_v   = sqrt(S_v/A_v);
b_v     = A_v*MAC_v;
taper_v = taper_w;
root_v  = 3/2*MAC_v*((1+taper_v)/(1+taper_v+taper_v^2));
tip_v   = taper_v*root_v;

%% OUTPUT STRUCTURE
vstab.croot = root_v;
vstab.b     = b_v;
vstab.xLE   = pos.xCG + lv - wing.xMAC*root_v;
vstab.xAC   = vstab.xLE + wing.xMAC*vstab.croot;

end

