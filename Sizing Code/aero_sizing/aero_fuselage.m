function fuselage = aero_fuselage(assumptions,mission,wing,pos)

cm2m = 1e-2;
g2kg = 1e-3;

%% PARAMETERS
S_w      = wing.S; 
Df       = assumptions.Df;     
q        = mission.q;
V        = mission.V;
mu       = mission.mu;

%% FUSELAGE LENGHT
Lf_nose  = assumptions.Lf_nose;
Lf_rear  = assumptions.Lf_rear;
Lf_body  = pos.Lf_body;
Lf       = Lf_nose + Lf_rear + Lf_body;

%% ZERO-LIFT DRAG COEFFICIENT

% Slenderness
slender = Lf/Df;

% Wing/fuselage interference factor
Rwf = 1;

% Turbulent flat plate skin friction coefficient of the fuselage
Re = V*Lf/mu; 
Cf = 0.455/(log10(Re))^2.58;

% Wetted area of the fuselage
Swet = fus_wetted(Df,Lf_nose,Lf_body,Lf_rear);

% Fuselage zero-lift drag coefficient
CD0 = Rwf*Cf*(1+60/(slender^3) + 0.0025*slender)*Swet/S_w;
D   = q*S_w*CD0;

% tail_LE_distance         = Lf_rear; %m, from some reference of the fuselage box
% fuselage_nature          = [1,Df,Df,Lf];  %So, the fuselage_nature format is: [type,radius/base of ellipse/width,0/height of ellipse/height,lenght of body]
% [CD_Fuselage,D_Fuselage] = Drag_Fuselage(V,S_w,tail_LE_distance,fuselage_nature);

%% OUTPUT STRUCTURE
fuselage.Lf       = Lf;
fuselage.Lf_nose  = Lf_nose;
fuselage.Lf_body  = Lf_body;
fuselage.Lf_rear  = Lf_rear;
fuselage.Df       = Df;
fuselage.CD0      = CD0;
fuselage.D        = D;


end

