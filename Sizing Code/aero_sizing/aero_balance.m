function cg = aero_balance(Df,Lf_nose,Lf_body,Lf_rear,battery_m)

cm2m = 1e-2;
g2kg = 1e-3; 

Lf_nose = Lf_nose/cm2m;
Lf_body = Lf_body/cm2m;
Lf_rear = Lf_rear/cm2m;
Lf      = Lf_nose + Lf_body + Lf_rear;

%battery
weights(1).mass = battery_m;
weights(1).lx   = 14.73;
weights(1).ly   = 5.46;
weights(1).lz   = 4.32;
weights(1).x    = Lf_nose + 0.5*weights(1).lx;
weights(1).y    = 0;
weights(1).z    = 0.5*weights(1).lz;

%marcopolo
weights(2).mass = 12;
weights(2).lx   = 6.4;
weights(2).ly   = 2.24;
weights(2).lz   = 1.27;
weights(2).x    = Lf_nose + weights(1).lx - 0.5*weights(2).lx;
weights(2).y    = 0;
weights(2).z    = 0.5*Df + 0.5*weights(2).lz;

%camera+gimbal
weights(3).mass = 295;
weights(3).lx   = 18.7;
weights(3).ly   = 9.35;
weights(3).lz   = 6.17;
weights(3).x    = Lf_nose + 0.5*Lf_body;
weights(3).y    = 0;
weights(3).z    = 0.5*weights(3).lz;

%ardupilot
weights(4).mass = 28;
weights(4).lx   = 7;
weights(4).ly   = 4.5;
weights(4).lz   = 1.5;
weights(4).x    = Lf_nose + Lf_body - 0.5*weights(4).lx;
weights(4).y    = 0;
weights(4).z    = 0.5*weights(4).lz;

%receiver
weights(5).mass = 4.4;
weights(5).lx   = 4;
weights(5).ly   = 1.9;
weights(5).lz   = 0.9;
weights(5).x    = Lf_nose + Lf_body - 0.5*weights(5).lx;
weights(5).y    = 0;
weights(5).z    = weights(4).lz + 0.5*weights(5).lz;

%telemetry
weights(6).mass = 4;
weights(6).lx   = 5.55;
weights(6).ly   = 2.67;
weights(6).lz   = 1.33;
weights(6).x    = Lf_nose + Lf_body + 0.5*weights(6).lx;
weights(6).y    = 0;
weights(6).z    = 0.5*Df + 0.5*weights(6).lz;

%gps module
weights(7).mass = 16.8;
weights(7).lx   = 3.8;
weights(7).ly   = 3.8;
weights(7).lz   = 0.85;
weights(7).x    = Lf_nose + Lf_body + 0.5*weights(7).lx;
weights(7).y    = 0;
weights(7).z    = 0.5*Df + weights(6).lz + 0.5*weights(7).lz;

%speed controller
weights(8).mass = 25;
weights(8).lx   = 5.5;
weights(8).ly   = 1.9;
weights(8).lz   = 1;
weights(8).x    = Lf_nose + Lf_body + weights(6).lx + 0.5*weights(8).lx ;
weights(8).y    = 0;
weights(8).z    = 0.5*Df + 0.5*weights(8).lz;

%motor
weights(9).mass = 53;
weights(9).lx   = 3.3;
weights(9).ly   = 2.8;
weights(9).lz   = 2.8;
weights(9).x    = Lf - 0.5*weights(9).lx;
weights(9).y    = 0;
weights(9).z    = 0.5*Df + 0.5*weights(9).lz;

%% CG computation
nmasses = size(weights,2);
M = 0; Xcg = 0; Ycg = 0; Zcg = 0;
for i = 1:nmasses
    M   = M + weights(i).mass*g2kg;
    Xcg = Xcg + weights(i).mass*g2kg*weights(i).x*cm2m;
    Ycg = Ycg + weights(i).mass*g2kg*weights(i).y*cm2m;
    Zcg = Zcg + weights(i).mass*g2kg*weights(i).z*cm2m;
end

Xcg = Xcg/M;
Ycg = Ycg/M;
Zcg = Zcg/M;

%% Inertia Tensor computation
Ixx = 0;
Iyy = 0;
for i = 1:nmasses
    xx  = (weights(i).x*cm2m - Xcg)*(weights(i).x*cm2m - Xcg);
    yy  = (weights(i).y*cm2m - Ycg)*(weights(i).y*cm2m - Ycg);
    zz  = (weights(i).z*cm2m - Zcg)*(weights(i).z*cm2m - Zcg);
    m   = weights(i).mass*g2kg;
    Ixx = Ixx + m*(yy+zz);
    Iyy = Iyy + m*(xx + zz);
end

%% OUTPUT STRUCTURE
cg.mass = M;
cg.x    = Xcg;
cg.y    = Ycg;
cg.z    = Zcg;
cg.Iyy  = Iyy;


