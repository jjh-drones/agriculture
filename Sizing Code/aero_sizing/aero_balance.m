function cg = aero_balance(assumptions,battery_m,wing,xAC,copy,massf)

fname = 'OUTPUTS\INTERNAL_LAYOUT';

cm2m = 1e-2;
g2kg = 1e-3; 
k    = 0;

Df      = assumptions.Df;
Lf_nose = assumptions.Lf_nose;
Lf_body = assumptions.Lf_body;
Lf_rear = assumptions.Lf_rear;
Lf      = Lf_nose + Lf_body + Lf_rear;

dens_foam        = 20.824;
density_carbon   = 1480;

%battery
k = k + 1;
weights(k).name = 'Battery';
weights(k).mass = battery_m;
weights(k).lx   = 14.73*cm2m;
weights(k).ly   = 5.46*cm2m;
weights(k).lz   = 4.32*cm2m;
weights(k).x    = Lf_nose + 0.5*weights(1).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*weights(1).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%marcopolo
k = k + 1;
weights(k).name = 'MarcoPolo';
weights(k).mass = 12*g2kg;
weights(k).lx   = 6.4*cm2m;
weights(k).ly   = 2.24*cm2m;
weights(k).lz   = 1.27*cm2m;
weights(k).x    = Lf_nose + weights(1).lx - 0.5*weights(2).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*Df + 0.5*weights(2).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%camera+gimbal
k = k + 1;
weights(k).name = 'Camera+Gimbal';
weights(k).mass = 295*g2kg;
weights(k).lx   = 18.7*cm2m;
weights(k).ly   = 9.35*cm2m;
weights(k).lz   = 6.17*cm2m;
weights(k).x    = Lf_nose + 0.5*Lf_body;
weights(k).y    = 0;
weights(k).z    = 0.5*weights(3).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%ardupilot
k = k + 1;
weights(k).name = 'Ardupilot';
weights(k).mass = 28*g2kg;
weights(k).lx   = 7*cm2m;
weights(k).ly   = 4.5*cm2m;
weights(k).lz   = 1.5*cm2m;
weights(k).x    = Lf_nose + Lf_body - 0.5*weights(4).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*weights(4).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%receiver
k = k + 1;
weights(k).name = 'Receiver';
weights(k).mass = 4.4*g2kg;
weights(k).lx   = 4*cm2m;
weights(k).ly   = 1.9*cm2m;
weights(k).lz   = 0.9*cm2m;
weights(k).x    = Lf_nose + Lf_body - 0.5*weights(5).lx;
weights(k).y    = 0;
weights(k).z    = weights(4).lz + 0.5*weights(5).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%telemetry
k = k + 1;
weights(k).name = 'Receiver';
weights(k).mass = 4*g2kg;
weights(k).lx   = 5.55*cm2m;
weights(k).ly   = 2.67*cm2m;
weights(k).lz   = 1.33*cm2m;
weights(k).x    = Lf_nose + Lf_body + 0.5*weights(6).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*Df + 0.5*weights(6).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%gps module
k = k + 1;
weights(k).name = 'GPS_module';
weights(k).mass = 16.8*g2kg;
weights(k).lx   = 3.8*cm2m;
weights(k).ly   = 3.8*cm2m;
weights(k).lz   = 0.85*cm2m;
weights(k).x    = Lf_nose + Lf_body + 0.5*weights(7).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*Df + weights(6).lz + 0.5*weights(7).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%speed controller
k = k + 1;
weights(k).name = 'Speed Controller';
weights(k).mass = 25*g2kg;
weights(k).lx   = 5.5*cm2m;
weights(k).ly   = 1.9*cm2m;
weights(k).lz   = 1*cm2m;
weights(k).x    = Lf_nose + Lf_body + weights(6).lx + 0.5*weights(8).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*Df + 0.5*weights(8).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

%motor
k = k + 1;
weights(k).name = 'Motor';
weights(k).mass = 53*g2kg;
weights(k).lx   = 3.3*cm2m;
weights(k).ly   = 2.8*cm2m;
weights(k).lz   = 2.8*cm2m;
weights(k).x    = Lf - 0.5*weights(9).lx;
weights(k).y    = 0;
weights(k).z    = 0.5*Df + 0.5*weights(9).lz;
weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);


if assumptions.given
    Data = xlsread('OUTPUTS\NEW_WEIGHT');
    
    k = k + 1;
    weights(k).lx    = 0;
    weights(k).ly    = 0;
    weights(k).lz    = 0;
    weights(k).mass  = Data(1);
    weights(k).x     = Data(2);
    weights(k).y     = Data(3);
    weights(k).z     = Data(4);
    weights(k).Ix    = Data(5);
    weights(k).Iy    = Data(6);
    weights(k).Iz    = Data(7);
else
    %main wing
    k = k + 1;
    airfoil_thicknes = 0.12;
    rod_r            = 0.004;
    weights(k).lx    = wing.croot;
    weights(k).ly    = wing.b;
    weights(k).lz    = wing.croot*airfoil_thicknes;
    weights(k).mass  = dens_foam*weights(k).lx*weights(k).ly*weights(k).lz + 2*density_carbon*pi*rod_r^2*weights(k).ly;
    weights(k).x     = xAC + 0.2*weights(k).lx;
    weights(k).y     = 0;
    weights(k).z     = 0.5*Df + 0.5*weights(10).lz;
    weights(k).Ix    = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
    weights(k).Iy    = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
    weights(k).Iz    = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

    %hstab
    k = k + 1;
    airfoil_thicknes = 0.06;
    rod_r            = 0.004;
    weights(k).lx    = assumptions.ch_mul*wing.croot;
    weights(k).ly    = assumptions.ch_mul*wing.croot*assumptions.Ah;
    weights(k).lz    = assumptions.ch_mul*wing.croot*airfoil_thicknes;
    weights(k).mass  = dens_foam*weights(k).lx*weights(k).ly*weights(k).lz + 2*density_carbon*pi*rod_r^2*weights(k).ly;
    weights(k).x     = assumptions.tail_Lhinge - (assumptions.xHinge_h-assumptions.xMAC_h)*assumptions.ch_mul*wing.croot;
    weights(k).y     = 0;
    weights(k).z     = assumptions.tail_height - 0.5*Df;
    weights(k).Ix    = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
    weights(k).Iy    = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
    weights(k).Iz    = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

    %vstab
    k = k + 1;
    airfoil_thicknes = 0.06;
    rod_r            = 0.004;
    weights(k).lx    = assumptions.ch_mul*wing.croot;
    weights(k).ly    = assumptions.ch_mul*wing.croot*airfoil_thicknes;
    weights(k).lz    = 0.5*assumptions.ch_mul*wing.croot*assumptions.Ah;
    weights(k).mass  = dens_foam*weights(k).lx*weights(k).ly*weights(k).lz + 2*density_carbon*pi*rod_r^2*weights(k).lz;
    weights(k).x     = assumptions.tail_Lhinge - (assumptions.xHinge_h-assumptions.xMAC_h)*assumptions.ch_mul*wing.croot;
    weights(k).y     = 0;
    weights(k).z     = Df/2 + assumptions.xHinge_h*weights(k).lz;
    weights(k).Ix    = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
    weights(k).Iy    = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
    weights(k).Iz    = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);

    %fuselage
    k = k + 1;
    weights(k).lx   = Lf;
    weights(k).ly   = Df;
    weights(k).lz   = Df;
    weights(k).mass = dens_foam*weights(k).lx*weights(k).ly*weights(k).lz + 4*density_carbon*pi*rod_r^2*weights(k).lx;
    weights(k).x    = 0.5*weights(k).lx;
    weights(k).y    = 0;
    weights(k).z    = 0;
    weights(k).Ix   = (1/12)*weights(k).mass*((weights(k).ly)^2+(weights(k).lz)^2);
    weights(k).Iy   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).lz)^2);
    weights(k).Iz   = (1/12)*weights(k).mass*((weights(k).lx)^2+(weights(k).ly)^2);
end

%% CG computation
nmasses = size(weights,2);
M = 0; Xcg = 0; Ycg = 0; Zcg = 0;
for i = 1:nmasses
    M   = M + weights(i).mass;
    Xcg = Xcg + weights(i).mass*weights(i).x;
    Ycg = Ycg + weights(i).mass*weights(i).y;
    Zcg = Zcg + weights(i).mass*weights(i).z;
end

Xcg = Xcg/M;
Ycg = Ycg/M;
Zcg = Zcg/M;

%% Inertia Tensor computation
Ixx = 0;
Iyy = 0;
Izz = 0;
for i = 1:nmasses
    xx  = (weights(i).x - Xcg)*(weights(i).x - Xcg);
    yy  = (weights(i).y - Ycg)*(weights(i).y - Ycg);
    zz  = (weights(i).z - Zcg)*(weights(i).z - Zcg);
    m   = weights(i).mass;
    Ixx = Ixx + m*(yy + zz) + weights(i).Ix;
    Iyy = Iyy + m*(xx + zz) + weights(i).Iy;
    Izz = Izz + m*(xx + yy) + weights(i).Iz;
end
%% OUTPUT STRUCTURE
cg.mass = M;
cg.x    = Xcg;
cg.y    = Ycg;
cg.z    = Zcg;
cg.Ixx  = Ixx;
cg.Iyy  = Iyy;
cg.Izz  = Izz;

%% Write report
k  = 1;
nc = 10;
while k < nc
    A(nc*(k-1)+ 1, :) = {'NAME: ',weights(k).name};
    A(nc*(k-1)+ 2, :) = {'MASS: ',weights(k).mass};
    A(nc*(k-1)+ 3, :) = {'LX: ',weights(k).lx};
    A(nc*(k-1)+ 4, :) = {'LY: ',weights(k).ly};
    A(nc*(k-1)+ 5, :) = {'LZ: ',weights(k).lz};
    A(nc*(k-1)+ 6, :) = {'X: ',weights(k).x};
    A(nc*(k-1)+ 7, :) = {'Y: ',weights(k).y};
    A(nc*(k-1)+ 8, :) = {'Z: ',weights(k).z};
    k = k+1;
1;
end

if copy
    xlswrite(fname,A);
end

%% Export to AVL Mass file
if massf == 1,
    create_mass_avl_file(weights);
end
end




