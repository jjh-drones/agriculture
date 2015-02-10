%% MISSION REQUIREMENTS
mission.V          = 18.4;
mission.name       = 'brick';
mission.V_second   = 7;
mission.H          = 112;
mission.H_second   = 3;
mission.t          = 31*60;
mission.mu         = 16.04*10^(-6);
mission.g          = 9.81;                                
R                  = 287.058;  
atm                = get_atm(mission.H);
atm_second         = get_atm(mission.H_second);
mission.rho        = atm.rho;
mission.rho_second = atm_second.rho;
mission.T          = atm.Ta;
mission.T_second   = atm_second.Ta;
mission.a          = (1.4*R*mission.T)^0.5;
mission.a_second   = (1.4*R*mission.T_second)^0.5;
mission.q          = 0.5*mission.rho*mission.V^2;
mission.q_second   = 0.5*mission.rho*mission.V_second^2;                         
mission.M          = mission.V/mission.a;
mission.M_second   = mission.V_second/mission.a_second;
mission.turn_angle = 90;
mission.turn_sec   = 0.55;

%% ASSUMPTIONS
assumptions.weight        = 2;
assumptions.Aw            = 7;
assumptions.Vh            = 0.5;
assumptions.Ah_mul        = 2/3;
assumptions.ch_mul        = 0.5;
assumptions.tail_Clalpha  = 2*pi; 
assumptions.Vv            = 0.025;
assumptions.Av            = 1.5;
assumptions.eta_h         = 0.98;    
assumptions.tail_Cd0      = 0.00338; %Based on NACA0006
assumptions.StaticMargin  = 0.2;
assumptions.battery_m     = 808;
assumptions.Df            = 12e-2;
assumptions.back_angle    = 12;
assumptions.tail_height   = 10e-2;
assumptions.Lf_body       = 62e-2;
assumptions.Lf_nose       = 1.5*assumptions.Df;
assumptions.Lf_rear       = assumptions.Df/tan(assumptions.back_angle*pi/180);
assumptions.Turn_radius   = 20; %Meters, to fullfil control points
assumptions.tail_Lhinge   = assumptions.tail_height/tan(assumptions.back_angle*pi/180);


%% COMPUTATION
wing     = aero_wing(assumptions,mission);

pos      = aero_pos2(assumptions,wing);     

fuselage = aero_fuselage(assumptions,mission,wing,pos);

hstab    = aero_hstab(assumptions,mission,wing,pos,fuselage);

vstab    = aero_vstab(wing,hstab,pos,assumptions);

aileron  = aileron_sizing(wing,pos,hstab,vstab,mission,atm); 

%% VISUALIZATION
m2cm    = 100;
rad2deg = 180/pi;

wing.xAC  = pos.xAC;
wing.xLE  = wing.xAC - wing.xMAC*wing.croot;

%Fuselage
display('FUSELAGE')
display('--------')
fprintf('%15s%15.6f\n','Height: ',assumptions.Df*m2cm);
fprintf('%15s%15.6f\n','L_nose: ',assumptions.Lf_nose*m2cm);
fprintf('%15s%15.6f\n','L_body: ',fuselage.Lf_body*m2cm);
fprintf('%15s%15.6f\n','L_rear: ',assumptions.Lf_rear*m2cm);
fprintf('%15s%15.6f\n','L_total: ',fuselage.Lf*m2cm);
fprintf('%15s%15.6f\n','Angle: ',assumptions.back_angle );
fprintf('\n')

%Wing
display('WING')
display('--------')
fprintf('%15s%15.6f\n','x_LE: ',wing.xLE*m2cm);
fprintf('%15s%15.6f\n','x_AC: ',wing.xAC*m2cm);
fprintf('%15s%15.6f\n','Chord root: ',wing.croot*m2cm);
fprintf('%15s%15.6f\n','Span: ',wing.b*m2cm);
fprintf('\n')

%Hstab
display('HSTAB')
display('--------')
fprintf('%15s%15.6f\n','x_LE: ',hstab.xLE*m2cm);
fprintf('%15s%15.6f\n','x_AC: ',hstab.xAC*m2cm);
fprintf('%15s%15.6f\n','z_AC: ',(hstab.zAC-0.05)*m2cm);
fprintf('%15s%15.6f\n','Chord root: ',hstab.croot*m2cm);
fprintf('%15s%15.6f\n','Span: ',hstab.b*m2cm);
fprintf('%15s%15.6f\n','Incidence: ',hstab.ih);
fprintf('\n')

%Vstab
display('VSTAB')
display('--------')
fprintf('%15s%15.6f\n','x_LE: ',vstab.xLE*m2cm);
fprintf('%15s%15.6f\n','x_AC: ',vstab.xAC*m2cm);
fprintf('%15s%15.6f\n','Chord root: ',vstab.croot*m2cm);
fprintf('%15s%15.6f\n','Span: ',vstab.b*m2cm);
fprintf('\n')

%Drag
display('DRAG BREAKDOWN')
display('--------')
fprintf('%15s%15.6f\n','Wing: ',wing.D_cruise);
fprintf('%15s%15.6f\n','Hstab: ',hstab.D);
fprintf('%15s%15.6f\n','Fuselage: ',fuselage.D);
fprintf('%15s%15.6f\n','Total: ',wing.D_cruise + hstab.D + fuselage.D);
fprintf('\n')

%Longitudinal Stability
cg = aero_balance(assumptions.Df,assumptions.Lf_nose,fuselage.Lf_body,assumptions.Lf_rear,assumptions.battery_m,wing);
ls = stab_long(cg.x,pos,wing,hstab,assumptions);
matlab2avl(mission,wing,hstab,cg,aileron)
display('LONG STABILITY (DESIGN)')
display('--------')
fprintf('%15s%15.6f\n','Battery Mass: ',assumptions.battery_m);
fprintf('%15s%15.6f\n','x_CG: ',cg.x*m2cm);
fprintf('%15s%15.6f\n','Cm_alpha: ',ls.Cmalpha);
fprintf('%15s%15.6f\n','NP: ',ls.NP*wing.MAC*m2cm);
fprintf('%15s%15.6f\n','SM: ',ls.SM*wing.MAC*m2cm);
fprintf('\n')

%Longitudinal Test
battery_m = 600;
cg = aero_balance(assumptions.Df,assumptions.Lf_nose,fuselage.Lf_body,assumptions.Lf_rear,battery_m,wing);
ls = stab_long(cg.x,pos,wing,hstab,assumptions);
display('LONG STABILITY (TEST)')
display('--------')
fprintf('%15s%15.6f\n','Battery Mass: ',battery_m);
fprintf('%15s%15.6f\n','x_CG: ',cg.x*m2cm);
fprintf('%15s%15.6f\n','Cm_alpha: ',ls.Cmalpha);
fprintf('%15s%15.6f\n','NP: ',ls.NP*wing.MAC*m2cm);
fprintf('%15s%15.6f\n','SM: ',ls.SM*wing.MAC*m2cm);
fprintf('\n')





