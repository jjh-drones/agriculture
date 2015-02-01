%% MISSION REQUIREMENTS
mission.V          = 18.4;
mission.V_second   = 6;
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

%% ASSUMPTIONS
assumptions.weight        = 2;
assumptions.Aw            = 7;
assumptions.Vh            = 0.;
assumptions.Kc            = 1.2;
assumptions.Ah_mul        = 2/3;
assumptions.tail_Clalpha  = 2*pi; 
assumptions.Vv            = 0.04;
assumptions.Av            = 1.5;
assumptions.Df            = 12e-2;
assumptions.mul_nose      = 1.5;
assumptions.mul_rear      = 2.0;
assumptions.Lf_body       = 0.5;
assumptions.wing_pos      = 0.5;
assumptions.eta_h         = 0.98;    
assumptions.tail_Cd0      = 0.00338; %Based on NACA0006
assumptions.StaticMargin  = 0.2;
assumptions.battery_m     = 808;
assumptions.Lf_nose       = assumptions.mul_nose*assumptions.Df;
assumptions.Lf_rear       = assumptions.mul_rear*assumptions.Df;
% assumptions.Lf_rear       = 33;

%% COMPUTATION
wing     = aero_wing(assumptions,mission);

pos      = aero_pos(assumptions,wing);     

fuselage = aero_fuselage(assumptions,mission,wing,pos);

hstab    = aero_hstab(assumptions,mission,wing,pos,fuselage);

vstab    = aero_vstab(wing,hstab,pos,assumptions);

%% VISUALIZATION
m2cm    = 100;
rad2deg = 180/pi;

wing.xAC  = pos.xAC;
wing.xLE  = wing.xAC - wing.xMAC*wing.croot;

%Fuselage
display('FUSELAGE')
display('--------')
fprintf('%15s%15.6f\n','Diameter: ',assumptions.Df*m2cm);
fprintf('%15s%15.6f\n','L_nose: ',assumptions.Lf_nose*m2cm);
fprintf('%15s%15.6f\n','L_body: ',fuselage.Lf_body*m2cm);
fprintf('%15s%15.6f\n','L_rear: ',assumptions.Lf_rear*m2cm);
fprintf('%15s%15.6f\n','L_total: ',fuselage.Lf*m2cm);
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
fprintf('%15s%15.6f\n','Wing: ',wing.D);
fprintf('%15s%15.6f\n','Hstab: ',hstab.D);
fprintf('%15s%15.6f\n','Fuselage: ',fuselage.D);
fprintf('%15s%15.6f\n','Total: ',wing.D + hstab.D + fuselage.D);
fprintf('\n')

%Longitudinal Stability
cg = aero_balance(assumptions.Df,assumptions.Lf_nose,fuselage.Lf_body,assumptions.Lf_rear,assumptions.battery_m);
ls = stab_long(cg.x,pos,wing,hstab,assumptions);
matlab2avl(mission,wing,hstab,cg)
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
cg = aero_balance(assumptions.Df,assumptions.Lf_nose,fuselage.Lf_body,assumptions.Lf_rear,battery_m);
ls = stab_long(cg.x,pos,wing,hstab,assumptions);
display('LONG STABILITY (TEST)')
display('--------')
fprintf('%15s%15.6f\n','Battery Mass: ',battery_m);
fprintf('%15s%15.6f\n','x_CG: ',cg.x*m2cm);
fprintf('%15s%15.6f\n','Cm_alpha: ',ls.Cmalpha);
fprintf('%15s%15.6f\n','NP: ',ls.NP*wing.MAC*m2cm);
fprintf('%15s%15.6f\n','SM: ',ls.SM*wing.MAC*m2cm);
fprintf('\n')





