%% MISSION
mission.V          = 18.4;
mission.name       = 'brick';
mission.V_second   = 7;
mission.H          = 112;
mission.H_second   = 3;
mission.t          = 31*60;
mission.mu         = 16.04*10^(-6);
mission.g          = 9.81;                                
R                  = 287.058;  

% atm                = get_atm(mission.H);
% atm_second         = get_atm(mission.H_second);
atm.rho         = 1.21188;
atm.Ta          = 287.422;
atm_second.rho  = 1.22465;
atm_second.Ta   = 288.130;

mission.rho        = atm.rho;
mission.rho_second = atm_second.rho;
mission.T          = atm.Ta;
mission.T_second   = atm_second.Ta;
mission.a          = (1.4*R*mission.T)^0.5;
mission.a_second   = (1.4*R*mission.T_second)^0.5;
mission.q          = 0.5*mission.rho*mission.V^2;
mission.q_second   = 0.5*mission.rho_second*mission.V_second^2;                         
mission.M          = mission.V/mission.a;
mission.M_second   = mission.V_second/mission.a_second;

% aileron sizing
mission.turn_angle      = 90;
mission.turn_sec        = 0.75;

% elevator sizing
mission.rot_launch_sec  = 1;
mission.thrust_launch   = 5;

%% ASSUMPTIONS

%Direct
assumptions.weight            = 2;
assumptions.Aw                = 7;
assumptions.Vh                = 0.5;
assumptions.Ah_mul            = 2/3;
assumptions.nose_mul          = 1.5;
assumptions.Clalpha_h         = 2*pi;
assumptions.Cd0_h             = 0.00338;  %Based on NACA0006
assumptions.Vv                = 0.025;
assumptions.Av                = 1.5;
assumptions.eta_h             = 0.98;    
assumptions.StaticMargin      = 0.2;
assumptions.battery_m         = 808*1e-3;
assumptions.Df                = 12e-2;
assumptions.back_angle        = 15;
assumptions.tail_height       = 10e-2;
assumptions.Turn_radius       = 20;       %Meters, to fullfil control points
assumptions.Lf_body           = 63e-2;
assumptions.xHinge_h          = 0.75;
assumptions.xMAC_h            = 0.25;     
assumptions.ch_mul            = 0.5;
assumptions.deltaE_max        = -25;
assumptions.bratio_E          = 1;
assumptions.da_max            = 25;    %+- 25 degrees for ailerons
assumptions.b_end_max_perc    = 0.95;  %maximum external position of aileron
assumptions.aileronC_ratio    = 0.25;  %percentage of aileron chord with respect to wing
assumptions.b_beggininga_perc = 0.15;  %start of aileron withing semi span in percentage
assumptions.b_begginingb_perc = 0.8;   %maximum start offset from wing span in percentage
assumptions.airfoil_h         = 'NACA0006';
assumptions.airfoil_v         = 'NACA0006';
assumptions.taper_v           = 0.8;
assumptions.given             = 0;

%Derived
assumptions.Ah            = assumptions.Ah_mul*assumptions.Aw;
assumptions.CLalpha_h     = assumptions.Clalpha_h/(1+(assumptions.Clalpha_h/(pi*assumptions.Ah )));
assumptions.CLalpha_v     = assumptions.CLalpha_h;
assumptions.Lf_nose       = assumptions.nose_mul*assumptions.Df;
assumptions.Lf_rear       = assumptions.Df/tan(assumptions.back_angle*pi/180);
assumptions.tail_Lhinge   = assumptions.tail_height/tan(assumptions.back_angle*pi/180);

%Avl
avl.alpha        = 3;
avl.beta         = 0;
avl.roll_rate    = 0;
avl.pitch_rate   = 0;
avl.yaw_rate     = 0;
avl.aileron      = 0;
avl.elevator     = 0;
avl.rudder       = 0;

%% COMPUTATION

%wing
wing        = aero_wing(assumptions,mission);

xAC_initial = 0.2984;
estFcn      = @(xAC) xAC_finder(xAC,assumptions, wing);
xAC         = fzero(estFcn,xAC_initial);
cg          = aero_balance(assumptions,assumptions.battery_m,wing,xAC,1,1);
wing.xAC    = xAC;
wing.xLE    = wing.xAC - wing.xMAC*wing.croot;

%fuselage
fuselage = aero_fuselage(assumptions,mission,wing);

%tail
hstab    = aero_hstab(assumptions,mission,wing,fuselage,cg);
vstab    = aero_vstab(assumptions,mission,wing,hstab,cg);

%control surfaces
elevator = elevator_sizing(assumptions,mission,wing,fuselage,cg,hstab);
aileron  = aileron_sizing(wing,cg,hstab,vstab,mission,assumptions);
rudder   = rudder_sizing();

%force distributions
mat2excel(wing,'wing');
mat2excel(hstab,'hstab');
mat2excel(vstab,'vstab');

%calling avl
matlab2avl(mission,wing,hstab,vstab,cg,aileron,elevator,rudder)
matlab2avl_launch('brick',avl);

%% VISUALIZATION
m2cm    = 100;
rad2deg = 180/pi;

if 1
fname = 'OUTPUTS\EXTERNAL_LAYOUT';
    
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

k = 1;
B(k, :) = {'FUSELAGE',[]};
k = k+1;
B(k, :) = {'Height: ',assumptions.Df};
k = k+1;
B(k, :) = {'L_nose: ',assumptions.Lf_nose};
k = k+1;
B(k, :) = {'L_body: ',fuselage.Lf_body};
k = k+1;
B(k, :) = {'L_rear: ',assumptions.Lf_rear};
k = k+1;
B(k, :) = {'L_total: ',fuselage.Lf};
k = k+1;
B(k, :) = {'Angle: ',assumptions.back_angle };
k = k+1;
B(k, :) = {[],[]};
k = k+1;
B(k, :) = {[],[]};
k = k+1;

%Wing
display('WING')
display('--------')
fprintf('%15s%15.6f\n','x_LE: ',wing.xLE*m2cm);
fprintf('%15s%15.6f\n','x_AC: ',wing.xAC*m2cm);
fprintf('%15s%15.6f\n','Chord root: ',wing.croot*m2cm);
fprintf('%15s%15.6f\n','Span: ',wing.b*m2cm);
fprintf('\n')

B(k, :) = {'WING',[]};
k = k+1;
B(k, :) = {'x_LE: ',wing.xLE};
k = k+1;
B(k, :) = {'x_AC: ',wing.xAC};
k = k+1;
B(k, :) = {'Chord root: ',wing.croot};
k = k+1;
B(k, :) = {'Span: ',wing.b};
k = k+1;
B(k, :) = {'Aileron_span_start: ',aileron.start};
k = k+1;
B(k, :) = {'Aileron_span_end: ',aileron.start+aileron.span};
k = k+1;
B(k, :) = {'Aileron_chord_ratio: ',assumptions.aileronC_ratio};
k = k+1;
B(k, :) = {'Aileron_angle_max: ',assumptions.da_max};
k = k+1;
B(k, :) = {'Aileron_angle_min: ',-assumptions.da_max};
k = k+1;

B(k, :) = {[],[]};
k = k+1;
B(k, :) = {[],[]};
k = k+1;

%Hstab
display('HSTAB')
display('--------')
fprintf('%15s%15.6f\n','x_LE: ',hstab.xLE*m2cm);
fprintf('%15s%15.6f\n','x_AC: ',hstab.xAC*m2cm);
fprintf('%15s%15.6f\n','z_AC: ',(hstab.zAC-0.05)*m2cm);
fprintf('%15s%15.6f\n','Chord root: ',hstab.croot*m2cm);
fprintf('%15s%15.6f\n','Span: ',hstab.b*m2cm);
fprintf('%15s%15.6f\n','Incidence: ',hstab.incidence*rad2deg);
fprintf('\n')

B(k, :) = {'HSTAB',[]};
k = k+1;
B(k, :) = {'x_LE: ',hstab.xLE};
k = k+1;
B(k, :) = {'x_AC: ',hstab.xAC};
k = k+1;
B(k, :) = {'z_AC: ',(hstab.zAC-0.05)};
k = k+1;
B(k, :) = {'Chord root: ',hstab.croot};
k = k+1;
B(k, :) = {'Span: ',hstab.b};
k = k+1;
B(k, :) = {'Incidence: ',hstab.incidence*rad2deg};
k = k+1;
B(k, :) = {'Elevator_span_start: ',0};
k = k+1;
B(k, :) = {'Elevator_span_end: ',1};
k = k+1;
B(k, :) = {'Elevator_choord_ratio: ',elevator.cratio_E};
k = k+1;
B(k, :) = {'Elevator_angle_maxup: ',elevator.deltaE_up};
k = k+1;
B(k, :) = {'Elevator_angle_maxdown: ',elevator.deltaE_down};
k = k+1;
B(k, :) = {[],[]};
k = k+1;
B(k, :) = {[],[]};
k = k+1;

%Vstab
display('VSTAB')
display('--------')
fprintf('%15s%15.6f\n','x_LE: ',vstab.xLE*m2cm);
fprintf('%15s%15.6f\n','x_AC: ',vstab.xAC*m2cm);
fprintf('%15s%15.6f\n','Chord root: ',vstab.croot*m2cm);
fprintf('%15s%15.6f\n','Span: ',vstab.b*m2cm);
fprintf('\n')

B(k, :) = {'VSTAB',[]};
k = k+1;
B(k, :) = {'x_LE: ',vstab.xLE};
k = k+1;
B(k, :) = {'x_AC: ',vstab.xAC};
k = k+1;
B(k, :) = {'Chord root: ',vstab.croot};
k = k+1;
B(k, :) = {'Span: ',vstab.b};
k = k+1;
B(k, :) = {'Rudder_span_start: ',0};
k = k+1;
B(k, :) = {'Rudder_span_end: ',1};
k = k+1;
B(k, :) = {'Rudder_choord_ratio: ',0.25};
k = k+1;
B(k, :) = {'Rudder_angle_maxup: ',20};
k = k+1;
B(k, :) = {'Rudder_angle_maxdown: ',-20};
k = k+1;
B(k, :) = {[],[]};
k = k+1;
B(k, :) = {[],[]};
k = k+1;

%Drag
display('DRAG BREAKDOWN')
display('--------')
fprintf('%15s%15.6f\n','Wing: ',wing.D_cruise);
fprintf('%15s%15.6f\n','Hstab: ',hstab.D);
fprintf('%15s%15.6f\n','Fuselage: ',fuselage.D);
fprintf('%15s%15.6f\n','Total: ',wing.D_cruise + hstab.D + fuselage.D);
fprintf('\n')

B(k, :) = {'DRAG BREAKDOWN',[]};
k = k+1;
B(k, :) = {'Wing: ',wing.D_cruise};
k = k+1;
B(k, :) = {'Hstab: ',hstab.D};
k = k+1;
B(k, :) = {'Fuselage: ',fuselage.D};
k = k+1;
B(k, :) = {'Total: ',wing.D_cruise + hstab.D + fuselage.D};
k = k+1;
B(k, :) = {[],[]};
k = k+1;
B(k, :) = {[],[]};
k = k+1;

xlswrite(fname,B);

%Longitudinal Stability
cg = aero_balance(assumptions,assumptions.battery_m,wing,wing.xAC,0,0);
ls = stab_long(cg.x,wing,hstab,assumptions);
display('LONG STABILITY (DESIGN)')
display('--------')
fprintf('%15s%15.6f\n','Battery Mass: ',assumptions.battery_m);
fprintf('%15s%15.6f\n','x_CG: ',cg.x*m2cm);
fprintf('%15s%15.6f\n','Cm_alpha: ',ls.Cmalpha);
fprintf('%15s%15.6f\n','NP: ',ls.NP*wing.MAC*m2cm);
fprintf('%15s%15.6f\n','SM: ',ls.SM*wing.MAC*m2cm);
fprintf('\n')

%Longitudinal Test
battery_m = 600e-3;
cg = aero_balance(assumptions,battery_m,wing,wing.xAC,0,0);
ls = stab_long(cg.x,wing,hstab,assumptions);
display('LONG STABILITY (TEST)')
display('--------')
fprintf('%15s%15.6f\n','Battery Mass: ',battery_m);
fprintf('%15s%15.6f\n','x_CG: ',cg.x*m2cm);
fprintf('%15s%15.6f\n','Cm_alpha: ',ls.Cmalpha);
fprintf('%15s%15.6f\n','NP: ',ls.NP*wing.MAC*m2cm);
fprintf('%15s%15.6f\n','SM: ',ls.SM*wing.MAC*m2cm);
fprintf('\n')
end




