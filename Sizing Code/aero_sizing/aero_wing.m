function wing = aero_wing(assumptions,mission)

Airfoil_List           = {'Eppler_560';'SD7062';'NACA0009';'W_FX60'};
target_speed           = mission.V;             %meters per second'
secondary_target_speed = mission.V_second;      %meters per second
weight                 = assumptions.weight;    %kilograms
fuselage_thickness     = assumptions.Df ;
AR                     = assumptions.Aw;        %balance between efficiency and constructibility

%%Internal Variables
g                   = mission.g;
q                   = mission.q;
q_secondary         = mission.q_second;
kinematic_viscosity = mission.mu;
M                   = mission.M;
delta_CL            = 0.0;                                   %Additional CL with FLAPS
Re_min              = 100000;                                %other values NOT POSSIBLE
Re_max              = 1000000;
step_size           = 2000;
delta_Re            = floor((Re_max-Re_min)/step_size);
Re_options          = [50000, 100000, 200000, 500000, 1000000];
delta               = 0.25;

%%Manouverability Calculations

wing.n = (assumptions.Turn_radius*mission.g/(mission.V)^2)^(-2)+1;

if 1
for kk = 1:size(Airfoil_List,1),
    normalized_params = [];
    P                 = zeros(250,7,size(Re_options,2));
    airfoil_name      = char(Airfoil_List(kk));

    for i = 1:size(Re_options,2),
        fileID = fopen([pwd,'\',airfoil_name,'\',num2str(Re_options(i)),'.txt']);
        p = cell2mat(textscan(fileID,'%f %f %f %f %f %f %f','headerlines',12));
        P(1:size(p,1),1:size(p,2),i) = p; 
    end
    end_Param = zeros(size(P,3),1);
    for i = 1:size(P,3)
        flag = 0;
        for j = 1:size(P,1)-1
            if abs(P(j+1,1,i)-P(j,1,i)) == 0 && flag == 0,
                end_Param(i,1) = j-1;
                flag = 1;
            end
        end
    end

    for i = 1:size(P,3)
        for j = 1:size(P,1)-1
            if abs(P(j+1,1,i)-P(j,1,i)) > delta && j+1<=end_Param(i,1),
                P(j+2:end_Param(i,1)+1,:,i) = P(j+1:end_Param(i,1),:,i);
                P(j+1,:,i) = [mean([P(j,1,i),P(j+2,1,i)]),mean([P(j,2,i),P(j+2,2,i)]),mean([P(j,3,i),P(j+2,3,i)]),mean([P(j,4,i),P(j+2,4,i)]),...
                    mean([P(j,5,i),P(j+2,5,i)]),mean([P(j,6,i),P(j+2,6,i)]),mean([P(j,7,i),P(j+2,7,i)])];
            end
        end
        CL_Cutoff(i) = 0.85*max(P(:,2,i))+delta_CL; %Find the Cutoff CL Max of the aircraft
    end
    U = min(max(P(:,1,:)));
    L = max(min(P(:,1,:)));
    angle_min = L; %controlling the range of survey for the look up tables on the airfoils
    angle_max = U;
    Tag = zeros(2,size(P,3));
    for i = 1:size(P,3)
        for j = 1:size(P,1)
            if P(j,1,i) == angle_min,
                Tag(1,i) = j;
            end
            if P(j,1,i) == angle_max,
                Tag(2,i) = j;
            end
        end
        normalized_params(:,:,i) = P(Tag(1,i):Tag(2,i),:,i);
    end

    cla_alpha_min = -3;

    cla_alpha_max = 7;

    Tag = zeros(2,1);
    Tag2 = zeros(2,1);
    kfactor_min = -5;
    kfactor_max = 5;

    for j = 1:size(normalized_params,1)
        if normalized_params(j,1,1) == cla_alpha_min,
            Tag(1,1) = j;
        end
        if normalized_params(j,1,1) == cla_alpha_max,
            Tag(2,1) = j;
        end
        if normalized_params(j,1,1) == kfactor_min,
            Tag2(1,1) = j;
        end
        if normalized_params(j,1,1) == kfactor_max,
            Tag2(2,1) = j;
        end
    end
    Cl0 = zeros(4,size(normalized_params,3)); %[index, alpha at Cl = 0, 0, Cl at alpha=0]
    Cla = zeros(1,size(normalized_params,3)); %infinite wing lift curve slope
    for j = 1:size(normalized_params,3)
        small = 1000000;
        x = normalized_params(Tag(1,1):Tag(2,1),1,j);
        y = normalized_params(Tag(1,1):Tag(2,1),2,j);
        cx = cov(x,y);
        vx = var(x);
        Cla(1,j) = cx(1,2)/vx;
        for i = 1:size(normalized_params,1)
            if abs(normalized_params(i,2,j)) <= small,
                small = abs(normalized_params(i,2,j));
                Cl0(1,j) = i;
                Cl0(2,j) = normalized_params(i,1,j);
                Cl0(3,j) = normalized_params(i,2,j);
            end    
            if abs(normalized_params(i,1,j)) == 0,
                Cl0(4,j) = normalized_params(i,2,j);
            end
        end
        if Cl0(3,j) < 0,
            Cl0(2,j) = interp1([normalized_params(Cl0(1,j),2,j),normalized_params(Cl0(1,j)+1,2,j)],[normalized_params(Cl0(1,j),1,j),normalized_params(Cl0(1,j)+1,1,j)],0);
            Cl0(3,j) = 0;
        elseif Cl0(3,j) > 0,
            Cl0(2,j) = interp1([normalized_params(Cl0(1,j)-1,2,j),normalized_params(Cl0(1,j),2,j)],[normalized_params(Cl0(1,j)-1,1,j),normalized_params(Cl0(1,j),1,j)],0);
            Cl0(3,j) = 0;
        end
    end
        %%Calculate the K Factor
    for j = 1:size(normalized_params,3)
        for i = 1:size(normalized_params,1)
            if normalized_params(i,1,j) == 0,
                zero_a = i;
            end
        end
        Cl_0(j) = normalized_params(zero_a,2,j);
        for i = 1:size(normalized_params,1)
            Cl_Diff(i) = (normalized_params(i,2,j)-Cl_0(j))^2;
            Cd_nom(i)  = normalized_params(i,3,j);
        end
        Cl_Diff = Cl_Diff';
        Cd_nom = Cd_nom';
        x = Cl_Diff(Tag2(1,1):Tag2(2,1));
        y = Cd_nom(Tag2(1,1):Tag2(2,1));
        cx = cov(x,y);
        vx = var(x);
        k_factor(j) = cx(1,2)/vx;
    end    
    G = 1;
    %% Run calculations
    for i = Re_min:delta_Re:Re_max,
        chord(G)            = i*kinematic_viscosity/(target_speed);
        wing_area(G)        = AR*chord(G)^2;    %Assuming rectangular wing
        span(G)             = AR*chord(G);
        CL_req(G)           = g*weight/(q*wing_area(G));                  
        CL_req_secondary(G) = g*weight/(q_secondary*wing_area(G));
        Cla_temp(G)         = interp1(Re_options,Cla(1,:),i)*180/(pi);
        K_Factor            = interp1(Re_options,k_factor(1,:),i);
        a0_temp             = interp1(Re_options,Cl0(2,:),i);
        CL_Cutoff_Specific  = interp1(Re_options,CL_Cutoff,i);
        k                   = Cla_temp(G)/(2*pi); 
        CLa(G)              = ((2*pi*AR)/(2+(((AR^2*(1-M^2)/k^2))+4)^0.5))*(pi/180);
        alpha_cruise(G)     = (CL_req(G))/CLa(G) + a0_temp;
        CL_req_secondary(G) = g*weight/(q_secondary*wing_area(G));
        alpha_takeoff(G)    = (CL_req_secondary(G))/CLa(G) + a0_temp;
        
        if CL_req_secondary(G) > CL_Cutoff_Specific,
            alpha_cruise(G) = 35;   %If the secondary contrains are NOT MET, just discard the iteration
        end
        Cl0_temp = -CLa(G)*a0_temp;
        if alpha_cruise(G) > angle_max,
            CD_airfoil(G) = 0.99;
        else
            for h = 1:size(Re_options,2),
                Cd_temp_airfoil(h) = interp1(normalized_params(:,1,h),normalized_params(:,3,h),alpha_cruise(G));
                Cm_temp_airfoil(h) = interp1(normalized_params(:,1,h),normalized_params(:,5,h),alpha_cruise(G));
            end
            CD_airfoil(G)   = interp1(Re_options,Cd_temp_airfoil,i);
            CM_airfoil(G)   = interp1(Re_options,Cm_temp_airfoil,i);
        end
        s                    = 1-2*(fuselage_thickness/span(G))^2;
        u                    = 0.99; %Theoretical oswald efficiency factor
        e_invicid            = 1/(u*s);
        K                    = 0.38;  %Factor from flight test data
        e_viscous            = K*CD_airfoil(G);
        e_total_wing(G)      = (e_invicid + e_viscous*pi*AR)^(-1);
        K_wing               = (pi*AR*e_total_wing(G))^(-1);
        CD_Wing(G)           = CD_airfoil(G) + (K_wing)*(CL_req(G))^2 + K_Factor*(CL_req(G)-Cl0_temp)^2;
        CD_Wing_secondary(G) = CD_airfoil(G) + (K_wing)*(CL_req_secondary(G))^2 + K_Factor*(CL_req_secondary(G)-Cl0_temp)^2;        
        D_Wing(G)            = q*wing_area(G)*CD_Wing(G);
        D_Wing_secondary(G)  = q_secondary*wing_area(G)*CD_Wing_secondary(G);
        L_req(G)             = q*wing_area(G)*CL_req(G);
        L_req_secondary(G)   = q_secondary*wing_area(G)*CL_req_secondary(G);  %assuming rectangular (no MAC, but chord)
        %         CD_Wing(G) = CD_airfoil(G) + (K_wing)*(CL_req(G))^2; %No K Factor for comparison
        G = G + 1;
    end
    D_Min = 500;
    for i = 1:size(D_Wing,2)
        if D_Wing(i) <= D_Min,
            D_Min   = D_Wing(i);

            wing.CLalpha      = CLa(i)*180/pi;
            wing.Clalpha      = Cla_temp(i);
            wing.CL_cruise    = CL_req(i);
            wing.CL_launch    = CL_req_secondary(i);
            wing.CD_cruise    = CD_Wing(i);
            wing.CD_launch    = CD_Wing_secondary(i);
            wing.CM           = CM_airfoil(i);
            wing.L_cruise     = L_req(i);
            wing.L_launch     = L_req_secondary(i);
            wing.D_cruise     = D_Wing(i);
            wing.D_launch     = D_Wing_secondary(i);
            wing.M_cruise     = q*wing_area(i)*chord(i)*CM_airfoil(i);
            wing.M_launch     = q_secondary*wing_area(i)*chord(i)*CM_airfoil(i);
            wing.S            = wing_area(i);
            wing.A            = AR;
            wing.croot        = chord(i); 
            wing.b            = span(i);
            wing.MAC          = chord(i);
            wing.xMAC         = 0.25;
            wing.taper        = 1;
            wing.sweep        = 0;
            wing.dihedral     = 0;         
            wing.alpha_cruise = alpha_cruise(i)*pi/180;
            wing.alpha_launch = alpha_takeoff(i)*pi/180;
            wing.incidence    = 0;

        end
    end
    if kk == 1,
        optimized_wing = wing;
    elseif wing.D_cruise <= optimized_wing.D_cruise,
       optimized_wing = wing;
       index_winner   = kk;
    end
end
    wing = optimized_wing;
    wing.airfoil_name = Airfoil_List(index_winner);
    wing.eps0_launch  = 2*wing.CL_launch/(pi*wing.A);
    wing.eps0_cruise  = 2*wing.CL_cruise/(pi*wing.A);
    wing.eps_dalpha   = 2*wing.CLalpha/(pi*wing.A);
    wing.eps_launch   = wing.eps0_launch + wing.eps_dalpha*wing.alpha_launch;
    wing.eps_cruise   = wing.eps0_cruise + wing.eps_dalpha*wing.alpha_cruise;
    wing.L_max        = wing.n*mission.g*assumption.weight;
        
else
%% HARCODED VALUES
wing.n            = 3.9776475148099;
wing.CLalpha      = 4.6506973085265;
wing.Clalpha      = 6.1133327276269;
wing.CL_cruise    = 0.197400702814798;
wing.CL_launch    = 1.34969578612598;
wing.CD_cruise    = 0.0121414476864852;
wing.CD_launch    = 0.105967541334586;
wing.CM           = -0.118447819005208;
wing.L_cruise     = 26.487;
wing.L_launch     = 26.487;
wing.D_cruise     = 1.62912553140022;
wing.D_launch     = 2.07955177468948;
wing.M_cruise     = -4.85815681938764;
wing.M_launch     = -0.710533129309266;
wing.S            = 0.654061834848571;
wing.A            = 7;
wing.croot        = 0.305675326086957; 
wing.b            = 2.1397272826087;
wing.MAC          = 0.305675326086957;
wing.xMAC         = 0.25;
wing.taper        = 1;
wing.sweep        = 0;
wing.dihedral     = 0;         
wing.alpha_cruise = -0.0334951273920181;
wing.alpha_launch = 0.214273111834944;
wing.incidence    = 0;
wing.airfoil_name = {'W_FX60'};
wing.eps0_launch  = 0.122749003447001;
wing.eps0_cruise  = 0.0179527414987367;
wing.eps_dalpha   = 0.422960837414952;
wing.eps_launch   = 0.213378138264216;
wing.eps_cruise   = 0.00378561436768828;

end

end

