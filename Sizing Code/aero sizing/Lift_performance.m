function wing = Lift_performance(airfoil_name,target_speed,secondary_target_speed,weight,fuselage_thickness,AR)
    %%Internal Variables
    density      = 1.225; %kg/m3 sea level
    temperature         = 30+273.15;            %degrees kelvin
    kinematic_viscosity = 16.04*10^(-6); %Conditions for 30 celsius
    g                   = 9.81;          %m/s2
    R                   = 287.058;       %Gas constant for air
    speed_sound         = (1.4*R*temperature)^0.5;
    M                   = target_speed/speed_sound;
    q = 0.5*density*target_speed^2;
    q_secondary = 0.5*density*secondary_target_speed^2;
    delta_CL    = 0.3;       %Additional CL with FLAPS
    Re_min = 100000;     %other values NOT POSSIBLE
    Re_max = 1000000;

    step_size = 2000;

    delta_Re = floor((Re_max-Re_min)/step_size);

    %%Load Airfoil
    Re_options = [50000, 100000, 200000, 500000, 1000000];

    P = zeros(250,7,size(Re_options,2));




    delta = 0.25;

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


    for j = 1:size(normalized_params,1)
        if normalized_params(j,1,1) == cla_alpha_min,
            Tag(1,1) = j;
        end
        if normalized_params(j,1,1) == cla_alpha_max,
            Tag(2,1) = j;
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
    G = 1;
    %% Run calculations
    for i = Re_min:delta_Re:Re_max,
        chord(G) = i*kinematic_viscosity/(target_speed);
        wing_area(G) = AR*chord(G)^2;    %Assuming rectangular wing
        span(G) = AR*chord(G);
        CL_req(G) = g*weight/(q*wing_area(G));                  
        CL_req_secondary(G) = g*weight/(q_secondary*wing_area(G));
        Cla_temp(G) = interp1(Re_options,Cla(1,:),i)*180/(pi);
        a0_temp = interp1(Re_options,Cl0(2,:),i);
        CL_Cutoff_Specific = interp1(Re_options,CL_Cutoff,i);
        k = Cla_temp(G)/(2*pi); 
        CLa(G)     = ((2*pi*AR)/(2+(((AR^2*(1-M^2)/k^2))+4)^0.5))*(pi/180);
        alpha_cruise(G) = (CL_req(G))/CLa(G) + a0_temp;
        CL_req_secondary(G) = g*weight/(q_secondary*wing_area(G));
        alpha_takeoff(G) = (CL_req_secondary(G)-0.3)/CLa(G) + a0_temp;
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
        s = 1-2*(fuselage_thickness/span(G))^2;
        u = 0.99; %Theoretical oswald efficiency factor
        e_invicid = 1/(u*s);
        K = 0.38;  %Factor from flight test data
        e_viscous = K*CD_airfoil(G);

        e_total_wing(G) = (e_invicid + e_viscous*pi*AR)^(-1);
        K_wing     = (pi*AR*e_total_wing(G))^(-1);
        CD_Wing(G) = CD_airfoil(G) + (K_wing)*(CL_req(G))^2;
        D_Wing(G)  = q*wing_area(G)*CD_Wing(G);
        G = G + 1;
    end
    D_Min = 500;
    for i = 1:size(D_Wing,2)
       if D_Wing(i) <= D_Min,
           D_Min   = D_Wing(i);
           wing.CLa    = CLa(i)*180/pi;
           wing.chord  = chord(i); 
           wing.span = span(i);
           wing.Cla    = Cla_temp(i);
           wing.CD_Cruise = CD_Wing(i);
           wing.CL_Cruise = CL_req(i);
           wing.CM_Cruise = CM_airfoil(i);
           wing.S_Cruise  = wing_area(i);
           wing.CL_Launch = CL_req_secondary(i);
           wing.alpha_Cruise = alpha_cruise(i);
           wing.alpha_Takeoff = alpha_takeoff(i);
           wing.Drag_Cruise = D_Wing(i);
           wing.AR          = AR;
       end
    end
end
