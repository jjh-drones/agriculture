function drag_force = drag_nikolai(airfoil_lift,aifoil_drag,wing_area,wing_aspect_ratio,tail_lift,tail_drag,tail_area,tail_aspect_ratio,cross_section_nature,angle_of_attack,dynamic_pressure,fuselage_lenght,mode)
    %first, we need to add the information of the wings
    airfoil_wing = csvread('file_wing');
    %second, we poceed to add the information of the elevator
    airfoil_tail = csvread('file_tail');
    
    %So, the fuselage_nature format is: [type,radius/base of
    %ellipse/width,0/height of ellipse/height]
    if fuselage_nature(1) == 1,
    elseif fuselage_nature(1) == 2,
    elseif fuselage_nature(1) == 3,    
    end
    
    wing_span = (wing_aspect_ratio*wing_area)^0.5;
    s = 1-2*(fuselage_thickness/wing_span)^2;
    u = 0.99; %Theoretical oswald efficiency factor
    e_invicid = 1/(u*s);
    K = 0.38;  %Factor from flight test data
    e_viscous = K*Cd_0;

    e_total_wing = (e_invicid + e_viscous*pi*AR)^(-1);

    interference = 0.2; %replace with actual calculations on tail cone end
    tail_span = (tail_aspect_ratio*tail_area)^0.5;
    s = 1-2*(interference*fuselage_thickness/tail_span)^2;
    u = 0.99; %Theoretical oswald efficiency factor
    e_invicid = 1/(u*s);
    K = 0.38;  %Factor from flight test data
    e_viscous = K*Cd_0;
    
    e_total_tail = (e_invicid + e_viscous*pi*AR)^(-1);
    
    

    
    
    
    K_wing = (pi*wing_aspect_ratio*e_total_wing)^(-1);
    K_tail = (pi*wing_aspect_ratio*e_total_tail)^(-1);
    CD = CD_min + (K_wing)*(CL_wing-CL_min_wing)^2+(K_tail)*(CL_tail-CL_min_tail)^2;
    
    drag_force = dynamic_pressure*wing_area*CD;
end