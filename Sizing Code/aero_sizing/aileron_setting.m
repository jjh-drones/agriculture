function [t_roll,Pss,bank_angle_till_steady] = aileron_setting(CL_alpha_wing,C_aileron,C_wing,inner_edge_ail_perc,outer_edge_ail_perc,S_wing,b_wing,d_aileron,q_flight,rho_flight,S_horiontal,S_vertical,Ixx,dersired_roll_angle)
    CDR = 0.9;
    Yd = 0.4*(b_wing/2);
    deg2rad = pi/180;
    rad2deg = 180/pi;
    G = csvread('Ail_Eff.csv');
    C_ratio = C_aileron/C_wing;
    Ta = interp1(G(:,1),G(:,2),C_ratio);
    Y_1 = inner_edge_ail_perc*(b_wing/2);
    Y_0 = outer_edge_ail_perc*(b_wing/2);
    Cl_da = ((2*CL_alpha_wing*Ta*C_wing)/(S_wing*b_wing))*((Y_0^2/2)-(Y_1^2/2));
    Cl  = Cl_da*d_aileron;
    LA  = q_flight*S_wing*Cl*b_wing;

    Pss = ((2*LA)/(rho_flight*(S_horiontal+S_vertical+S_wing)*CDR*Yd^3))^0.5;
    bank_angle_till_steady = (Ixx/(rho_flight*Yd^3*(S_horiontal+S_vertical+S_wing)*CDR))*log(Pss^2);
    P_dot = Pss^2/(2*bank_angle_till_steady);
    
    if deg2rad*dersired_roll_angle < bank_angle_till_steady,
        t_roll = (2*deg2rad*dersired_roll_angle/P_dot)^0.5;
    else
        t_roll = (2*bank_angle_till_steady/P_dot)^0.5 +(deg2rad*dersired_roll_angle-bank_angle_till_steady)/Pss;
    end
    bank_angle_till_steady = bank_angle_till_steady*rad2deg;
    Pss = rad2deg*Pss;
end