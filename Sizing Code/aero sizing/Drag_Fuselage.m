function [CD_Fuselage,D_Fuselage] = Drag_Fuselage(target_speed,S_wing,tail_LE_distance,fuselage_nature)
%     target_speed = 18.3; %meters per second'
%     S_wing       = 0.6; %meters2
%     tail_LE_distance = 0.2; %m, from some reference of the fuselage box
    density = 1.225; %kg/m3
    Closure_angle = 14; %Degrees
    kinematic_viscosity = 16.04*10^(-6);
    q = 0.5*density*target_speed^2;
    nosecone_lenght_ratio = 0.2; %percentage of body lenght
    
       %So, the fuselage_nature format is: [type,radius/base of
        %ellipse/width,0/height of ellipse/height,lenght of body]
        
    tailcone_lenght = max(fuselage_nature(2),fuselage_nature(3))/(2*tan(Closure_angle*pi/180));   
    tail_lenght = max(tailcone_lenght,tail_LE_distance);
    nose_lenght = nosecone_lenght_ratio*fuselage_nature(4);
    if fuselage_nature(1) == 0,
        Aera_Base        = pi*fuselage_nature(2)*fuselage_nature(3)/4;
        a = fuselage_nature(2)/2;
        b = fuselage_nature(3)/2;
        main_circ = pi*(3*(a+b)-((3*a+b)*(a+3*b))^0.5);
        Wetted_area_body = pi*(3*(a+b)-((3*a+b)*(a+3*b))^0.5)*fuselage_nature(4);
        if tail_LE_distance < tailcone_lenght,
            a_small = (tailcone_lenght-tail_LE_distance)*a/tailcone_lenght;
            b_small = (tailcone_lenght-tail_LE_distance)*b/tailcone_lenght;
            small_circ = pi*(3*(a_small+b_small)-((3*a_small+b_small)*(a_small+3*b_small))^0.5);
            s = ((tail_LE_distance)^2+(a-a_small)^2)^0.5;
            s_2 = ((tail_LE_distance)^2+(b-b_small)^2)^0.5;
            s_mean = (s+s_2)/2;
            Wetted_Area_tail = (s_mean/2)*(small_circ+main_circ);
        else
            a_small = 0;
            b_small = 0;
            small_circ = 0;
            s = ((tailcone_lenght)^2+(a)^2)^0.5;
            s_2 = ((tailcone_lenght)^2+(b)^2)^0.5;
            s_mean = (s+s_2)/2; %three pointer, in the sense that I really dont know how the slanted lenght is used or if both are used
            Wetted_Area_tail = (s_mean/2)*(main_circ);
        end
        s = ((tailcone_lenght)^2+(a)^2)^0.5;
        s_2 = ((tailcone_lenght)^2+(b)^2)^0.5;
        s_mean = (s+s_2)/2; %three pointer, in the sense that I really dont know how the slanted lenght is used or if both are used
        Wetted_Area_nose = (s_mean/2)*(main_circ);
    elseif fuselage_nature(1) == 1,
        Aera_Base = fuselage_nature(2)*fuselage_nature(3);
        Wetted_area_body = (2*fuselage_nature(2)+2*fuselage_nature(3))*fuselage_nature(4);
        main_perim = (2*fuselage_nature(2)+2*fuselage_nature(3));
        if tail_LE_distance < tailcone_lenght,
            h_small = (tailcone_lenght-tail_LE_distance)*fuselage_nature(3)/tailcone_lenght;
            w_small = (tailcone_lenght-tail_LE_distance)*fuselage_nature(2)/tailcone_lenght;
            small_perim = 2*h_small+2*w_small;
            s = ((tail_LE_distance)^2+(fuselage_nature(2)/2-w_small/2)^2)^0.5;
            s_2 = ((tail_LE_distance)^2+(fuselage_nature(3)/2-h_small/2)^2)^0.5;
        else
            h_small = 0;
            w_small = 0;
            small_perim = 0;
            s = ((tailcone_lenght)^2+(fuselage_nature(2)/2)^2)^0.5;
            s_2 = ((tailcone_lenght)^2+(fuselage_nature(3)/2)^2)^0.5;
        end
        Wetted_Area_tail = (fuselage_nature(2)+w_small)*s_2+(fuselage_nature(3)+h_small)*s;
        s = ((nose_lenght)^2+(fuselage_nature(2)/2)^2)^0.5;
        s_2 = ((nose_lenght)^2+(fuselage_nature(3)/2)^2)^0.5;
        Wetted_Area_nose = 1.3*((fuselage_nature(2))*s_2+(fuselage_nature(3))*s);
    end
    fuselage_lenght = tail_lenght+fuselage_nature(4)+nose_lenght;
    Total_Wetted_Area = Wetted_Area_nose+Wetted_Area_tail+Wetted_area_body;
    Reynolds_Fuselage = target_speed*fuselage_lenght/kinematic_viscosity; %assuming turbulent BL
    D = 2*(Aera_Base/pi)^0.5;
    Cf = 0.455/(log10(Reynolds_Fuselage))^2.58;
    FR = fuselage_lenght/D;
    FF = 1+(60/FR^3)+(0.0025*FR);

    CD_Fuselage = Cf*FF*Total_Wetted_Area/S_wing;
    D_Fuselage = q*S_wing*CD_Fuselage;
end
