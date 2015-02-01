Airfoil_List           = {'Eppler_560';'SD7062';'NACA0009';'W_FX60'};
target_speed           = 18.3; %meters per second'
secondary_target_speed = 6; %meters per second
weight                 = 2;    %kilograms
fuselage_thickness     = 0.1188; %meters, given by Juan Manuel
AR                     = 7;    %balance between efficiency and constructibility


for i = 1:size(Airfoil_List,1),
    wing = Lift_performance(char(Airfoil_List(i)),target_speed,secondary_target_speed,weight,fuselage_thickness,AR);
    if i == 1,
        optimized_wing = wing;
    elseif wing.Drag_Cruise <= optimized_wing.Drag_Cruise,
       optimized_wing = wing;
    end
end

S_wing       = optimized_wing.S_Cruise; %meters2
tail_LE_distance = 0.2; %m, from some reference of the fuselage box
fuselage_nature = [1,0.115,0.08,0.4572];  %So, the fuselage_nature format is: [type,radius/base of
                                      %ellipse/width,0/height of ellipse/height,lenght of body]
[CD_Fuselage,D_Fuselage] = Drag_Fuselage(target_speed,S_wing,tail_LE_distance,fuselage_nature);