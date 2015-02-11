function create_mass_avl_file(weights,mission)
    cm2m               = 0.01;
    g2kg               = 0.001;

    name               = mission.name;
    f                  = '%11.7f ';
    d                  = '%2d';
    fid                = fopen([name,'.mass'],'w');

    % GLOBAL

    fprintf(fid,['Lunit = ', f, 'm \n'],1);
    fprintf(fid,['Munit = ', f, 'kg \n'],1);
    fprintf(fid,['Tunit = ', f, 's \n'],1);
    fprintf(fid,['g = ', f, ' \n'],mission.g);
    fprintf(fid,['rho = ', f, ' \n'],mission.rho);
    fprintf(fid,'#  mass   x     y     z      Ixx     Iyy    Izz   [  Ixy   Ixz   Iyz ]\n');
    fprintf(fid,['* ',f,f,f,f,f,f,f,f,f,f,'\n'],1,1,1,1,1,1,1,1,1,1);
    fprintf(fid,['+ ',f,f,f,f,f,f,f,f,f,f,'\n'],0,0,0,0,0,0,0,0,0,0);
    for i = 1:size(weights,2),
       fprintf(fid,[f,f,f,f,f,f,f,'\n'],g2kg*weights(i).mass,cm2m*weights(i).x,cm2m*weights(i).y,cm2m*weights(i).z,weights.Ix,weights.Iy,weights.Iz); 
%        fprintf([f,f,f,f,f,f,f,'\n'],g2kg*weights(i).mass,cm2m*weights(i).x,cm2m*weights(i).y,cm2m*weights(i).z,weights(i).Ix,weights(i).Iy,weights(i).Iz); 
    end
end
