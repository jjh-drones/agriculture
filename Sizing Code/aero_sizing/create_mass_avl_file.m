function create_mass_avl_file(cg,weights,mission)
    cm2m               = 0.01;
    g2kg               = 0.001;

    name               = mission.name;
    f                  = '%8.4f';
    d                  = '%2d';
    fid                = fopen([name,'.mass'],'w');

    % GLOBAL

    fprintf(fid,['Lunit = ', f, 'm \n'],1);
    fprintf(fid,['Munit = ', f, 'kg \n'],1);
    fprintf(fid,['Tunit = ', f, 's \n'],1);
    fprintf(fid,['g = ', f, 's \n'],mission.g);
    fprintf(fid,['rho = ', f, 's \n'],mission.rho);
    fprintf(fid,'#  mass   x     y     z      Ixx     Iyy    Izz   [  Ixy   Ixz   Iyz ]\n');
    fprintf(fid,['* ',f,f,f,f,f,f,f,f,f,f],1,1,1,1,1,1,1,1,1,1);
    fprintf(fid,['+ ',f,f,f,f,f,f,f,f,f,f],0,0,0,0,0,0,0,0,0,0);
    for i = 1:size(weights,2),
       fprintf(fid,['  ',f,f,f,f,f,f,f],g2kg*weights.mass(i),cm2m*weights.x(i),cm2m*weights.y(i),cm2m*weights.z(i),weights.Ix,weights.Iy,weights.Iz); 
    end
    
close(fid);
end
