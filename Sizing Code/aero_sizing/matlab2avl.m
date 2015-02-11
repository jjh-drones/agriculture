function matlab2avl(mission,wing,hstab,cg,aileron)

name               = mission.name;
f                  = '%8.4f';
d                  = '%2d';
fid                = fopen([name,'.avl'],'w');
path_wing_airfoil  = [cell2mat(wing.airfoil_name),'_avl.dat']; 
path_hstab_airfoil = 'NACA0006_avl.dat';

% GLOBAL
fprintf(fid,[name,'\n']);
fprintf(fid,[f,'\n'],mission.M);
fprintf(fid,[d,d,d,'\n'],0,0,0);
fprintf(fid,[f,f,f,'\n'],wing.S,wing.MAC,wing.b);
fprintf(fid,[f,f,f,'\n'],cg.x,cg.y,cg.z);
fprintf(fid,[f,'\n\n'],0);

%WING
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,-2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'YDUPLICATE\n');
fprintf(fid,[d,'\n'],0);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);


fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,aileron.start,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);

fprintf(fid,'CONTROL\n');
fprintf(fid,['aileron',f,f,f,f,f,f,'\n'],1,(1-aileron.chord_perc),0,1,0,-1);

fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,aileron.start+aileron.span,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);

fprintf(fid,'CONTROL\n');
fprintf(fid,['aileron',f,f,f,f,f,f,'\n'],1,(1-aileron.chord_perc),0,1,0,-1);

fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0.5*wing.b,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n\n']);

%HSTAB
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING3\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,-2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'YDUPLICATE\n');
fprintf(fid,[d,'\n'],0);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0,0,hstab.croot,hstab.incidence,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n']);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0.5*hstab.b,0,hstab.croot,hstab.incidence,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n\n']);
end

