function matlab2avl(mission,wing,hstab,cg)

name               = 'brick';
f                  = '%8.4f';
d                  = '%2d';
fid                = fopen([name,'.avl'],'w');
path_wing_airfoil  = 'SD7062.dat'; 
path_hstab_airfoil = 'NACA0006.dat';

% GLOBAL
fprintf(fid,[name,'\n']);
fprintf(fid,[f,'\n'],mission.M);
fprintf(fid,[d,d,d,'\n'],0,0,0);
fprintf(fid,[f,f,f,'\n'],wing.S,wing.MAC,wing.b);
fprintf(fid,[f,f,f,'\n'],cg.x,0,0);
fprintf(fid,[f,'\n\n'],0);

% RIGHT WING
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,-2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0.5*wing.b,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n\n']);

% LEFT WING
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING2\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,-0.5*wing.b,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0,0,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n\n']);

% RIGHT HSTAB
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING3\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,-2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0,0,hstab.croot,hstab.ih,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n']);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0.5*hstab.b,0,hstab.croot,hstab.ih,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n\n']);

% LEFT HSTAB
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING4\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,-0.5*hstab.b,0,hstab.croot,hstab.ih,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n']);
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0,0,hstab.croot,hstab.ih,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n\n']);
end

