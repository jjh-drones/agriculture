function matlab2avl(assumptions,mission,wing,hstab,vstab,cg,aileron,elevator,rudder)

name               = mission.name;
f                  = '%8.4f';
d                  = '%2d';
fid                = fopen(['OUTPUTS\',name,'.avl'],'w');
path_wing_airfoil  = [cell2mat(wing.airfoil_name),'_avl.dat']; 
path_hstab_airfoil = [hstab.airfoil_name,'_avl.dat'];
path_vstab_airfoil = [vstab.airfoil_name,'_avl.dat']; 

% GLOBAL
fprintf(fid,[name,'\n']);
fprintf(fid,[f,'\n'],mission.M);
fprintf(fid,[d,d,d,'\n'],0,0,0);
fprintf(fid,[f,f,f,'\n'],wing.S,wing.MAC,wing.b);
fprintf(fid,[f,f,f,'\n'],cg.x,cg.y,cg.z);
fprintf(fid,[f,'\n\n'],0);

%% WING
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,-2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'YDUPLICATE\n');
fprintf(fid,[d,'\n'],0);

% root chord
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0,wing.zAC,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);

% start aileron
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,aileron.start,wing.zAC,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);
fprintf(fid,'CONTROL\n');
fprintf(fid,['aileron',f,f,f,f,f,f,'\n'],1,(1-aileron.chord_perc),0,1,0,-1);

% end aileron
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,aileron.start+aileron.span,wing.zAC,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n']);
fprintf(fid,'CONTROL\n');
fprintf(fid,['aileron',f,f,f,f,f,f,'\n'],1,(1-aileron.chord_perc),0,1,0,-1);

% tip chord
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],wing.xLE,0.5*wing.b,wing.zAC,wing.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_wing_airfoil,'\n\n\n']);

%% HSTAB
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING2\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,-2);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],2);
fprintf(fid,'YDUPLICATE\n');
fprintf(fid,[d,'\n'],0);

% root
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0,assumptions.tail_height,hstab.croot,hstab.incidence,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n']);
fprintf(fid,'CONTROL\n');
fprintf(fid,['elevator',f,f,f,f,f,f,'\n'],1,(1-elevator.cratio_E),0,1,0,1);

% tip
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],hstab.xLE,0.5*hstab.b,assumptions.tail_height,hstab.croot,hstab.incidence,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_hstab_airfoil,'\n']);
fprintf(fid,'CONTROL\n');
fprintf(fid,['elevator',f,f,f,f,f,f,'\n\n\n'],1,(1-elevator.cratio_E),0,1,0,1);

%% VSTAB
fprintf(fid,'SURFACE\n');
fprintf(fid,'WING3\n');
fprintf(fid,[f,f,f,f,'\n'],18,1,30,1);
fprintf(fid,'ANGLE\n');
fprintf(fid,[f,'\n'],0);
fprintf(fid,'COMPONENT\n');
fprintf(fid,[d,'\n'],1);

% root
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],vstab.xLE,0,assumptions.tail_height,vstab.croot,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_vstab_airfoil,'\n']);
fprintf(fid,'CONTROL\n');
fprintf(fid,['rudder',f,f,f,f,f,f,'\n'],1,(1-rudder.cratio_R),0,0,1,1);

% tip
fprintf(fid,'SECTION\n');
fprintf(fid,[f,f,f,f,f,f,f,'\n'],vstab.xLE + vstab.croot*(1-vstab.taper),0,assumptions.tail_height+vstab.b,vstab.croot*vstab.taper,0,0,0);
fprintf(fid,'AFIL\n');
fprintf(fid,[path_vstab_airfoil,'\n']);
fprintf(fid,'CONTROL\n');
fprintf(fid,['rudder',f,f,f,f,f,f,'\n'],1,(1-rudder.cratio_R),0,0,1,1);
end

