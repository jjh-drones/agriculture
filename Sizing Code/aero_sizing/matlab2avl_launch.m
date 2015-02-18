function matlab2avl_launch(Name,avl)
    
    
    
    fname = [Name,'.dat'];
    fid = fopen(['OUTPUTS\',fname], 'wt');
    
    % LOAD
    fprintf(fid,['load ',Name,'.avl\n']);
    fprintf(fid,['mass ',Name,'.mass\n']);
    
    % EXECUTE
    fprintf(fid,'oper\n');
    fprintf(fid,['a a ',num2str(avl.alpha),'\n']);
    fprintf(fid,['b b ',num2str(avl.beta),'\n']);
    fprintf(fid,['r r ',num2str(avl.roll_rate),'\n']);
    fprintf(fid,['p p ',num2str(avl.pitch_rate),'\n']);
    fprintf(fid,['y y ',num2str(avl.yaw_rate),'\n']);
    fprintf(fid,['d1 d1 ',num2str(avl.aileron),'\n']);
    fprintf(fid,['d2 d2 ',num2str(avl.elevator),'\n']);
    fprintf(fid,['d3 d3 ',num2str(avl.rudder),'\n']);
    fprintf(fid,'x\n');
    
    % - ft
    fprintf(fid,'FT\n');
    fprintf(fid,['temporal',Name,'\n']);
    fclose(fid);
    
    % LAUNCH
    cd([pwd,'\OUTPUTS'])
    system(['avl.exe <' fname]);
    cd('..')
    fclose('all');
end