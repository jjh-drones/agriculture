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
    
    % DOCUMENTS
    fprintf(fid,'FT\n');
    fprintf(fid,['avl_totalforces_',Name,'\n']);
    fprintf(fid,'FN\n');
    fprintf(fid,['avl_surfaceforces_',Name,'\n']);
    fprintf(fid,'FS\n');
    fprintf(fid,['avl_stripforces_',Name,'\n']);
    fprintf(fid,'FE\n');
    fprintf(fid,['avl_elementforces_',Name,'\n']);
    fprintf(fid,'FB\n');
    fprintf(fid,['avl_bodyforces_',Name,'\n']);
    fprintf(fid,'HM\n');
    fprintf(fid,['avl_hingemoments_',Name,'\n']);
    fprintf(fid,'VM\n');
    fprintf(fid,['avl_stripshear_',Name,'\n']);
    fprintf(fid,'ST\n');
    fprintf(fid,['avl_stability_',Name,'\n']);
    fprintf(fid,'SB\n');
    fprintf(fid,['avl_bodyaxis_',Name,'\n']);
    fprintf(fid,'RE\n');
    fprintf(fid,['avl_reference_',Name,'\n']);
    
    % LAUNCH
    fclose(fid);
    cd([pwd,'\OUTPUTS'])
    system(['avl.exe <' fname]);
    cd('..')
    fclose('all');
end