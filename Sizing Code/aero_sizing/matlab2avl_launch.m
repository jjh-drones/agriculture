function matlab2avl_launch(Name,avl)
    
    alpha = avl.alpha;
    
    fname = [Name,'.dat'];
    fid = fopen(['OUTPUTS\',fname], 'wt');
    fprintf(fid,['load ',Name,'.avl\n']);
    fprintf(fid,['mass ',Name,'.mass\n']);
    fprintf(fid,'oper\n');
    fprintf(fid,['a a ',num2str(alpha),'\n']);
    fprintf(fid,'x\n');
    fprintf(fid,'FT\n');
    fprintf(fid,['temporal',Name,'\n']);
    fclose(fid);
    cd([pwd,'\OUTPUTS'])
    system(['avl.exe <' fname]);
    cd('..')
    fclose('all');
end