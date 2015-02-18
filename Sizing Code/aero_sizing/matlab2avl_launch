function AVL_feeder(Name,alpha)
    fname = 'thafile';
    fid = fopen(fname, 'wt');
    fprintf(fid,['load ',Name,'.avl\n']);
    fprintf(fid,['mass ',Name,'.mass\n']);
    fprintf(fid,'oper\n');
    fprintf(fid,['a a ',num2str(alpha),'\n']);
    fprintf(fid,'x\n');
    fprintf(fid,'FT\n');
    fprintf(fid,['temporal',Name,'\n']);
    fclose(fid);
    system(['avl.exe <' fname]);
    fileDL = fopen(['temporal',Name],'r');
    C = textscan(fileDL, '%s', 'delimiter', '\n');
    sublines = size(C{1,1},1);
    Lines = 1;
    for J=1:sublines
        S = C{1,1}{J,1};
        if isempty(S) || size(S,2) <= 1,
            TF = 0;
        else
            TF = strcmp(S(1:5),'CLtot');

            if TF == 1,
                OutputAVL(i,1) = alfa(i);
                OutputAVL(i,2) = str2double(S(10:17));
                SS = C{1,1}{J+1,1};
                OutputAVL(i,3) = str2double(SS(10:17));
                Lines = Lines + 1;
            end
        end
    end
    fclose('all');
    delete('thafile',['temporal',Name]);
end