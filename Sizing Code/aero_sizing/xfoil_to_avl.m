function xfoil_to_avl(name)

fileID           = fopen([name,'.dat']);
airfo_name       = fgets(fileID);
Data             = cell2mat(textscan(fileID,'%f %f','headerlines',0));
tag              = 0;
fclose(fileID);
for i = 1:size(Data,1)
    if Data(i,1) == 1 && tag == 0,
        tag = 1;
        index_one = i;
    end
end
g         = index_one;
remainder = rem(g,2);
if remainder == 0,
    for i = 1:index_one/2
        baffer    = Data(i,:);
        Data(i,:) = Data(index_one-i+1,:);
        Data(index_one-i+1,:) = baffer;
    end
else
    for i = 1:(index_one-1)/2
        baffer    = Data(i,:);
        Data(i,:) = Data(index_one-i+1,:);
        Data(index_one-i+1,:) = baffer;
    end
end
fileID = fopen([name,'_avl.dat'],'wt');
fwrite(fileID,airfo_name);
for i = 1:size(Data,1)
    fprintf(fileID,'%f %f\n',Data(i,:));
end
fclose(fileID);
end