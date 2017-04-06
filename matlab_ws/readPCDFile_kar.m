function Data = readPCDFile_kar(file_path)
disp('Reading the point cloud .... ');
% File pointer
fid = fopen(file_path, 'rt');
% Assuming that point cloud is organized
num = 307200;
i=1;
% Ignore the 11 lines of the standard pcd file
while(i<=11)
     scrap = fgetl(fid);
     %disp(scrap);
     i=i+1;
end
% Format specifier
format = ['%f %f %f\n'];
% One line to load the point cloud
C = textscan(fid,format);
Data= cell2mat(C);
fclose(fid);
disp('Done');
end