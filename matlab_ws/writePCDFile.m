function writePCDFile(D_data, file_name)
fileID2 = fopen(file_name, 'w');
fprintf(fileID2, '%s\n', '# .PCD v0.7 - Point Cloud Data file format');
fprintf(fileID2, '%s\n', 'VERSION 0.7');
fprintf(fileID2, '%s\n', 'FIELDS x y z');
fprintf(fileID2, '%s\n', 'SIZE 4 4 4');
fprintf(fileID2, '%s\n', 'TYPE F F F');
fprintf(fileID2, '%s\n', 'COUNT 1 1 1');
fprintf(fileID2, '%s\n', 'WIDTH 1');
fprintf(fileID2, '%s\n', ['HEIGHT ',num2str(size(D_data, 1))]);
fprintf(fileID2, '%s\n', 'VIEWPOINT 0 0 0 1 0 0 0');
fprintf(fileID2, '%s\n', ['POINTS ', num2str(size(D_data, 1))]);
fprintf(fileID2, '%s\n', 'DATA ascii');

for i=1:size(D_data, 1)
    fprintf(fileID2, '%2.4f %2.4f %2.4f\n', D_data(i, 1), D_data(i, 2), D_data(i, 3));
    
end
end