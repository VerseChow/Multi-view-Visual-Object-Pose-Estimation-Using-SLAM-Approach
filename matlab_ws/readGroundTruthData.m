function readGroundTruthData(data_path)
source_dir = pwd;
disp([source_dir, data_path, '/poses/*pose.txt']);
d_g= dir([source_dir, data_path, '/poses/*pose.txt']);
path = [source_dir, data_path, '/poses/'];
d_g = {d_g.name};
global gt_Data;
gt_Data = [];
for i=1:size(d_g, 2)
   disp(['Reading ', strcat(path, d_g(i))]);
   fid = fopen(char(strcat(path, d_g(i))), 'rt');
   line = fgetl(fid);
   obj_info = strsplit(line, ' ');
   obj_pose = zeros(6, 1);
   for j=2:7
    obj_pose(j-1, 1) = str2double(obj_info(j));
   end
   gt_Data(:, end+1) = obj_pose;
   
end

end