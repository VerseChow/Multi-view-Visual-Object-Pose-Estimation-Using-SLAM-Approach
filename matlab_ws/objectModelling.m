function objectModelling(image, pcd_file, pcd_odom_file)

%Get the rgb image
curr_img = imread(image);

%Get the pcd data
%pcd_data = readPCDFile(pcd_file);
%disp('read the pcd_data');

%Get the pcd odom data
pcd_odom_data = readPCDFile(pcd_odom_file);
disp('reade the pcd_odom_data');

%extract features from the rgb image
I_curr = single(rgb2gray(curr_img));
[f_curr, d_curr] = vl_sift(I_curr);

%check if the 3d point in the odom data is belong to the object
f_3d_curr = zeros(3, size(f_curr, 2));

disp('before the forloop');
for i=1:size(f_curr, 2)
    i
    f_3d_curr(:, i)=get3DPoint(int8(f_curr(2, i)), int8(f_curr(1, i)), pcd_odom_data)';
end
%store the descriptor, x, y, x, y, z, frame, id

end