function hash_frame = objectFeaturesPerFrame(image, pcd_file, pcd_odom_file, plot_flag)
%Table height
table_z = 0.79;


%Get the rgb image
curr_img = imread(image);

%Get the pcd data
pcd_data = readPCDFile_kar(pcd_file);

% %Get the pcd odom data
pcd_odom_data = readPCDFile_kar(pcd_odom_file);
disp('reade the pcd_odom_data');

%extract features from the rgb image
I_curr = single(rgb2gray(curr_img));
[f_curr, d_curr] = vl_sift(I_curr);

%check if the 3d point in the odom data is belong to the object
f_3d_curr = zeros(3, size(f_curr, 2));

disp('before the forloop');
for i=1:size(f_curr, 2)
    disp([int32(f_curr(1, i)), int32(f_curr(2, i))]);
    pt_3d = get3DPoint(int32(f_curr(1, i)), int32(f_curr(2, i)), pcd_odom_data)';
    if(pt_3d(3)>table_z) %If the object is above the table
        f_3d_curr(:, i)=pt_3d;
    end    
end

%store the descriptor, x, y, x, y, z, frame, id
indices = find(f_3d_curr(1, :)~=0);
hash_frame.rgb_pix = f_curr(:, indices);
hash_frame.rgb_feat = d_curr(:, indices);
hash_frame.depth_loc = f_3d_curr(:, indices);

%plot the features on the image
if(plot_flag==1)
    figure(1);
    imshow(curr_img); hold on;
    plot(int32(f_curr(1, :)), int32(f_curr(2, :)), 'r*'); hold on;
    plot(int32(f_curr(1, indices)), int32(f_curr(2, indices)), 'g*'); hold on;
end

end