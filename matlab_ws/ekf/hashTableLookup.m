function z = hashTableLookup(observed_image, observed_pcd, observed_pcd_base, transform_matrix)
global Table;
%observed_image is in the mat format after imread 640x480
%observed_pcd is the point cloud in mat 307200x3 format
features_orig = getFeaturesOnObject(observed_image, observed_pcd, observed_pcd_base);

%transform_matrix is in the 4x4 format already to convert camera to
%base_link (if the observed_pcd_base) is not available.

%match with the Table data to get features
[matches, scores] = vl_ubcmatch(Table.hash_table.rgb_feat, features_orig.rgb_feat, 2) ;
z = [];
z = [features_orig.depth_loc(:, matches(2, :))', matches(1, :)'];
writePCDFile(z(:, 1:3), 'observed_features_1.pcd');

end

function hash_frame = getFeaturesOnObject(observed_image, observed_pcd, observed_pcd_base)
table_z = 0.79;
%extract features from the rgb image
I_curr = single(rgb2gray(observed_image));
[f_curr, d_curr] = vl_sift(I_curr);

%check if the 3d point in the odom data is belong to the object
f_3d_curr = zeros(3, size(f_curr, 2));

for i=1:size(f_curr, 2)
    %disp([int32(f_curr(1, i)), int32(f_curr(2, i))]);
    pt_3d = get3DPoint(int32(f_curr(1, i)), int32(f_curr(2, i)), observed_pcd_base)';
    if(pt_3d(3)>table_z) %If the object is above the table
        f_3d_curr(:, i)=pt_3d;
    end    
end
indices = find(f_3d_curr(1, :)~=0);
hash_frame=[];
%store the descriptor, x, y, x, y, z, frame, id
hash_frame.rgb_pix = f_curr(:, indices);
hash_frame.rgb_feat = d_curr(:, indices);
hash_frame.depth_loc = f_3d_curr(:, indices);

end