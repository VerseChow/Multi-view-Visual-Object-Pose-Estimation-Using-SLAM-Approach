function hash_table = objectModelling(folder_path, hash_table_name)

%list all the image files in the folder
image_list = dir([folder_path, '/scene*.png']);

%list all the pcd files in the folder
pcd_list = dir([folder_path, '/scene_*.pcd']);

%list all the pcd_odom files in the folder
pcd_odom_list = dir([folder_path, '/scene_*_odom.pcd']);

%set to get the list of non odom files
pcd_list = setdiff({pcd_list.name}, {pcd_odom_list.name});

num_images = length({image_list.name});
%prev frame
prev_image = [folder_path, '/', image_list(1).name];
prev_pcd = char(strcat(folder_path, '/', pcd_list(1)));
prev_odom_pcd = [folder_path, '/', pcd_odom_list(1).name];

hash_table=[];
hash_table.rgb_pix=[];
hash_table.rgb_feat = [];
hash_table.depth_loc = [];

for i=2:num_images-1
    %curr_frame
    curr_image = [folder_path, '/', image_list(i).name];
    curr_pcd = char(strcat(folder_path, '/', pcd_list(i)));
    curr_odom_pcd = [folder_path, '/', pcd_odom_list(i).name]; 
    
    %next_frame
    next_image = [folder_path, '/', image_list(i+1).name];
    next_pcd = char(strcat(folder_path, '/', pcd_list(i+1)));
    next_odom_pcd = [folder_path, '/', pcd_odom_list(i+1).name]; 
        
    %get the hash of the frames
    hash_prev = objectFeaturesPerFrame(prev_image, prev_pcd, prev_odom_pcd, 0);
    hash_curr = objectFeaturesPerFrame(curr_image, curr_pcd, curr_odom_pcd, 0);
    hash_next = objectFeaturesPerFrame(next_image, next_pcd, next_odom_pcd, 0);
    
    %match 2 and 1 to the temp_2
    [matches21, scores] = vl_ubcmatch(hash_curr.rgb_feat, hash_prev.rgb_feat, 2) ;
    figure(1);
    subplot(2, 1, 1);
    show2DMatches(curr_image, prev_image, matches21, hash_curr, hash_prev);
    temp_2=[];
    temp_2.rgb_pix = hash_curr.rgb_pix(:, matches21(1, :));
    temp_2.rgb_feat = hash_curr.rgb_feat(:, matches21(1, :));
    temp_2.depth_loc = hash_curr.depth_loc(:, matches21(1, :));
    
    %match temp_2 and 3 to the final_match
    [matches23, scores] = vl_ubcmatch(temp_2.rgb_feat, hash_next.rgb_feat, 2) ;
    %figure(2);
    subplot(2, 1, 2);
    show2DMatches(curr_image, next_image, matches23, temp_2, hash_next);
    best_2=[];
    best_2.rgb_pix = temp_2.rgb_pix(:, matches23(1, :));
    best_2.rgb_feat = temp_2.rgb_feat(:, matches23(1, :));
    best_2.depth_loc = temp_2.depth_loc(:, matches23(1, :));    
    best_2.depth_loc = getFeatures3DwrtObjectCenter(best_2.depth_loc, i, folder_path);
    
    hash_table.rgb_pix = [hash_table.rgb_pix, best_2.rgb_pix];
    hash_table.rgb_feat = [hash_table.rgb_feat, best_2.rgb_feat];
    hash_table.depth_loc = [hash_table.depth_loc, best_2.depth_loc];
    
end
writePCDFile(hash_table.depth_loc', [folder_path, '/features_3d_hashed.pcd']);
save([folder_path, '/', hash_table_name], 'hash_table');
end


function show2DMatches(image_A, image_B, matches, hash_A, hash_B)
    im_A = imread(image_A);
    im_B = imread(image_B);
    imshow([im_A, im_B]); hold on;
    plot(int32(hash_A.rgb_pix(1, matches(1, :))), int32(hash_A.rgb_pix(2, matches(1, :))), 'r*'); hold on;
    plot(int32(hash_B.rgb_pix(1, matches(2, :)))+640, int32(hash_B.rgb_pix(2, matches(2, :))), 'r*'); hold on;
    x1 = int32(hash_A.rgb_pix(1, matches(1, :)));
    x2 = int32(hash_B.rgb_pix(1, matches(2, :)))+640;
    y1 = int32(hash_A.rgb_pix(2, matches(1, :)));
    y2 = int32(hash_B.rgb_pix(2, matches(2, :)));    
    line([x1; x2], [y1; y2]);    
end
