x = 340:424;
y = 94:221;
list_pixels=[];
for i=x
    for j=y
        list_pixels(end+1, :) = [i, j];
    end
end

D_data = readPCDFile('/home/kar/Dropbox/Michigan_research/MRF_data_exp/scene_test/scene_0.pcd');
list_3d = get3Dfeaturesfrom2D(D_data, list_pixels);
writePCDFile(list_3d, '/home/kar/Dropbox/Michigan_research/MRF_data_exp/scene_test/sugar_0.pcd');


load('/home/kar/Dropbox/Michigan_research/MRF_data_exp/scene_test/rcnn_output.mat');
N = size(label_output, 1);
for k=1:N
   if(label_output(k, 5)>0.9)
       k
       D_data = readPCDFile('/home/kar/Dropbox/Michigan_research/MRF_data_exp/scene_test/scene_0.pcd');
       x = label_output(k, 1):label_output(k, 2); 
       y = label_output(k, 3):label_output(k, 4);
       list_pixels=[];
        for i=x
            for j=y
                list_pixels(end+1, :) = [i, j];
            end
        end
        list_3d = get3Dfeaturesfrom2D(D_data, list_pixels);
        writePCDFile(list_3d, ['/home/kar/Dropbox/Michigan_research/MRF_data_exp/scene_test/segment_k_', num2str(label_output(k, 6)), '.pcd']);
   end
end

%% Feature extract

%put a list of images into 'data' folder
run('./vlfeat/toolbox/vl_setup.m')
data_path = '../data/eecs568_data/1/';
img_list = dir([data_path, 'scene_1_*.png']);
imgs = {img_list.name};
feats = extract_features(imgs,data_path);


%% Observation function
global Table;
Table = load('hash_table_1');
image_path2 = '/home/kar/workspace/eecs568_final/data/eecs568_data/1/scene_1_1.png';
pcd_odom_path2 ='/home/kar/workspace/eecs568_final/data/eecs568_data/1/scene_1_1_odom.pcd';
pcd_path2 ='/home/kar/workspace/eecs568_final/data/eecs568_data/1/scene_1_1.pcd';
observed_image = imread(image_path2);
observed_pcd = readPCDFile_kar(pcd_path2);
observed_pcd_odom = readPCDFile_kar(pcd_odom_path2);
z = hashTableLookup(observed_image, observed_pcd, observed_pcd_odom, []);
