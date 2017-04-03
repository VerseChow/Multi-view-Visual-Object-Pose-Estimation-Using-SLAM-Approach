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
run('vlfeat-0.9.20/toolbox/vl_setup')
img_list = dir('data/*.png');
imgs = string({img_list.name});
extract_features(imgs,'data');
