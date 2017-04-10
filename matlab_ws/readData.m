function readData( data_path )
%READDATA Summary of this function goes here
%   get the data from txt file, and convert it into tanspose matrix
%   data_path should be the format of "/***/", and should depend on the
%   root file path.
    global Data;
    source_dir = pwd;
    d_u= dir([source_dir, data_path, '*odom_base_tf.txt']);
    d_cam_tf = dir([source_dir, data_path, '*base_cam_tf.txt']);
    d_img = dir([source_dir, data_path, '*.png']);
    
    pcd_list = dir([source_dir, data_path, '/scene_*.pcd']);

    %list all the pcd_odom files in the folder
    pcd_odom_list = dir([source_dir, data_path, '/scene_*_odom.pcd']);

    count = length(d_u);
    fprintf('Found %d txt files, preparing for data\n', count);
    if count == 0
       disp('No data found, check the data path'); 
       return;
    else
        Data.base_transpose_Matrix = cell(count, 1);
        Data.cam_transpose_Matrix = cell(count, 1);
        Data.image = cell(count, 1);
        Data.G = cell(count-1, 1);
        Data.pcd = cell(count, 1);
        Data.pcd_base = cell(count, 1);
        Data.num = count;
        Data.theta = [];
        for i = 1:count
            FileName_u = d_u(i).name;
            FileName_cam = d_cam_tf(i).name;
            FileName_img = d_img(i).name;
            
            fprintf(1, 'Now reading %s\n', FileName_u);
            fid=fopen(fullfile(source_dir, data_path, FileName_u));
            temp = textscan(fid, '%f');
            transpose_Matrix = eye(4);
            rotm = quatern2rotMat(temp{1}(4:end)');
            transpose_Matrix(1:3, 4) = temp{1}(1:3);
            transpose_Matrix(1:3, 1:3) = rotm;
            Data.base_transpose_Matrix{i} = transpose_Matrix;
                Data.theta = [Data.theta acos(transpose_Matrix(1,1))];

            %%%read cam_base transform
            fprintf(1, 'Now reading %s\n', FileName_cam);
            fid=fopen(fullfile(source_dir, data_path, FileName_cam));
            temp = textscan(fid, '%f');
            transpose_Matrix = eye(4);
            rotm = quatern2rotMat(temp{1}(4:end)');
            transpose_Matrix(1:3, 4) = temp{1}(1:3);
            transpose_Matrix(1:3, 1:3) = rotm;
            Data.cam_transpose_Matrix{i} = transpose_Matrix;
            
            %%%read image
            fprintf(1, 'Now reading %s\n', FileName_img);
            Data.image{i} = imread(fullfile(source_dir, data_path, d_img(i).name));
            
            %%%read pcd 
            fprintf('Now reading pcd file');
            Data.pcd{i} = readPCDFile_kar(fullfile(source_dir, data_path, pcd_list(i).name));
            Data.pcd_base{i} = readPCDFile_kar(fullfile(source_dir, data_path, pcd_odom_list(i).name));
        end
        for i = 1:count-1
            temp =  inv(Data.base_transpose_Matrix{i+1})*Data.base_transpose_Matrix{i};
            Data.G{i} = temp;
        end
    end

end

function R = quatern2rotMat(q)
%QUATERN2ROTMAT Converts a quaternion orientation to a rotation matrix
%
%   R = quatern2rotMat(q)
%
%   Converts a quaternion orientation to a rotation matrix.
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#quaternions
%
%	Date          Author          Notes
%	27/09/2011    SOH Madgwick    Initial release
%   q = x y z w
    x = q(1);
    y = q(2);
    z = q(3);
    w = q(4);
    R(1,1) = 1-2*y*y-2*z*z;
    R(1,2) = 2*(x*y+w*z);
    R(1,3) = 2*(x*z-w*y);
    R(2,1) = 2*(x*y-w*z);
    R(2,2) = 1-2*x*x-2*z*z;
    R(2,3) = 2*(y*z+w*x);
    R(3,1) = 2*(x*z+w*y);
    R(3,2) = 2*(y*z-w*x);
    R(3,3) = 1-2*x*x-2*y*y;
end