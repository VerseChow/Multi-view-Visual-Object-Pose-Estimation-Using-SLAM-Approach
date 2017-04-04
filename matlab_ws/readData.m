function readData( data_path )
%READDATA Summary of this function goes here
%   get the data from txt file, and convert it into tanspose matrix
%   data_path should be the format of "/***/", and should depend on the
%   root file path.
    global Data;
    source_dir = pwd;
    d_u= dir([source_dir, data_path, '*odom_base_tf.txt']);
    d_cam_tf = dir([source_dir, data_path, '*base_cam_tf.txt']);
    count = length(d_u);
    fprintf('Found %d txt files, preparing for data\n', count);
    if count == 0
       disp('No data found, check the data path'); 
       return;
    else
        Data.base_transpose_Matrix = cell(count,1);
        Data.cam_transpose_Matrix = cell(count, 1);
        
        Data.u = cell(count-1, 1);
        for i = 1:count
            FileName_u = d_u(i).name;
            FileName_cam = d_cam_tf(i).name;
            fprintf(1, 'Now reading %s\n', FileName_u);
            fid=fopen(fullfile(source_dir, data_path, FileName_u));
            temp = textscan(fid, '%f');
            transpose_Matrix = eye(4);
            rotm = quatern2rotMat(temp{1}(4:end)');
            transpose_Matrix(1:3, 4) = temp{1}(1:3);
            transpose_Matrix(1:3, 1:3) = rotm;
            Data.base_transpose_Matrix{i} = transpose_Matrix;
            
            %%%read cam_base transform
            fprintf(1, 'Now reading %s\n', FileName_cam);
            fid=fopen(fullfile(source_dir, data_path, FileName_cam));
            temp = textscan(fid, '%f');
            transpose_Matrix = eye(4);
            rotm = quatern2rotMat(temp{1}(4:end)');
            transpose_Matrix(1:3, 4) = temp{1}(1:3);
            transpose_Matrix(1:3, 1:3) = rotm;
            Data.cam_transpose_Matrix{i} = transpose_Matrix;
        end
        for i = 1:count-1
            temp = Data.base_transpose_Matrix{i+1}(1:3,4) - Data.base_transpose_Matrix{i}(1:3,4);
            Data.u{i} = temp;
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

    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(1,2,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(2,2,:) = 2.*q(:,1).^2-1+2.*q(:,3).^2;
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
end