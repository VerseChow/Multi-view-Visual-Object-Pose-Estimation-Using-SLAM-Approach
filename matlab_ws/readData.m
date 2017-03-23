function readData( data_path )
%READDATA Summary of this function goes here
%   get the data from txt file, and convert it into tanspose matrix
%   data_path should be the format of "/***/", and should depend on the
%   root file path.
    global Data;
    source_dir = pwd;
    d= dir([source_dir, data_path, '*.txt']);
    count = length(d);
    fprintf('Found %d txt files, preparing for data\n', count);
    if count == 0
       dip('No data found, check the data path'); 
       return;
    else
        Data.transpose_Matirix = cell(count, 1);
        Data.u = cell(count-1, 1);
        for i = 1:count
            FileName = d(i).name;
            fprintf(1, 'Now reading %s\n', FileName);
            fid=fopen(fullfile(source_dir, data_path, FileName));
            temp = textscan(fid, '%f');
            transpose_Matirix = eye(4);
            rotm = quatern2rotMat(temp{1}(4:end)');
            transpose_Matirix(1:3, 4) = temp{1}(1:3);
            transpose_Matirix(1:3, 1:3) = rotm;
            Data.transpose_Matirix{i} = transpose_Matirix;
        end
        for i = 1:count-1
            temp = Data.transpose_Matirix{i+1}(1:3,4) - Data.transpose_Matirix{i}(1:3,4);
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