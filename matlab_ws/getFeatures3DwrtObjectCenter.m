function feature_3d_out = getFeatures3DwrtObjectCenter(feature_3d_in, file)
%use the id to load the 3d point of the object annotated by Linh
%obj_poses_path = [path, '/poses/scene_1_', num2str(id), '_obj_pose.txt'];
fid = fopen(file, 'rt');
format = '%s %f %f %f %f %f %f\n';
C = textscan(fid, format);
obj_pose_odom = cell2mat(C(2:end));
obj_pose_odom = obj_pose_odom(1, :); %T_{obj}^{odom}
T_obj_odom = [euler2rotMat(obj_pose_odom(4:6)), obj_pose_odom(1:3)'; 0 0 0 1];
fclose(fid);

feature_3d_out = inv(T_obj_odom)*[feature_3d_in; ones(1, size(feature_3d_in, 2))];
feature_3d_out = feature_3d_out(1:3, :);
% use the id to load the transformation 
% odom_2_cam = [path, '/scene_1_', num2str(id), '_cam_inv_tf.txt'];
% fid2 = fopen(odom_2_cam, 'rt');
% format2 = ['%f %f %f\n %f %f %f %f'];
% D = textscan(fid2, format2);
% odom_2_cam_trans = cell2mat(D);
% odom_2_cam_pos = odom_2_cam_trans(1:3);
% odom_2_cam_quat = quatern2rotMat(odom_2_cam_trans(4:7));
% fclose(fid2);

%transform the point using this odom_2_cam_quat transformation

%now subtract the xyz from the object center to get the position with

%respect to the object pose 

%TODO how to handle the pose of the object.
%feature_3d_out = feature_3d_in;

end

function R = euler2rotMat(e)
    Rx = [1 0 0; 0 cos((e(1))) -sin((e(1))); 0 sin((e(1))) cos((e(1)))];
    Ry = [cos((e(2))) 0 sin((e(2))); 0 1 0; -sin((e(2))) 0 cos((e(2)))];
    Rz = [cos((e(3))) -sin((e(3))) 0; sin((e(3))) cos((e(3))) 0; 0 0 1];
    R = Rz*Ry*Rx;
    
    
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