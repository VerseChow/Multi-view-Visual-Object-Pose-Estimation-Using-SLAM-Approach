function list_3d_points = get3Dfeaturesfrom2D(pcd_file, list_pixels)
% This code is going to take the [list_pixels] which has collection of 2D
% features of image and use the corresponding point cloud file to get the
% 3D locations of the features

% IMPORTANT: We are using organized point cloud for this function to work


end

function [x, y, z] = get3DPoint(i, j, pcd_file)
% pcd_file is ranging from 1:307200
point_cloud_data = readPCDFile(pcd_file);

end

function pc = readPCDFile(pcd_file)
% Read the point cloud data

end