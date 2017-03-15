function list_3d_points = get3Dfeaturesfrom2D(D_data, list_pixels)
% This code is going to take the [list_pixels] which has collection of 2D
% features of image and use the corresponding point cloud file to get the
% 3D locations of the features

% IMPORTANT: We are using organized point cloud for this function to work
list_3d_points = [];
for i=1:size(list_pixels, 1)
   X = get3DPoint(list_pixels(i,1),list_pixels(i,2), D_data);
   if(X(1)~=NaN)
       list_3d_points(end+1, :) = X;
   end
end

end


