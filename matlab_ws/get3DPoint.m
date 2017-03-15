function X = get3DPoint(i, j, D_data)
% 3D_data is ranging 307200x3
if(size(D_data, 1)~=307200)
    disp('The point cloud data is not organized point cloud');
    return;
end
ind = i+(j-1)*640;
X = D_data(ind, :);
end
