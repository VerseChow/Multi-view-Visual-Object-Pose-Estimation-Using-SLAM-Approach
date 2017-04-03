function [Li] = da_known(z)
% EKF-SLAM data association with known correspondences
    global State;
    num = size(z,2);
    Li = zeros(1, num);
    for i=1:num
        temp = find(State.Ekf.sL == z(i));
        if ~isempty(temp)
            Li(i) = temp;
        else
            Li(i) = 0;
        end
    end
        
end




