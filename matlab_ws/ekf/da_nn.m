function Li = da_nn(z, R)

    global Param;
    global State;
    num = size(z,2);
    Li = zeros(1,num);
    for i = 1:num
        Dist_min = flintmax;
        nearest = 0;
        for j = 1:State.Ekf.nL
            [Sigma, zhat] = get_SigmaZhat(j);
            Dist_ij = mahalanobis2(z(:,i), zhat, Sigma);%mahalanobis distance calculation
            if Dist_ij < Dist_min 
                nearest = j;
                Dist_min = Dist_ij;
            end
        end
        if Dist_min <= chi2inv(0.95,Param.maxObs);
            Li(i) = nearest;%find existing landmark
        elseif Dist_min >= chi2inv(0.99999,Param.maxObs);
            Li(i) = 0;%add new landmark
        else
            Li(i) = -1;%ambiuitious section, do nothing
        end
    end
end

function dist = mahalanobis2(z,zhat,Sigma)
    delta_z = z-zhat;
    delta_z(2) = minimizedAngle(delta_z(2));
    dist = delta_z'/Sigma*delta_z;
end

function [Sigma, zhat] = get_SigmaZhat(index)
    global State;
    global Param;
    robot_pose = State.Ekf.mu(State.Ekf.iR);
    
    deltaX = State.Ekf.mu(2+2*index)-State.Ekf.mu(1);
    deltaY = State.Ekf.mu(3+2*index)-State.Ekf.mu(2);
    
    Ht = zeros(2,3+2*size(State.Ekf.sL,2));
    q = deltaX^2+deltaY^2;
    q_sqrt = sqrt(q);
    
    Ht(1:2,1:3) = [-deltaX/sqrt(q), -deltaY/sqrt(q), 0; deltaY/q, -deltaX/q, -1];
    Ht(1:2,2*index+2:3+2*index) = [deltaX/sqrt(q), deltaY/sqrt(q);-deltaY/q, deltaX/q];
    
    Sigma = Ht*State.Ekf.Sigma*Ht' + Param.R;
    zhat = [q_sqrt;
            minimizedAngle(atan2(deltaY, deltaX) - robot_pose(3))];
end
