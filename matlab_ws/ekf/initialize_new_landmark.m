function idex = initialize_new_landmark(z, R)
    global Param;
    global State;
    State.Ekf.nL = State.Ekf.nL + 1;
    State.Ekf.sL = [State.Ekf.sL, z(3)];
    robot_pose = State.Ekf.mu(State.Ekf.iR);
    new_landmark = zeros(2,1);
    new_landmark(1) = robot_pose(1) + z(1)*cos(minimizedAngle(z(2)+robot_pose(3)));
    new_landmark(2) = robot_pose(2) + z(1)*sin(minimizedAngle(z(2)+robot_pose(3)));
    State.Ekf.mu = [State.Ekf.mu; new_landmark];
    landmarkSigma = flintmax.*eye(2) ;
    State.Ekf.Sigma= blkdiag(State.Ekf.Sigma, landmarkSigma);
    idex = State.Ekf.nL;
%     angle = minimizedAngle(z(2)+State.Ekf.mu(3));
%     L = zeros(2, 3);
%     W = zeros(2, 2);
%     L(1:2,1:2) = eye(2);
%     L(1,3) = -z(1)*sin(angle);
%     L(2,3) = z(1)*cos(angle); 
%     W(1,1) = cos(angle);
%     W(2,1) = sin(angle);
%     W(1,2) = -z(1)*sin(angle);
%     W(2,2) = z(1)*sin(angle);
%     a = L*State.Ekf.Sigma(1:3,1:3)*L'+W*R*W';
%     s = size(State.Ekf.Sigma);
%     if s>3
%         State.Ekf.Sigma = [State.Ekf.Sigma,...
%                           [State.Ekf.Sigma(1:3,1:3)*L';State.Ekf.Sigma(4:s,1:3)*L']];
% 
%         State.Ekf.Sigma = [State.Ekf.Sigma;...
%                           [L*State.Ekf.Sigma(1:3,1:3),L*State.Ekf.Sigma(1:3,4:s),a]];
%     else
%         State.Ekf.Sigma = [State.Ekf.Sigma,...
%                            State.Ekf.Sigma(1:3,1:3)*L'];
%         State.Ekf.Sigma = [State.Ekf.Sigma;...
%                           [L*State.Ekf.Sigma(1:3,1:3),a]];
%     end  
%     
end
