function idex = initialize_new_landmark(z)
%z contains x,y,z and index from hash table
    global State;
    global Table;
    global Param;
    State.nL = State.nL + 1;
    State.sL = [State.sL, z(4)];
    robot_pose = State.mu(State.iR);
    
    new_landmark = robot_pose(1:3) + Table.hash_table.depth_loc(:,z(4));
    
    State.mu = [State.mu; new_landmark];
    L = zeros(3, 4);
    W = zeros(3, 3);
    L(1:3,1:3) = eye(3);
    W = eye(3);
    a = L*State.Sigma(1:4,1:4)*L'+W*Param.R*W';
    s = size(State.Sigma);
    if s>4
        State.Sigma = [State.Sigma,...
                          [State.Sigma(1:4,1:4)*L';State.Sigma(5:s,1:4)*L']];

        State.Sigma = [State.Sigma;...
                          [L*State.Sigma(1:4,1:4),L*State.Sigma(1:4,5:s),a]];
    else
        State.Sigma = [State.Sigma,...
                           State.Sigma(1:4,1:4)*L'];
        State.Sigma = [State.Sigma;...
                          [L*State.Sigma(1:4,1:4),a]];
    end  
    idex = State.nL;

    State.iL{idex} = 3*State.nL+2:4+3*State.nL;
    State.iM = [State.iM; [3*State.nL+2:4+3*State.nL]'];
    
end
