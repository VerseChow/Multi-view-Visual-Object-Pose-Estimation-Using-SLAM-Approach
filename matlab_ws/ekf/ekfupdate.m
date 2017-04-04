function count_landmark = ekfupdate(z)
    global Param;
    global State;
    % batch update

    % returns state vector indices pairing observations with landmarks      
    Li = da_nn(z(1:3,:), Param.R);
    
    num = size(z,2);
    count = 0;
    
    for i=1:num
        if Li(i)==0
            index = initialize_new_landmark(z(:,i), Param.R);
            Li(i) = index;%update the new landmark index
        end
        if Li(i)==-1
            count = count+1;
        end
    end

    count = num-count;%Valid landmark number
        
    
    zhat = zeros(3*count,1);
    dz = zhat;
    Ht = zeros(3*count,4+3*State.nL);
    j = 1;
    R = [];
    for i=1:num
        if Li(i)~=-1
%%%%%%%%%%%%%Stack the observation to batch type
            df = State.feature_dist(3*Li(i)-2:3*Li(i));
            zhat(3*j-2) = State.mu(1)+df(1);
            zhat(3*j-1) = State.mu(2)+df(2);
            zhat(3*j) = State.mu(3)+df(3);
            dz(3*j-2) = z(1,i)-zhat(3*j-2);
            dz(3*j-1) = z(2,i)-zhat(3*j-1);
            dz(3*j) = z(3,i)-zhat(3*j);
            Ht(3*j-2:3*j,1:3) = eye(3);
            Ht(3*j-2:3*j,3*Li(i)+2:3*Li(i)+4) = eye(3);
            R = blkdiag(R, Param.R);
            j = j+1;
        end
    end

%%%%%%%%%%%%%Correction Step of Batch
    S = Ht*State.Sigma*Ht'+R;
    Kt = State.Sigma*Ht'/S;
    State.mu = State.mu+Kt*dz;
    temp = eye(size(State.Sigma))-Kt*Ht;
    State.Sigma = temp*State.Sigma*temp'+Kt*R*Kt';
   
    count_landmark = count;
end
%%%%%%%%%%%%%%Construct Ht matrix
function [Ht, zt] = getHt(index)
    global State;
    Ht = zeros(2,3+2*size(State.Ekf.sL,2));
    zt = zeros(2,1);
    deltaX = State.Ekf.mu(2+2*index)-State.Ekf.mu(1);
    deltaY = State.Ekf.mu(3+2*index)-State.Ekf.mu(2);
    q = deltaX^2+deltaY^2;
    Ht(1:2,1:3) = [-deltaX/sqrt(q), -deltaY/sqrt(q), 0; deltaY/q, -deltaX/q, -1];
    Ht(1:2,2*index+2:3+2*index) = [deltaX/sqrt(q), deltaY/sqrt(q);-deltaY/q, deltaX/q];
    zt(1) = sqrt(q);
    zt(2) = minimizedAngle(atan2(deltaY, deltaX)-State.Ekf.mu(3));
end
