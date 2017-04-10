function count_landmark = ekfupdate(z)
    global Param;
    global State;
    global Table;
    % batch update

    % returns state vector indices pairing observations with landmarks      
    Li = da_nn(z);
    
    num = size(z,1);
    count = 0;
    
    for i=1:num
        if Li(i)==0
            index = initialize_new_landmark(z(i,:));
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

            df = Table.hash_table.depth_loc(:,State.sL(Li(i)));
            zhat(3*j-2) = State.mu(1)+df(1);
            zhat(3*j-1) = State.mu(2)+df(2);
            zhat(3*j) = State.mu(3)+df(3);
            dz(3*j-2) = z(i,1)-zhat(3*j-2)-0.05;
            dz(3*j-1) = z(i,2)-zhat(3*j-1)+0.05;
            dz(3*j) = z(i,3)-zhat(3*j);
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
    State.Sigma = temp*State.Sigma;
   
    count_landmark = count;
end
