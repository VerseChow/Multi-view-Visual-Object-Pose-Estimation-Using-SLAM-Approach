function predictMot3ion(u)
%PREDICTMU Summary of this function goes here
%   Detailed explanation goes here 
    global State
    global Param

    M = zeros(3,3);
    M(1,1) = Param.alphas(1)*u(1)^2+Param.alphas(2)*u(2)^2;
    M(2,2) = Param.alphas(3)*u(2)^2+Param.alphas(4)*(u(1)^2+u(3)^2);
    M(3,3) = Param.alphas(1)*u(1)^2+Param.alphas(2)*u(2)^2;
    
    num_landmark = State.nL;
    Sigma = State.Sigma;
    Mu = State.mu; 
    predMu = Mu;
    predMu(1) = Mu(1)+u(2)*cos(Mu(4)+u(1));
    predMu(2) = Mu(2)+u(2)*sin(Mu(4)+u(1));
    predMu(4) = wrapToPi(Mu(4)+u(1)+u(3));
    
    for i = 1:State.nL
        predMu(State.iL{i}) = Mu(State.iL{i})+[ u(2)*cos(Mu(4)+u(1)), u(2)*sin(Mu(4)+u(1)), 0 ]';
    end
    
    
    Gt = eye(4+3*num_landmark);
    Vt = zeros(4+3*num_landmark,3);
    Gt(1:4, 1:4) = eye(4);
    Gt(1,4) = -u(2)*sin(predMu(4)+u(1));
    Gt(2,4) = u(2)*cos(predMu(4)+u(1));

    Vt(1,1) = -u(2)*sin(predMu(4)+u(1));
    Vt(1,2) = cos(predMu(4)+u(1));
    Vt(2,1) = u(2)*cos(predMu(4)+u(1));
    Vt(2,2) = sin(predMu(4)+u(1));
    Vt(4,1) = 1;
    Vt(4,3) = 1;
    
    for i = 1:State.nL
        V_m = zeros(3,3);
        V_m(1,1) = -u(2)*sin(predMu(4)+u(1));
        V_m(1,2) = cos(predMu(4)+u(1));
        V_m(2,1) = u(2)*cos(predMu(4)+u(1));
        V_m(2,2) = sin(predMu(4)+u(1));
        Vt(State.iL{i}, :) = V_m;
    end
    
    
    State.mu = predMu;
    
    State.Sigma = Gt*Sigma*Gt'+Vt*M*Vt';
end

