function predictMotion(u)
%PREDICTMU Summary of this function goes here
%   Detailed explanation goes here 
    global State
    global Param
    
    %Qf = Param.Qf;
    
    num_landmark = State.nL;
    Sigma = State.Sigma;
    Mu = State.mu; 
    predMu = Mu;
    
    Gt = eye(4+3*num_landmark);
    Vt = zeros(4+3*num_landmark,3);
    Gt(1:4, 1:4) = u+Param.M;
    Vt(1:2,1:2) = eye(2);
    
    predMu(1:4) = u * predMu(1:4);
    State.mu = predMu;
    
    State.Sigma = Gt*Sigma*Gt';
end

