function predictMotion(u)
%PREDICTMU Summary of this function goes here
%   Detailed explanation goes here 
    global State
    global Param
    
    %Qf = Param.Qf;
    
    M = Param.M;
    
    num_landmark = State.nl;
    Sigma = State.sigma;
    Mu = State.mu; 
    predMu = Mu;
    
    Gt = eye(4+3*num_landmark);
    Vt = zeros(4+3*num_landmark,3);
    
    Vt(1:2,1:2) = eye(2);
    
    predMu(1) = predMu(1)-u(1);
    predMu(2) = predMu(2)-u(2);
    predMu(3) = predMu(3);
    predMu(4) = predMu(4);
    State.mu = predMu;
    
    State.sigma = Gt*Sigma*Gt'+Vt*M*Vt';
end

