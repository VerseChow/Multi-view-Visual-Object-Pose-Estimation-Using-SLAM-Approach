function predictMotion_pure(u)
%PREDICTMU Summary of this function goes here
%   Detailed explanation goes here 
    global State
    global Param
        
    Mu = State.mu; 
    predMu = Mu;
  
    predMu(1:4) = u * predMu(1:4);
    State.mu = predMu;
    
end

