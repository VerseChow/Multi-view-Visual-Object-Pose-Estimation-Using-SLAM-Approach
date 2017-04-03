function  [predMu, predSigma] = predictMotion( Mu, Sigma, u, alpha, M, Qf)
%PREDICTMU Summary of this function goes here
%   Detailed explanation goes here 
    predMu = Mu;
    Gt = eye(4);
    Gt(4, 2) = -u(1)*(Mu(2)-u(2))^2;
    Vt = zeros(4,2);
    Vt(1,1) = -1;
    Vt(2,2) = sin(alpha);
    Vt(3,2) = -cos(alpha);
    Vt(4,1) = 1/(Mu(2)-u(2));
    Vt(4,2) = Mu(2)/(Mu(2)-u(2))^2;
    predMu(1) = predMu(1)-u(1);
    predMu(2) = predMu(2)+u(2)*sin(alpha);
    predMu(3) = predMu(3)-u(2)*cos(alpha);
    predMu(4) = predMu(4)+u(1)/(Mu(2)-u(2));
    predSigma = Gt*Sigma*Gt'+Vt*M*Vt'+Qf;
end

