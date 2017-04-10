function [TR, TT] = icp_standard(q,p)
%Perform ICP in a bruteforce manner

iter = 15;

dim = size(q,1);

Np = size(p,2);

% Transformed point cloud
pt = p;

% Temporary transform vec and matrix
T = zeros(dim,1);
R = eye(dim,dim);

% Init temporary transform vectors and matrices

TT = zeros(dim,1,iter + 1);
TR = repmat(eye(dim,dim),[1,1,iter+1]);

weights = ones(size(p,2));

for k=1:iter
    
    [match, mindist] = match_bruteForce(q,pt, dim);
    p_idx = true(1, Np);
    q_idx = match;    
    
    % Point to point minimization
    [R,T] = eq_point(q(:,q_idx),pt(:,p_idx), weights(p_idx), dim);
    
    TR(:,:,k+1) = R*TR(:,:,k);
    TT(:,:,k+1) = R*TT(:,:,k)+T;
    
    pt = TR(:,:,k+1) * p + repmat(TT(:,:,k+1),1,Np);
    
end

TR = TR(:,:,end);
TT = TT(:,:,end);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [match mindist] = match_bruteForce(q, p, dim)
    m = size(p,2);
    n = size(q,2);    
    match = zeros(1,m);
    mindist = zeros(1,m);
    for ki=1:m
        d=zeros(1,n);
        for ti=1:dim
            d=d+(q(ti,:)-p(ti,ki)).^2;
        end
        [mindist(ki),match(ki)]=min(d);
    end
    
    mindist = sqrt(mindist);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [R,T] = eq_point(q,p,weights,dim)

m = size(p,2);
n = size(q,2);

% normalize weights
weights = weights ./ sum(weights);

% find data centroid and deviations from centroid
q_bar = q * transpose(weights);
q_mark = q - repmat(q_bar, 1, n);
% Apply weights
q_mark = q_mark .* repmat(weights, dim, 1);

% find data centroid and deviations from centroid
p_bar = p * transpose(weights);
p_mark = p - repmat(p_bar, 1, m);
% Apply weights
%p_mark = p_mark .* repmat(weights, 3, 1);

N = p_mark*transpose(q_mark); % taking points of q in matched order

[U,~,V] = svd(N); % singular value decomposition

R = V*diag([1 1 det(U*V')])*transpose(U);

T = q_bar - R*p_bar;

end