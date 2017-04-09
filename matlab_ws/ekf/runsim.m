function [path_ekf,path_gt]=runsim() %pauseLen, makeVideo)
%readData('/0_base_link/');
addpath('./../');
addpath('./../vlfeat-0.9.20/toolbox');
vl_setup;

global Param;
global Data;
global State;
global Table;

Table = load('hash_table_1.mat');
% if ~exist('pauseLen', 'var') || isempty(pauseLen)
%     pauseLen = [];
% end
% if ~exist('makeVideo','var') || isempty(makeVideo)
%     makeVideo = false;
% end
%===================================================
State.t     = 0;          % time
State.mu    = zeros(4,1); % robot initial pose
State.Sigma = zeros(4,4); % robot initial covariance
State.iR    = 1:4;        % 3 vector containing robot indices
State.iM    = [];         % 2*nL vector containing map indices
State.iL    = {};         % nL cell array containing indices of landmark i
State.sL    = [];         % nL vector containing signatures of landmarks
State.nL    = 0;          % scalar number of landmarks
%===================================================
% if ~exist('pauseLen','var')
%     pauseLen = 0.3; % seconds
% end

% if makeVideo
%     try
%         votype = 'avifile';
%         vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
%     catch
%         votype = 'VideoWriter';
%         vo = VideoWriter('video', 'MPEG-4');
%         set(vo, 'FrameRate', min(5, 1/pauseLen));
%         open(vo);
%     end
% end
% Initalize Params
%===================================================
Param.initialStateMean = [0.392657 0.751307 0.861234 0]';%%！！！！！！！！！！！！！！！！initialize with first feature detected

% Motion noise.
Param.M = diag([0.01 0.01 0.01].^2); % std of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
Param.beta = [0.1, 0.1, 0.1]; % [m, m, m]
Param.R = diag(Param.beta.^2);

% Step size between filter updates, can be less than 1.
Param.deltaT=0.1; % [s]

%===================================================

% Initialize State
%===================================================
State.mu = Param.initialStateMean;
State.Sigma = diag([0.1, 0.1, 0.1, 0.1]);%%%%%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!!!initialize later with first feature detected

%z = hashTableLookup(Data.image{1}, Data.pcd{1}, Data.pcd_base{1});

path_ekf = zeros(2,Data.num-1);
path_gt = zeros(2,Data.num-1);
for t = 1:Data.num-1
    
    State.t = t;
    %=================================================
    % data available to your filter at this time step
    %=================================================
    z = hashTableLookup(Data.image{t}, Data.pcd{t}, Data.pcd_base{t});
    u = Data.u{t};
    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    predictMotion(u);
    ekfupdate(z);
    path_ekf(:, t) = State.mu(1:2);
    
    disp(State.mu(State.iR));
    %=================================================
    %TODO: plot and evaluate filter results here
    %=================================================
%     drawnow;
%     if pauseLen > 0
%         pause(pauseLen);
%     end
%     
%     if makeVideo
%         F = getframe(gcf);
%         switch votype
%           case 'avifile'
%             vo = addframe(vo, F);
%           case 'VideoWriter'
%             writeVideo(vo, F);
%           otherwise
%             error('unrecognized votype');
%         end
%     end
end
State.mu = Param.initialStateMean;
State.Sigma = zeros(4);
    
for t = 1:Data.num-1
    
    State.t = t;
    u = Data.u{t};

    predictMotion(u);
    path_gt(:, t) = State.mu(1:2);

    disp(State.mu(State.iR));
end
plot(path_ekf(1,:), path_ekf(2,:));
hold on;
plot(path_gt(1,:), path_gt(2,:))




