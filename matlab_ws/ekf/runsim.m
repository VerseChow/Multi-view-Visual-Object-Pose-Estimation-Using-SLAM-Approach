function varargout = runsim(t, pauseLen, makeVideo)

global Param;
global Data;
global State;
if ~exist('pauseLen', 'var') || isempty(pauseLen)
    pauseLen = [];
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end
%===================================================
State.t     = t;          % time
State.mu    = zeros(4,1); % robot initial pose
State.Sigma = zeros(4,4); % robot initial covariance
State.iR    = 1:4;        % 3 vector containing robot indices
State.iM    = [];         % 2*nL vector containing map indices
State.iL    = {};         % nL cell array containing indices of landmark i
State.sL    = [];         % nL vector containing signatures of landmarks
State.nL    = 0;          % scalar number of landmarks
%===================================================
if ~exist('pauseLen','var')
    pauseLen = 0.3; % seconds
end

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end
% Initalize Params
%===================================================
Param.initialStateMean = [0 0 0 0]';%%！！！！！！！！！！！！！！！！initialize with first feature detected

% Motion noise.
Param.M = diag([0.01 0.01 0].^2); % std of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
Param.beta = [0.1, 0.1, 0.1]; % [m, m, m]
Param.R = diag(Param.beta.^2);

% Step size between filter updates, can be less than 1.
Param.deltaT=0.1; % [s]

%===================================================

% Initialize State
%===================================================
State.mu = Param.initialStateMean;
State.Sigma = zeros(4);%%%%%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!!!initialize later with first feature detected

for t = 1:numSteps

    State.t = t;
    %=================================================
    % data available to your filter at this time step
    %=================================================
    u = getControl(t);
    z = getObservations(t);
    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    predictMotion(u);
    ekfupdate(z);
    
    %=================================================
    %TODO: plot and evaluate filter results here
    %=================================================

    drawnow;
    if pauseLen > 0
        pause(pauseLen);
    end
    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end
if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);v
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end
if nargout >= 1
    varargout{1} = Data;
end

%==========================================================================
function u = getControl(t)
global Data;
% noisefree control command
u = zeros(3,1);
u(1:2) = Data.u{t};  % 3x1 [drot1; dtrans; drot2]


%==========================================================================
function z = getObservations(t)
global Data;
% noisy observations
z = Data.realObservation(:,:,t); % 3xn [range; bearing; landmark id]
ii = find(~isnan(z(1,:)));
z = z(:,ii);

