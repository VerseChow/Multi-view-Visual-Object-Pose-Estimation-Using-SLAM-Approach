function varargout = run(numSteps, choice, pauseLen, da, makeVideo)
% RUN PS3 EKF Feature-Based SLAM
%   RUN(ARG)
%   RUN(ARG, CHOICE, PAUSELEN)
%   RUN(ARG, CHOICE, PAUSELEN, DA)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data structure from a previous run.
%      CHOICE - is either 'sim' or 'vp' for simulator or Victoria Park
%               data set, respectively.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      DA - data assocation, is one of either:
%           'known' - only available in simulator
%           'nn'    - incremental maximum likelihood nearest neighbor
%           'nndg'  - nn double gate on landmark creation
%                     (throws away ambiguous observations)
%           'jcbb'  - joint compatability branch and bound
%
%   DATA = RUN(ARG, CHOISE, PAUSELEN, DA)
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.
%
%   Note: more parameters can be controlled in the run.m file itself via
%   fields of the Param structure.

%   (c) 2009-2015
%   Ryan M. Eustice
%   University of Michigan
%   eustice@umich.edu
close all;
addpath('./slamsim');
addpath('./vicpark');

if ~exist('pauseLen', 'var') || isempty(pauseLen)
    pauseLen = [];
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

clear global Param State Data;
global Param;
global State;
global Data;


% select which data association method to use in ekfupdate.m, choices are:
%   known - only available in simulator
%   nn    - incremental maximum likelhood nearest neighbor
%   nndg  - nn double gate on landmark creation (throws away ambiguous observations)
%   jcbb  - joint compatability branch and bound
if ~exist('da','var') || isempty(da)
    da = 'known';
end
Param.dataAssociation = da;

% select which update method to use in ekfupdate.m, choices are:
%   batch  - batch updates
%   seq    - sequential updates
Param.updateMethod = 'seq';

% size of bounding box for VP data set plotting
Param.bbox = 0; % bbox = 20 [m] speeds up graphics

% Structure of global State variable
%===================================================
State.Ekf.t     = 0;          % time
State.Ekf.mu    = zeros(3,1); % robot initial pose
State.Ekf.Sigma = zeros(3,3); % robot initial covariance
State.Ekf.iR    = 1:3;        % 3 vector containing robot indices
State.Ekf.iM    = [];         % 2*nL vector containing map indices
State.Ekf.iL    = {};         % nL cell array containing indices of landmark i
State.Ekf.sL    = [];         % nL vector containing signatures of landmarks
State.Ekf.nL    = 0;          % scalar number of landmarks
%===================================================

switch lower(choice)
    case 'sim'
        Data = runsim(numSteps,pauseLen,makeVideo);
        if nargout > 0
            varargout{1} = Data;
        end
    case 'vp'
        runvp(numSteps,pauseLen, makeVideo);
    otherwise
        error('unrecognized selection "%s"', choice);
end
figure(2)
det_array = zeros(1,State.Ekf.nL);
for i = 1:State.Ekf.nL    
    det_array(i) = det(State.Ekf.Sigma(2*i+2:2*i+3,2*i+2:2*i+3));    
end
plot(1:State.Ekf.nL, det_array, '-x');
xlabel('feature id');
ylabel('Determinant');
set(gca,'XTick',[1:1:State.Ekf.nL]);
figure(3)
subplot(1,2,1);
correlate_matrix = corrcov(State.Ekf.Sigma(4:2*State.Ekf.nL+3,4:2*State.Ekf.nL+3));
image = (ones(size(correlate_matrix))-correlate_matrix)*255;
imshow(image, [0 255]);
subplot(1,2,2);
hold on;
for i = 1:State.Ekf.nL
    cor_array = [];
    for j = 1:State.Ekf.nL
        cor_array = [cor_array, correlate_matrix(2*i-1,2*j-1)];
    end
    plot(1:State.Ekf.nL, cor_array);
    set(gca,'XTick',[1:1:State.Ekf.nL]);
end
