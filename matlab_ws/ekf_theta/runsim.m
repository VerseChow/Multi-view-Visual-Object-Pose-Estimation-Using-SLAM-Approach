function [path_ekf,path_gt]=runsim(data_path, pauseLen)%, makeVideo)
%readData('/data_4_8/');
close all;
addpath('./../');
addpath('./../vlfeat-0.9.20/toolbox');
vl_setup;
%%%read the data
% readData(data_path);

%%%read the groundtruth data
% readGroundTruthData(data_path);

global tp_link;
global Param;
global Data;
global State;
global Table;

tp_link = readPCDFile_kar('tp_link_d.pcd');
Table = load('hash_table_1.mat');
makeVideo = 0;

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
State.iM    = [];         % nL*3 vector containing map indices
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
        vo = VideoWriter('video', 'Uncompressed AVI');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end
%Initalize Params
%===================================================
Param.initialStateMean = [Data.groundtruth(:,1);0];%%！！！！！！！！！！！！！！！！initialize with first feature detected

% Motion noise.
%Param.M = diag([1, 1, 1].*0.1); % std of noise proportional to alphas
Param.alphas = [0.05 0.001 0.05 0.01].^2;
% Standard deviation of Gaussian sensor noise (independent of distance)
Param.beta = [1, 1, 1].*0.1; % [m, m, m]
Param.R = diag(Param.beta.^1);

% Step size between filter updates, can be less than 1.
Param.deltaT=0.1; % [s]

%===================================================

% Initialize State
%===================================================
State.mu = Param.initialStateMean;
path_ekf = zeros(2,Data.num-1);
path_gt = zeros(2,Data.num-1);
path_gt1 = zeros(3,Data.num-1);
for t = 1:Data.num-1
    
    State.t = t;
    u = Data.u(:,t);
    predictMotion(u);
    path_gt(:, t) = State.mu(1:2);
    path_gt1(:, t) = State.mu(1:3);
end
State.mu = Param.initialStateMean;
State.Sigma = diag([1, 1, 0, 0.1].*0.001);%%%%%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!!!initialize later with first feature detected

%z = hashTableLookup(Data.image{1}, Data.pcd{1}, Data.pcd_base{1});


    
State.mu = Param.initialStateMean;
robot_traj = [];
obj_traj = [];
sigma_forplot = [];

for t = 1:Data.num-1
    
    State.t = t;
    %=================================================
    % data available to your filter at this time step
    %=================================================
    [z, features_orig, indices] = hashTableLookup(Data.image{t+1}, Data.pcd{t+1}, Data.pcd_base{t+1});

    u = Data.u(:,t);
    %=================================================
    %TODO: get motion control from data, do EKF prediction
    % and correction      
    %=================================================
    predictMotion(u);
    pred_obj=[path_gt1(:,t); 1];
    pred_Sigma = State.Sigma(1:2, 1:2);
    ekfupdate(z);
    path_ekf(:, t) = State.mu(1:2);
    sigma_forplot =[sigma_forplot State.Sigma(1:2, 1:2)];
    %=================================================
    %TODO: plot and evaluate filter results here
    %=================================================
    % Trajectory of robot
    temp = cell2mat(Data.base_transpose_Matrix(t+1))*[0; 0; 0; 1];
    robot_traj(:, end+1)= temp(1:3, 1);
    temp2 = cell2mat(Data.base_transpose_Matrix(t+1))*[Data.groundtruth(1:3, t+1); 1];
    obj_traj(:, end+1) = temp2;
    z_traj =  cell2mat(Data.base_transpose_Matrix(t+1))*[z(:, 1:3)'; ones(1, size(z, 1))];
    %Estimated object pose
    pred_obj = cell2mat(Data.base_transpose_Matrix(t+1))*pred_obj; 
    est_obj = cell2mat(Data.base_transpose_Matrix(t+1))*[State.mu(1), State.mu(2), State.mu(3), 1]';
     
    if makeVideo
        plotting(robot_traj, obj_traj, est_obj, pred_obj, pred_Sigma, z_traj, z, features_orig, indices, t+1, pauseLen,makeVideo,votype,vo);
    else
        plotting(robot_traj, obj_traj, est_obj, pred_obj, pred_Sigma, z_traj, z, features_orig, indices, t+1, pauseLen, 0, 0, 0);
    end
        drawnow;    
  
end

if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end


figure(2);
plot(path_ekf(1,:), path_ekf(2,:));
hold on;
plot(path_gt(1,:), path_gt(2,:));
plot(Data.groundtruth(1,:), Data.groundtruth(2,:));
legend('after','before','ground truth');
axis equal;
%%%plot the covriance on the path
% for i=1:20
%         centerX = path_ekf(1,i);
%         centerY = path_ekf(2,i);
%         cov = sigma_forplot(1:2,2*i-1:2*i);
%         h3 = plotcov2d(centerX, centerY, cov, 'c', 0, 0, 0, 1);
%         hold on;
% end
error_ekf = abs(path_ekf-Data.groundtruth(1:2,2:21));
error_pred = abs(path_gt-Data.groundtruth(1:2,2:21));
figure(3);
subplot(2,1,1)
hold on

plot((1:20),error_pred(1,:),'LineWidth',10);

plot((1:20),error_ekf(1,:), 'LineWidth',10);
legend('error before using EKF SLAM for x','error after using EKF SLAM for x');
title('Error for x');
subplot(2,1,2)
hold on

plot((1:20),error_pred(2,:), 'LineWidth',10);

plot((1:20),error_ekf(2,:), 'LineWidth',10);
legend('error before using EKF SLAM for y','error after using EKF SLAM for y');
title('Error for y');
end

function plotting(robot_traj, obj_traj, est_obj, pred_obj, pred_Sigma, z_traj, z, features_orig, indices, t, pauseLen,makeVideo,votype,vo)
  global Data;
  global tp_link;
  global Table;
  global State;
  %figure('units','normalized','outerposition',[0 0 1 1])
  
  subplot(3, 4, [1:3, 5:7, 9:11]);
  %robot and obj trajs
  plot(robot_traj(1, :), robot_traj(2, :), 'r','LineWidth',2); hold on;
  %plot(obj_traj(1, :), obj_traj(2, :), 'b*');
  line([robot_traj(1, end), est_obj(1)], [robot_traj(2, end), est_obj(2)], 'Color', 'g','LineWidth',2);
  %3d points projected into 2d
  plot(z_traj(1, :), z_traj(2, :), 'g*');
  plotrobot(robot_traj(1, end), robot_traj(2, end), 0, 'b', 1, 'y'); hold on;
  plotcircle([obj_traj(1, end), obj_traj(2, end)], 0.01, 100, 'k', 1, 'r');  
  %sigma of the object pose
  plotcov2d(est_obj(1), est_obj(2), State.Sigma(1:2, 1:2), 'r', 0, 'r', 0.5, 1);
  plotcov2d(pred_obj(1), pred_obj(2), pred_Sigma(1:2, 1:2), 'b', 0, 'b', 0.5, 1);
  plotObj(obj_traj(1, end), obj_traj(2, end), 0.285, 0.065, 'g');
  plotObj(pred_obj(1), pred_obj(2), 0.285, 0.065, 'b');
  plotObj(est_obj(1), est_obj(2), 0.285, 0.065, 'r');
  axis([[-0.3, 1.2], [-0.15, 1]]);
  pbaspect([1 1 1])
  
  %sigma of landmark covariance
%   for i = 1:State.nL
%       landmark_mu = cell2mat(Data.base_transpose_Matrix(t))*[State.mu(State.iL{i}); 1];
%       Sigma = State.Sigma(State.iL{i}, State.iL{i});
%       plot(landmark_mu(1), landmark_mu(2), 'b*');
%       plotcov2d(landmark_mu(1), landmark_mu(2), Sigma(1:2, 1:2), 'b', 0, 'b', 0.5, 0.1);
%   end
  
  subplot(3, 4, 4);
  plot(z_traj(1, :), z_traj(2, :), 'g*'); hold on;
  plotcircle([obj_traj(1, end), obj_traj(2, end)], 0.01, 100, 'k', 1, 'r');  
%   plotcov2d(est_obj(1), est_obj(2), State.Sigma(1:2, 1:2), 'r', 0, 'r', 0.5, 3);
%   plotcov2d(pred_obj(1), pred_obj(2), pred_Sigma(1:2, 1:2), 'b', 0, 'b', 0.5, 3);
  plotObj(obj_traj(1, end), obj_traj(2, end), 0.285, 0.065, 'g');
  plotObj(pred_obj(1), pred_obj(2), 0.285, 0.065, 'b');
  plotObj(est_obj(1), est_obj(2), 0.285, 0.065, 'r');
  axis([[0.4, 0.8], [0.7, 1]]);
  
  subplot(3, 4, 8);
  imshow(Data.image{t}); hold on;
  plot(features_orig.rgb_pix(1, indices), features_orig.rgb_pix(2, indices), 'r*');
  
  subplot(3, 4, 12);

  scatter3(tp_link(:, 1), -tp_link(:, 3), tp_link(:, 2), 'g','filled'); hold on;
  scatter3(Table.hash_table.depth_loc(1, :), -Table.hash_table.depth_loc(3, :), Table.hash_table.depth_loc(2, :),150, 'r', 'filled'); hold on;
  scatter3(Table.hash_table.depth_loc(1, z(:, 4)), -Table.hash_table.depth_loc(3, z(:, 4)), Table.hash_table.depth_loc(2, z(:, 4)),150, 'b', 'filled');
  view(-20, 16);
  pbaspect([1.5 1 1])
    
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
  drawnow;
  if pauseLen > 0
         pause(pauseLen);
  end
  hold off;
  
  clf;
end


