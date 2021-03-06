function [path_ekf,path_gt]=runsim(data_path, pauseLen)%, makeVideo)
%readData('/0_base_link/');
close all;
addpath('./../');
addpath('./../vlfeat-0.9.20/toolbox');
addpath('./../icp');
vl_setup;

%%%read the data
readData(data_path);

%%%read the groundtruth data
%readGroundTruthData(data_path);

global tp_link;
global Param;
global Data;
global State;
global Table;

tp_link = readPCDFile_kar('./tp_link.pcd');
Table = load('hash_4_8_data.mat');


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
%Param.initialStateMean = [0.56415 0.85277 0.86562 1]';%%！！！！！！！！！！！！！！！！initialize with first feature detected
Param.initialStateMean = [0.566201746463776;0.849269747734070;0.859818756580353; 1];
% Motion noise.
Param.M = diag([1, 1, 1, 0].*0.01); % std of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
Param.beta = [1, 1, 1].*0.001; % [m, m, m]
Param.R = diag(Param.beta.^1);

% Step size between filter updates, can be less than 1.
Param.deltaT=0.1; % [s]

%===================================================

% Initialize State
%===================================================
State.mu = Param.initialStateMean;
State.Sigma = diag([1, 1, 0, 0].*0.01);%%%%%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!!!initialize later with first feature detected

%z = hashTableLookup(Data.image{1}, Data.pcd{1}, Data.pcd_base{1});

path_ekf = zeros(2,Data.num-1);
path_gt = zeros(2,Data.num-1);
    
State.mu = Param.initialStateMean;
robot_traj = [];
obj_traj = [];

for t = 1:Data.num-1
    
    State.t = t;
    %=================================================
    % data available to your filter at this time step
    %=================================================
    [z, features_orig, indices] = hashTableLookup(Data.image{t+1}, Data.pcd{t+1}, Data.pcd_base{t+1});
%    figure;
%    size(z(:,1))
%    scatter3(z(:,1),z(:,2),z(:,3),'b');
%    obs = unique([z(:,1),z(:,2),z(:,3)]);
    obs_indices = z(:,4);
    ground_filtered = Table.hash_table.depth_loc(:,obs_indices);
    obs = z(:,1:3);
    ground = [ground_filtered(1,:);ground_filtered(2,:);ground_filtered(3,:)];
%    hold on
%    scatter3(Table.hash_table.depth_loc(1,:),...
%        Table.hash_table.depth_loc(2,:),Table.hash_table.depth_loc(3,:),'r');
%    [TR TT ER] = icp_standard(obs',ground);
    [TR TT] = icp_std(obs',ground);
    gt = Data.groundtruth(:,t);

    [TR_std TT_std] = icp(obs',ground,15);
    [TR_kd TT_kd] = icp(obs',ground,15,'Matching','kDtree','Extrapolation',true);    
    Dicp_closed = TR * ground + TT;
    Dicp_std = TR_std * ground + TT_std;
    Dicp_kd = TR_kd * ground + TT_kd
    plot_icp(obs',ground,Dicp_closed, Dicp_std, Dicp_kd);
    
     u = Data.G{t};
    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    predictMotion(u);
    pred_obj=[State.mu(1:3); 1];
    pred_Sigma = State.Sigma(1:2, 1:2);
    ekfupdate(z);
    path_ekf(:, t) = State.mu(1:2);
    
%     disp(State.mu(State.iR));
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
%     robot_obj_translate = temp(1:3,1) - temp2(1:3,1);
%     computed = TT - robot_obj_translate;
%     final = [computed, gt]
%     Dicp = TR * ground + TT;
%    plot_icp(obs',ground,Dicp);
%    plot_landmark(z);
     
    
    plotting(robot_traj, obj_traj, est_obj, pred_obj, pred_Sigma, ...
        z_traj, z, features_orig, indices, t+1, pauseLen);

end
State.mu = Param.initialStateMean;

for t = 1:Data.num-1
    
    State.t = t;
    u = Data.G{t};
    predictMotion(u);
    path_gt(:, t) = State.mu(1:2);
    
end
figure(2);
plot(path_ekf(1,:), path_ekf(2,:));
hold on;
plot(path_gt(1,:), path_gt(2,:));
plot(Data.groundtruth(1,:), Data.groundtruth(2,:));
legend('after','before','ground truth');
axis equal;
end

function plotting(robot_traj, obj_traj, est_obj, pred_obj, pred_Sigma, z_traj, z, features_orig, indices, t, pauseLen)
  global Data;
  global tp_link;
  global Table;
  global State;
  %figure('units','normalized','outerposition',[0 0 1 1])
  
  subplot(3, 4, [1:3, 5:7, 9:11]);
  %robot and obj trajs
  plot(robot_traj(1, :), robot_traj(2, :), 'r'); hold on;
  %plot(obj_traj(1, :), obj_traj(2, :), 'b*');
  line([robot_traj(1, end), est_obj(1)], [robot_traj(2, end), est_obj(2)], 'Color', 'g');
  %3d points projected into 2d
  plot(z_traj(1, :), z_traj(2, :), 'g*');
  plotrobot(robot_traj(1, end), robot_traj(2, end), 0, 'b', 1, 'y'); hold on;
  plotcircle([obj_traj(1, end), obj_traj(2, end)], 0.01, 100, 'k', 1, 'r');  
  %sigma of the object pose
  plotcov2d(est_obj(1), est_obj(2), State.Sigma(1:2, 1:2), 'r', 0, 'r', 0.5, 3);
  plotcov2d(pred_obj(1), pred_obj(2), pred_Sigma(1:2, 1:2), 'b', 0, 'b', 0.5, 3);
  
  plotObj(obj_traj(1, end), obj_traj(2, end), 0.285, 0.065, 'g');
  plotObj(pred_obj(1), pred_obj(2), 0.285, 0.065, 'b');
  plotObj(est_obj(1), est_obj(2), 0.285, 0.065, 'r');
  axis([[-0.3, 1.2], [-0.15, 1]]);
  pbaspect([1 1 1])
  
  subplot(3, 4, 4);
  plot(z_traj(1, :), z_traj(2, :), 'g*'); hold on;
  plotcircle([obj_traj(1, end), obj_traj(2, end)], 0.01, 100, 'k', 1, 'r');  
  plotcov2d(est_obj(1), est_obj(2), State.Sigma(1:2, 1:2), 'r', 0, 'r', 0.5, 3);
  plotcov2d(pred_obj(1), pred_obj(2), pred_Sigma(1:2, 1:2), 'b', 0, 'b', 0.5, 3);
  plotObj(obj_traj(1, end), obj_traj(2, end), 0.285, 0.065, 'g');
  plotObj(pred_obj(1), pred_obj(2), 0.285, 0.065, 'b');
  plotObj(est_obj(1), est_obj(2), 0.285, 0.065, 'r');
  axis([[0.45, 0.8], [0.8, 1]]);
  
  subplot(3, 4, 8);
  imshow(Data.image{t}); hold on;
  plot(features_orig.rgb_pix(1, indices), features_orig.rgb_pix(2, indices), 'r*');
  
  subplot(3, 4, 12);
  scatter3(tp_link(:, 1), -tp_link(:, 3), tp_link(:, 2), '.'); hold on;
  scatter3(Table.hash_table.depth_loc(1, :), -Table.hash_table.depth_loc(3, :), Table.hash_table.depth_loc(2, :), 'r', 'filled'); hold on;
  scatter3(Table.hash_table.depth_loc(1, z(:, 4)), -Table.hash_table.depth_loc(3, z(:, 4)), Table.hash_table.depth_loc(2, z(:, 4)), 'y', 'filled');
  view(-20, 16);
  pbaspect([1.5 1 1])
  
%   subplot(3,5,5);
%   plot3(M(1,:),M(2,:),M(3,:),'b.',D(1,:),D(2,:),D(3,:),'r.');
%   axis equal;
%   xlabel('x'); ylabel('y'); zlabel('z');
%   title('Original');
% 
%   % Plot the results
%   subplot(3,5,10);
%   plot3(M(1,:),M(2,:),M(3,:),'b.',Dicp(1,:),Dicp(2,:),Dicp(3,:),'r.');
%   axis equal;
%   xlabel('x'); ylabel('y'); zlabel('z');
%   title('ICP result');
  
  drawnow;
  if pauseLen > 0
         pause(pauseLen);
  end
  hold off;
  
  clf;
end

function plot_icp(M,D,Dicp_1,Dicp_2,Dicp_3)
subplot(1,4,1);
plot3(M(1,:),M(2,:),M(3,:),'b.',D(1,:),D(2,:),D(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Original features');

% Plot the results
subplot(1,4,2);
plot3(M(1,:),M(2,:),M(3,:),'b.',Dicp_1(1,:),Dicp_1(2,:),Dicp_1(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Closed Form');

subplot(1,4,3);
plot3(M(1,:),M(2,:),M(3,:),'b.',Dicp_2(1,:),Dicp_2(2,:),Dicp_2(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Brute Force');

subplot(1,4,4);
plot3(M(1,:),M(2,:),M(3,:),'b.',Dicp_3(1,:),Dicp_3(2,:),Dicp_3(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('KD Tree');
end

function plot_landmark(z)
  global Table
  subplot(1,2,1);
  scatter3(Table.hash_table.depth_loc(1, :), -Table.hash_table.depth_loc(3, :), Table.hash_table.depth_loc(2, :), 'r', 'filled'); hold on;
  title('Features in object frame');
  subplot(1,2,2);
  scatter3(z(:,1),z(:,2),z(:,3),'b','filled');
  title('Observations in robot frame');
end

function [TR, TT] = icp_std(p,q)
mu_x = mean(p,2);
mu_y = mean(q,2);

pt = p - mu_x;
qt = q - mu_y;

W = pt*qt';
[U, S, V] = svd(W);
TR = U*V';
TT = mu_x - TR * mu_y;
end


