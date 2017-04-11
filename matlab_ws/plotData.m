function plotData(data_path)
global Data;
global gt_Data;
%%%read the data
readData(data_path);

%%%read the groundtruth data
readGroundTruthData(data_path);

% Trajectory of robot
robot_traj = [];
obj_traj = [];
for t=1:size(Data.base_transpose_Matrix, 1)
    temp = cell2mat(Data.base_transpose_Matrix(t))*[0; 0; 0; 1];
    robot_traj(:, end+1)= temp(1:3, 1);
    temp2 = cell2mat(Data.base_transpose_Matrix(t))*[gt_Data(1:3, t); 1];
    obj_traj(:, end+1) = temp2;
end
figure(1);
plot(robot_traj(1, :), robot_traj(2, :), 'r'); hold on;
plot(robot_traj(1, :), robot_traj(2, :), 'r*'); hold on;
plot(obj_traj(1, :), obj_traj(2, :), 'b*'); hold on;
for i=1:size(robot_traj, 2)
    text(robot_traj(1, i), robot_traj(2, i)+0.01, num2str(i));  
    line([robot_traj(1, i), obj_traj(1, i)], [robot_traj(2, i), obj_traj(2, i)]);
end
axis equal;
figure(2);
plot3(obj_traj(1, :), obj_traj(2, :), obj_traj(3, :), 'r'); hold on
for i=1:size(robot_traj, 2)
    text(obj_traj(1, i), obj_traj(2, i), obj_traj(3, i)+0.001, num2str(i));      
end
scatter3(obj_traj(1, :), obj_traj(2, :), obj_traj(3, :), 'filled'); 
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

% obj_traj=zeros(3, size(gt_Data, 2));
% robot_traj=zeros(3, size(gt_Data, 2));
% 
% robot_traj(:, 1) = [0, 0, 0]';
% obj_traj(:, 1) = gt_Data(1:3, 1)+robot_traj(:, 1);
% for i=2:size(gt_Data, 2)
%     robot_traj(:, i) = robot_traj(:, i-1)+U(:, i-1);
%     obj_traj(:, i) = gt_Data(1:3, i)+robot_traj(:, i);    
% end
% 
% %% plot the data 
% figure(1);
% 
% %x,y plot
% plot(robot_traj(1, :), robot_traj(2, :), 'r*'); hold on;
% plot(obj_traj(2, :), obj_traj(1, :), 'b*');
% for i=1:size(robot_traj, 2)
%     text(robot_traj(1, i), robot_traj(2, i), num2str(i));
%     text(obj_traj(2, i), obj_traj(1, i), num2str(i));
%     line([robot_traj(1, i), obj_traj(2, i)], [robot_traj(2, i), obj_traj(1, i)]);
% end
% axis equal;

end