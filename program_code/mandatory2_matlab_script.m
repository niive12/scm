%% Load simple statistics and data from pre-imported file
clear all;
load('imported_statistics005-3.5.mat');

%% Plot Path Execution Time vs. extend
figure(1);
plot(data_extend,data_path_exec_time,'x');
grid on;
hold on;
plot(extend,path_time_median,'r-');
xlabel('\epsilon (rad)');
ylabel('Path execution time (s)');
hold off;
legend('raw data','median');

%% Plot Jointspace Traveled Distance vs. extend
figure(2);
plot(data_extend,data_robot_distance_jointspace,'x');
grid on;
hold on;
plot(extend,jointspace_distance_median,'r-');
xlabel('\epsilon (rad)');
ylabel('Jointspace traveled distance');
hold off;
legend('raw data','median');

%% Plot Gripper Traveled Distance vs. extend
figure(3);
plot(data_extend,data_gripper_distance_cart,'x');
grid on;
hold on;
plot(extend,gripper_distance_median,'r-');
xlabel('\epsilon (rad)');
ylabel('Gripper traveled distance (m)');
hold off;
legend('raw data','median');

%% Plot RRT Execution Time vs. extend
figure(4);
plot(data_extend,data_RRT_execution_time/1000,'x');
grid on;
hold on;
plot(extend,rrt_exec_time_median/1000,'r-');
ylim([0,0.75]);
xlabel('\epsilon (rad)');
ylabel('RRT execution time (s)');
hold off;
legend('raw data','median');

%% Plot Number of States vs. extend
figure(5);
plot(data_extend,data_number_of_states,'x');
grid on;
hold on;
plot(extend,number_of_states_median,'r-');
xlabel('\epsilon (rad)');
ylabel('Number of states in path');
ylim([0 100]);
hold off;
legend('raw data','median');

