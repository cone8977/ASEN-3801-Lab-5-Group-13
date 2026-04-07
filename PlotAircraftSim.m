% Contributors: Liz Thompson
% Course number: ASEN 3801
% File name: PlotAircraftSim
% Created: 3/2/26

function PlotAircraftSim(time,aircraft_state_array,control_input_array,fig, col,Task)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = Time corresponding to the nth set of variables
%           aircraft_state_array = 12 x n array of aircraft states
%           control_input_array = 4 x n array of control inputs (Zc,Lc,Mc,Nc)
%           fig = 6x1 vector of figure numbers to plot over
%           col = string indicating the plotting option for each plot
%           Task = what Task and part for figure labeling                 
% Outputs:  NO NUMERICAL. Only plots the following 6 figures:
%               - Four figures each with 3 subplots for intertial position,
%               euler angles, intertial velocity, and angular velocity
%               - One figure with subplots for each control input variable
%               - One figure that shows the 3D path for the aircraft, using
%               up as positive, and marking the start (green) and end (red)
%
% Methodology: Using inputted data from a simulation run, plot the aircraft
% states, control inputs, and 3D location over time, WITHOUT clearing the
% figures before plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assign Useful Names to Variables (x,y,z,phi,theta,etc...)

x = aircraft_state_array(:,1); y = aircraft_state_array(:,2); 
z = aircraft_state_array(:,3); phi = aircraft_state_array(:,4);
theta = aircraft_state_array(:,5); psi = aircraft_state_array(:,6);
u = aircraft_state_array(:,7); v = aircraft_state_array(:,8);
w = aircraft_state_array(:,9); p = aircraft_state_array(:,10);
q = aircraft_state_array(:,11); r = aircraft_state_array(:,12);
Zc = control_input_array(:,1); Lc = control_input_array(:,2);
Mc = control_input_array(:,3); Nc = control_input_array(:,4);

%% Plotting A/C State Vector Components (4 Figs w/Subplots)

figure(fig(1)); % Figure 1 for A/C Inertial Positions
hold on; grid on;
sgtitle('Inertial Position Components vs Time')
subplot(3,1,1); plot(time,x,col(1),LineWidth=1); grid on;
ylabel('X-Position (m)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,y,col(2),LineWidth=1); grid on;
ylabel('Y-Position (m)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,z,col(3),LineWidth=1); grid on;
ylabel('Z-Position (m)'); xlim([0,max(time)]); xlabel('Time (s)');
print(sprintf('ASEN3801_L4_%sa',Task),'-dpng','-r300')
figure(fig(2)); % Figure 2 for A/C Euler Angles
hold on; grid on;
sgtitle('3-2-1 Body Euler Angles vs Time')
subplot(3,1,1); plot(time,phi,col(4),LineWidth=1); grid on;
ylabel('Roll Angle (rad)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,theta,col(5),LineWidth=1); grid on;
ylabel('Pitch Angle (rad)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,psi,col(6),LineWidth=1); grid on;
ylabel('Yaw Angle (rad)'); xlim([0,max(time)]); xlabel('Time (s)');
print(sprintf('ASEN3801_L4_%sb',Task),'-dpng','-r300')
figure(fig(3)); % Figure 3 for A/C Air Relative Velocity
hold on; grid on;
sgtitle('Air-Relative Velocity Components')
subplot(3,1,1); plot(time,u,col(1),LineWidth=1); grid on;
ylabel('X-Velocity (m/s)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,v,col(2),LineWidth=1); grid on;
ylabel('Y-Velocity (m/s)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,w,col(3),LineWidth=1); grid on;
ylabel('Z-Velocity (m/s)'); xlim([0,max(time)]); xlabel('Time (s)');
print(sprintf('ASEN3801_L4_%sc',Task),'-dpng','-r300')
figure(fig(4)); % Figure 4 for A/C Angular Rates
hold on; grid on;
sgtitle('3-2-1 Body Angular Rates')
subplot(3,1,1); plot(time,p,col(4),LineWidth=1); grid on;
ylabel('Roll Rate (rad/s)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,q,col(5),LineWidth=1);  grid on;
ylabel('Pitch Rate (rad/s)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,r,col(6),LineWidth=1); grid on;
ylabel('Yaw Rate (rad/s)'); xlim([0,max(time)]); xlabel('Time (s)');
print(sprintf('ASEN3801_L4_%sd',Task),'-dpng','-r300')
%% Plotting Control Inputs

figure(fig(5)); % Figure 5 for A/C Control Inputs
hold on;
sgtitle('Control Inputs')
subplot(4,1,1); plot(time,Zc,col(3),LineWidth=1); 
ylabel('Z-Force'); xlim([0,max(time)]); grid on;
subplot(4,1,2); plot(time,Lc,col(4),LineWidth=1); 
ylabel('X-Moment'); xlim([0,max(time)]); grid on;
subplot(4,1,3); plot(time,Mc,col(5),LineWidth=1); 
ylabel('Y-Moment'); xlim([0,max(time)]); grid on;
subplot(4,1,4); plot(time,Nc,col(6),LineWidth=1); 
ylabel('Z-Moment'); xlim([0,max(time)]); xlabel('Time (s)'); grid on;
print(sprintf('ASEN3801_L4_%se',Task),'-dpng','-r300')
%% Plotting 3D Trajectory 

figure(fig(6)); % Figure 6 for A/C 3D Trajectory
hold on; grid on; view(3);
title('3D Trajectory')
plot3(x,y,-z,'k',LineWidth=1); % 3D Trajectory
plot3(x(1),y(1),-z(1),'go'); plot3(x(end),y(end),-z(end),'ro') % Markers
xlabel('X-Positon (m)'); ylabel('Y-Position (m)'); zlabel('Altitude (m)')
print(sprintf('ASEN3801_L4_%sf',Task),'-dpng','-r300')
end