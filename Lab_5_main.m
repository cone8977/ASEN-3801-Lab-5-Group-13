% Contributors: Tyler Emmons, Wang Long Lee, Cody Newton, Carson Sutton 
% Course number: ASEN 3801
% File name: PlotAircraftSim
% Created: 4/7/26

clc;
clear;
close all;

% Aircraft Parameter structure 
a_params=ttwistor();


Velocity=1; %[m/s]
[wind_angles] = WindAnglesFromVelocityBody(Velocity);



