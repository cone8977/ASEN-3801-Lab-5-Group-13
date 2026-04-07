% Contributors: Tyler Emmons, Wang Long Lee, Cody Newton, Carson Sutton 
% Course number: ASEN 3801
% File name: PlotAircraftSim
% Created: 4/7/26

clc;
clear;
close all;

% Aircraft Parameter structure 
a_params=ttwistor();
t=[0 20];
aircraft_surfaces=[0;0;0;0];
wind_inertial=[0;0;0];
rad=pi./180;
Adjust=[1./rad,1./rad,1./rad,1];
Image=0;
%% Task 2 
%Problem 1

x=[0 ; 0 ; -1609.34
    0 ; 0 ; 0
    21 ; 0 ; 0
    0 ; 0 ; 0];

func=@ (t,aircraft_state)AircraftEOM(t, aircraft_state, aircraft_surfaces, wind_inertial, a_params);

[T,y] = ode45(func,t,x);

Controls=ones(length(T),4);
Controls=Controls.*aircraft_surfaces';
PlotAircraftSim(T,y,Controls,Image+1:Image+6, ['b' 'b' 'b' 'b' 'b' 'b'],"21")
Image=Image+6;

% Problem 2 

aircraft_surfaces=[0.1079;0;0;0.3182];
x=[0 ; 0 ; -1800
    0 ; 0.0278 ; 0
    20.99 ; 0 ; 0.5837
    0 ; 0 ; 0];

func=@ (t,aircraft_state)AircraftEOM(t, aircraft_state, aircraft_surfaces, wind_inertial, a_params);

[T,y] = ode45(func,t,x);

Controls=ones(length(T),4);
Controls=Controls.*aircraft_surfaces';
PlotAircraftSim(T,y,Controls.*Adjust,Image+1:Image+6, ['b' 'b' 'b' 'b' 'b' 'b'],"22")
Image=Image+6;

% Problem 3 

aircraft_surfaces=[5*rad;2*rad;-13*rad;0.3];
x=[0 ; 0 ; -1800
    15*rad ; -12*rad ; 270*rad
    19 ; 3 ; -2
    0.08*rad ; -0.2*rad ; 0];

func=@ (t,aircraft_state)AircraftEOM(t, aircraft_state, aircraft_surfaces, wind_inertial, a_params);

[T,y] = ode45(func,t,x);

Controls=ones(length(T),4);
Controls=Controls.*aircraft_surfaces';

PlotAircraftSim(T,y,Controls.*Adjust,Image+1:Image+6, ['b' 'b' 'b' 'b' 'b' 'b'],"23")
Image=Image+6;

%% Problem 3 

aircraft_surfaces=[0.1079;0;0;0.3182];
x=[0 ; 0 ; -1800
    0 ; 0.0278 ; 0
    20.99 ; 0 ; 0.5837
    0 ; 0 ; 0];
doublet_size=15*rad;
doublet_time=0.25;
func=@ (t,aircraft_state)AircraftEOMDoublet(t, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, a_params);

[T,y] = ode45(func,t,x);

Controls=ones(length(T),4);
Controls=Controls.*aircraft_surfaces';

PlotAircraftSim(T,y,Controls.*Adjust, Image+1:Image+6, ['b' 'b' 'b' 'b' 'b' 'b'],"23")
Image=Image+6;