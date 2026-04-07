function [xdot,Controls] = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size,doublet_time, wind_inertial, aircraft_parameters)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = simulation time
%           Aircraft_state = 12x1 state vector [x,y,z,phi,theta,psi,u,v,w,p,q,r] 
%           Wind_inertial = 3x1 wind velocity vector  
%           Aircraft surfaces = 4x1 array of motor forces [Elevator
%           (Deg),Alerion (Deg),Rudder (Deg),Throttle (0-1)]
%           Aircraft_parameters: Aircraft Parameter Struct
% 
% Output:   xdot: 12x1 derivative of the state vector
%
% Methodology: Using derivations from class, calculate the dotted state
% vaiables given values for the previous step and control forces and
% moments. This function will be used by ode45 to simulate the QR flight. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assign Useful Names to aircraft_stateiables (x,y,z,phi,theta,etc...)

State.x=aircraft_state(1); State.y=aircraft_state(2); State.z=aircraft_state(3);             % Position
State.phi=aircraft_state(4); State.theta=aircraft_state(5); State.psi=aircraft_state(6);     % Attitude
State.u=aircraft_state(7); State.v=aircraft_state(8); State.w=aircraft_state(9);             % Velocity
State.p=aircraft_state(10); State.q=aircraft_state(11); State.r=aircraft_state(12);          % Angular Rate
density=stdatmo(State.z);
Gamma =gammaCalc(aircraft_parameters);
g=aircraft_parameters.g;
m=aircraft_parameters.m;
%% Set Up Euler Angle Structure 

% Calculate Trig values of Psi
Trig.cpsi = cos(State.psi); Trig.spsi = sin(State.psi);
Trig.tpsi = tan(State.psi);

% Calculate Trig values of Theta
Trig.ctheta = cos(State.theta); Trig.stheta = sin(State.theta);
Trig.ttheta = tan(State.theta);

% Calculate Trig values of Phi
Trig.cphi = cos(State.phi); Trig.sphi = sin(State.phi);
Trig.tphi = tan(State.phi);

%% Extracting Inertia Values

In.x = aircraft_parameters.Ix; In.y = aircraft_parameters.Iy; In.z = aircraft_parameters.Iz;
In.xy = aircraft_parameters.Ixz;

%% Calcaulte Control Force/Moments
if time<=doublet_time
aircraft_surfaces(1)=aircraft_surfaces(1)+doublet_size;
elseif doublet_time < time<=(2.*doublet_time)
aircraft_surfaces(1)=aircraft_surfaces(1)- 2.*doublet_size;
else
aircraft_surfaces(1)=aircraft_surfaces(1)+doublet_size;
end

[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
Controls=[];

%% X_dot, Y_dot, Z_dot
Pos_dot=[Trig.ctheta.*Trig.cpsi, (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi), (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi); ...
                     Trig.ctheta.*Trig.spsi, (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi), (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi); ...
                     -Trig.stheta, Trig.ctheta.*Trig.sphi, Trig.ctheta.*Trig.cphi] * [State.u;State.v;State.w];

X_dot = Pos_dot(1);
Y_dot = Pos_dot(2);
Z_dot = Pos_dot(3);


%% Phi_dot, Theta_dot, Psi_dot
Angle_dot=[ 1, Trig.sphi.*Trig.ttheta, Trig.cphi.*Trig.ttheta;
            0, Trig.cphi, -Trig.sphi
            0, Trig.sphi./Trig.ctheta, Trig.cphi./Trig.ctheta]*[State.p;State.q;State.r;]; 


Phi_dot = Angle_dot(1);
Theta_dot = Angle_dot(2);
Psi_dot = Angle_dot(3);


%% U_dot, V_dot, W_dot

v_dot= cross([State.u,State.v,State.w],[State.p,State.q,State.r])' + g.*[-Trig.stheta;Trig.ctheta.*Trig.sphi ;Trig.ctheta.*Trig.cphi] +aero_forces./m;

U_dot = v_dot(1);
V_dot = v_dot(2);
W_dot = v_dot(3);


%% P_dot, Q_dot, R_dot

P_dot = Gamma.one.*State.p.*State.q -Gamma.two.*State.q.*State.r +Gamma.three.*aero_moments(1) +Gamma.four.*aero_moments(3);
Q_dot = Gamma.five.*State.p.*State.r -Gamma.six.*(State.p.^2-State.r.^2) + 1./In.y .*aero_moments(2);
R_dot = Gamma.seven .*State.p.*State.q - Gamma.one.*State.q.*State.r +Gamma.four .*aero_moments(1) + Gamma.eight.*aero_moments(3);

%% Compilation
xdot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];

end