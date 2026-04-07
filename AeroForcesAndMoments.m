function [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
%
% 
% aircraft_state = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]
%   NOTE: The function assumes the veolcity is the air relative velocity
%   vector. When used with simulink the wrapper function makes the
%   conversion.
%
% aircraft_surfaces = [de da dr dt];
%
% Lift and Drag are calculated in Wind Frame then rotated to body frame
% Thrust is given in Body Frame
% Sideforce calculated in Body Frame


%%% redefine states and inputs for ease of use
ap = aircraft_parameters;

State.x=aircraft_state(1); State.y=aircraft_state(2); State.z=aircraft_state(3);             % Position
State.phi=aircraft_state(4); State.theta=aircraft_state(5); State.psi=aircraft_state(6);     % Attitude
State.u=aircraft_state(7); State.v=aircraft_state(8); State.w=aircraft_state(9);             % Velocity
State.p=aircraft_state(10); State.q=aircraft_state(11); State.r=aircraft_state(12); 


Trig.cpsi = cos(State.psi); Trig.spsi = sin(State.psi);
Trig.tpsi = tan(State.psi);

% Calculate Trig values of Theta
Trig.ctheta = cos(State.theta); Trig.stheta = sin(State.theta);
Trig.ttheta = tan(State.theta);

% Calculate Trig values of Phi
Trig.cphi = cos(State.phi); Trig.sphi = sin(State.phi);
Trig.tphi = tan(State.phi);


wind_body = [Trig.ctheta.*Trig.cpsi, (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi), (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi); ...
                     Trig.ctheta.*Trig.spsi, (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi), (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi); ...
                     -Trig.stheta, Trig.ctheta.*Trig.sphi, Trig.ctheta.*Trig.cphi] *wind_inertial;
air_rel_vel_body = aircraft_state(7:9,1) - wind_body;

[wind_angles] = WindAnglesFromVelocityBody(air_rel_vel_body);
V = wind_angles(1,1);
beta = wind_angles(2,1);
alpha = wind_angles(3,1);

p = aircraft_state(10,1);
q = aircraft_state(11,1);
r = aircraft_state(12,1);

de = aircraft_surfaces(1,1);
da = aircraft_surfaces(2,1);
dr = aircraft_surfaces(3,1);
dt = aircraft_surfaces(4,1);

alpha_dot = 0;

%Q = ap.qbar;
Q = 0.5*density*V*V;

sa = sin(alpha);
ca = cos(alpha);

%%% determine aero force coefficients
CL = ap.CL0 + ap.CLalpha*alpha + ap.CLq*q*ap.c/(2*V) + ap.CLde*de;
%CD = ap.CD0 + ap.CDalpha*alpha + ap.CDq*q*ap.c/(2*V) + ap.CDde*de;
CD = ap.CDpa + ap.K*CL*CL;

CX = -CD*ca + CL*sa;
CZ = -CD*sa - CL*ca;

CY = ap.CY0 + ap.CYbeta*beta + ap.CYp*p*ap.b/(2*V) + ap.CYr*r*ap.b/(2*V) + ap.CYda*da + ap.CYdr*dr;

%%Thrust = .5*density*ap.Sprop*ap.Cprop*((ap.kmotor*dt)^2 - V^2);
Thrust = density*ap.Sprop*ap.Cprop*(V + dt*(ap.kmotor - V))*dt*(ap.kmotor-V); %%Changed 5/2/15;New model as described in http://uavbook.byu.edu/lib/exe/fetch.php?media=shared:propeller_model.pdf


%%% determine aero forces from coeffficients 
X = Q*ap.S*CX + Thrust;
Y = Q*ap.S*CY;
Z = Q*ap.S*CZ;

aero_forces = [X;Y;Z];

%%% determine aero moment coefficients
Cl = ap.b*[ap.Cl0 + ap.Clbeta*beta + ap.Clp*p*ap.b/(2*V) + ap.Clr*r*ap.b/(2*V) + ap.Clda*da + ap.Cldr*dr];
Cm = ap.c*[ap.Cm0 + ap.Cmalpha*alpha + ap.Cmq*q*ap.c/(2*V) + ap.Cmde*de];
Cn = ap.b*[ap.Cn0 + ap.Cnbeta*beta + ap.Cnp*p*ap.b/(2*V) + ap.Cnr*r*ap.b/(2*V) + ap.Cnda*da + ap.Cndr*dr];

%%% determine aero moments from coeffficients
aero_moments = Q*ap.S*[Cl; Cm; Cn];%[l;m;n];

