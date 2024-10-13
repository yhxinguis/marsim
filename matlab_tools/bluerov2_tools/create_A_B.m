%% BlueROV2 State Space Matrices Calculation
% This script calculates the A and B matrices for a state-space model of the BlueROV2 underwater vehicle.
%
% The script generates constant coefficient matrices for a 12-state model:
% States: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
% Inputs: 6 thruster forces [T1, T2, T3, T4, T5, T6]
%
% Key features:
% - Assumes a constant coefficient model suitable for small deviations from equilibrium
% - Includes kinematic relationships in the A matrix
% - B matrix accounts for thruster positions and orientations
%
% Inputs:
% - No direct inputs; parameters are defined within the script
%
% Outputs:
% - A: 12x12 state matrix
% - B: 12x6 input matrix
%
% Usage:
% Run the script to generate and display the A and B matrices.
% The matrices can be used for further control system design and analysis.
%
% Note:
% - The model assumes a body-fixed coordinate system
% - Thruster positions and directions are defined relative to the center of gravity
% - The script uses predefined constants for mass, moments of inertia, and thruster configurations

clear all;

% ROV Parameters
mass = 10.0;  % kg
volume = 0.01;  % m^3 (assuming slightly positively buoyant)
rho_water = 1000;  % kg/m^3 (freshwater density)
Ixx = 0.099;  % kg*m^2
Iyy = 0.178;  % kg*m^2
Izz = 0.269;  % kg*m^2

% Initialize matrices
A = zeros(12, 12);
B = zeros(12, 6);

%% A matrix calculation

% Position rates (rows 1-3)
A(1:3, 7:9) = eye(3);

% Attitude rates (rows 4-6)
A(4:6, 10:12) = eye(3);

% Note: All other elements in A remain zero

%% B matrix calculation

% COG position
cog = [0; 0; 0.011];

% Thruster positions (relative to origin)
t1 = [0.14; -0.092; 0.0];
t2 = [0.14; 0.092; 0.0];
t3 = [-0.15; -0.092; 0.0];
t4 = [-0.15; 0.092; 0.0];
t5 = [0.0; -0.109; 0.077];
t6 = [0.0; 0.109; 0.077];

% Thruster directions (unit vectors)
d1 = [cosd(-45); sind(-45); 0];
d2 = [cosd(45); sind(45); 0];
d3 = [cosd(135); sind(135); 0];
d4 = [cosd(225); sind(225); 0];
d5 = [0; 0; -1];
d6 = [0; 0; -1];

% Force effects (rows 7-9)
B(7:9, 1) = d1 / mass;
B(7:9, 2) = d2 / mass;
B(7:9, 3) = d3 / mass;
B(7:9, 4) = d4 / mass;
B(7:9, 5) = d5 / mass;
B(7:9, 6) = d6 / mass;

% Moment effects (rows 10-12)
B(10:12, 1) = cross(t1 - cog, d1) ./ [Ixx; Iyy; Izz];
B(10:12, 2) = cross(t2 - cog, d2) ./ [Ixx; Iyy; Izz];
B(10:12, 3) = cross(t3 - cog, d3) ./ [Ixx; Iyy; Izz];
B(10:12, 4) = cross(t4 - cog, d4) ./ [Ixx; Iyy; Izz];
B(10:12, 5) = cross(t5 - cog, d5) ./ [Ixx; Iyy; Izz];
B(10:12, 6) = cross(t6 - cog, d6) ./ [Ixx; Iyy; Izz];

% Display results
disp('A matrix:');
disp(A);
disp('B matrix:');
disp(B);



