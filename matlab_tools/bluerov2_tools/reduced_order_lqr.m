%% BlueROV2 Reduced-Order LQR Controller
% This script designs a Linear Quadratic Regulator (LQR) controller for the BlueROV2 
% underwater vehicle using a reduced-order model based on the controllable subspace.
%
% The script performs the following main tasks:
% 1. Computes the controllable subspace of the system
% 2. Creates a reduced-order model
% 3. Designs an LQR controller for the reduced-order model
% 4. Transforms the controller gain back to the full-state space
% 5. Simulates and visualizes the closed-loop system response
%
% Inputs:
% - A: 12x12 state matrix (from create_A_B.m)
% - B: 12x6 input matrix (from create_A_B.m)
% - Q: 12x12 state weighting matrix for LQR cost function
% - R: 6x6 input weighting matrix for LQR cost function
%
% Outputs:
% - K: 6x12 LQR gain matrix for the full-state system
% - Plots of system response (position, orientation, velocities, control inputs)
% - Closed-loop system eigenvalues
%
% Key components:
% - Controllability analysis
% - Model order reduction
% - LQR controller design
% - Closed-loop system simulation
% - Stability analysis
%
% Usage:
% Ensure A, B, Q, and R matrices are defined in the workspace before running this script.
% Run the script to design the controller, simulate the system, and visualize the results.
%
% Note:
% - The script assumes a 12-state model for the BlueROV2
% - The reduced-order model is based on the controllable subspace of the system
% - Simulation uses a unit step input to all control channels

% Compute controllability matrix
co = ctrb(A, B);

% Compute rank and get the controllable subspace
r = rank(co);
Vc = orth(co);  % Orthonormal basis for controllable subspace

% Create reduced-order model
Ar = Vc' * A * Vc;
Br = Vc' * B;

% Create reduced Q and R matrices
Qr = Vc' * Q * Vc;
R = 0.1 * eye(6);  % Keep R the same

% Compute LQR gain for reduced system
Kr = lqr(Ar, Br, Qr, R);

% Transform gain back to original coordinates
K = Kr * Vc';

% Display results
disp('Reduced A matrix:');
disp(Ar);

disp('Reduced B matrix:');
disp(Br);

disp('LQR gain matrix K:');
disp(K);

% Simulate closed-loop system
sys_cl = ss(A - B*K, B, eye(12), zeros(12,6));

% Step response
t = 0:0.1:10;
[y, t, x] = step(sys_cl, t);

% Plot results
figure;
subplot(3,2,1);
plot(t, y(:,1:3));
title('Position');
legend('x', 'y', 'z');

subplot(3,2,2);
plot(t, y(:,4:6));
title('Orientation');
legend('roll', 'pitch', 'yaw');

subplot(3,2,3);
plot(t, y(:,7:9));
title('Linear Velocity');
legend('vx', 'vy', 'vz');

subplot(3,2,4);
plot(t, y(:,10:12));
title('Angular Velocity');
legend('wx', 'wy', 'wz');

subplot(3,2,5:6);
plot(t, x(:,1:6));
title('Control Inputs');
legend('T1', 'T2', 'T3', 'T4', 'T5', 'T6');

% Check closed-loop stability
eig_cl = eig(A - B*K);
disp('Closed-loop eigenvalues:');
disp(eig_cl);
