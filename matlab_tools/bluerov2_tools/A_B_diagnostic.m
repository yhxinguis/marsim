%% BlueROV2 A and B Matrix Diagnostic
% This script performs a diagnostic analysis on the A and B matrices of the BlueROV2 
% state-space model, as well as the Q and R matrices used for LQR control design.
%
% The script performs the following main tasks:
% 1. Checks and displays the dimensions of A, B, Q, and R matrices
% 2. Verifies compatibility of matrix dimensions
% 3. Displays sample elements from each matrix
% 4. Attempts to calculate the LQR gain
% 5. Checks system controllability
% 6. Examines eigenvalues of the A matrix
%
% Inputs:
% - A: 12x12 state matrix (should be in workspace)
% - B: 12x6 input matrix (should be in workspace)
% - Q and R matrices are defined within the script
%
% Outputs:
% - Printed diagnostic information about matrix dimensions and properties
% - Controllability analysis results
% - Eigenvalue analysis of A matrix
% - LQR calculation attempt results
%
% Key components:
% - Matrix dimension checks
% - Controllability analysis
% - Eigenvalue analysis
% - LQR calculation attempt
%
% Usage:
% Ensure A and B matrices are defined in the workspace before running this script.
% Run the script to perform diagnostic checks and display results.
%
% Note:
% - This script is intended for debugging and verification purposes
% - It assumes a 12-state model for the BlueROV2 with 6 control inputs
% - The script defines Q and R matrices internally for LQR testing

% Display dimensions of A and B
disp('Dimensions of A:');
disp(size(A));

disp('Dimensions of B:');
disp(size(B));

% Define Q and R matrices as provided
Q = diag([10, 10, 10, ... % x, y, z position
          0, 0, 5, ... % roll, pitch, yaw
          1, 1, 1, ... % vx, vy, vz
          0, 0, 0.1]); % wx, wy, wz
R = 0.1 * eye(6); % Assuming 6 thrusters

disp('Dimensions of Q:');
disp(size(Q));

disp('Dimensions of R:');
disp(size(R));

% Check if dimensions are compatible
if size(A,1) ~= size(Q,1)
    disp('Warning: Dimensions of A and Q do not match');
end

if size(B,2) ~= size(R,1)
    disp('Warning: Dimensions of B and R do not match');
end

% Display first few elements of each matrix for sanity check
disp('First few elements of A:');
disp(A(1:min(5,size(A,1)), 1:min(5,size(A,2))));

disp('First few elements of B:');
disp(B(1:min(5,size(B,1)), 1:min(5,size(B,2))));

disp('First few elements of Q:');
disp(Q(1:min(5,size(Q,1)), 1:min(5,size(Q,2))));

disp('First few elements of R:');
disp(R(1:min(5,size(R,1)), 1:min(5,size(R,2))));

% Try to calculate LQR gain
try
    K = lqr(A, B, Q, R);
    disp('LQR calculation successful');
    disp('Dimensions of K:');
    disp(size(K));
catch ME
    disp('Error in LQR calculation:');
    disp(ME.message);
end

% Check controllability
co = ctrb(A, B);
controllability_rank = rank(co);
system_rank = size(A, 1);

disp(['Controllability matrix rank: ', num2str(controllability_rank)]);
disp(['System rank: ', num2str(system_rank)]);

if controllability_rank == system_rank
    disp('The system is controllable');
else
    disp('The system is not fully controllable');
end

% Check for imaginary eigenvalues
eig_A = eig(A);
if any(imag(eig_A) ~= 0)
    disp('A has imaginary eigenvalues:');
    disp(eig_A);
else
    disp('A does not have imaginary eigenvalues');
end
