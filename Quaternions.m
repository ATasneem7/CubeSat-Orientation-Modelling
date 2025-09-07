%% QUATERNIONS Capturing the Rotational Dynamics of a Tumbling Satellite in Terms of Quaternions

function s_dot = Quaternions(t, s, I)
q = s(1:4);                   % Quaternion Vector, containing the Euler Parameters
omega = s(5:7);               % Vector Containing the Angular Velocities
% Coefficient Matrix for Kinematics of Euler Parameters
Quat_Kinematics = [ 0         -omega(1)   -omega(2)  -omega(3);
                   omega(1)        0       omega(3)  -omega(2);
                   omega(2)   -omega(3)       0       omega(1);       
                   omega(3)    omega(2)   -omega(1)       0];
q_dot = 0.5*Quat_Kinematics*q;           % Kinematic Equations for Euler Parameters
omega_dot = inv(I)*(-cross(omega, I*omega));           % Rotational EoMs
s_dot = [q_dot; omega_dot];
end