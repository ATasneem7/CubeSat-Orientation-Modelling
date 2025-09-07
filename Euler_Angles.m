%% EULER_ANGLES Capturing the Rotational Dynamics of a Tumbling Satellite in Terms of Euler Angles 

function s_dot = Euler_Angles(t, s, I)
theta = s(1:3);             % Vector containing the Euler Angles
omega = s(4:6);             % Vector Containing the Angular Velocities
M = [0;0;0];                % No Moment Applied to the Satellite
% Coefficient Matrix of the Kinematic Equations
Kinematics = [1           0                  -sin(theta(2));
              0     cos(theta(1))     cos(theta(2))*sin(theta(1));         
              0    -sin(theta(1))     cos(theta(2))*cos(theta(1))];
theta_dot = inv(Kinematics)*omega;                     % Kinematics Equations
omega_dot = inv(I)*(M-cross(omega, I*omega));           % Rotational EoMs
s_dot = [theta_dot; omega_dot];
end