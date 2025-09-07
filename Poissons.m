%% POISSONS Capturing the Rotational Dynamics of a Tumbling Satellite in Terms of DCM 

function s_dot = Poissons(t, s, I)
C = s(1:9);                  % Vector containing DCM Elements
omega = s(10:12);            % Vector Containing the Angular Velocities
omega_skew = [    0        omega(3)   -omega(2);
              -omega(3)       0        omega(1);      
               omega(2)   -omega(1)        0];
Poissons_Kinematics = reshape(C, 3,3);
C_dot = omega_skew * Poissons_Kinematics;           % Poisson's Kinematic Equations
omega_dot = inv(I)*(-cross(omega, I*omega));        % Rotational EoMs
s_dot = [reshape(C_dot,9,1); omega_dot];
end