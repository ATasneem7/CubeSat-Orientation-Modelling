%% Defining the Geometry & the Inertial Properties of the CubeSat

% Geometric Properties 
l = 0.35;                       % length of the CubeSat in (m) along X-axis
w = 0.15;                       % width of the CubeSat in (m) along Y-axis
t = 0.10;                       % height of the CubeSat in (m) along Z-axis

% Inertial Properties
m = 12;                        % Mass of the CubeSat in (kg)
i_xx = (w^2 + t^2)*m/12;       % MOI about the x-axis passing through the CoM
i_yy = (l^2 + t^2)*m/12;       % MOI about the y-axis passing through the CoM
i_zz = (w^2 + l^2)*m/12;       % MOI about the z-axis passing through the CoM
I = [i_xx 0 0;...
     0 i_yy 0;...
     0 0 i_zz];                 % MOI Matrix about the CoM           
%% Creating the Geometry of the CubeSat in 3D Space

% Setting up the Properties of the Inertial Reference Frame 
axis equal
xlabel("X-axis"); ylabel("Y-axis"); zlabel("Z-axis");
xlim([-55 55]);
ylim([-55 60]);
zlim([-55 55]);
xticks(-55:20:55);
yticks(-55:20:60);
zticks(-55:20:55);
grid on
title('CubeSat Geometry with Attached Body Frame',....
      'Color',"k", 'FontSize', 11, 'FontWeight','bold')

% Setting the View Angle of the 3D Plot
AZ = 130;
EL = 10;
view(AZ,EL);
hold on

% Creating Body Frame Orthogonal Axis System
origin = [0 0 0];                  % Body Axis System passes through the CoM
xaxis = 100*[0.5,0,0];             % Coordinates of the x-axis of body frame
yaxis = 100*[0,0.55,0];            % Coordinates of the y-axis of body frame
zaxis = 100*[0,0,0.5];             % Coordinates of the z-axis of body frame

% Draw Body-Attacted Frame x-Axis
h1 = mArrow3(origin,xaxis,'color','#D2FF72', 'stemWidth', 1, 'tipWidth', 2);
hold on
% Draw Body-Attacted Frame y-Axis
h2 = mArrow3(origin,yaxis,'color','#DA498D', 'stemWidth', 1, 'tipWidth', 2);
hold on 
% Draw Body-Attacted Frame z-Axis
h3 = mArrow3(origin,zaxis,'color','#FF1E1E', 'stemWidth', 1, 'tipWidth', 2);
camlight
hold on
%% *Creating the Geometry of the CubeSat with Extended Fins in 3D Space*

% Defining the Coordinates of the CubeSat Body Vertices
v1 = 100*[-l -w -t]/2;       v2 = 100*[l -w -t]/2;
v3 = 100*[l -w t]/2;         v4 = 100*[-l -w t]/2;   
v5 = 100*[-l w -t]/2;        v6 = 100*[l w -t]/2;   
v7 = 100*[l w t]/2;          v8 = 100*[-l w t]/2;

body_v = [v1;v2;v3;v4;v5;v6;v7;v8];   % 8x3 Matrix containing Body Vertices 

% Defining the Vertices of CubeSat Faces 
f = [1 2 3 4;...       % bottom face
     5 6 7 8;...       % top face
     1 2 6 5;...       % right-hand face
     3 4 8 7;...       % left-hand face
     1 4 8 5;...       % front face            
     2 3 7 6];         % back face              

patch('Vertices',body_v,'Faces', f(1:4,:),'FaceColor','#DBDFEA',...
      'EdgeColor','k','lineWidth', 1);
patch('Vertices',body_v,'Faces', f(5:6,:),'FaceColor','#7A73D1',...
      'EdgeColor','k','lineWidth', 1);
hold on

% Fin Dimensions
finL = 0.40;                                % Length of the fins in (m)
finW = 0.1;                                 % Width of the fins in (m)

% Defining the Coordinates of Satellite Fin-1 Vertices  
fin1_v1 = 100*[finW 0 0];         fin1_v2 = 100*[-finW 0 0];
fin1_v3 = 100*[-finW finL 0];     fin1_v4 = 100*[finW finL 0]; 

fin1_v = [fin1_v1; fin1_v2; fin1_v3; fin1_v4];   % 4x3 Fin-1 Vertices Matrix

CubeSat_Fin1 = patch('Vertices', fin1_v, 'Faces', [1 2 3 4],...
               'FaceColor', 'yellow','EdgeColor', 'black');
hold on 

% Defining the Coordinates of Satellite Fin-2 Vertices 
fin2_v1 = 100*[finW 0 0];          fin2_v2 = 100*[-finW 0 0];
fin2_v3 = 100*[-finW -finL 0];     fin2_v4 = 100*[finW -finL 0]; 

fin2_v = [fin2_v1; fin2_v2; fin2_v3; fin2_v4];    % 4x3 Fin-2 Vertices Matrix

CubeSat_Fin2 = patch('Vertices', fin2_v, 'Faces', [1 2 3 4],...
              'FaceColor','yellow','EdgeColor', 'black');
light
hold off
%% *Simulating 3D Orientations of the CubeSat in Terms of Euler Angles*

% Defining CubeSat Initial State Variables
theta0 = [0;0;0];                            % CubeSat Initial Orientations
omega0 = [0.1;15; 0.1];                      % CubeSat Initial Angular Velocity 
s0 = [theta0; omega0];                       % Initial States Vector

% Defining the time window
tspan = 0:0.01:30;                % in secs

% Error Tolerance
tolerance = 1e-9;
options = odeset("RelTol",tolerance, "AbsTol", tolerance);

% Implementation of ODE45 Numerical Solver
[t, s] = ode45(@Euler_Angles, tspan, s0, options, I);

Phi = s(:,1);                         
Theta = s(:, 2);             % Extracting the Euler Angles from the State Vector
Psi = s(:, 3);
%% Visualizing the Angular Velocity Components of the CubeSat over Time 

plot(t, s(:, 4),"Color","#2a9d8f","LineWidth",2.15);
hold on
plot(t, s(:, 5),"Color","#ffb703","LineWidth",2.15);
hold on
plot(t, s(:, 6),"Color","#ff8fab","LineWidth",2.15);
hold off
xlim([0 15])
xticks(0:3:15)
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Variation of CubeSat Angular Velocity with Time",...
      'Color',"w", 'FontSize', 11, 'FontWeight','bold')
legend("Wx","Wy","Wz")
grid on
%% Visualizing the CubeSat Orientation over Time 

plot(t, Phi,"Color", "#ff8fab", "LineWidth", 2.25);
hold on
plot(t, Theta,"Color", "#2a9d8f", "LineWidth", 2.25);
hold on 
plot(t, Psi, "Color", "#ffb703", "LineWidth", 2.25);
xlim([0 5]);
xticks(0:0.5:5);
ylim([-10 70]);
yticks(-20:10:70);
xlabel("Time (s)");
ylabel("Euler Angles (rad)");
title("Variation of CubeSat Orientation with Time")
legend("Roll (Phi)","Pitch (Theta)","Yaw (Psi)")
grid on
hold off
%% Validating the Simulation Results Using the Conservation of Rotational Energy

% Computing Rotational Kinetic Energy at each time step
Energy = zeros(length(t), 1);
for i = 1:length(t)
    omega = s(i, 4:6);
    Energy(i) = 0.5 * omega * I * omega';
end

plot(t, Energy, 'LineWidth', 1.25);
xlabel("Time (s)");
ylabel("Energy (J)");
title("Variation of CubeSat Rotational Kinetic Energy over Time")
grid on
%% Validating the Simulation Results Using the Conservation of Angular Momentum

% Computing Angular Momentum at each time step
H_BodyFrame = zeros(length(t),3);      % Angular Momentum Components in Body Frame 
H_InertialFrame = zeros(length(t),3);  % Angular Momentum Components in Inertial Frame

for i = 1:length(t)
    omega = s(i, 4:6);
    H_BodyFrame(i,:) = (I*omega')';

    C1_Phi = [1 0 0; 0 cos(Phi(i)) -sin(Phi(i)); 0 sin(Phi(i)) cos(Phi(i))];
    C2_Theta = [cos(Theta(i)) 0 sin(Theta(i)); 0 1 0;-sin(Theta(i)) 0 cos(Theta(i))];
    C3_Psi = [cos(Psi(i)) -sin(Psi(i))  0; sin(Psi(i)) cos(Psi(i))  0; 0 0 1]; 
    
    CB2I = C3_Psi * C2_Theta * C1_Phi;      % Body 3-2-1 Direction Cosine Matrix

    H_InertialFrame(i, :) = (CB2I* H_BodyFrame(i,:)')';
end

% Visualizing the Variation of the Angular Momentum Components in the Body Frame
plot(t,H_BodyFrame(:,1), 'LineWidth', 1.65, 'Color', "#ffd166");
hold on
plot(t, H_BodyFrame(:,2), 'LineWidth', 1.65, 'Color', "#83c5be");
hold on
plot(t,H_BodyFrame(:,3), 'LineWidth', 1.65, 'Color', "#b23a48")
hold off
xlim([0 9]);
xticks(0:1:9);
ylim([-2.5 2.5])
xlabel("Time (s)")
ylabel("H-Components (in Body Frame)");
title("Variation of Angular Momentum Components in Body Frame");
legend("H_x","H_y", "H_z");
grid on
% Visualizing the Variation of the Angular Momentum Components in the Inertial Frame   
plot(t,H_InertialFrame(:,1), 'LineWidth', 1.45, 'Color', "red");
hold on
plot(t, H_InertialFrame(:,2), 'LineWidth', 1.45, 'Color', "blue");
hold on
plot(t,H_InertialFrame(:,3), 'LineWidth', 1.35, 'Color', "yellow")
hold off
xlim([0 5]);
xticks(0:0.5:5);
ylim([-4 4]);
yticks(-3:1.5:3)
xlabel("Time (s)")
ylabel("H-Components (in Inertial Frame)");
title("Variation of Angular Momentum Components in Body Frame");
legend("H_x","H_y", "H_z");
grid on
%% Computing the Elements of the Euler Angle Direction Cosine Matrix (DCM)

n = length(t);

% Setting up the vector for each element of DCM
DCM11 = zeros(n,1);  DCM12 = zeros(n,1);  DCM13 = zeros(n,1);
DCM21 = zeros(n,1);  DCM22 = zeros(n,1);  DCM23 = zeros(n,1);
DCM31 = zeros(n,1);  DCM32 = zeros(n,1);  DCM33 = zeros(n,1);

for i = 1:n
    C1_Phi = [1 0 0; 0 cos(Phi(i)) -sin(Phi(i)); 0 sin(Phi(i)) cos(Phi(i))];
    C2_Theta = [cos(Theta(i)) 0 sin(Theta(i)); 0 1 0;-sin(Theta(i)) 0 cos(Theta(i))];
    C3_Psi = [cos(Psi(i)) -sin(Psi(i))  0; sin(Psi(i)) cos(Psi(i))  0; 0 0 1]; 
    
    CB2I = C3_Psi * C2_Theta * C1_Phi;      % Body 3-2-1 Direction Cosine Matrix

    DCM11(i, 1) = CB2I(1,1);  DCM12(i, 1) = CB2I(1,2);  DCM13(i, 1) = CB2I(1,3);
    DCM21(i, 1) = CB2I(2,1);  DCM22(i, 1) = CB2I(2,2);  DCM23(i, 1) = CB2I(2,3);
    DCM31(i, 1) = CB2I(3,1);  DCM32(i, 1) = CB2I(3,2);  DCM33(i, 1) = CB2I(3,3);
end
%% *Simulating 3D Orientations of the CubeSat in Terms of Quaternions*

% Defining Quaternion for CubeSat Nominal Orinetation
quat0 = [1;0;0;0];                     % Euler Parameters for Nominal Orientation
omega0 = [0.1;15; 0.1];                % CubeSat Initial Angular Velocity 
s00 = [quat0; omega0];                 % Initial States Vector

% Defining the time window
tspan = 0:0.01:30;                % in secs

% Error Tolerance
tolerance = 1e-9;
options = odeset("RelTol",tolerance, "AbsTol", tolerance);

% Implementation of ODE45 Numerical Solver
[t, s] = ode45(@Quaternions, tspan, s00, options, I);

% Extracting the Euler Parameters
q4 = s(:,1);
q1 = s(:,2);
q2 = s(:,3);
q3 = s(:,4);
%% Computing the Elements of the Quaternion Direction Cosine Matrix (DCM)

n = length(t);

% Setting up the vector for each element of DCM
Quat_DCM11 = zeros(n,1);  Quat_DCM12 = zeros(n,1);  Quat_DCM13 = zeros(n,1);
Quat_DCM21 = zeros(n,1);  Quat_DCM22 = zeros(n,1);  Quat_DCM23 = zeros(n,1);
Quat_DCM31 = zeros(n,1);  Quat_DCM32 = zeros(n,1);  Quat_DCM33 = zeros(n,1);

for i = 1:n
    % Direction Cosine Matrix for Quaternions
    Quat_CB2I = [   1-2*(q2(i)^2 + q3(i)^2)     2*(q1(i)*q2(i)-q3(i)*q4(i))  2*(q1(i)*q3(i)+q2(i)*q4(i));
                  2*(q1(i)*q2(i)+q3(i)*q4(i))     1-2*(q1(i)^2 + q3(i)^2)    2*(q2(i)*q3(i)-q1(i)*q4(i));
                  2*(q1(i)*q3(i)-q2(i)*q4(i))   2*(q2(i)*q3(i)+q1(i)*q4(i))    1-2*(q1(i)^2 + q2(i)^2)];  

    Quat_DCM11(i, 1) = Quat_CB2I(1,1);  Quat_DCM12(i, 1) = Quat_CB2I(1,2);  Quat_DCM13(i, 1) = Quat_CB2I(1,3);
    Quat_DCM21(i, 1) = Quat_CB2I(2,1);  Quat_DCM22(i, 1) = Quat_CB2I(2,2);  Quat_DCM23(i, 1) = Quat_CB2I(2,3);
    Quat_DCM31(i, 1) = Quat_CB2I(3,1);  Quat_DCM32(i, 1) = Quat_CB2I(3,2);  Quat_DCM33(i, 1) = Quat_CB2I(3,3);
end
%% Comparing the Variation of Body Frame in Inertial Frame for Euler Angles & Quaternions

% Plotting the Body Frame X-Axis Variation
Fig1 = figure("Name", "Variation of Body Frame X-axis in Inertial Frame over Time");

subplot(3,1,1);
plot(t, DCM11, t, Quat_DCM11, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (11)")
title('Variation of Body X-axis in Inertial Frame Over Time');
legend('Euler Angles','Quaternion'); 
grid on;

subplot(3,1,2);
plot(t, DCM12, t, Quat_DCM12, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (12)")
legend('Euler Angles','Quaternion'); 
grid on;

subplot(3,1,3);
plot(t, DCM13, t, Quat_DCM13, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (13)")
legend('Euler Angles','Quaternion'); 
grid on;
% Plotting the Body Frame Y-Axis Variation
Fig2 = figure("Name", "Variation of Body Frame Y-axis in Inertial Frame over Time");

subplot(3,1,1);
plot(t, DCM21, t, Quat_DCM21, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (21)")
title('Variation of Body Y-axis in Inertial Frame Over Time');
legend('Euler Angles','Quaternion'); 
grid on;

subplot(3,1,2);
plot(t, DCM22, t, Quat_DCM22, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (22)")
legend('Euler Angles','Quaternion'); 
grid on;

subplot(3,1,3);
plot(t, DCM23, t, Quat_DCM23, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (23)")
legend('Euler Angles','Quaternion'); 
grid on;
% Plotting the Body Frame Z-Axis Variation
Fig3 = figure("Name", "Variation of Body Frame Z-axis in Inertial Frame over Time");

subplot(3,1,1);
plot(t, DCM31, t, Quat_DCM31, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (31)")
title('Variation of Body Z-axis in Inertial Frame For Euler Angles & Quaternions');
legend('For Euler Angles','For Quaternions'); 
grid on;

subplot(3,1,2);
plot(t, DCM32, t, Quat_DCM32, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (32)")
legend('For Euler Angles','For Quaternions'); 
grid on;

subplot(3,1,3);
plot(t, DCM33, t, Quat_DCM33, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (33)")
legend('For Euler Angles','For Quaternions'); 
grid on;
%% *Simulating CubeSat 3D Orientations in Terms of Poisson's Kinematics*

C0 = eye(3);                          % Initial DCM, (Body Frame Aligned with the Inertial Frame)
omega0 = [0.1;15; 0.1];               % CubeSat Initial Angular Velocity Components
s000 = [reshape(C0, 9,1); omega0];    % Initial States Vector

% Defining the time window
tspan = 0:0.01:30;                % in secs

% Error Tolerance
tolerance = 1e-9;
options = odeset("RelTol",tolerance, "AbsTol", tolerance);

% Implementation of ODE45 Numerical Solver
[t, s] = ode45(@Poissons, tspan, s000, options, I);

% Extracting the DCM Elements 
C11 = s(:,1); C12 = s(:,2); C13 = s(:,3);
C21 = s(:,4); C22 = s(:,5); C23 = s(:,6);
C31 = s(:,7); C32 = s(:,8); C33 = s(:,9);
%% Comparing the Variation of Body Frame for Euler Angles & DCM in Inertial Frame 

% Plotting the Body Frame X-Axis Variation
Fig4 = figure("Name", "Variation of Body Frame X-axis in Inertial Frame over Time");

subplot(3,1,1);
plot(t, DCM11, t, C11, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (11)")
title('Variation of Body X-axis in Inertial Frame For Euler Angles & DCM');
legend('For Euler Angles','For DCM'); 
grid on;

subplot(3,1,2);
plot(t, DCM12, t, C12, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (12)")
legend('For Euler Angles','For DCM'); 
grid on;

subplot(3,1,3);
plot(t, DCM13, t, C13, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (13)")
legend('For Euler Angles','For DCM'); 
grid on;
% Plotting the Body Frame Y-Axis Variation
Fig5 = figure("Name", "Variation of Body Frame Y-axis in Inertial Frame over Time");

subplot(3,1,1);
plot(t, DCM21, t, C21, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (21)")
title('Variation of Body Y-axis in Inertial Frame For Euler Angles & DCM');
legend('For Euler Angles','For DCM'); 
grid on;

subplot(3,1,2);
plot(t, DCM22, t, C22, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (22)")
legend('For Euler Angles','For DCM'); 
grid on;

subplot(3,1,3);
plot(t, DCM23, t, C23, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (23)")
legend('For Euler Angles','For DCM'); 
grid on;
% Plotting the Body Frame Z-Axis Variation
Fig6 = figure("Name", "Variation of Body Frame Z-axis in Inertial Frame over Time");

subplot(3,1,1);
plot(t, DCM31, t, C31, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (31)")
title('Variation of Body Z-axis in Inertial Frame For Euler Angles & DCM');
legend('For Euler Angles','For DCM'); 
grid on;

subplot(3,1,2);
plot(t, DCM32, t, C32, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (32)")
legend('For Euler Angles','For DCM'); 
grid on;

subplot(3,1,3);
plot(t, DCM33, t, C33, 'LineWidth', 1.25);
xlim([0 15]); xticks(0:3:15);
ylim([-2 2]); yticks(-2:0.5:2);
xlabel("Time (sec)");
ylabel("DCM Element (33)")
legend('For Euler Angles','For DCM'); 
grid on;
%% Animating the Variations in the Orientation of the CubeSat 
% To animate the rotational motion of the CubeSat, we will use the Direction 
% Cosine Matrix, which will be computed at each timestep to update the orientation 
% of the cubesat geometry in a 3D plot. Besides, we'll also animate the rotation 
% of the satellite fins, keeping them fixed relative to the satelite body so they 
% rotate together. Before we animate the dynamics of the cubesat, we need to initialize 
% certain features of MATLAB.
% Setting up the Animation Window

figure('Name', 'CubeSat Rotational Animation with Fins',...
       'Position', [100, 100, 800, 600]);
axis equal
xlabel("X-axis"); ylabel("Y-axis"); zlabel("Z-axis");
xlim([-0.5 0.5]*100); ylim([-0.5 0.5]*100); zlim([-0.5 0.5]*100);
xticks(-50:20:50);
yticks(-50:20:50);
zticks(-50:20:50);
view(130,10); 
grid on;
camlight;
hold on;

% Add annotation
initial_omega = [0.1; 15; 0.1];
initial_Euler = [0; 0; 0];

%txt = sprintf('Initial \\omega = [%.2f, %.1f, %.2f] rad/s\nInitial Euler Angles = [%.2f, %.2f, %.2f]', ...
%               initial_omega(1), initial_omega(2), initial_omega(3), ...
%               initial_Euler(1), initial_Euler(2), initial_Euler(3));

% annotation('textbox', [0.01, 0.01, 0.2, 0.5], 'String', txt, ...
%            'Color', 'w', 'EdgeColor', 'w', 'BackgroundColor', 'k', ...
%            'FontSize', 10, 'FontWeight', 'bold', 'FitBoxToText', 'on');

% Set figure background
fig = gcf;
set(fig, 'Color', 'k');  % gcf = get current figure

% Set axes background and color properties
ax = gca;                 % gca = get current axes
set(ax, 'Color', 'k');    % axes background to black
set(ax, 'XColor', 'w');   % x-axis to white
set(ax, 'YColor', 'w');   % y-axis to white
set(ax, 'ZColor', 'w');   % z-axis to white
set(ax, 'GridColor', 'w','LineWidth', 2.5);
title("Animated Visuals of a CubeSat Attitude Dynamics",...
     'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold')

%% Defining Satellite Body & Fin Geometry in Separate Variables for Animation Loops

% Base body and fin vertex sets (scaled by 100 already)
V0_body = body_v;
V0_fin1 = fin1_v;
V0_fin2 = fin2_v;
% Initializing the Video File to Record the Animation

% Set up the video writer
v = VideoWriter('Tumbling_CubeSat_Rotation_Animation.mp4', 'MPEG-4');
v.FrameRate = 5;         % Adjust for smoothness vs file size
open(v);
% Loop Through Time to Animate the Orientations of the CubeSat

for i = 1:4:length(t)
    cla; % Clear axes

    % Compute rotation matrix from Euler angles
    C1_Phi = [1 0 0; 0 cos(Phi(i)) -sin(Phi(i)); 0 sin(Phi(i)) cos(Phi(i))];
    C2_Theta = [cos(Theta(i)) 0 sin(Theta(i)); 0 1 0; -sin(Theta(i)) 0 cos(Theta(i))];
    C3_Psi = [cos(Psi(i)) -sin(Psi(i)) 0; sin(Psi(i)) cos(Psi(i)) 0; 0 0 1];
    R = C3_Psi * C2_Theta * C1_Phi;

    % Apply rotation to body and fins
    V_rot_body = (R * V0_body')';
    V_rot_fin1 = (R * V0_fin1')';
    V_rot_fin2 = (R * V0_fin2')';

    % Draw rotated CubeSat
    patch('Vertices', V_rot_body,'Faces', f(1:4,:),'FaceColor','#DBDFEA','EdgeColor','k','lineWidth', 1);
    patch('Vertices', V_rot_body,'Faces', f(5:6,:),'FaceColor','#7A73D1','EdgeColor','k','lineWidth', 1);

    % Draw rotated fins
    patch('Vertices', V_rot_fin1, 'Faces', [1 2 3 4], ...
          'FaceColor', 'yellow', 'EdgeColor', 'black');
    patch('Vertices', V_rot_fin2, 'Faces', [1 2 3 4], ...
          'FaceColor', 'yellow', 'EdgeColor', 'black');

    % Draw rotated body frame axes
    scale = 50;
    mArrow3(origin, scale * R(1,:), "color", '#D2FF72', 'stemWidth', 1.5, 'tipWidth', 3); % x-axis
    mArrow3(origin, scale * R(2,:), "color", '#DA498D', 'stemWidth', 1.5, 'tipWidth', 3); % y-axis
    mArrow3(origin, scale * R(3,:), "color", '#FF1E1E', 'stemWidth', 1.5, 'tipWidth', 3); % z-axis

    % Optional: show current time
    text(-40, 40, 40, sprintf('Time: %.2f s', t(i)), 'FontSize', 10,...
        'FontWeight', 'bold', "Color","W");
    drawnow;
    frame = getframe(gcf);    % Capture current figure
    writeVideo(v, frame);     % Write it to the video file
end
close(v);
winopen('Tumbling_CubeSat_Rotation_Animation.mp4')
