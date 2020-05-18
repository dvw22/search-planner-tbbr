function [search_result] = simulate_static_search(init_pose, search_path)
%simulate_static_search Simulates a mobile robot statically executing a search path on the map.
%   Detailed explanation goes here

%% Simulation Setup
% Define Vehicle
wheel_radius = 0.1;                        % Wheel [m]
wheel_base = 0.5;                        % [m]
mobile_robot = DifferentialDrive(wheel_radius,wheel_base);

% Time Array
sample_time = 0.1;              % [s]
end_time = 45;                  % [s]
time_vector = 0:sample_time:end_time;

% Pose Array
pose = zeros(3,numel(time_vector));   % Initialise array, [x, y, theta] x time vector
pose(:,1) = init_pose;          % Add initial condition

% Map


% Simulation Visualiser
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';

% Path Following Controller
controller = controllerPurePursuit;
controller.Waypoints = search_path;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Simulation Loop
rate = rateControl(1/sample_time);
for i = 2:numel(time_vector)    % start index at 2nd element
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [ref_fwd_speed,ref_ang_speed] = controller(pose(:,i-1));          % Calculate forward and angular speeds
    [wL,wR] = inverseKinematics(mobile_robot,ref_fwd_speed,ref_ang_speed);        % Calculate individual wheel speeds 
    
    % Compute the velocities
    [fwd_speed,ang_speed] = forwardKinematics(mobile_robot,wL,wR);
    body_velocity = [fwd_speed;0;ang_speed]; % Body velocities [vx;vy;w]
    world_velocity = bodyToWorld(body_velocity,pose(:,i-1));  % Convert from body coordinates to world coordinates
    
    % Perform forward discrete integration step
    world_distance = world_velocity*sample_time;    % Calculate distance moved
    pose(:,i) = pose(:,i-1) + world_distance;    % Calculate new pose
    
    % Update visualization
    viz(pose(:,i),search_path)
    waitfor(rate);
    
    % Check if object is found
    search_result = true;
end


end

