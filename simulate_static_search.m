function [search_result] = simulate_static_search(init_pose, search_path)
%simulate_static_search Simulates a mobile robot statically executing a search path on the map.
%   Detailed explanation goes here

%% Environment Setup
% Define OPI
opi = [7, 8, 1];        % [x, y, label]

% Object Detector sensor
detector = ObjectDetector;
detector.fieldOfView = pi/4;    % [rad]

% Simulation Visualiser
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachObjectDetector(viz,detector);
viz.objectColors = [1 0 0;0 1 0;0 0 1];
viz.objectMarkers = 'so^';

%% Vehicle Setup
% Define Vehicle
wheel_radius = 0.1;                        % [m]
wheel_base = 0.5;                        % [m]
mobile_robot = DifferentialDrive(wheel_radius,wheel_base);

% Path Following Controller
controller = controllerPurePursuit;
controller.Waypoints = search_path;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Simulation Loop Setup

% Time Array
sample_time = 0.1;              % [s]
end_time = 300;                  % Arbitrarily set to 5 minutes [s]
time_vector = 0:sample_time:end_time;

% Pose Array
pose = zeros(3,numel(time_vector));   % Initialise array, [x, y, theta] x time vector
pose(:,1) = init_pose;          % Add initial condition

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
    viz(pose(:,i),search_path,opi)
    waitfor(rate);
    
    % Check if OPI is found
    detections = detector(pose(:,i),opi);
    if ~isempty(detections)
        disp('OPI detected. Ending search.');
        search_result = true;
        break;
    end
    
    % End simulation if last waypoint is reached
    dist_between = [pose(1,i),pose(2,i);search_path(end,1),search_path(end,2)];
    if pdist(dist_between,'euclidean') < 0.1    % search ends if robot within 0.1m radius of waypoint
        disp('End of search path reached. Ending search.');
        search_result = false;
        break;
    end 
    
end


end

