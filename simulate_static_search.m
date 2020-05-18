function [search_result] = simulate_static_search(init_pose, search_path, map)
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
pose = zeros(3,numel(time_vector));   % Initialise array, [x, y, angle] x time vector
pose(:,1) = init_pose;          % Add initial condition

% Map
load map;

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

end

