clear

%% Map Setup
% Occupancy matrix from map
% load exampleMap;
load complexMap;
% load OccupancyMap;

%% Other Setup
% Search Robot Object
Search_robot = SearchRobot();

% Starting Positions
Search_robot.pose = [0.75,0.75,pi/2];  % [x, y, theta]
start_position = [Search_robot.pose(1), Search_robot.pose(2)];  % [x, y]
opi = [8, 8, 1];  % [x, y, label]

%% Create Search Planner Object (Decomposes Map and Plans Cell Order)
Search_planner = SearchPlanner(map);

%% Plan Search Path
[complete_waypoints, segment_idx] = Search_planner.complete_search_path(start_position);

%% Plot Path
% Simulation Visualiser
Search_planner.plot_path(complete_waypoints)

%% Simulate Search
result = simulate_static_search(Search_robot,opi,complete_waypoints,segment_idx);
