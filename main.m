clear

%% Map Setup
% Occupancy matrix from map
load exampleMap;
% load complexMap;
% load OccupancyMap;

%% Other Setup
% Search Robot Object
Search_robot = SearchRobot();

% Starting Positions
Search_robot.pose = [0.75,0.75,pi/2];  % [x, y, theta]
opi = [8, 8, 1];  % [x, y, label]

% Test Suite
Test_suite = SearchTestSuite(map);

%% Create Search Planner Object
Search_planner = OfflineSearchPlanner(map);

%% Plan Search Path
Search_planner.update_search_path(Search_robot.pose);

%% Plot Path
% Simulation Visualiser
% Search_planner.plot_search_path()

%% Simulate Search
result = simulate_static_search(Search_robot,Test_suite,Search_planner,opi);
