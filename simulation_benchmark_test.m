clear

%% Map Setup
load exampleMap;
% load complexMap;
% load OccupancyMap;

% Inflate map
inflated_map = copy(map);
inflate(inflated_map,1,'grid')

%% Other Setup
% Search Robot Object
Search_robot = SearchRobot();

% Create Search Planner Object
Search_planner = OfflineSearchPlanner(inflated_map);

% Test Suite
Test_suite = SearchTestSuite(map);

% Starting Positions
Search_robot.pose = [2; 2; pi/2];  % [x, y, theta]
opi = [0.25, 0.25, 1];  % [x, y, label]

%% Plan Search Path and Measure Computation Time
tic
Search_planner.update_search_path(Search_robot.pose);
Test_suite.computation_time = toc;

%% Visualisation
% Plot Path
% Search_planner.plot_search_path()

% Simulate Search
result = simulate_offline_search(Search_robot,Test_suite,Search_planner,opi);
