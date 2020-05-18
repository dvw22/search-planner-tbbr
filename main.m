% Most of the content happens here

%% Setup
% Mobile Robot
initial_pose = [2,2,0];     % [x, y, theta]

% Map
load exampleMap;
occupancy_matrix = occupancyMatrix(map);
[area, num_cells] = unoccupied_area(map);

%% Compute Search Path
search_path = [2,2;
               3,3;
               3,5;
               6,8];

%% Simulate Search
result = simulate_static_search(initial_pose, search_path);