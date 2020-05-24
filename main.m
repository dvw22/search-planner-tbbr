clear

%% Setup
% Occupancy matrix from map
% load exampleMap;
load complexMap;
occ_matrix = occupancyMatrix(map);
bi_occ_matrix = round(occ_matrix);  % convert to binary
resolution = map.Resolution;

% Binary occupancy map from matrix
bi_occ_map = binaryOccupancyMap(bi_occ_matrix,resolution);

% % Map
% load OccupancyMap;
% occ_map = occVal;
% bi_occ_map = round(occ_map);  % convert to binary
% resolution = 1;

% Physical
initial_pose = [0.75,0.75,pi/2];  % [x, y, theta]
opi = [8, 8, 1];  % [x, y, label]
planner = mobileRobotPRM(bi_occ_map);

% [area, num_cells] = unoccupied_area(map);

%% Decompose Map
% Perform cell decomposition on map
[decomposed_map, graph, num_cells] = btd_cell_decomposition(bi_occ_matrix);
display_decomposed_map(decomposed_map)

%% Plan Cell Order
cell_order = plan_cell_order(graph, num_cells);

%% Plan Search Path
[map_waypoints, segment_idx] = map_search_path(cell_order,decomposed_map,resolution,planner);

%% Simulate Search
result = simulate_static_search(initial_pose, opi, map_waypoints, segment_idx);
