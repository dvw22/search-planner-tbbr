clear

%% Setup
% Mobile Robot
initial_pose = [2,2,0];     % [x, y, theta]

% Map
load exampleMap;
occ_map = occupancyMatrix(map);
bi_occ_map = round(occ_map);  % convert to binary
% [area, num_cells] = unoccupied_area(map);

% % Map
% load OccupancyMap;
% occ_map = occVal;
% bi_occ_map = round(occ_map);  % convert to binary
% % [area, num_cells] = unoccupied_area(map);

%% Compute Search Path
% Perform cell decomposition on map
[decomposed_map, num_cells] = btd_cell_decomposition(bi_occ_map);
display_decomposed_map(decomposed_map)

% Get boustrophedon waypoints for cell
% search_path = cell_search_path(decomposed_map, 5);

% search_path = [2.5,2;
%                3,3;
%                1,5;
%                3,6];

%% Simulate Search
% result = simulate_static_search(initial_pose, search_path);