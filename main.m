clear

%% Setup
% Physical
initial_pose = [0.75,0.75,pi/2];  % [x, y, theta]
opi = [8, 8, 1];  % [x, y, label]

% Map
load exampleMap;
% load complexMap
occ_map = occupancyMatrix(map);
bi_occ_map = round(occ_map);  % convert to binary
resolution = map.Resolution;

% % Map
% load OccupancyMap;
% occ_map = occVal;
% bi_occ_map = round(occ_map);  % convert to binary
% resolution = 1;

% [area, num_cells] = unoccupied_area(map);

%% Decompose Map
% Perform cell decomposition on map
[decomposed_map, num_cells] = btd_cell_decomposition(bi_occ_map);
display_decomposed_map(decomposed_map)

%% Simulate Search
for cell = 1:num_cells
    % Get boustrophedon waypoints for cell
    search_path = cell_search_path(decomposed_map, cell, resolution);
    
    % Simulate
    result = simulate_static_search(initial_pose, search_path, opi);
end
% search_path = cell_search_path(decomposed_map, 10, resolution);
