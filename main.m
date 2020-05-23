clear

%% Setup
% Physical
initial_pose = [0.75,0.75,pi/2];  % [x, y, theta]
opi = [8, 8, 1];  % [x, y, label]

% Map
load exampleMap;
occ_map = occupancyMatrix(map);
bi_occ_map = round(occ_map);  % convert to binary
resolution = map.Resolution;
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
search_path = [];
for cell = 1:num_cells
    search_path = [search_path; cell_search_path(decomposed_map, cell, resolution)];
end
% search_path = cell_search_path(decomposed_map, 1, resolution);

% search_path = [2.5,2;
%                3,3;
%                1,5;
%                3,6];

%% Simulate Search
% result = simulate_static_search(initial_pose, search_path, opi);