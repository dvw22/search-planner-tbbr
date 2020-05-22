%% Testing setup

% map = occupancyMatrix(map);
% occupancy_map = round(occupancy_map);

% binary_map = round(occVal);

occupancy_map = [1,1,1,1,1,1,1,1,1,1,1;
                 1,0,0,0,0,0,0,0,0,0,1;
                 1,0,0,0,1,1,1,1,1,0,1;
                 1,0,0,1,1,1,0,0,1,0,1;
                 1,0,1,1,1,1,0,0,1,0,1;
                 1,0,0,1,1,1,1,1,1,0,1;
                 1,0,0,0,1,1,0,0,0,0,1;
                 1,1,1,1,1,1,1,1,1,1,1];

[decomposed_map, num_cells] = btd_cell_decomposition(occupancy_map);

% imshow(map)
display_decomposed_map(decomposed_map)


