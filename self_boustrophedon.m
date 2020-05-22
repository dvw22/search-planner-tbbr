%% Testing setup

occupancy_map = occupancyMatrix(map);

binary_map = round(occupancy_map);

[decomposed_map, num_cells] = btd_cell_decomposition(binary_map);

% imshow(map)
display_decomposed_map(decomposed_map)


