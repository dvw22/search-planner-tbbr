function [unoccupied_area,num_unoccupied_cells] = unoccupied_area(map)
%unoccupied_area Calculates the total free area in the map
%   Detailed explanation

occupancy_matrix = occupancyMatrix(map);

binary_occupancy_matrix = occupancy_matrix>map.ProbabilitySaturation(1);

num_occupied_cells = sum(sum(binary_occupancy_matrix));

num_unoccupied_cells = numel(occupancy_matrix) - num_occupied_cells;      % calculate number of unsearched cells
unoccupied_area = (1/map.Resolution)^2 * num_unoccupied_cells;    % convert to area

end

