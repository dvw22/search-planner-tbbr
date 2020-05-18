function [searched_area,num_searched_cells] = searched_area(search_matrix,map)
%unoccupied_area Calculates the total searched area in the map
%   Detailed explanation

num_searched_cells = sum(sum(search_matrix));
searched_area = (1/map.Resolution)^2 * num_searched_cells;    % convert to area
end

