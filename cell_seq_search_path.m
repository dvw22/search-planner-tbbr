function [cell_seq_waypoints, num_cells] = cell_seq_search_path(cell_seq,decomposed_map,resolution,planner)
% cell_seq_search_path Outputs waypoints for a set of cells in an optimised order
%   Gets a row in from a cell order plan matrix. Slices off zeroed
%   elements. Inputs each element as a cell number to cell_search_path.
%   Appends the result to a full cell sequence waypoint matrix.

%% Preparation
% Slice off zeros
cell_seq = cell_seq(:,any(cell_seq,1));

%% Initialise
% Initialise
cell_seq_waypoints = [];  % reset search path
num_cells = size(cell_seq,2);

%% Waypoint Generation
% Get waypoints for each cell and append
for i = 1:num_cells
    [cell_waypoints, num_waypoints] = cell_search_path(decomposed_map,cell_seq(i),resolution,planner);
    cell_seq_waypoints = [cell_seq_waypoints; cell_waypoints];
end

end

