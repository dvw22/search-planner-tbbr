function [map_waypoints,segment_idx] = map_search_path(cell_order,decomposed_map,resolution,planner)
% map_search_path Outputs a full waypoint list with a segment indices matrix
% to access segmented regions during the waypoint planning process.
%   The segments are either a cell sequence path or a shortest path between
%   cells.

%% Waypoint Generation
% Initialise
map_waypoints = [];
segment_idx = [];
num_cell_seq = size(cell_order,1);

% Get waypoints for each cell sequence and path between and append
for i = 1:2:num_cell_seq
    % Append cell sequence waypoints
    [cell_seq_waypoints, num_cells] = cell_seq_search_path(cell_order(i,:),decomposed_map,resolution);
    map_waypoints = [map_waypoints; cell_seq_waypoints];
    
    % Save indices
    
    % Append travel waypoints
    if i < num_cell_seq
        % Get next cell sequence waypoints
        [next_cell_seq_waypoints, num_cells] = cell_seq_search_path(cell_order(i+1,:),decomposed_map,resolution);
        
        % Get start and end points for travel
        start_point = cell_seq_waypoints(end,:);  % starting at end of last cell sequence
        end_point = next_cell_seq_waypoints(1,:);  % ending at start of next cell sequence
        
        % Append travel waypoints
        travel_waypoints = findpath(planner,start_point,end_point);
        map_waypoints = [map_waypoints; travel_waypoints];
        
        % Save indices
        
        % Append next cell sequence waypoints
        map_waypoints = [map_waypoints; cell_seq_waypoints];
    
        % Save indices
    end
    

end

