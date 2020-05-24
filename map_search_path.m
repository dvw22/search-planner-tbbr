function [map_waypoints,segment_idx] = map_search_path(cell_order,decomposed_map,resolution,planner, start_pos)
% map_search_path Outputs a full waypoint list with a segment indices matrix
% to access segmented regions during the waypoint planning process.
%   The segments are either a cell sequence path or a shortest path between
%   cells.

%% Waypoint Generation
% Initialise
map_waypoints = [];
segment_idx = [];
num_cell_seq = size(cell_order,1);

last_end_idx = 0;
start_idx = 1;

% Get waypoints for each cell sequence and path between and append
for i = 1:num_cell_seq
    % Calculate waypoints before travel first time only
    if i == 1
        % Append cell sequence waypoints
        [cell_seq_waypoints, num_cells] = cell_seq_search_path(cell_order(i,:),decomposed_map,resolution);
        map_waypoints = [map_waypoints; cell_seq_waypoints];

        % Append cell sequence indices
        num_waypoints = size(cell_seq_waypoints,1);
        end_idx = last_end_idx + num_waypoints;
        segment_idx = [segment_idx; [start_idx, end_idx]];
        start_idx = end_idx + 1;  % store
        last_end_idx = end_idx;  % store
        
        % Plan path to first waypoint
        travel_waypoints = findpath(planner,start_pos,map_waypoints(1,:));
        % The planner may not always find a path if the map is too complex.
        % This condition is to prevent the code from breaking.
        if isempty(travel_waypoints) == 0
            travel_waypoints(1,:) = [];  % Must clear source
            travel_waypoints(end,:) = [];  % Must clear destination of travel to avoid duplicate
        else
            display(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
        end
        map_waypoints = [travel_waypoints; map_waypoints];
        
        % Append travel indices
        num_waypoints = size(travel_waypoints,1);
        end_idx = last_end_idx + num_waypoints;
        segment_idx = [segment_idx; [start_idx, end_idx]];
        start_idx = end_idx + 1;  % store
        last_end_idx = end_idx;  % store
        
    % Just finished calculating a travel segment
    else
        % Append cell sequence waypoints
        map_waypoints = [map_waypoints; cell_seq_waypoints];

        % Append cell sequence indices
        num_waypoints = size(cell_seq_waypoints,1);
        end_idx = last_end_idx + num_waypoints;
        segment_idx = [segment_idx; [start_idx, end_idx]];
        start_idx = end_idx + 1;  % store
        last_end_idx = end_idx;  % store
    end
    
    % Append travel waypoints
    if i < num_cell_seq
        % Get next cell sequence waypoints
        [next_cell_seq_waypoints, num_cells] = cell_seq_search_path(cell_order(i+1,:),decomposed_map,resolution);
        
        % Get start and end points for travel
        start_waypoint = cell_seq_waypoints(end,:);  % starting at end of last cell sequence
        end_waypoint = next_cell_seq_waypoints(1,:);  % ending at start of next cell sequence
        
        % Append travel waypoints
        travel_waypoints = findpath(planner,start_waypoint,end_waypoint);
        % The planner may not always find a path if the map is too complex.
        % This condition is to prevent the code from breaking.
        while isempty(travel_waypoints) == 1
            % Publish info
            display(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
            display('Increasing nodes and connection distance')
            
            % Increase nodes
            planner.NumNodes = planner.NumNodes * 2;
            planner.ConnectionDistance = planner.ConnectionDistance * 2;
            
            % Calculate again
            travel_waypoints = findpath(planner,start_waypoint,end_waypoint);
        end    
        travel_waypoints(1,:) = [];  % Must clear source of travel to avoid duplicate
        travel_waypoints(end,:) = [];  % Must clear destination of travel to avoid duplicate
        map_waypoints = [map_waypoints; travel_waypoints];
        
        % Append travel indices
        num_waypoints = size(travel_waypoints,1);
        end_idx = last_end_idx + num_waypoints;
        segment_idx = [segment_idx; [start_idx, end_idx]];
        start_idx = end_idx + 1;  % store
        last_end_idx = end_idx;  % store
        
        % Store next cell sequence waypoints
        cell_seq_waypoints = next_cell_seq_waypoints;
    end

end

