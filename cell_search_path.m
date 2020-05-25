function [cell_waypoints, num_waypoints] = cell_search_path(decomposed_map,cell,resolution,planner)
% cell_search path Generates a search path within a cell
%   Detailed explanation goes here

[ceiling_idx, floor_idx] = ceil_floor_cell(decomposed_map, cell);  

num_ceiling = size(ceiling_idx,1);
num_waypoints = num_ceiling*2;

cell_indices = zeros(num_waypoints,2);  % [row1,col1; row2,col2]
cell_waypoints = cell_indices;  % [x1,y1; x2,y2; ...]

%% Populate with matrix indices

% Start at bottom left
cell_indices(1,:) = floor_idx(1,:);
    
% Initialise
add_ceilings = true;
j = 1;  % different rate counter

% For waypoint planning in between edges
insertion_indices = [];

for i = 2:num_waypoints-1
    % Adding ceiling points
    if add_ceilings == true
        % Add a ceiling point
        cell_indices(i,:) = ceiling_idx(j,:);
        
        % Get next row of ceiling_idx (even)
        if mod(i,2) == 0
            j = j+1; 
        % Reset index for floor_idx counting every other time (odd)
        else
            % Check pair for edges
            if cell_indices(i,1) ~= cell_indices(i-1,1)
                insertion_indices = [insertion_indices; i];  % store insertion index
            end
            
            % Reset for next pair
            add_ceilings = false;
            j = j-1;
        end 
    % Adding floor points
    else
        % Add a floor point
        cell_indices(i,:) = floor_idx(j+1,:);
        
        % Check if both pairs of ceiling and floor are added
        if mod(i-1,4) == 0
            j = j+2;
        end
        
        % Get next row of floor_idx (even)
        if mod(i,2) == 0
            j = j+1;
        % Reset index for ceiling_idx counting every other time (odd)
        else
            % Check pair for edges
            if cell_indices(i,1) ~= cell_indices(i-1,1)
                insertion_indices = [insertion_indices; i];  % store insertion index
            end
            
            % Reset for next pair
            add_ceilings = true;
            j = j-1;
        end
    end
end

% end with last ceiling/floor
if add_ceilings == true
    cell_indices(num_waypoints,:) = ceiling_idx(num_ceiling,:);
elseif add_ceilings == false
    cell_indices(num_waypoints,:) = floor_idx(num_ceiling,:);
end

%% Post processing to insert waypoint paths between edges
% insertion_offset = 0;  % initialise offset due to path insertions
% for i = 1:size(insertion_idx,1)


% update num_waypoints


%% Convert from matrix indices [row, col] to map waypoints [x, y]

% Rows are y reference, columns are x reference (swap)
cell_waypoints(:,1) = cell_indices(:,2);
cell_waypoints(:,2) = cell_indices(:,1);

% Flip y points
cell_waypoints(:,2) = (size(decomposed_map,1)+1) - cell_waypoints(:,2);

% Shift waypoints to centres of grid units
cell_waypoints = cell_waypoints - 0.5;

% Perform scaling for resolution
cell_waypoints = cell_waypoints/resolution;


end

