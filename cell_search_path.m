function [cell_waypoints] = cell_search_path(decomposed_map,cell,resolution)
% cell_search path Generates a search path within a cell
%   Detailed explanation goes here

[ceiling_idx, floor_idx] = ceil_floor_cell(decomposed_map, cell);  

num_ceiling = size(ceiling_idx,1);
num_indices = num_ceiling*2;

cell_indices = zeros(num_indices,2);  % [row1,col1; row2,col2]
cell_waypoints = cell_indices;  % [x1,y1; x2,y2; ...]

%% Populate with matrix indices

% start at bottom left
cell_indices(1,:) = floor_idx(1,:);

% Initialise different rate counter
j = 1;

for i = 2:4:num_indices-1
    
    % Add two next ceiling points
    cell_indices(i,:) = ceiling_idx(j,:);
    cell_indices(i+1,:) = ceiling_idx(j+1,:);

    % Add two next floor points
    cell_indices(i+2,:) = floor_idx(j+1,:);
    cell_indices(i+3,:) = floor_idx(j+2,:);
    
    % Update different rate counter
    j = j+2;
    
end

% end at top right
cell_indices(num_indices,:) = ceiling_idx(num_ceiling,:);

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

