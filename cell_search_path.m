function [cell_waypoints] = cell_search_path(decomposed_map,cell,resolution)
% cell_search path Generates a search path within a cell
%   Detailed explanation goes here

[ceiling_idx, floor_idx] = ceil_floor_cell(decomposed_map, cell);

num_ceiling = size(ceiling_idx,1);
num_waypoints = num_ceiling*2;

cell_waypoints = zeros(num_waypoints,2);  % [x1,y1; x2,y2; ...]

%% Populate with matrix indices

% start at bottom left
cell_waypoints(1,:) = floor_idx(1,:);

% Initialise different rate counter
j = 1;

for i = 2:4:num_waypoints-1
    
    % Add two next ceiling points
    cell_waypoints(i,:) = ceiling_idx(j,:);
    cell_waypoints(i+1,:) = ceiling_idx(j+1,:);

    % Add two next floor points
    cell_waypoints(i+2,:) = floor_idx(j+1,:);
    cell_waypoints(i+3,:) = floor_idx(j+2,:);
    
    % Update different rate counter
    j = j+1;
    
end

% end at top right
cell_waypoints(num_waypoints,:) = ceiling_idx(num_ceiling,:);

%% Convert from matrix indices to map waypoints [x, y]

% Perform scaling for resolution
cell_waypoints = cell_waypoints/resolution;

end

