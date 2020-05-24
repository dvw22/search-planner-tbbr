function [cell_order] = plan_cell_order(decomposed_map, reeb_graph, num_cells)
% plan_cell_order Determines what order the cells should be searched for
% complete coverage
%   Detailed explanation goes here

% Initialise
cell_order = zeros(num_cells);

% Get starting cell number
start = 1;
cell_order(1) = start;

for cell_order_idx = 1:num_cells
    % Initialise
    unsearched_neighbor = false;
    
    % Get adjacent cells
    adj_cells = neighbors(reeb_graph,cell);
    
    if 
    
    % Try find a neighbour that hasn't been visited
    for i = 1:size(adj_cells,1)
        % Cell hasn't been visited
        if ismember(adj_cells(i),cell_order) == 0
            unsearched_neighbor = true;
            cell = adj_cells(i);
            cell_order(list_idx) = cell;
            break  % greedy
        end
    end
    
    % No unsearched neighbours found. Move to lowest unsearched cell
    if unsearched_neighbor == false
        
    end
    
    
end

    

end

