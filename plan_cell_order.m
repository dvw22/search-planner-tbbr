function [cell_order] = plan_cell_order(decomposed_map, reeb_graph, num_cells)
% plan_cell_order Determines what order the cells should be searched for
% complete coverage
%   Detailed explanation goes here

% Initialise
cell_order = zeros(num_cells,1);
unsearched_cells = [1:num_cells];

% Get starting cell number
start = 1;
cell = start;
cell_order(1) = cell;
unsearched_cells(start) = [];  % remove start cell

for cell_order_idx = 2:num_cells
    % Initialise
    unsearched_neighbor = false;
    
    % Check if there are next cells
    if outdegree(reeb_graph,cell) > 0
        % Store next cell
        next_cells = successors(reeb_graph,cell);
        cell = next_cells(1);  % ASSUMPTION: TAKE THE FIRST ALWAYS
        
        % Update
        unsearched_neighbor = true;
        unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
        cell_order(cell_order_idx) = cell;
    end
    
    % No unsearched neighbours found. Move to lowest unsearched cell
    if unsearched_neighbor == false
        % Add lowest unsearched cell
        cell = min(unsearched_cells);
        unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
        cell_order(cell_order_idx) = cell;
    end
    
end

    

end

