function [cell_order] = plan_cell_order(decomposed_map, reeb_graph, num_cells)
% plan_cell_order Determines what order the cells should be searched for
% complete coverage
%   Detailed explanation goes here

% Initialise
cell_order = [];
unsearched_cells = (1:num_cells);

% Get starting cell number
start = 1;
cell = start;
cell_order_idx = 1;
cell_seq_idx = 2;
unsearched_cells(start) = [];  % remove start cell

cell_seq = zeros(1,num_cells);
cell_seq(1) = start;

% Continue while there are still unsearched cells
while true
    % Initialise
    all_searched = true;
    
    % The cell is not a dead end
    if outdegree(reeb_graph,cell) > 0
        % Check successors
        next_cells = successors(reeb_graph,cell);
        
        % Check if there is an unsearched successor
        for i = 1:size(next_cells,2)
            % A successor hasn't been searched yet
            if find(unsearched_cells==next_cells(i))
                all_searched = false;
                cell = next_cells(i);
                
                % Update lists
                cell_seq(cell_seq_idx) = cell;  % append to continuous cell sequence
                unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
                break
            end
        end
        
        % All the successors have already been searched
        if all_searched == true
            % Add lowest unsearched cell
            cell = min(unsearched_cells);
            
            % Add sequence to cell order
            cell_order = [cell_order; cell_seq];
            cell_order_idx = cell_order_idx + 1;

            % Reset cell sequence
            cell_seq(1,:) = 0;  % clear sequence
            cell_seq_idx = 1;  % restart index

            % Add lowest unsearched cell
            cell = min(unsearched_cells);
            unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
            cell_seq(cell_seq_idx) = cell;
        end     
        
    % The cell is a dead end
    else
        % Add sequence to cell order
        cell_order = [cell_order; cell_seq];
        cell_order_idx = cell_order_idx + 1;
        
        % Reset cell sequence
        cell_seq(1,:) = 0;  % clear sequence
        cell_seq_idx = 1;  % restart index
            
        % Add lowest unsearched cell
        cell = min(unsearched_cells);
        unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
        cell_seq(cell_seq_idx) = cell;
    end
    
    % Increment loop
    cell_seq_idx = cell_seq_idx + 1;
    
    if isempty(unsearched_cells) == 1
        % Add last cell
        cell_order = [cell_order; cell_seq];
        break
    end
    
end

end

