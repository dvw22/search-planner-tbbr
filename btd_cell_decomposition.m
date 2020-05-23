function [decomposed_map, cell_counter] = btd_cell_decomposition(occupancy_map)
% btd_cell_decomposition Uses boustrophedon cell decomposition to output a
% region divided occupancy map
%   The occupancy map must have a border of occupied cells for it to work.
%   For the output, a 0 is an obstacle and the integers are the
%   corresponding cell number.

%% Initialise variables
decomposed_map = zeros(size(occupancy_map));
last_connectivity = 0;
last_connections = [];
cell_counter = 0;  % tracks the number of cells total
last_cells = [];  % tracks which cells were in the last slice
current_cells = [];  % tracks which cells are in the current slice

for col = 1:size(occupancy_map,2)
    %% Check connectivity of the slice
    slice = occupancy_map(:,col);
    [connectivity, connections] = slice_connectivity(slice);
    
    %% 1. Check if we are coming out of a full obstacle slice
    if last_connectivity == 0
        % Reset current_cells for population
        current_cells = [];
        
        % loop through number of connections in the new slice and add these
        % cells
        for i = 1:size(connections,1)
            cell_counter = cell_counter + 1;
            current_cells = [current_cells, cell_counter];  % append cells
        end
        
    %% 2. Check if we are still in a full obstacle slice
    elseif connectivity == 0
        % No cells in full obstacle slice
        current_cells = [];
    
    %% Determine sweep line splitting/joining
    % Each individual connection in the slice can be checked for splitting
    % or joining. This is done using the adjacency matrix. The sum of a row
    % indicates how many right slice connections a left slice connections
    % is adjacent to. The sum of a column indicates how many left slice
    % connections a right slice connection is adjacent to.
    else
        %% 3. Connection(s) present in current or last slice
        % Compare slices using adjacency matrix
        adj_matrix = connections_adjacency(last_connections,connections);
        
        % Initialise
        insertion = [];
        replacement = [];
        changes = 0;  % tracks how many elements have been lost/gained in the current_cells

        % Compare slices left to right using adjacency matrix row
        for i = 1:size(adj_matrix,1)
            % Reset the insertion
            insertion = [];
                
            % 3a. The connection split: IN condition
            if sum(adj_matrix(i,:)) > 1
                % Find the replacement index
                cell_of_interest = last_cells(i);  % find the number of the cell
                index_of_interest = find(current_cells==cell_of_interest);  % desired index of replacement
                
                % Check how many new cells are produced and track
                for j = 1:sum(adj_matrix(i,:))
                    cell_counter = cell_counter + 1;
                    insertion = [insertion, cell_counter];
                end
                
                % Replace the old cell with the new replacement cells
                before_insertion = current_cells(1:index_of_interest-1);
                after_insertion = current_cells(index_of_interest+1:end);
                current_cells = [before_insertion,insertion,after_insertion];
                
            % 3b. The connection does not split or join (dead end)
            elseif sum(adj_matrix(i,:)) == 0
                % Find the removal index
                cell_of_interest = last_cells(i);  % find the number of the cell
                index_of_interest = find(current_cells==cell_of_interest);
                
                % Remove the cell from the current cells array
                current_cells(index_of_interest) = [];
                
            end
        end

        % Compare slices right to left using adjacency matrix columns
        for i = 1:size(adj_matrix,2)
            % 3c. The connection joined: OUT condition
            if sum(adj_matrix(:,i)) > 1
                % Find the replacement index
                cell_of_interest = last_cells(i);  % find the number of the cell
                index_of_interest = find(current_cells==cell_of_interest);
                
                % Track new cells (replaces other cells this time)
                cell_counter = cell_counter + 1;
                replacement = cell_counter;
                
                % Replace cells that merged with new cell and update
                % current_cell matrix
                before_replacement = current_cells(1:index_of_interest-1);
                after_replacement = current_cells(index_of_interest+sum(adj_matrix(:,i)):end);
                current_cells = [before_replacement, replacement, after_replacement];
                
            % 3d. A new connection formed from an obstacle: IN condition
            elseif sum(adj_matrix(:,i)) == 0
                % add just one count to the cell_counter
                cell_counter = cell_counter + 1;
                insertion = cell_counter;
                
                % insert this new cell in the correct place in current_cell
                % array
                before_insertion = current_cells(1:i-1);
                after_insertion = current_cells(i:size(current_cells,2));
                current_cells = [before_insertion, insertion, after_insertion];
            end
        end
    end
    
    %% Update map
    % No connectivity means a full line of obstacles
    if connectivity == 0
        decomposed_map(:,col) = 0;  % Obstacles are 0
    else
        for i = 1:size(connections,1)
            % Fill connectivity segments with corresponding cell numbers
            start_fill = connections(i,1);
            end_fill = connections(i,2)-1;
            decomposed_map(start_fill:end_fill,col) = current_cells(i);
        end
    end
        
    %% Store previous states
    last_connectivity = connectivity;
    last_connections = connections;
    last_cells = current_cells;
    last_slice = slice;
    debug_slices = [last_slice, slice];
    
end

end

