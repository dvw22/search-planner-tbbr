function [decomposed_map, cell_counter] = btd_cell_decomposition(occupancy_map)
% btd_cell_decomposition Uses boustrophedon cell decomposition to output a
% region divided occupancy map
%   The occupancy map must have a border of occupied cells for it to work.
%   For the output, a 0 is an obstacle and the integers are the
%   corresponding cell number.

%% Initialise variables
decomposed_map = zeros(size(occupancy_map));
reeb_graph = graph;
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
        % Reset current_cells for re-population
        current_cells = [];
        
        % Loop through number of connections in the new slice and add these
        % cells
        for i = 1:size(connections,1)
            cell_counter = cell_counter + 1;
            current_cells = [current_cells, cell_counter];  % append cells
        end
        
        % Update graph
        reeb_graph = reeb_graph.addnode(size(current_cells,2));
        
    %% 2. Check if we are in a full obstacle slice
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

        % Compare slices left to right using adjacency matrix row
        for i = 1:size(adj_matrix,1)
            % 3a. The connection split: IN condition
            if sum(adj_matrix(i,:)) > 1
                % Reset the split cells
                split = [];
                
                % Find the split index
                cell_of_interest = last_cells(i);  % find the number of the cell
                index_of_interest = find(current_cells==cell_of_interest);  % desired index of replacement            
                
                % Check how many new cells are produced and track
                for j = 1:sum(adj_matrix(i,:))
                    cell_counter = cell_counter + 1;
                    split = [split, cell_counter];
                end
                
                % Replace the old cell with the new split cells
                before_split = current_cells(1:index_of_interest-1);
                after_split = current_cells(index_of_interest+1:end);
                current_cells = [before_split,split,after_split];
                
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
                % Find the join index
                index_of_interest = i;
                
                % Track new cells (replaces other cells this time)
                cell_counter = cell_counter + 1;
                join = cell_counter;
                
                % Replace cells that joined with new cell and update
                % current_cell matrix
                before_join = current_cells(1:index_of_interest-1);
                after_join = current_cells(index_of_interest+sum(adj_matrix(:,i)):end);
                current_cells = [before_join, join, after_join];
                
            % 3d. A new connection formed from an obstacle: IN condition
            elseif sum(adj_matrix(:,i)) == 0
                % Find the insert index
                index_of_interest = i;
                
                % Add just one count to the cell_counter
                cell_counter = cell_counter + 1;
                insert = cell_counter;
                
                % Insert this new cell in the correct place in current_cell
                % array
                before_insert = current_cells(1:index_of_interest-1);
                after_insert = current_cells(index_of_interest:end);
                current_cells = [before_insert, insert, after_insert];
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
    
end

end

