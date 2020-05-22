%% Testing setup
clear

map = [1,1,1,1,1,1,1,1,1,1,1,1
       1,0,0,0,0,0,0,0,0,0,0,1;
       1,0,0,1,1,1,0,1,1,1,0,1;
       1,0,0,1,1,1,0,1,1,1,0,1;
       1,0,0,1,1,1,0,1,1,1,0,1;
       1,0,0,0,0,0,0,1,1,1,0,1;
       1,0,0,0,0,0,0,0,0,0,0,1;
       1,1,1,1,1,1,1,1,1,1,1,1];

   
new_map = zeros(size(map));   
slice_L = map(:,1);
slice_R = map(:,4);
last_connectivity = 0;
last_connections = [];
cell_counter = 0;  % tracks the number of cells total
current_cells = [];  % tracks which cells are in a slice

for col = 1:size(map,2)
    %% Check connectivity of the slice
    slice = map(:,col);
    [connectivity, connections] = slice_connectivity(slice);
    
    %% Check if we are coming out of a full obstacle slice
    if last_connectivity == 0
        % loop through number of connections in the new slice and add these
        % cells
        for i = 1:size(connections,1)
            cell_counter = cell_counter + 1;
            current_cells = [current_cells, cell_counter];  % append cells
        end
        
    %% Check if we are still in a full obstacle slice
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
        %% Compare slices using adjacency matrix
        adj_matrix = connections_adjacency(last_connections,connections);

        % Compare slices left to right using adjacency matrix row
        for i = 1:size(adj_matrix,1)
            % The connection does not split
            if sum(adj_matrix(i,:)) == 1
                % no cells need to be added?
                
            % The connection split: IN condition
            elseif sum(adj_matrix(i,:)) > 1
                % look at how many new connections this connection is
                % adjacent to
                % add a count to the cell_counter for each one
                % replace old connection's cell number with new cell 
                % numbers in correct place in current_cell array
            end
        end

        % Compare slices right to left using adjacency matrix columns
        for i = 1:size(adj_matrix,2)
            % The connection joined: OUT condition
            if sum(adj_matrix(:,i)) > 1
                % add just one count to the cell_counter
                % replace old connections' cell numbers with new cell
                % number in correct place in current_cell array
                
            % A new connection formed inside an obstacle: IN condition
            elseif sum(adj_matrix(:,i)) == 0
                % add just one count to the cell_counter
                % insert this new cell in the correct place in current_cell
                % array
            end
        end
    end
    
    %% Update map
    % No connectivity means a full line of obstacles
    if connectivity == 0
        new_map(:,col) = 0;  % Obstacles are 0
    else
        for i = 1:size(connections,1)
            % Fill connectivity segments with corresponding cell numbers
        end
    end
        
    %% Store previous states
    last_connectivity = connectivity;
    last_connections = connections;
    
end



