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
new_cells = []; % stores new cells that are formed for each connection

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
        insertion = [];
        replacement = [];

        % Compare slices left to right using adjacency matrix row
        for i = 1:size(adj_matrix,1)
            % The connection does not split
            if sum(adj_matrix(i,:)) == 1
                % Make sure the current_cell array keeps the same cell
                % number for this connection
                
            % The connection split: IN condition
            elseif sum(adj_matrix(i,:)) > 1
                % Check how many new cells are produced and track
                for j = 1:sum(adj_matrix(i,:))
                    cell_counter = cell_counter + 1;
                    insertion = [insertion, cell_counter];
                end
                
                % Insert the new cells in order into the current cells
                % array
                before_insertion = current_cells(1:i-1);
                after_insertion = current_cells(i+1:size(current_cells,2));
                current_cells = [before_insertion,insertion,after_insertion];
                
            end
        end

        % Compare slices right to left using adjacency matrix columns
        for i = 1:size(adj_matrix,2)
            % The connection joined: OUT condition
            if sum(adj_matrix(:,i)) > 1
                % Track new cells (replaces other cells this time)
                cell_counter = cell_counter + 1;
                replacement = cell_counter;
                
                % Replace cells that merged with new cell and update
                % current_cell matrix
                before_replacement = current_cells(1:i-1);
                after_replacement = current_cells(i+sum(adj_matrix(:,i)):size(current_cells,2));
                current_cells = [before_replacement, replacement, after_replacement];
                
            % A new connection formed inside an obstacle: IN condition
            elseif sum(adj_matrix(:,i)) == 0
                % add just one count to the cell_counter
                % insert this new cell in the correct place in current_cell
                % array
                cell_counter = cell_counter + 1;
                insertion = cell_counter;
                
                before_insertion = current_cells(1:i-1);
                after_insertion = current_cells(i+1:size(current_cells,2));
                current_cells = [before_insertion, insertion, after_insertion];
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
            start_fill = connections(i,1);
            end_fill = connections(i,2)-1;
            new_map(start_fill:end_fill,col) = current_cells(i);
        end
    end
        
    %% Store previous states
    last_connectivity = connectivity;
    last_connections = connections;
    
end



