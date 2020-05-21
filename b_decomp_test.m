%% Testing setup
clear

map = [0,0,0,0,0,0,0
       0,0,1,1,1,0,0;
       0,1,1,1,1,1,0;
       0,1,1,0,0,0,0;
       0,0,1,1,1,1,0;
       0,0,0,0,0,0,0];
   
slice_index = 2;
slice_L = map(:,slice_index);
slice_R = map(:,slice_index+1);

%% Initialise
last_connectivity = 0;
last_connections = [];
current_cell = 1;
current_cells = [];  % tracks the cells occupying a slice

%% Loop
for col = 1:size(map,2)
    current_slice = map(:,col);  % get slice
    [connectivity, connections] = slice_connectivity(current_slice);  % check

    % New cells are produced if the previous slice had no splits in
    % connectivity
    if last_connectivity == 0
        current_cells = [];  % reset current cells

        % If there is a split in connectivity, track new cells produced
        for i = 1:size(connectivity,1)
            current_cells = [current_cells, current_cell];
            current_cell = current_cell + 1;
        end

    % ...unless the current cell also has no splits in connectivity
    elseif connectivity == 0
        current_cells = [];  % reset current cells

    % Otherwise, the slice is passing through a split region
    else
        % We use an adjacency matrix to check whether connections have
        % split or joined
        adj_matrix = connections_adjacency(last_connections, connections);
        new_cells = zeros(1,length(connections));  % initialise cells

        % Step through adjacency matrix rows to see how connections have
        % changed from left to right
        for i = 1:size(adj_matrix,1)
            % A connection does not split or join
            if sum(adj_matrix(i,:)) == 1
                % PROBABLY WRONG
               [x, y] = find(adj_matrix(i,:));
               new_cells(x(1)) = current_cells(i);
               
            % A connection has split, meaning an IN occurred
            elseif sum(adj_matrix(i,:)) > 1
                [x, y] = find(adj_matrix(i,:));
                for index = 1:size(x,2)
                    % PROBABLY WRONG
                    new_cells(x(index)) = current_cell;
                    current_cell = current_cell + 1;
                end
                
            end
        end

        % Step through adjacency matrix columns to see how connections have
        % changed from right to left
        for i = 1:size(adj_matrix,2)
            % A connection has joined, meaning an OUT occurred
            if sum(adj_matrix(:,i)) > 1
                new_cells(i) = current_cell;
                current_cell = current_cell + 1;
                
            % ???
            elseif sum(adj_matrix(:,i)) == 0
                new_cells(i) = current_cell;
                current_cell = current_cell + 1;
                
            end
        
        % Update cells list
        current_cells = new_cells;
        
        end
        
    end

    last_connectivity = connectivity;
    last_connections = connections;
    
end

