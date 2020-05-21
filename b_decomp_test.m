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
        % We use an adjacency matrix to 
        adj_matrix = connections_adjacency(last_connections, connections)
        new_cells = zeros(1,length(connections))  % initialise cells

        for i = 1:size(adj_matrix,1)
            if sum(adj_matrix(i,:)) == 1
                adj_matrix
            end
        end

    end

    last_connectivity = connectivity;
    last_connections = connections;
    
end


