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
cell_counter = 1;
current_cells = [];

for col = 1:size(map,2)
    %% Check connectivity of the slice
    [connectivity, connections] = slice_connectivity(map(:,col));
    
    %% Compare slices using adjacency matrix
    adj_matrix = connections_adjacency(last_connections,connections);
     
    %% Determine slice splitting/joining
    % Each individual connection in the slice can be checked for splitting
    % or joining. This is done using the adjacency matrix. The sum of a row
    % indicates how many right slice connections a left slice connections
    % is adjacent to. The sum of a column indicates how many left slice
    % connections a right slice connection is adjacent to.
    
    % Compare slices left to right using adjacency matrix row
    for i = 1:size(adj_matrix,1)
        % The connection does not split
        if sum(adj_matrix(i,:)) == 1
            sum(adj_matrix(i,:))
        % The connection split: IN condition
        elseif sum(adj_matrix(i,:)) > 1
            sum(adj_matrix(i,:))
        end
    end
    
    % Compare slices right to left using adjacency matrix columns
    for i = 1:size(adj_matrix,2)
        % The connection joined: OUT condition
        if sum(adj_matrix(:,i)) > 1
            sum(adj_matrix(:,i))
        % A new connection formed inside an obstacle: IN condition
        elseif sum(adj_matrix(:,i)) == 0
            sum(adj_matrix(:,i))
        end
    end
    
    % Update map
    % No connectivity means a full line of obstacles
    if connectivity == 0
        new_map(:,col) = 0;  % Obstacles are 0
    else
        for i = 1:size(connections,1)
            % Fill connectivity segments with corresponding cell numbers
        end
    end
        
    last_connectivity = connectivity;
    last_connections = connections;
    
end



