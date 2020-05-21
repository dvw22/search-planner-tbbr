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
    
    adjacency_matrix = connections_adjacency(last_connections,connections);
     
    %% Determine slice splitting/joining
    if connectivity > last_connectivity
        % New cell(s) has/have been formed
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



