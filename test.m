% fixing adjacency connections

map = [1,1,1,1,1,1,1,1,1,1,1,1
       1,0,0,0,0,0,0,0,0,0,0,1;
       1,0,0,1,1,1,0,1,1,1,0,1;
       1,0,0,1,1,1,0,1,1,1,0,1;
       1,0,0,1,1,1,0,1,1,1,0,1;
       1,0,0,0,0,0,0,1,1,1,0,1;
       1,0,0,0,0,0,0,0,0,0,0,1;
       1,1,1,1,1,1,1,1,1,1,1,1];

   
new_map = zeros(size(map));   
slice_L = map(:,3);
slice_R = map(:,4);

[connectivity_L, connections_L] = slice_connectivity(slice_L);
[connectivity_R, connections_R] = slice_connectivity(slice_R);

adj_matrix = connections_adjacency(connections_L, connections_R)

