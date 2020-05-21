%% Testing setup
clear

map = [0,0,0,0,0,0,0
       0,0,1,1,1,0,0;
       0,1,1,1,1,1,0;
       0,1,1,1,1,1,0;
       0,0,0,0,0,0,0];
   
slice_index = 1;
slice_L = map(:,slice_index);
slice_R = map(:,slice_index+1);

% Initialise
last_connectivity = 0;
last_connections = [];
current_cell = 1;
current_cells = [];

current_slice = slice_R;  % get slice
[connectivity, connections] = slice_connectivity(current_slice)  % check