%% Testing setup
clear

map = [0,1,1,1,1,1,1,1,1,1;
       0,0,1,1,1,1,1,1,1,1;
       0,0,0,0,1,1,0,0,1,1;
       0,0,1,0,1,1,0,0,1,1;
       0,0,1,1,1,1,1,1,1,1;
       0,1,1,1,1,0,0,0,0,1];

slice_L = map(:,1);
slice_R = map(:,4);


[connectivity, connections] = slice_connectivity(slice_R)



