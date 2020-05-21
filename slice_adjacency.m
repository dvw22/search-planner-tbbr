function [adjacency_matrix] = slice_adjacency(con_seg_L,con_seg_R)
%slice_adjacency Returns the adjacency matrix for two slices with segmented
%sections
%   Detailed explanation goes here

%% Initialise Matrix
adjacency_matrix = zeros(length(con_seg_L), length(con_seg_R)); 

%% Looping
for i = 1:length(con_seg_L)
    for j = 1:length(con_seg_R)
        if min(con_seg_L(i,2), con_seg_R(j,2)) - max(con_seg_L(i,1), con_seg_R(j,1)) > 0
            adjacency_matrix(i, j) = 1;
        end
    end
end

end

