function [adjacency_matrix] = connections_adjacency(connections_L,connections_R)
% connections_adjacency Returns the adjacency matrix for two neighbouring
% connectivity slices
%   Detailed explanation goes here

%% Initialise Matrix
adjacency_matrix = zeros(length(connections_L), length(connections_R)); 

%% Looping
for i = 1:length(con_seg_L)
    for j = 1:length(con_seg_R)
        if min(connections_L(i,2), connections_R(j,2)) - max(connections_L(i,1), connections_R(j,1)) > 0
            adjacency_matrix(i, j) = 1;
        end
    end
end

end

