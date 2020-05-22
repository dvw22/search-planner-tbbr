function [cell_paths] = generate_paths(decomposed_map)
% generate_paths Generates boustrophedon paths in each cell in a
% boustrophedon cell decomposed occupancy map
%   Detailed explanation goes here

cell = 1;

% Zero all other cells
zeroed_except_cell = decomposed_map==cell;
col_cut = zeroed_except_cell(:,any(zeroed_except_cell,1));  % cut zero columns
truncated_cell = col_cut(any(col_cut,2),:);  % cut zero rows



