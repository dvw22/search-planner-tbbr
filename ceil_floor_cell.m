function [cell_ceiling_idx,cell_floor_idx] = ceil_floor_cell(decomposed_map,cell)
% ceil_floor_cell Generates boustrophedon paths in each cell in a
% boustrophedon cell decomposed occupancy map
%   Returns indices of matrix [row1,col1; row2,col2; ...]

% Truncate matrix to cell of interest size
zeroed_except_cell = decomposed_map==cell;  % zero all other cells
col_cut = zeroed_except_cell(:,any(zeroed_except_cell,1));  % cut zero columns
truncated_cell = col_cut(any(col_cut,2),:);  % cut zero rows

% Find cell length and height
cell_length = size(truncated_cell,2);

% Initialise ceiling and floor arrays to track indices
cell_ceiling_idx = zeros(cell_length,2);  % [row1,row1; col2,col2; ...]
cell_floor_idx = cell_ceiling_idx;

% Find ceiling offset of cell
for i = 1:size(zeroed_except_cell,1)
    if ~all(zeroed_except_cell(i,:)==0)
        ceiling_offset = i-1;
        break
    end
end

% Find left wall offset of cell
for i = 1:size(zeroed_except_cell,2)
    if ~all(zeroed_except_cell(:,i)==0)
        leftwall_offset = i-1;
        break
    end
end

% Look for ceiling and floor indices within cell
for col = 1:cell_length
    % Look down through each row until top of cell is found in column slice
    for i = 1:size(truncated_cell,1)
        if truncated_cell(i,col)==1
            cell_ceiling_idx(col,1) = i + ceiling_offset;  % store row index
            cell_ceiling_idx(col,2) = col + leftwall_offset;  % store col index
            break
        end
    end

    % Look up through each row until bottom of cell is found in column
    % slice
    for i = size(truncated_cell,1):-1:1
        if truncated_cell(i,col)==1
            cell_floor_idx(col,1) = i + ceiling_offset;  % store row index
            cell_floor_idx(col,2) = col + leftwall_offset;  % store col index
            break
        end
    end
end

end





