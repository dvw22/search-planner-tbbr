function [connectivity,connections] = slice_connectivity(slice)
% slice_connectivity Calculates the number of connected segments and
% returns their indices
%   If the previous index is a 0 and this one is a 1, a connectivity has
%   opened. Log this point as a start point. Otherwise, if the previous 
%   index is a 1 and this one is a 0 and a connectivity was opened, a 
%   connectivity has closed. Log this point as an end point. 
%   Try to track the number of connectivities. The start and end of a
%   connectivity should occupy a row.

%% Initialise
connectivity = 0;
last_data = 0;
open_part = false;
connections = []; % unknown length

%% Loop Through
for i = 1:length(slice)
    data = slice(i);
    
    % Connectivity opens
    if data == 0 && open_part == false
        open_part = true;
        connectivity = connectivity + 1;
        start_point = i;
    % Connectivity closes
    elseif last_data == 0 && data == 1 && open_part == true
        open_part = false;
        end_point = i;
        connections(connectivity, 1) = start_point;
        connections(connectivity, 2) = end_point;
    % Logs the part closing if it reaches the end of the slice
    elseif i == length(slice) && open_part == true
        open_part = false;
        end_point = i;
        connections(connectivity, 1) = start_point;
        connections(connectivity, 2) = end_point;
    end
        
    % Store data
    last_data = data;
    
end
        
end

