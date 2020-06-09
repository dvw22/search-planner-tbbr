% Pre-process Map

% The map resolution is currently 10 (10 grid units per meter)
% I want a resolution of 2 (2 grid units per meter)
% This is the same as resizing the image so that it is 5x smaller
% However, my matrix uses occupancy map values
% I should try to use the resize function to resize the map iteratively and
% then the desired result is achieved.

% A manual resize will be used.
% By selecting every other row/column element, the size will be halved


%% Setup

clear

% Initialise
load 'lidarMap'

% Convert to binary occupancy map
occ_matrix = occupancyMatrix(map);
if map.DefaultValue < map.OccupiedThreshold
    bi_occ_matrix = occ_matrix >= map.DefaultValue;
else
    bi_occ_matrix = occ_matrix >= map.OccupiedThreshold;
end

% Inflate map by robot dimension
inflate(map,2,'grid');

%% Remove gaps/noise
% Close noise gaps
structuring_element = strel('square',2);
processed_matrix = imclose(bi_occ_matrix,structuring_element);
% Remove noise obstacles
structuring_element = strel('square',3);
processed_matrix = imopen(bi_occ_matrix,structuring_element);

processed_occ_map = occupancyMap(processed_matrix,map.Resolution);

% Inflate map by robot dimension
inflate(processed_occ_map,2,'grid');

show(processed_occ_map)


%% Reduce resolution

% Reduction variable
reduction = 5;

% Get map matrix
num_row = size(bi_occ_matrix,1);
num_col = size(bi_occ_matrix,2);

% Initialise halved occ_matrix
reduced_matrix = zeros(num_row/reduction,num_col/reduction);
i = 1;
j = 1;

for row = 1:reduction:num_row
    for col = 1:reduction:num_col
        reduced_matrix(i,j) = processed_matrix(row,col);
        j = j+1;
    end
    
    % Increment inner loop row counter
    i = i+1;
    % Reset inner loop col counter
    j = 1;
end

reduced_occ_map = occupancyMap(reduced_matrix,map.Resolution/reduction);
inflate(reduced_occ_map,1,'grid');

show(reduced_occ_map)





