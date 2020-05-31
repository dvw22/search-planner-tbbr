function [search_matrix] = update_search_matrix(search_matrix,bi_occ_map,pose)
% update_search_matrix Adds the grid units currently in the object 
% detector's FoV and DoF to the search matrix
%   Within the search matrix:
%   0: unsearched
%   1: occupied
%   2: searched

%% Setup
% Object Detector sensor
detector = ObjectDetector;
detector.fieldOfView = pi/4;    % [rad]

% Camera Config
rays = 10;
scan_angles = linspace(-(detector.fieldOfView)/2,(detector.fieldOfView)/2,rays);
max_range = detector.maxRange;

% Initialise Ranges of Rays
ranges = zeros(size(scan_angles,2),1);

%% Use rays within FoV / DoF to find indices
% Get ranges of each ray in x,y coordinates
ranges_xy = rayIntersection(bi_occ_map,pose,scan_angles,max_range);

% Convert ranges of rays to euclidean distances
for i = 1:size(ranges,1)
    if isnan(ranges_xy(i,1))
        ranges(i) = max_range;
    else
        dist_between = [pose(1,1),pose(2,1);ranges_xy(i,1),ranges_xy(i,2)];
        ranges(i) = pdist(dist_between,'euclidean');
    end
end

% Find which grid units the rays intersect and update search matrix
for i = 1:size(ranges,1)
    [end_unit, grid_units] = raycast(bi_occ_map,pose,ranges(i),scan_angles(i));

    % Convert midpoints to linear indices
    row_idx = [grid_units(:,1); 0];
    col_idx = [grid_units(:,2); 0];
    % Add endpoint too
    row_idx(end) = end_unit(1,1);
    col_idx(end) = end_unit(1,2);
    linear_idx = sub2ind(bi_occ_map.GridSize,row_idx,col_idx);

    % Replace raycast indices with 2 to indicate searched
    search_matrix(linear_idx) = 2;
end

%% Post-processing: Fix Flooding into Occupied Spaces by Repopulating
% Get logical mask of occupancy map
bi_occ_matrix = occupancyMatrix(bi_occ_map);
% Replace with 1 for obstacles
search_matrix(bi_occ_matrix) = 1;

end

