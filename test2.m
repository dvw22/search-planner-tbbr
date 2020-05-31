% Occupancy matrix from map
load exampleMap;
% load complexMap;
occ_matrix = occupancyMatrix(map);
bi_occ_matrix = round(occ_matrix);  % convert to binary
resolution = map.Resolution;

% Binary occupancy map from matrix
bi_occ_map = binaryOccupancyMap(bi_occ_matrix,resolution);

% Lidar
% Object
area_measure = LidarSensor;
area_measure.sensorOffset = [0,0];
area_measure.scanAngles = linspace(-pi/6,pi/6,6);
area_measure.maxRange = 5;
% Homemade
scan_angles = linspace(-pi/6,pi/6,6);
max_range = 5;
ranges = zeros(size(scan_angles,2),1);

% Pose
initPose = [1; 3; pi/4];        % Initial pose (x y theta)
pose = initPose;

% Initialise Search Matrix
search_matrix = bi_occ_matrix;
size_map = size(search_matrix);

% Create visualizer
viz = Visualizer2D;
attachLidarSensor(viz,area_measure);


% Get ranges
ranges_xy = rayIntersection(map,pose,scan_angles,max_range);
% Convert to distance between
for i = 1:size(ranges,1)
    if isnan(ranges_xy(i,1))
        ranges(i) = 5;
    else
        dist_between = [pose(1,1),pose(2,1);ranges_xy(i,1),ranges_xy(i,2)];
        ranges(i) = pdist(dist_between,'euclidean');
    end
end

% Update search map
for i = 1:size(ranges,1)
    % Check for NaN
    if isnan(ranges(i))
        [end_unit, grid_units] = raycast(map,pose,area_measure.maxRange,area_measure.scanAngles(i));
    else
        [end_unit, grid_units] = raycast(map,pose,ranges(i),scan_angles(i));
    end

    % Convert to linear indices
    row_idx = grid_units(:,1);
    col_idx = grid_units(:,2);
    linear_idx = sub2ind(size_map,row_idx,col_idx);

    % Replace raycast indices with 2 to indicate searched
    search_matrix(linear_idx) = 2;
end

% Visualise
search_map = binaryOccupancyMap(search_matrix,resolution);
% viz.mapName = 'map';
viz.mapName = 'search_map';
viz(pose,ranges)