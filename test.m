% Area measure
area_measure = LidarSensor;
area_measure.sensorOffset = [0,0];
area_measure.scanAngles = linspace(-pi/6,pi/6,9);
area_measure.maxRange = 5;

% Create visualizer
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,area_measure);

search_matrix = occupancyMatrix(map);

%% Simulation parameters
sampleTime = 0.1;              % Sample time [s]
initPose = [1; 3; pi/4];        % Initial pose (x y theta)
pose = initPose;

% Initialize time, input, and pose arrays
tVec = 0:sampleTime:10;         % Time array

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)       
    % Update lidar and visualization
    ranges = area_measure(pose);
    
    for i = 1:size(ranges,1)
        % Check for NaN
        if isnan(ranges(i))
            [end_unit, grid_units] = raycast(map,pose,area_measure.maxRange,area_measure.scanAngles(i));
        else
            [end_unit, grid_units] = raycast(map,pose,ranges(i),area_measure.scanAngles(i));
        end
        
        % Convert to linear indices
        row_idx = grid_units(:,1);
        col_idx = grid_units(:,2);
        size_map = size(search_matrix);
        linear_idx = sub2ind(size_map,row_idx,col_idx);
        
        % Replace raycast indices with 2 to indicate searched
        search_matrix(linear_idx) = 2;
    end
    
    viz(pose,ranges)
    waitfor(r);
end