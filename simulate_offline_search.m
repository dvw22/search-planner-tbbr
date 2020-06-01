function [search_result] = simulate_offline_search(Search_robot,Test_suite,Search_planner,opi)
%simulate_static_search Simulates a mobile robot executing an offline
%search path on the map.

%% Simulation Loop Setup
% Time Array
sample_time = 0.1;  % [s]
end_time = 1800;  % Arbitrarily set to 30 minutes [s]
time_vector = 0:sample_time:end_time;

% Pose Array
pose = zeros(3,numel(time_vector));  % Initialise array, [x, y, theta] * time vector
pose(:,1) = Search_robot.pose;  % Add initial condition

% Initialise Search Map
Test_suite.add_searched_area(pose(:,1));

% Search Waypoint Indexing and Flags and Info
segment = 1;
new_segment = true;
new_waypoint = true;
num_segments = size(Search_planner.segment_idx,1);

%% Environment Setup
% Simulation Visualiser
Viz = Visualizer2D;
Viz.hasWaypoints = true;
Viz.mapName = 'map';
attachObjectDetector(Viz,Search_robot.Detector);
Viz.objectColors = [1 0 0;0 1 0;0 0 1];
Viz.objectMarkers = 'so^';

% Waypoint Reached Radius
waypoint_radius = 0.5;  % [m]

%% Simulation Loop
rate = rateControl(1/sample_time);
for i = 2:numel(time_vector)  % start index at 2nd element
    %% Segment Updating
    % Get new search path if starting a new segment
    if new_segment == true
        % Get new segment
        seg_start_idx = Search_planner.segment_idx(segment,1);
        seg_end_idx = Search_planner.segment_idx(segment,2);
        search_path = Search_planner.complete_waypoints(seg_start_idx:seg_end_idx,:);
        
        % Update indices and flag
        segment = segment + 1;  % move to next segment next time
        waypoint_idx = 1;  % reset waypoint index
        new_segment = false;
        
        % Get search path size info
        num_waypoints = size(search_path,1);
    end
    
    %% Waypoint Updating
    % Get new waypoint from cell search path
    if new_waypoint == true
        % Get new waypoint
        waypoint = search_path(waypoint_idx,:);
        Search_robot.Controller.Waypoints = waypoint;
        
        % Update indices and flag
        waypoint_idx = waypoint_idx + 1;
        new_waypoint = false;
    end
    
    %% Simulation Updating
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [ref_fwd_speed,ref_ang_speed] = Search_robot.Controller(pose(:,i-1));  % Calculate forward and angular speeds
    [wL,wR] = inverseKinematics(Search_robot.MobileRobot,ref_fwd_speed,ref_ang_speed);  % Calculate individual wheel speeds 
    
    % Compute the velocities
    [fwd_speed,ang_speed] = forwardKinematics(Search_robot.MobileRobot,wL,wR);
    body_velocity = [fwd_speed;0;ang_speed];  % Body velocities [vx;vy;w]
    world_velocity = bodyToWorld(body_velocity,pose(:,i-1));  % Convert from body coordinates to world coordinates
    
    % Perform forward discrete integration step
    world_distance = world_velocity*sample_time;  % Calculate distance moved
    pose(:,i) = pose(:,i-1) + world_distance;  % Calculate new pose
    Search_robot.pose = pose(:,i);  % Update search robot object
    
    % Update visualization
    Viz(pose(:,i),Search_robot.Controller.Waypoints,opi)
    waitfor(rate);
    
    %% Test Suite Updating
    % Add searched areas to test suite search map
    Test_suite.add_searched_area(pose(:,i))

    % Add collisions to test suite collision counter
    Test_suite.update_collision(pose(:,i))
    
    % Add current search time to test suit
    Test_suite.search_time = time_vector(i);
    
    %% OPI Checking
    % Check if OPI is found
    detections = Search_robot.Detector(pose(:,i),opi);
    if ~isempty(detections)
        % Publish info
        disp('OPI detected. Ending search.');
        
        % Search succeeded
        search_result = true;
        break;
    end
    
    %% Waypoint Checking
    % Check distance between robot and waypoint
    dist_between = [pose(1,i),pose(2,i);search_path(waypoint_idx-1,1),search_path(waypoint_idx-1,2)];
    
    % Waypoint reached if robot within radius
    if pdist(dist_between,'euclidean') < waypoint_radius    
        % Publish info
        disp(['Waypoint ',num2str(waypoint_idx-1),'/',num2str(num_waypoints),' in segment ',num2str(segment-1),'/',num2str(num_segments),' reached.'])
                
        % Segment complete
        if waypoint_idx == num_waypoints
            % Publish info
            disp('End of segment reached.')
            
            % Last segment complete
            if segment == num_segments
                % Publish info
                disp('End of complete search path reached. Ending search.');
            
                % Search failed
                search_result = false;
                break;
            else
                % Next segment and waypoint flags
                new_waypoint = true;
                new_segment = true;
            end
            
        else
            % Next waypoint flag
            new_waypoint = true;
        end
        
    end 
    
end

end

