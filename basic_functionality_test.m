clear

%% Map Setup
load occupancy_maps/basicMap;

%% Other Setup
% Search Robot Object
Search_robot = SearchRobot();

% Test Suite
Test_suite = SearchTestSuite(map);

% Starting Positions
Search_robot.pose = [13; 10; pi/4];  % [x, y, theta]
opi = [3, 11, 1];  % [x, y, label]

% Custom Waypoints
movement_path = [14,11;
                 14.5,12;
                 14,13;
                 13,13.5;
                 12,13;
                 11.5,12;
                 12,11;
                 13,10;
                 14,9;
                 14.5,8;
                 14,7
                 13,6.5;
                 12,7;
                 11.5,8;
                 12,9;
                 13,10
                 
                 10,16
                 
                 6.1,13;
                 7,10.5;
                 6.1,8;
                 
                 8,4;

                 3,10];

%% Simulation Loop Setup
% Time Array
sample_time = 0.1;  % [s]
end_time = 1800;  % Arbitrarily set to 30 minutes [s]
time_vector = 0:sample_time:end_time;

% Pose Array
pose = zeros(3,numel(time_vector));  % Initialise array, [x, y, theta] * time vector
pose(:,1) = Search_robot.pose;  % Add initial condition

% Initialise Waypoint Flag and Index
new_waypoint = true;
next_waypoint_idx = 1;
last_num_collisions = 0;

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
    %% Waypoint Updating
    % Get new waypoint from cell search path
    if new_waypoint == true
        % Get new waypoint
        waypoint = movement_path(next_waypoint_idx,:);
        Search_robot.Controller.Waypoints = waypoint;
        
        % Update indices and flag
        next_waypoint_idx = next_waypoint_idx + 1;
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
    % Add collisions to test suite collision counter
    Test_suite.update_collision(pose(:,i))
    if Test_suite.num_collisions > last_num_collisions
        % Publish info
        disp(['Collision detected. Total collisions = ', num2str(Test_suite.num_collisions),'.']);
        
        % Update counter
        last_num_collisions = last_num_collisions + 1;
    end
    
    %% OPI Checking
    % Check if OPI is found
    detections = Search_robot.Detector(pose(:,i),opi);
    if ~isempty(detections)
        % Publish info
        range = detections(1,1);
        angle = detections(1,2);
        disp(['OPI detected ',num2str(range),'m away at angle ',num2str(angle),'rad. Ending search.']);
        
        % Search succeeded
        search_result = true;
        break;
    end
    
    %% Waypoint Checking
    % Check distance between robot and waypoint
    dist_between = [pose(1,i),pose(2,i);movement_path(next_waypoint_idx-1,1),movement_path(next_waypoint_idx-1,2)];
    
    % Waypoint reached if robot within radius
    if pdist(dist_between,'euclidean') < waypoint_radius    
        % Publish info
        disp(['Waypoint ',num2str(next_waypoint_idx-1),'/',num2str(size(movement_path,1)),' reached.'])

        % Check if last waypoint is found
        if next_waypoint_idx==size(movement_path,1)+1
            % Publish info
            disp('Last waypoint reached. Ending search.');
            
            % Search failed
            search_result = false;
            break;
        else
            % Next waypoint flag
            new_waypoint = true;
        end
    end 
end
