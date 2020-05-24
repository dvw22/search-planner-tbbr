function [search_result] = simulate_static_search(init_pose, opi, map_waypoints, segment_idx)
%simulate_static_search Simulates a mobile robot statically executing a search path on the map.
%   Detailed explanation goes here

%% Environment Setup
% Object Detector sensor
detector = ObjectDetector;
detector.fieldOfView = pi/4;    % [rad]

% Simulation Visualiser
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachObjectDetector(viz,detector);
viz.objectColors = [1 0 0;0 1 0;0 0 1];
viz.objectMarkers = 'so^';

%% Vehicle Setup
% Define Vehicle
wheel_radius = 0.1;                        % [m]
wheel_base = 0.5;                        % [m]
mobile_robot = DifferentialDrive(wheel_radius,wheel_base);

% Path Following Controller
controller = controllerPurePursuit;
controller.LookaheadDistance = 0.5*0.25;
controller.DesiredLinearVelocity = 0.75*2;
controller.MaxAngularVelocity = 1.5*16;
waypoint_radius = 0.5;

%% Simulation Loop Setup

% Time Array
sample_time = 0.1;              % [s]
end_time = 600;                  % Arbitrarily set to 10 minutes [s]
time_vector = 0:sample_time:end_time;

% Pose Array
pose = zeros(3,numel(time_vector));   % Initialise array, [x, y, theta] x time vector
pose(:,1) = init_pose;          % Add initial condition

% Search waypoint indexing and flags and info
segment = 1;
new_segment = true;
new_waypoint = true;
num_segments = size(segment_idx,1);

%% Simulation Loop
rate = rateControl(1/sample_time);
for i = 2:numel(time_vector)    % start index at 2nd element
    %% Segment Updating
    % Get new search path if starting a new segment
    if new_segment == true
        % Get new segment
        seg_start_idx = segment_idx(segment,1);
        seg_end_idx = segment_idx(segment,2);
        search_path = map_waypoints(seg_start_idx:seg_end_idx,:);
        
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
        controller.Waypoints = waypoint;
        
        % Update indices and flag
        waypoint_idx = waypoint_idx + 1;
        new_waypoint = false;
    end
    
    %% Simulation Updating
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [ref_fwd_speed,ref_ang_speed] = controller(pose(:,i-1));          % Calculate forward and angular speeds
    [wL,wR] = inverseKinematics(mobile_robot,ref_fwd_speed,ref_ang_speed);        % Calculate individual wheel speeds 
    
    % Compute the velocities
    [fwd_speed,ang_speed] = forwardKinematics(mobile_robot,wL,wR);
    body_velocity = [fwd_speed;0;ang_speed]; % Body velocities [vx;vy;w]
    world_velocity = bodyToWorld(body_velocity,pose(:,i-1));  % Convert from body coordinates to world coordinates
    
    % Perform forward discrete integration step
    world_distance = world_velocity*sample_time;    % Calculate distance moved
    pose(:,i) = pose(:,i-1) + world_distance;    % Calculate new pose
    
    % Update visualization
    viz(pose(:,i),controller.Waypoints,opi)
    waitfor(rate);
    
    %% OPI Checking
    % Check if OPI is found
    detections = detector(pose(:,i),opi);
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

