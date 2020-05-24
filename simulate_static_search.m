function [search_result] = simulate_static_search(init_pose, opi, decomposed_map, resolution, cell_order)
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
controller.LookaheadDistance = 0.5*0.5;
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

% Search waypoint indexing and flags
cell_order_row = 1;
new_cell_seq = true;
new_waypoint = true;

%% Simulation Loop
rate = rateControl(1/sample_time);
for i = 2:numel(time_vector)    % start index at 2nd element
    % Get new search path for cell if starting a new cell
    if new_cell_seq == true
        % Get new cell sequence
        cell_seq = cell_order(cell_order_row,:);    
        cell_seq = cell_seq(:,any(cell_seq,1));  % remove zeros to truncate cell sequence
        
        % Update indices and flag
        cell_order_row = cell_order_row + 1;  % move to next cell sequence next time
        waypoint_idx = 1;  % reset waypoint index
        new_cell_seq = false;
        
        % Get boustrophedon waypoints (search path) for new cell sequence
        search_path = [];  % reset search path
        for col = 1:size(cell_seq,2)
            cell_waypoints = cell_search_path(decomposed_map, cell_seq(col), resolution);
            search_path = [search_path; cell_waypoints];  % uses appending method, can be indexed later
        end
        
        % Get search path size info
        num_waypoints = size(search_path,1);
    end
    
    % Get new waypoint from cell search path
    if new_waypoint == true
        % Get new waypoint
        waypoint = search_path(waypoint_idx,:);
        controller.Waypoints = waypoint;
        
        % Update indices and flag
        waypoint_idx = waypoint_idx + 1;
        new_waypoint = false;
    end
    
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
    
    % Check if OPI is found
    detections = detector(pose(:,i),opi);
    if ~isempty(detections)
        disp('OPI detected. Ending search.');
        search_result = true;
        break;
    end
    
    % New event if end of waypoint is reached
    dist_between = [pose(1,i),pose(2,i);search_path(waypoint_idx-1,1),search_path(waypoint_idx-1,2)];
    if pdist(dist_between,'euclidean') < waypoint_radius    % waypoint reached if within radius
        % Update
        disp('Waypoint reached.')
        
        % Check if the last cell sequence waypoint has been reached
        if waypoint_idx == num_waypoints
            % Update
            disp('End of cell sequence search path reached.')
            
            % Check if the last cell sequence has been searched
            if cell_order_row == size(cell_order,1)
                % Update
                disp('End of complete search path reached. Ending search.');
            
                % End search
                search_result = false;
                break;
            else
                % Reset and prepare
                new_waypoint = true;  % start new waypoint
                new_cell_seq = true;  % start new cell
            end
            
        else
            % Move to next waypoint
            new_waypoint = true;
        end
        
    end 
    
end


end

