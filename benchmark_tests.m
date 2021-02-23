clear

%% Map Setup
while(true)
    % Prompt user
    fprintf(['Select map (press number and enter):\n', ...
             '1: MATLAB example map\n', ...
             '2: MATLAB complex map\n', ...
             '3: Indoor easy map\n', ...
             '4: Indoor medium map\n', ...
             '5: Indoor hard map\n', ...
             '6: Outdoor easy map\n', ...
             '7: Outdoor medium map\n', ...
             '8: Outdoor hard map\n'])
    selection = input('Selection: ','s');
    selection = str2double(selection);
    
    % Process selection
    switch selection
        case 1
            load occupancy_maps/exampleMap;
            break
        case 2
            load occupancy_maps/complexMap;
            break
        case 3
            load occupancy_maps/indoorEasyMap;
            break
        case 4
            load occupancy_maps/indoorMediumMap;
            break
        case 5
            load occupancy_maps/indoorHardMap;
            break
        case 6
            load occupancy_maps/outdoorEasyMap;
            break
        case 7
            load occupancy_maps/outdoorMediumMap;
            break
        case 8
            load occupancy_maps/outdoorHardMap;
            break
        otherwise
            disp('Invalid option. Please enter a single integer from 1-8')
    end
end

% Inflate map
inflated_map = copy(map);
inflate(inflated_map,1,'grid')

%% Other Setup
% Search Robot Object
Search_robot = SearchRobot();

% Create Search Planner Object
Search_planner = OfflineSearchPlanner(inflated_map);

% Test Suite
Test_suite = SearchTestSuite(map);

% Starting Positions
Search_robot.pose = [1.25; 18.75; pi/2];  % [x, y, theta]
opi = [0.25, 0.25, 1];  % [x, y, label]

%% Plan Search Path and Measure Computation Time
tic
Search_planner.update_search_path(Search_robot.pose);
Test_suite.computation_time = toc;

%% Visualisation
% Plot Path
Search_planner.plot_search_path()

% Simulate Search
result = simulate_offline_search(Search_robot,Test_suite,Search_planner,opi);
