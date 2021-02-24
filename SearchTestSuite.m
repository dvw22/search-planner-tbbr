classdef SearchTestSuite < handle
    %SearchTestSuite Measure search performance during simulation
    %   This class is a handle because the methods are designed to update 
    %   the object's properties. 
    %
    %   A SearchTestSuite object measures and collects relevant search 
    %   metrics during a search simulation. It tracks the search coverage
    %   of the robot and detects collisions with the environment. Other
    %   metrics such as search planner computation time and search duration
    %   are also tracked.
    
    properties
        search_time  % duration of completed search 
        computation_time  % time for complete search path generation
    end
    
    properties (SetAccess = private)
        bi_occ_map  % binary occupancy map object
        unoccupied_area  % area that can be visited by the robot (m^2)
        search_coverage  % percentage of available area searched (%)
        num_collisions  % tracks robot collisions with obstacles
        search_map  % displays area searched via an occupancy map
    end
    
    properties (Access = private)
        searched_area  % area covered by the object detector (m^2)
        search_matrix  % tracks area searched in the map
        last_collision  % collision count reset flag
        search_occ_matrix  % normalised search_matrix (0 to 1)
    end
    
    methods
        function obj = SearchTestSuite(occ_map)
            %SearchTestSuite Construct an instance of this class
            %   The object must be constructed with an occupancy map
            arguments
                occ_map (1,1) occupancyMap 
            end
            
            % Convert to binary occupancy map
            occ_matrix = occupancyMatrix(occ_map);
            bi_occ_matrix = occ_matrix >= occ_map.DefaultValue;  % convert to binary matrix
            bi_occ_map = binaryOccupancyMap(bi_occ_matrix,occ_map.Resolution);
            
            % Store binary occupancy map
            obj.bi_occ_map = bi_occ_map;
            
            % Calculate searchable area
            obj.unoccupied_area = obj.calc_unoccupied_area();  % [m^2]
            
            % Initialise searched area
            obj.searched_area = 0;  % [m^2]
            
            % Initialise search coverage
            obj.search_coverage = obj.searched_area / obj.unoccupied_area;  % [%]
            
            % Initialise search duration
            obj.search_time = 0;  % [s]
            
            % Initialise collisions
            obj.num_collisions = 0;
            
            % Initialise search matrix
            obj.search_matrix = double(occupancyMatrix(bi_occ_map));
            
            % Initialise search occupancy matrix
            obj.search_occ_matrix = obj.search_matrix;
            obj.search_occ_matrix(obj.search_matrix==2) = 0.5;
            
            % Initialise search map
            obj.search_map = occupancyMap(obj.search_occ_matrix,obj.bi_occ_map.Resolution);
            
            % Initialise computation speed
            obj.computation_time = 0;  % [s]
            
            % Initialise last collision
            obj.last_collision = false;
        end
        
        function [unoccupied_area] = calc_unoccupied_area(obj)
            %unoccupied_area Calculates the total free area in the map
            %   unoccupied_area - area visitable by the robot (m^2)
            
            bi_occ_matrix = occupancyMatrix(obj.bi_occ_map);
            
            num_occupied_units = sum(sum(bi_occ_matrix));

            num_unoccupied_units = numel(bi_occ_matrix) - num_occupied_units;  % calculate number of unsearched cells
            unoccupied_area = (1/obj.bi_occ_map.Resolution)^2 * num_unoccupied_units;  % convert to area
        end
        
        function add_searched_area(obj,pose)
            %update_search_matrix Add searched area at robot pose
            %   pose - robot's current pose [x,y,angle]
            %
            %   Adds the grid units currently in the mobile robot's object 
            %   detector's Field of View (FoV) and Depth of Field (DoF) to 
            %   the search matrix.
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
            ranges_xy = rayIntersection(obj.bi_occ_map,pose,scan_angles,max_range);

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
                [end_unit, grid_units] = raycast(obj.bi_occ_map,pose,ranges(i),scan_angles(i));

                % Convert midpoints to linear indices
                row_idx = [grid_units(:,1); 0];
                col_idx = [grid_units(:,2); 0];
                % Add endpoint too
                row_idx(end) = end_unit(1,1);
                col_idx(end) = end_unit(1,2);
                linear_idx = sub2ind(obj.bi_occ_map.GridSize,row_idx,col_idx);

                % Replace raycast indices with 2 to indicate searched
                obj.search_matrix(linear_idx) = 2;
            end

            %% Post-processing: Fix Flooding into Occupied Spaces by Repopulating
            % Get logical mask of occupancy map
            bi_occ_matrix = occupancyMatrix(obj.bi_occ_map);
            % Replace with 1 for obstacles
            obj.search_matrix(bi_occ_matrix) = 1;
            
            % Update search coverage statistic
            obj.update_search_coverage();
            % Update search map
            obj.update_search_map();
            
        end
        
        function update_collision(obj,pose)
            %update_collision Check potential collisions at position
            %   pose - robot's current pose [x,y,angle]
            %
            %   Using the robot's position and the occupancy map, this
            %   function detects whether a new collision has occurred. A
            %   new collision occurs when the robot moves from an
            %   unoccupied space to an occupied space in the map.
            
            % Pose is in an occupied space
            if getOccupancy(obj.bi_occ_map,[pose(1),pose(2)])==1
                % Add to counter if wasn't in a collision previously
                if obj.last_collision == false
                    obj.num_collisions = obj.num_collisions + 1;
                end

                % Update last collision flag
                obj.last_collision = true;
                
            % Pose is not in an occupied space
            else
                obj.last_collision = false;
            end
        end
    end
    
    methods (Access = private)
        function update_search_coverage(obj)
            %update_search_coverage Calculates the current search coverage
            
            obj.update_searched_area()
            
            % Calculate search coverage
            obj.search_coverage = 100 * (obj.searched_area/obj.unoccupied_area);
        end
        
        function update_searched_area(obj)
            %update_searched_area Calculates the current total searched area
            
            % Get logical mask of search grid units
            searched = obj.search_matrix==2;
            
            num_searched_units = sum(sum(searched));

            obj.searched_area = (1/obj.bi_occ_map.Resolution)^2 * num_searched_units;  % convert to area
        end
        
        function update_search_map(obj)
            %update_search_map Updates the search occupancy matrix and map
            
            obj.search_occ_matrix(obj.search_matrix==2) = 0.5;
            obj.search_map = occupancyMap(obj.search_occ_matrix,obj.bi_occ_map.Resolution);
        end
    end
end

