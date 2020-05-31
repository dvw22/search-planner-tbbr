classdef SearchTestSuite
    %SearchTestSuite Measures and collects relevant search metrics during
    %simulation.
    %   Detailed explanation goes here
    
    properties
        unoccupied_area
        searched_area
        search_coverage
        search_duration
        num_collisions
    end
    
    properties (Access = private)
        search_matrix
        
    end
    
    methods
        function obj = SearchTestSuite(occ_map)
            %SearchTestSuite Construct an instance of this class
            
            % Calculate searchable area
            obj.unoccupied_area = obj.calc_unoccupied_area(occ_map);  % [m^2]
            
            % Initialise searched area
            obj.searched_area = 0;  % [m^2]
            
            % Initialise search coverage
            obj.search_coverage = obj.searched_area / obj.unoccupied_area;  % [%]
            
            % Initialise search duration
            obj.search_duration = 0;  % [s]
            
            % Initialise collisions
            obj.num_collisions = 0;
        end
        
        function [unoccupied_area,num_unoccupied_cells] = calc_unoccupied_area(obj,occ_map)
            %unoccupied_area Calculates the total free area in the map
            
            % Transform occupancy map to binary matrix
            occ_matrix = occupancyMatrix(occ_map);
            bi_occ_matrix = round(occ_matrix);

            num_occupied_cells = sum(sum(bi_occ_matrix));

            num_unoccupied_cells = numel(bi_occ_matrix) - num_occupied_cells;  % calculate number of unsearched cells
            unoccupied_area = (1/occ_map.Resolution)^2 * num_unoccupied_cells;  % convert to area
        end
    end
end

