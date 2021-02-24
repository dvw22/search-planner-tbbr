classdef OfflineSearchPlanner < handle
    %SearchPlanner Robot planner for generating search paths in a map
    %   This class is a handle because the methods are designed to update 
    %   the object's properties. 
    %
    %   This is a 2D offline search planner so paths are planned once for a 
    %   complete, static map. Objects require an occupancy map during 
    %   instantiation, after which methods can be called to generate and
    %   interpret a search path. A search path is an ordered set of 2D 
    %   waypoints that can be provided to a mobile robot's mobility system.
    
    properties (SetAccess = private)
        bi_occ_map  % binary occupancy map of area to plan path in
        decomposed_matrix  % cell decomposed map (0=obstacles, 1+=cells)
        graph  % graph of cells in occupancy map
        num_cells  % number of cells in occupancy map
        cell_order  % order to search cells in occupancy map
        complete_waypoints  % complete list of search path waypoints
        segment_idx  % indices of path segments in complete search path
        init_pose  % initial pose of robot in occupancy map
        PathPlanner  % shortest path planner object
    end
    
    methods
        function obj = OfflineSearchPlanner(occ_map)
            %SearchPlanner Construct an instance of this class
            %   The object must be constructed with an occupancy map
            arguments
                occ_map (1,1) occupancyMap 
            end
            
            % Convert map to binary map
            occ_matrix = occupancyMatrix(occ_map);
            bi_occ_matrix = occ_matrix >= occ_map.DefaultValue;  % convert to binary matrix
            bi_occ_map = binaryOccupancyMap(bi_occ_matrix,occ_map.Resolution);
            
            % Store binary occupancy map
            obj.bi_occ_map = bi_occ_map;
            
            % Initialise cell decomposition properties
            obj.decomposed_matrix = [];
            obj.graph = [];
            obj.num_cells = [];
            
            % Initialise cell order properties
            obj.cell_order = [];
            
            % Initialise search path properties
            obj.complete_waypoints = [];
            obj.segment_idx = [];
            
            % Initialise start pose
            obj.init_pose = [];
            
            % Initialise shortest path planner
            obj.PathPlanner = mobileRobotPRM(bi_occ_map);
            obj.PathPlanner.NumNodes = 75*24;
            obj.PathPlanner.ConnectionDistance = 1;
        end 
        
        function update_search_path(obj,init_pose)
            %update_search_path Generate a search path within a map
            %   init_pose - starting pose of robot [x, y, angle]
            %
            %   Calculates and stores a set of search path waypoints within
            %   the "complete_waypoints" property.
            
            % Save init pose
            obj.init_pose = init_pose;
            
            % Perform cell decomposition of map
            [obj.decomposed_matrix, obj.graph, obj.num_cells] = obj.btd_cell_decomposition();
            
            % Perform cell order calculation
            obj.cell_order = obj.plan_cell_order();
            
            % Calculate waypoints in map
            [obj.complete_waypoints, obj.segment_idx] = obj.complete_search_path();
        end
        
        function plot_search_path(obj)
            %plot_path Plots the path as waypoints on the occupancy map
            %   The comet function is used to animate the search path and
            %   indicate their order on the occupancy map.
            
            % Display map
            figure, show(obj.bi_occ_map)
            hold on
            % Plot points
            scatter(obj.complete_waypoints(:,1),obj.complete_waypoints(:,2),25,'r','x')
            % Plot waypoint connection animation
            comet(obj.complete_waypoints(:,1),obj.complete_waypoints(:,2))
            hold off
        end
        
        function show_decomposed_map(obj)
            %display_decomposed_map Display segmented map for path planning
            %   Displays the 'cells' of a decomposed map generated during
            %   Boustrophedon cell decomposition as different greyscale
            %   shades
            
            % Scale cell numbers to values between 0 and just under 1
            max_cell_value = 0.9;
            divisor = obj.num_cells/max_cell_value;
            scaled_decomposed = obj.decomposed_matrix/divisor;
            
            % Transform 0s (obstacles) in decomposed matrix into 1s
            scaled_decomposed(scaled_decomposed==0) = 1;
            
            % Create map object and show
            decomposed_map = occupancyMap(scaled_decomposed,obj.bi_occ_map.Resolution);
            figure, show(decomposed_map);
        end
    end
    
    methods (Access = private)
        function [complete_waypoints,segment_idx] = complete_search_path(obj)
            %map_search_path Combine and generate complete search path
            %   complete_waypoints - ordered list of waypoints [[x,y], ...]
            %   segment_idx - location of segments in complete waypoints
            %
            %   Outputs a list of complete waypoint together with a list of
            %   'segment' locations in the complete list. The complete 
            %   search path is built from 'segments', which are either a 
            %   set of waypoints generated within along a sequence of 
            %   'cells' or a shortest path connecting 'cells'
            
            % Transform pose to just x,y position
            init_xy = [obj.init_pose(1), obj.init_pose(2)];
            
            
            %% Waypoint Generation
            % Initialise
            complete_waypoints = [];
            segment_idx = [];
            num_cell_seq = size(obj.cell_order,1);

            last_end_idx = 0;
            start_idx = 1;

            % Get waypoints for each cell sequence and path between and append
            for i = 1:num_cell_seq
                % Calculate waypoints before travel first time only
                if i == 1
                    % Append cell sequence waypoints
                    [cell_seq_waypoints, ~] = obj.cell_seq_search_path(obj.cell_order(i,:));
                    complete_waypoints = [complete_waypoints; cell_seq_waypoints];

                    % Append cell sequence indices
                    num_waypoints = size(cell_seq_waypoints,1);
                    end_idx = last_end_idx + num_waypoints;
                    segment_idx = [segment_idx; [start_idx, end_idx]];
                    start_idx = end_idx + 1;  % store
                    last_end_idx = end_idx;  % store

                    % Plan path to first waypoint
                    travel_waypoints = findpath(obj.PathPlanner,init_xy,complete_waypoints(1,:));
                    % The planner may not always find a path if the map is too complex.
                    % This condition is to prevent the code from breaking.
                    if isempty(travel_waypoints) == 0
                        travel_waypoints(1,:) = [];  % Must clear source
                        travel_waypoints(end,:) = [];  % Must clear destination of travel to avoid duplicate
                    else
                        disp(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
                    end
                    complete_waypoints = [travel_waypoints; complete_waypoints];

                    % Append travel indices
                    num_waypoints = size(travel_waypoints,1);
                    end_idx = last_end_idx + num_waypoints;
                    segment_idx = [segment_idx; [start_idx, end_idx]];
                    start_idx = end_idx + 1;  % store
                    last_end_idx = end_idx;  % store

                % Just finished calculating a travel segment
                else
                    % Append cell sequence waypoints
                    complete_waypoints = [complete_waypoints; cell_seq_waypoints];

                    % Append cell sequence indices
                    num_waypoints = size(cell_seq_waypoints,1);
                    end_idx = last_end_idx + num_waypoints;
                    segment_idx = [segment_idx; [start_idx, end_idx]];
                    start_idx = end_idx + 1;  % store
                    last_end_idx = end_idx;  % store
                end

                % Append travel waypoints
                if i < num_cell_seq
                    % Get next cell sequence waypoints
                    [next_cell_seq_waypoints, ~] = obj.cell_seq_search_path(obj.cell_order(i+1,:));

                    % Get start and end points for travel
                    start_waypoint = cell_seq_waypoints(end,:);  % starting at end of last cell sequence
                    end_waypoint = next_cell_seq_waypoints(1,:);  % ending at start of next cell sequence

                    % Append travel waypoints
                    travel_waypoints = findpath(obj.PathPlanner,start_waypoint,end_waypoint);
                    % The planner may not always find a path if the map is too complex.
                    % This condition is to prevent the code from breaking.
                    while isempty(travel_waypoints) == 1
                        % Publish info
                        disp(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
                        disp('Increasing nodes.')

                        % Increase nodes and connection distances
                        obj.PathPlanner.NumNodes = obj.PathPlanner.NumNodes + 100;

                        % Calculate again
                        travel_waypoints = findpath(obj.PathPlanner,start_waypoint,end_waypoint);
                    end    
                    travel_waypoints(1,:) = [];  % Must clear source of travel to avoid duplicate
                    travel_waypoints(end,:) = [];  % Must clear destination of travel to avoid duplicate
                    complete_waypoints = [complete_waypoints; travel_waypoints];

                    % Append travel indices
                    num_waypoints = size(travel_waypoints,1);
                    end_idx = last_end_idx + num_waypoints;
                    segment_idx = [segment_idx; [start_idx, end_idx]];
                    start_idx = end_idx + 1;  % store
                    last_end_idx = end_idx;  % store

                    % Store next cell sequence waypoints
                    cell_seq_waypoints = next_cell_seq_waypoints;
                end
            end
        end
        
        function [cell_seq_waypoints, num_cells_in_seq] = cell_seq_search_path(obj,cell_seq)
            % cell_seq_search_path Connect adjacent cell sequence waypoints
            %   cell_seq - list of adjacent cells [int,int,int,...,0,0,0]
            %   
            %   cell_waypoints - ordered list of cell waypoints [[x,y],...]
            %   num_waypoints - number of waypoints in the cell
            % 
            %   Outputs a list of waypoints connecting a sequence of cells.

            %% Preparation
            % Slice off unnecessary zeros in cell_seq
            cell_seq = cell_seq(:,any(cell_seq,1));

            %% Initialise
            % Initialise
            cell_seq_waypoints = [];  % reset search path
            num_cells_in_seq = size(cell_seq,2);

            %% Waypoint Generation
            % Get waypoints for each cell and append
            for i = 1:num_cells_in_seq
                % First cell calculation
                if i == 1
                    % Get first cell waypoints
                    [cell_waypoints, ~] = obj.cell_search_path(cell_seq(i));

                    % Append to cell sequence
                    cell_seq_waypoints = [cell_seq_waypoints; cell_waypoints];   
                % Just finished calculating a travel segment
                else
                    % Append previously stored cell waypoints
                    cell_seq_waypoints = [cell_seq_waypoints; cell_waypoints];
                end
                
                % Calculate path to start waypoint of next cell if not last
                % cell
                if i < num_cells_in_seq
                    % Get next cell waypoints 
                    [next_cell_waypoints, ~] = obj.cell_search_path(cell_seq(i+1));
                    
                    % Get start and end points for travel
                    start_waypoint = cell_waypoints(end,:);  % starting at end of last cell sequence
                    end_waypoint = next_cell_waypoints(1,:);  % ending at start of next cell sequence
                   
                    % Append travel waypoints
                    travel_waypoints = findpath(obj.PathPlanner,start_waypoint,end_waypoint);
                    % The planner may not always find a path if the map is too complex.
                    % This condition is to prevent the code from breaking.
                    while isempty(travel_waypoints) == 1
                        % Publish info
                        disp(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
                        disp('Increasing nodes.')

                        % Increase nodes and connection distances
                        obj.PathPlanner.NumNodes = obj.PathPlanner.NumNodes + 100;

                        % Calculate again
                        travel_waypoints = findpath(obj.PathPlanner,start_waypoint,end_waypoint);
                    end    
                    travel_waypoints(1,:) = [];  % Must clear source of travel to avoid duplicate
                    travel_waypoints(end,:) = [];  % Must clear destination of travel to avoid duplicate
                    cell_seq_waypoints = [cell_seq_waypoints; travel_waypoints];

                    % Store next cell waypoints for next sequence
                    cell_waypoints = next_cell_waypoints;
                end
            end
        end     

        function [cell_waypoints, num_waypoints] = cell_search_path(obj,cell)
            %cell_search path Generates a search path within a cell
            %   cell - positive integer indicating number of cell
            %
            %   cell_waypoints - ordered list of cell waypoints [[x,y],...]
            %   num_waypoints - number of waypoints in the cell
            %
            %   Generates a collision-free search path within a cell. First
            %   acquires waypoints at the top and bottom of a cell, then 
            %   'zips' them together to form rectilinear Boustrophedon
            %   paths.
            %   Lastly, collision free paths are inserted between some
            %   waypoints prone to causing collisions.

            % Initialise
            cell_indices = [];  % uses appending method because easier for now
            insertion_indices = [];
            add_floor = true;  % start with floor
            
            [ceiling_idx, floor_idx] = obj.ceil_floor_cell(cell);  

            for i = 1:size(floor_idx,1)
                % The floor and ceiling are on the same row (tunnel)
                if floor_idx(i,1) == ceiling_idx(i,1)
                    % Add one of them to the cell waypoint list
                    cell_indices = [cell_indices; floor_idx(i,:)];
                    
                    % Check if there is an edge
                    if i>1
                        % Tunnel is after a floor point
                        if add_floor == true
                            % Compare floors
                            if floor_idx(i,1)~=floor_idx(i-1,1)
                                % Track index
                                insertion_indices = [insertion_indices; size(cell_indices,1)];
                            end
                        % Tunnel is after a ceiling point
                        else
                            % Compare ceilings
                            if ceiling_idx(i,1)~=ceiling_idx(i-1,1)
                                % Track index
                                insertion_indices = [insertion_indices; size(cell_indices,1)];
                            end
                        end
                    end
                else
                    % The floor index needs to be added first
                    if add_floor == true
                        % Check if there is an edge from previous floor
                        if i>1
                            if floor_idx(i,1)~=floor_idx(i-1,1)
                                % Track index
                                insertion_indices = [insertion_indices; size(cell_indices,1)+1];
                            end
                        end
                        
                        % Add pair with floor first
                        pair = [floor_idx(i,:); ceiling_idx(i,:)];
                        cell_indices = [cell_indices; pair];
                        
                        % Add ceiling next
                        add_floor = false;
                    % The ceiling index needs to be added first
                    else
                        % Check if there is an edge from previous ceiling
                        if i>1
                            if ceiling_idx(i,1)~=ceiling_idx(i-1,1)
                                % Track index
                                insertion_indices = [insertion_indices; size(cell_indices,1)+1];
                            end
                        end
                        
                        % Add pair with ceiling first
                        pair = [ceiling_idx(i,:); floor_idx(i,:)];
                        cell_indices = [cell_indices; pair];
                        
                        % Add floor next
                        add_floor = true;
                    end
                end
            end

            %% Convert from matrix indices [row, col] to map waypoints [x, y]
            cell_waypoints = cell_indices;  % [x1,y1; x2,y2; ...]
            cell_waypoints(:) = 0;

            % Rows are y reference, columns are x reference (swap)
            cell_waypoints(:,1) = cell_indices(:,2);
            cell_waypoints(:,2) = cell_indices(:,1);

            % Flip y points
            cell_waypoints(:,2) = (size(obj.decomposed_matrix,1)+1) - cell_waypoints(:,2);

            % Shift waypoints to centres of grid units
            cell_waypoints = cell_waypoints - 0.5;

            % Perform scaling for resolution
            cell_waypoints = cell_waypoints/obj.bi_occ_map.Resolution;


            %% Post processing to insert waypoint paths between edges
            insertion_offset = 0;  % initialise offset due to path insertions
            for i = 1:size(insertion_indices,1)
                % Get start and end points for travel 
                insertion_idx = insertion_indices(i);
                start_point = cell_waypoints(insertion_idx - 1 + insertion_offset,:);
                end_point = cell_waypoints(insertion_idx + insertion_offset,:);

                % Plan path between points
                inserted_path = findpath(obj.PathPlanner,start_point,end_point);
                while isempty(inserted_path) == 1
                    % Publish info
                    disp('Path planning failed between cell waypoints.')
                    disp('Increasing nodes.')

                    % Increase nodes and connection distances
                    obj.PathPlanner.NumNodes = obj.PathPlanner.NumNodes + 100;

                    % Calculate again
                    inserted_path = findpath(obj.PathPlanner,start_point,end_point);
                end
                inserted_path(1,:) = [];  % Must clear source of travel to avoid duplicate
                inserted_path(end,:) = [];  % Must clear destination of travel to avoid duplicate

                % Insert into cell_indices
                before_insertion = cell_waypoints(1:(insertion_idx+insertion_offset)-1,:);
                after_insertion = cell_waypoints(insertion_idx+insertion_offset:end,:);
                cell_waypoints = [before_insertion; inserted_path; after_insertion];

                % Track length change due to insertion for next insertion
                insertion_offset = insertion_offset + size(inserted_path,1);
            end

            % Get final number of waypoints
            num_waypoints = size(cell_waypoints,1);
        end
        
        function [cell_ceiling_idx,cell_floor_idx] = ceil_floor_cell(obj,cell)
            %ceil_floor_cell Generate waypoints on top and bottom of cell
            %   cell - positive integer indicating number of cell
            %   
            %   cell_ceiling_idx - ordered cell ceiling waypoints [x, y]
            %   cell_floor_idx - ordered cell floor waypoints [x, y]
            %
            %   After specifying a cell, waypoints are generated just below
            %   their top contour ('ceiling') and above their bottom 
            %   contour ('floor').

            % Truncate matrix to cell of interest size
            zeroed_except_cell = obj.decomposed_matrix==cell;  % zero all other cells
            col_cut = zeroed_except_cell(:,any(zeroed_except_cell,1));  % cut zero columns
            truncated_cell = col_cut(any(col_cut,2),:);  % cut zero rows

            % Find cell length and height
            cell_length = size(truncated_cell,2);

            % Initialise ceiling and floor arrays to track indices
            cell_ceiling_idx = zeros(cell_length,2);  % [row1,row1; col2,col2; ...]
            cell_floor_idx = cell_ceiling_idx;

            % Find ceiling offset of cell
            for i = 1:size(zeroed_except_cell,1)
                if ~all(zeroed_except_cell(i,:)==0)
                    ceiling_offset = i-1;
                    break
                end
            end

            % Find left wall offset of cell
            for i = 1:size(zeroed_except_cell,2)
                if ~all(zeroed_except_cell(:,i)==0)
                    leftwall_offset = i-1;
                    break
                end
            end

            % Look for ceiling and floor indices within cell
            for col = 1:cell_length
                % Look down through each row until top of cell is found in column slice
                for i = 1:size(truncated_cell,1)
                    if truncated_cell(i,col)==1
                        cell_ceiling_idx(col,1) = i + ceiling_offset;  % store row index
                        cell_ceiling_idx(col,2) = col + leftwall_offset;  % store col index
                        break
                    end
                end

                % Look up through each row until bottom of cell is found in column
                % slice
                for i = size(truncated_cell,1):-1:1
                    if truncated_cell(i,col)==1
                        cell_floor_idx(col,1) = i + ceiling_offset;  % store row index
                        cell_floor_idx(col,2) = col + leftwall_offset;  % store col index
                        break
                    end
                end
            end
        end
        
        function [cell_order] = plan_cell_order(obj)
            %plan_cell_order Generate a cell searching order 
            %   cell_order - list of cell sequences [[cell_seq], ...]
            %
            %   Determines what order the cells should be searched for
            %   complete coverage. Each 'cell_seq' is a sequence of
            %   adjacent cells. They are initialised with zeros and number
            %   of elements equal to the total number of cells, to 
            %   preallocate list size.
            %   This is technically optimally solved by a TSP solver. For
            %   simplicity, this function tries to create long sequences of
            %   adjacent cells to minimise travel.
            
            % Initialise reeb graph
            reeb_graph = obj.graph;

            % Initialise arrays and counters
            cell_order = [];
            cell_seq = zeros(1,numnodes(reeb_graph));
            unsearched_cells = (1:numnodes(reeb_graph));
            cell_order_idx = 1;
            cell_seq_idx = 2;

            % Get starting cell number and add to first sequence
            start = 1;
            cell = start;
            cell_seq(1) = start;
            unsearched_cells(start) = [];  % remove start cell

            % Continue while there are still unsearched cells
            while true
                % Assume all successor cells have been searched
                all_searched = true;

                % The cell is not a dead end
                if outdegree(reeb_graph,cell) > 0
                    % Check successors cells
                    next_cells = successors(reeb_graph,cell);

                    % Check if there is an unsearched successor
                    for i = 1:size(next_cells,2)
                        % A successor hasn't been searched yet
                        if find(unsearched_cells==next_cells(i))
                            % Update flag
                            all_searched = false;

                            % Update arrays
                            cell = next_cells(i);
                            cell_seq(cell_seq_idx) = cell;  % append to continuous cell sequence

                            % New cell is no longer unsearched
                            unsearched_cells(unsearched_cells==cell) = [];  % remove cell
                            break
                        end
                    end

                    % All the successors have already been searched
                    if all_searched == true
                        % Add sequence to cell order
                        cell_order = [cell_order; cell_seq];  % add to cell order
                        cell_seq(1,:) = 0;  % clear sequence

                        % Update indices
                        cell_order_idx = cell_order_idx + 1;
                        cell_seq_idx = 1;

                        % Add lowest unsearched cell
                        cell = min(unsearched_cells);
                        cell_seq(cell_seq_idx) = cell;

                        % New cell is no longer unsearched
                        unsearched_cells(unsearched_cells==cell) = [];  % remove cell
                    end     

                % The cell is a dead end
                else
                    % Add sequence to cell order
                    cell_order = [cell_order; cell_seq];
                    cell_order_idx = cell_order_idx + 1;

                    % Reset cell sequence
                    cell_seq(1,:) = 0;  % clear sequence
                    cell_seq_idx = 1;  % restart index

                    % Add lowest unsearched cell
                    cell = min(unsearched_cells);
                    cell_seq(cell_seq_idx) = cell;

                    % New cell is no longer unsearched
                    unsearched_cells(unsearched_cells==cell) = [];  % remove cell
                end

                % Increment loop
                cell_seq_idx = cell_seq_idx + 1;

                % Escape condition
                if isempty(unsearched_cells) == 1
                    % Add last cell sequence
                    cell_order = [cell_order; cell_seq];
                    break
                end
            end
        end
        
        function [decomposed_matrix, reeb_graph, num_cells] = btd_cell_decomposition(obj)
            %btd_cell_decomposition Decompose a map into cells
            %   decomposed_matrix - % cell decomposed map
            %   reeb_graph - graph object describes the adjacency of cells
            %   num_cells - number of cells generated in the map
            %
            %   Boustrophedon cell decomposition divides a map into
            %   obstacle-free regions called 'cells'. The cells have
            %   amorphous top and bottom contours, but straight line left
            %   and right edges. Paths can then be planned in these cells.
            %   For this function to work, the occupancy map must have an 
            %   enclosing border of occupied cells.
            %   In the output matrix, a 0 is an obstacle and all other
            %   integers correspond to a specific cell.

            % Get binary occupancy matrix
            bi_occ_matrix = occupancyMatrix(obj.bi_occ_map);
            
            %% Initialise variables
            decomposed_matrix = zeros(size(bi_occ_matrix));
            reeb_graph = digraph;
            last_connectivity = 0;
            last_connections = [];
            cell_counter = 0;  % tracks the number of cells total
            last_cells = [];  % tracks which cells were in the last slice
            current_cells = [];  % tracks which cells are in the current slice

            for col = 1:size(bi_occ_matrix,2)
                %% Check connectivity of the slice
                slice = bi_occ_matrix(:,col);
                [connectivity, connections] = obj.slice_connectivity(slice);

                %% 1. Check if we are coming out of a full obstacle slice
                if last_connectivity == 0
                    % Reset current_cells for re-population
                    current_cells = [];

                    % Loop through number of connections in the new slice and add these
                    % cells
                    for i = 1:size(connections,1)
                        cell_counter = cell_counter + 1;
                        current_cells = [current_cells, cell_counter];  % append cells
                    end

                    % Update graph
                    reeb_graph = addnode(reeb_graph,size(current_cells,2));

                %% 2. Check if we are in a full obstacle slice
                elseif connectivity == 0
                    % No cells in full obstacle slice
                    current_cells = [];

                %% Determine sweep line splitting/joining
                % Each individual connection in the slice can be checked for splitting
                % or joining. This is done using the adjacency matrix. The sum of a row
                % indicates how many right slice connections a left slice connections
                % is adjacent to. The sum of a column indicates how many left slice
                % connections a right slice connection is adjacent to.
                else
                    %% 3. Connection(s) present in current or last slice
                    % Compare slices using adjacency matrix
                    adj_matrix = obj.connections_adjacency(last_connections,connections);

                    % Compare slices left to right using adjacency matrix row
                    for i = 1:size(adj_matrix,1)
                        % 3a. The connection split: IN condition
                        if sum(adj_matrix(i,:)) > 1
                            % Reset the split cells
                            split = [];

                            % Find the split index
                            cell_of_interest = last_cells(i);  % find the number of the cell
                            index_of_interest = find(current_cells==cell_of_interest);  % desired index of replacement            

                            % Check how many new cells are produced and track
                            for j = 1:sum(adj_matrix(i,:))
                                cell_counter = cell_counter + 1;
                                split = [split, cell_counter];
                            end

                            % Replace the old cell with the new split cells
                            before_split = current_cells(1:index_of_interest-1);
                            after_split = current_cells(index_of_interest+1:end);
                            current_cells = [before_split,split,after_split];

                            % Update graph
                            reeb_graph = addnode(reeb_graph,size(split,2));
                            reeb_graph = addedge(reeb_graph,cell_of_interest,split);

                        % 3b. The connection does not split or join (dead end)
                        elseif sum(adj_matrix(i,:)) == 0
                            % Find the removal index
                            cell_of_interest = last_cells(i);  % find the number of the cell
                            index_of_interest = current_cells==cell_of_interest;

                            % Remove the cell from the current cells array
                            current_cells(index_of_interest) = [];
                        end
                    end

                    % Compare slices right to left using adjacency matrix columns
                    for i = 1:size(adj_matrix,2)
                        % 3c. The connection joined: OUT condition
                        if sum(adj_matrix(:,i)) > 1
                            % Reset the joined cells
                            joined = [];

                            % Find the join index
                            index_of_interest = i;

                            % Track the cells being joined
                            for j = 1:size(adj_matrix,1)
                                if adj_matrix(j,i) == 1
                                    for k = 0:sum(adj_matrix(:,i))-1
                                        joined = [joined, last_cells(j+k)];
                                    end
                                    break
                                end
                            end

                            cell_counter = cell_counter + 1;
                            joint = cell_counter;

                            % Replace cells that joined with new cell and update
                            % current_cell matrix
                            before_join = current_cells(1:index_of_interest-1);
                            after_join = current_cells(index_of_interest+sum(adj_matrix(:,i)):end);
                            current_cells = [before_join, joint, after_join];

                            % Update graph
                            reeb_graph = addnode(reeb_graph,1);
                            reeb_graph = addedge(reeb_graph,joined,joint);

                        % 3d. A new connection formed from an obstacle: IN condition
                        elseif sum(adj_matrix(:,i)) == 0
                            % Find the insert index
                            index_of_interest = i;

                            % Add just one count to the cell_counter
                            cell_counter = cell_counter + 1;
                            insert = cell_counter;

                            % Insert this new cell in the correct place in current_cell
                            % array
                            before_insert = current_cells(1:index_of_interest-1);
                            after_insert = current_cells(index_of_interest:end);
                            current_cells = [before_insert, insert, after_insert];

                            % Update graph
                            reeb_graph = addnode(reeb_graph,1);

                        end
                    end              
                end
                
                %% Update map
                % No connectivity means a full line of obstacles
                if connectivity == 0
                    decomposed_matrix(:,col) = 0;  % Obstacles are 0
                else
                    for i = 1:size(connections,1)
                        % Fill connectivity segments with corresponding cell numbers
                        start_fill = connections(i,1);
                        end_fill = connections(i,2)-1;
                        decomposed_matrix(start_fill:end_fill,col) = current_cells(i);
                    end
                end

                %% Store previous states
                last_connectivity = connectivity;
                last_connections = connections;
                last_cells = current_cells;
            end
            
            %% Split/Join Post-processing
            % Split/join conditions (where a cell splits into two
            % new cells, but one of the new cells is also joined to
            % another previous cell) will add incorrect offsetting 
            % of the cell counter due to double counting of a
            % segment. To fix this, the following code is used.
            graph_cells = numnodes(reeb_graph);
            
            for i = 1:graph_cells
                cell_num_check = decomposed_matrix == i;
                
                % Check if the cell number doesn't exist
                if ~any(cell_num_check,'all')
                    % This cell number was removed by a split/join.
                    % Decrement all numbers after this 'skip'
                    subtraction = decomposed_matrix>i;
                    decomposed_matrix = decomposed_matrix-subtraction;
                    
                    % Update graph
                    reeb_graph = rmnode(reeb_graph,i);
                end
            end
            
            % Update number of cells
            num_cells = numnodes(reeb_graph);     
        end
        
        function [connectivity,connections] = slice_connectivity(obj,slice)
            %slice_connectivity Identifies connections in a slice of map
            %   slice - vertical slice in occupancy map
            %   
            %   connectivity - number of connections in a slice
            %   connections - starts and ends of connections in slice [[s,e],...]
            % 
            %   Connections are segments in an occupancy map with adjacent,
            %   unoccupied grid units. 
            %   This function calculates the number of connected segments 
            %   in a vertical slice of an occupancy map and returns their
            %   locations.
            %   If the previous index is a 0 and this one is a 1, a 
            %   connectivity has opened. Log this point as a start point. 
            %   Otherwise, if the previous index is a 1 and this one is a 0
            %   and a connectivity was opened, a connectivity has closed. 
            %   Log this point as an end point. 

            %% Initialise
            connectivity = 0;
            last_data = 0;
            open_part = false;
            connections = []; % unknown length

            %% Loop Through
            for i = 1:length(slice)
                data = slice(i);

                % Connectivity opens
                if data == 0 && open_part == false
                    open_part = true;
                    connectivity = connectivity + 1;
                    start_point = i;
                % Connectivity closes
                elseif last_data == 0 && data == 1 && open_part == true
                    open_part = false;
                    end_point = i;
                    connections(connectivity, 1) = start_point;
                    connections(connectivity, 2) = end_point;
                % Logs the part closing if it reaches the end of the slice
                elseif i == length(slice) && open_part == true
                    open_part = false;
                    end_point = i;
                    connections(connectivity, 1) = start_point;
                    connections(connectivity, 2) = end_point;
                end

                % Store data
                last_data = data;
            end
        end
        
        function [adjacency_matrix] = connections_adjacency(obj,connections_L,connections_R)
            %connections_adjacency Adjacency matrix of slices' connections
            %   connections_L - connections in left-hand slice
            %   connections_R - connections in right-hand slice
            %
            %   adjacency_matrix - n x n matrix
            %
            %   Returns the adjacency matrix describing adjacency of 
            %   connections between two neighbouring slices. The adjacency
            %   of connections between slices affects the generation of new
            %   cells during Boustrophedon cell decomposition.

            %% Initialise Matrix
            adjacency_matrix = zeros(size(connections_L,1), size(connections_R,1)); 

            %% Looping
            for i = 1:size(connections_L,1)
                for j = 1:size(connections_R,1)
                    if min(connections_L(i,2), connections_R(j,2)) - max(connections_L(i,1), connections_R(j,1)) > 0
                        adjacency_matrix(i, j) = 1;
                    end
                end
            end
        end 
    end
end

