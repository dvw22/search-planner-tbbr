classdef SearchPlanner
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        map_resolution
        decomposed_matrix
        graph
        num_cells
        cell_order
        PathPlanner
    end
    
    methods
        function obj = SearchPlanner(occ_map)
            %SearchPlanner Construct an instance of this class
            
            % Convert map to binary map
            obj.map_resolution = occ_map.Resolution;
            occ_matrix = occupancyMatrix(occ_map);
            bi_occ_matrix = round(occ_matrix);  % convert to binary matrix
            bi_occ_map = binaryOccupancyMap(bi_occ_matrix,obj.map_resolution);
            
            % Perform cell decomposition of map
            [obj.decomposed_matrix, obj.graph, obj.num_cells] = obj.btd_cell_decomposition(bi_occ_matrix);
            
            % Perform cell order calculation
            obj.cell_order = obj.plan_cell_order(obj.graph);
            
            % Initialise shortest path planner
            obj.PathPlanner = mobileRobotPRM(bi_occ_map);
            obj.PathPlanner.NumNodes = 75;
            obj.PathPlanner.ConnectionDistance = 5;
        end
        
        function [complete_waypoints,segment_idx] = complete_search_path(obj,init_pos)
            % map_search_path Outputs a full waypoint list with a segment indices matrix
            % to access segmented regions during the waypoint planning process.
            %   The segments are either a cell sequence path or a shortest path between
            %   cells.

            %% Waypoint Generation
            % Initialise
            complete_waypoints = [];
            segment_idx = [];
            num_cell_seq = size(obj.cell_order,1);
            original_num_nodes = obj.PathPlanner.NumNodes;
            original_conn_dis = obj.PathPlanner.ConnectionDistance;

            last_end_idx = 0;
            start_idx = 1;

            % Get waypoints for each cell sequence and path between and append
            for i = 1:num_cell_seq
                % Calculate waypoints before travel first time only
                if i == 1
                    % Append cell sequence waypoints
                    [cell_seq_waypoints, num_cells] = obj.cell_seq_search_path(obj.cell_order(i,:));
                    complete_waypoints = [complete_waypoints; cell_seq_waypoints];

                    % Append cell sequence indices
                    num_waypoints = size(cell_seq_waypoints,1);
                    end_idx = last_end_idx + num_waypoints;
                    segment_idx = [segment_idx; [start_idx, end_idx]];
                    start_idx = end_idx + 1;  % store
                    last_end_idx = end_idx;  % store

                    % Plan path to first waypoint
                    travel_waypoints = findpath(obj.PathPlanner,init_pos,complete_waypoints(1,:));
                    % The planner may not always find a path if the map is too complex.
                    % This condition is to prevent the code from breaking.
                    if isempty(travel_waypoints) == 0
                        travel_waypoints(1,:) = [];  % Must clear source
                        travel_waypoints(end,:) = [];  % Must clear destination of travel to avoid duplicate
                    else
                        display(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
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
                    [next_cell_seq_waypoints, num_cells] = obj.cell_seq_search_path(obj.cell_order(i+1,:));

                    % Get start and end points for travel
                    start_waypoint = cell_seq_waypoints(end,:);  % starting at end of last cell sequence
                    end_waypoint = next_cell_seq_waypoints(1,:);  % ending at start of next cell sequence

                    % Append travel waypoints
                    travel_waypoints = findpath(obj.PathPlanner,start_waypoint,end_waypoint);
                    % The planner may not always find a path if the map is too complex.
                    % This condition is to prevent the code from breaking.
                    while isempty(travel_waypoints) == 1
                        % Publish info
                        display(['Path planning failed between cell sequence ',num2str(i),' and ',num2str(i+1),'.'])
                        display('Increasing nodes and connection distance.')

                        % Increase nodes and connection distances
                        obj.PathPlanner.NumNodes = obj.PathPlanner.NumNodes * 2;
                        obj.PathPlanner.ConnectionDistance = obj.PathPlanner.ConnectionDistance * 2;

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
    end
       
    methods (Access = private)
        function [decomposed_matrix, reeb_graph, cell_counter] = btd_cell_decomposition(obj,bi_occ_matrix)
            %btd_cell_decomposition Uses boustrophedon cell decomposition to output a
            %region divided occupancy map
            %   The occupancy map must have a border of occupied cells for it to work.
            %   For the output, a 0 is an obstacle and the integers are the
            %   corresponding cell number.

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
                [connectivity, connections] = slice_connectivity(slice);

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
                    adj_matrix = connections_adjacency(last_connections,connections);

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
                            index_of_interest = find(current_cells==cell_of_interest);

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
        end
        
        function [cell_order] = plan_cell_order(obj,reeb_graph)
            % plan_cell_order Determines what order the cells should be searched for
            % complete coverage
            %   Detailed explanation goes here

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
                            unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
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
                        unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
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
                    unsearched_cells(find(unsearched_cells==cell)) = [];  % remove cell
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
        
        function [cell_seq_waypoints, num_cells] = cell_seq_search_path(obj,cell_seq)
            % cell_seq_search_path Outputs waypoints for a set of cells in an optimised order
            %   Gets a row in from a cell order plan matrix. Slices off zeroed
            %   elements. Inputs each element as a cell number to cell_search_path.
            %   Appends the result to a full cell sequence waypoint matrix.

            %% Preparation
            % Slice off zeros
            cell_seq = cell_seq(:,any(cell_seq,1));

            %% Initialise
            % Initialise
            cell_seq_waypoints = [];  % reset search path
            num_cells = size(cell_seq,2);

            %% Waypoint Generation
            % Get waypoints for each cell and append
            for i = 1:num_cells
                [cell_waypoints, num_waypoints] = obj.cell_search_path(cell_seq(i));
                cell_seq_waypoints = [cell_seq_waypoints; cell_waypoints];
            end
        end     

        function [cell_waypoints, num_waypoints] = cell_search_path(obj,cell)
            % cell_search path Generates a search path within a cell
            %   Detailed explanation goes here

            [ceiling_idx, floor_idx] = ceil_floor_cell(obj.decomposed_matrix, cell);  

            num_ceiling = size(ceiling_idx,1);
            num_waypoints = num_ceiling*2;

            cell_indices = zeros(num_waypoints,2);  % [row1,col1; row2,col2]

            %% Populate with matrix indices

            % Start at bottom left
            cell_indices(1,:) = floor_idx(1,:);

            % Initialise
            add_ceilings = true;
            j = 1;  % different rate counter

            % For waypoint planning in between edges
            insertion_indices = [];

            for i = 2:num_waypoints-1
                % Adding ceiling points
                if add_ceilings == true
                    % Add a ceiling point
                    cell_indices(i,:) = ceiling_idx(j,:);

                    % Get next row of ceiling_idx (even)
                    if mod(i,2) == 0
                        j = j+1; 
                    % Reset index for floor_idx counting every other time (odd)
                    else
                        % Check pair for edges
                        if cell_indices(i,1) ~= cell_indices(i-1,1)
                            insertion_indices = [insertion_indices; i];  % store insertion index
                        end

                        % Reset for next pair
                        add_ceilings = false;
                        j = j-1;
                    end 
                % Adding floor points
                else
                    % Add a floor point
                    cell_indices(i,:) = floor_idx(j+1,:);

                    % Check if both pairs of ceiling and floor are added
                    if mod(i-1,4) == 0
                        j = j+2;
                    end

                    % Get next row of floor_idx (even)
                    if mod(i,2) == 0
                        j = j+1;
                    % Reset index for ceiling_idx counting every other time (odd)
                    else
                        % Check pair for edges
                        if cell_indices(i,1) ~= cell_indices(i-1,1)
                            insertion_indices = [insertion_indices; i];  % store insertion index
                        end

                        % Reset for next pair
                        add_ceilings = true;
                        j = j-1;
                    end
                end
            end

            % End with last ceiling/floor
            if add_ceilings == true
                cell_indices(num_waypoints,:) = ceiling_idx(num_ceiling,:);
            elseif add_ceilings == false
                cell_indices(num_waypoints,:) = floor_idx(num_ceiling,:);
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
            cell_waypoints = cell_waypoints/obj.map_resolution;


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
                    disp('Increasing nodes and connection distance.')

                    % Increase nodes and connection distances
                    obj.PathPlanner.NumNodes = obj.PathPlanner.NumNodes * 2;
                    obj.PathPlanner.ConnectionDistance = obj.PathPlanner.ConnectionDistance * 2;

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

            % Update number of waypoints due to insertions
            num_waypoints = size(cell_waypoints,1);
        end

    end
end

