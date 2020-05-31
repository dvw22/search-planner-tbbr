classdef SearchPlanner
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        decomposed_matrix
        graph
        num_cells
        cell_order
    end
    
    methods
        function obj = SearchPlanner(occ_map)
            %SearchPlanner Construct an instance of this class
            
            % Process map
            occ_matrix = occupancyMatrix(occ_map);
            bi_occ_matrix = round(occ_matrix);  % convert to binary
            
            % Perform cell decomposition of map
            [obj.decomposed_matrix, obj.graph, obj.num_cells] = btd_cell_decomposition(bi_occ_matrix);
            
            % Perform cell order calculation
            obj.cell_order = plan_cell_order(obj.graph, obj.num_cells);
        end
    end
    
    methods (Access = private)
        function [decomposed_matrix, reeb_graph, cell_counter] = btd_cell_decomposition(bi_occ_matrix)
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
        
        function [cell_order] = plan_cell_order(reeb_graph)
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


    end
end

