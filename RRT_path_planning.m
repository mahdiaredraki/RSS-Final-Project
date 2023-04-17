function [path_cost, comp_time, path_cost_smooth, comp_time_smooth] = RRT_path_planning(map, rrt, robot,smooth_flag,start_time)
    % Initialize the tree
    tree = [map.start_position]; % initialize the tree with the start node
    treeCost = 0; % initialize the cost of the tree
    treeEdges = []; % initialize the edges of the tree
    s= [];
    t= [];
    weights = [];
    path =[];

    path_cost = 0;
    path_cost_smooth = 0;
    comp_time = inf;
    comp_time_smooth = inf;
    smooth_attempts = 4;

    % Define the main loop
    for i = 1:rrt.maxIterations
        % Sample a new node
        if rand() < rrt.groundNodeSamplingProb % sample a ground node
            newNode = sampleGroundNode(map);
        else % sample a flight node
            newNode = sampleFlightNode(map);
        end
        
        if rand() < rrt.goalNodeSamplingProb % sample goal node
            newNode = map.goal_position;
        end

        % Find the nearest node in the tree to the new node
        [nearestNode, nearestNodeIndex] = findNearestNode(newNode, tree);

        % Check if the edge between the nearest node and the new node is valid
        [valid_edge, newNode] = isValidEdge(nearestNode,newNode,rrt,map);
        if valid_edge

            % Calculate the cost of moving from the nearest node to the new node
            newNodeCost = calculateCost(nearestNode, newNode, robot);

            tree = [tree; newNode];
            treeCost = [treeCost; newNodeCost];

            x = [tree(nearestNodeIndex,1) newNode(1)];
            y = [tree(nearestNodeIndex,2) newNode(2)];
            z = [tree(nearestNodeIndex,3) newNode(3)];
            plot3(x,y,z, 'b', 'LineWidth', 0.5);

            s = [s nearestNodeIndex];
            t = [t size(tree,1)];
            weights = [weights newNodeCost];

            % Check if the goal has been reached
            if calculateDistance(newNode, map.goal_position) < rrt.stepSize/5
                % Add the goal node to the tree
                goalNodeCost = calculateCost(nearestNode, newNode, robot);
                tree = [tree; map.goal_position];
                treeCost = [treeCost; goalNodeCost];
                treeEdges = [treeEdges; size(tree, 1)-1, size(tree, 1)];
                s = [s nearestNodeIndex];
                t = [t size(tree,1)];
                weights = [weights goalNodeCost];

                G = graph(s,t,weights);

                % Find shortest (weighed) path to goal
                TR = shortestpathtree(G,1,size(tree,1));
                path = map.start_position;
                for k = 1:size(TR.Edges.EndNodes,1)
                    path = [path; tree(TR.Edges.EndNodes(k,2),:)];
                    path_cost = path_cost + TR.Edges.Weight(k);
                end
                break;
            end
        end
    end    

    %% Plot the path
    % plot3(tree(:,1), tree(:,2), tree(:,3), 'b', 'LineWidth', 2);
    if isempty(path)
        disp("No path found! Increase maxIteration!")
    else
        plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2)
        hold on;
        plot3(map.start_position(1), map.start_position(2), map.start_position(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
        plot3(map.goal_position(1), map.goal_position(2), map.goal_position(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        title('Path');
        % Uncomment the following line to display the final path cost
        disp(['RRT Path cost: ', num2str(path_cost)]);
        

    end
    comp_time = toc(start_time);

    %% Smooth RRT path
    if smooth_flag == true
        [smooth_path, path_cost_smooth] = smooth_paths(path, path_cost_smooth, robot, rrt, map, smooth_attempts);
        hold on;
        plot3(smooth_path(:,1),smooth_path(:,2),smooth_path(:,3),"g-",LineWidth=5)
        disp(['RRT Path cost with smoothing: ', num2str(path_cost_smooth)]);
        comp_time_smooth = toc(start_time);
    end

end
% Save the final path to a file
%csvwrite('path.csv', path);

% Define a function to find the nearest node in the tree to a given node
function [nearestNode, nearestNodeIndex] = findNearestNode(node, tree)
    dists = vecnorm((tree - node)'); % calculate the distances to all nodes in the tree
    [minDist, nearestNodeIndex] = min(dists); % find the index of the nearest node
    nearestNode = tree(nearestNodeIndex,:); % get the nearest node
end

function [valid, node2] = isValidEdge(node1, node2, rrt, map)
    % Check if the edge between node1 and node2 is valid
    direction = node2 - node1;
    distance = norm(direction);
    stepVector = (direction./distance).*(rrt.stepSize*distance);
    currentPoint = node1 + stepVector;

    % Discretize edge from node1 to currentPoint to check for collision
    % along edge
    increments_along_edge = 100;
    if distance >1
        increments_along_edge = increments_along_edge * distance;
    end    
    for i = 1:increments_along_edge
        stepVector = (direction./distance).*(i/increments_along_edge*(rrt.stepSize*distance));
        intermediate_node = node1 + stepVector;
        if checkOccupancy(map.occupancy_map, intermediate_node) ~= 1
            valid = true;
            node2 = currentPoint;
        else
            valid = false;
            node2 = node1;
            return
        end
    end
end

function [valid, node2] = isValidEdgesmooth(node1, node2, rrt, map)
    % Check if the edge between node1 and node2 is valid
    direction = node2 - node1;
    distance = norm(direction);

    % Discretize edge from node1 to currentPoint to check for collision
    % along edge
    increments_along_edge = 100;
    if distance > 1
        increments_along_edge = increments_along_edge * distance;
    end    
    for i = 1:increments_along_edge
        stepVector = (direction./distance).*(i/increments_along_edge*(distance));
        intermediate_node = node1 + stepVector;
        if checkOccupancy(map.occupancy_map, intermediate_node) ~= 1
            valid = true;
            node2 = node2;
        else
            valid = false;
            node2 = node1;
            return
        end
    end
end

function node = sampleGroundNode(map)
    % Sample a random node in the ground plane that is not in an obstacle
    x = rand()*map.map_limits(1);
    y = rand()*map.map_limits(2);
    while checkOccupancy(map.occupancy_map, [x,y,0]) == 1
        x = rand()*map.map_limits(1);
        y = rand()*map.map_limits(2);
    end
    node = [x, y, 0];
end

function node = sampleFlightNode(map)
    % Sample a random node in the 3D space that is not in an obstacle
    x = rand()*map.map_limits(1);
    y = rand()*map.map_limits(2);
    z = rand()*map.map_limits(3);
    while checkOccupancy(map.occupancy_map, [x,y,z]) == 1  %checkCollision([x,y,z], map)
        x = rand()*map.map_limits(1);
        y = rand()*map.map_limits(2);
        z = rand()*map.map_limits(3);
    end
    node = [x, y, z];
end

% Define a function to calculate the cost of moving from one node to another
function cost = calculateCost(node1, node2, robot)
    if node1(3) == 0 && node2(3) == 0 % if the nodes are on the ground
        cost = norm(node1-node2)*robot.costs.walking;
    else % if the nodes are in the air
        cost = norm(node1-node2)*robot.costs.flying;
    end
    if node1(3) == 0 && node2(3) ~= 0 % walking to flying
        cost = cost+ robot.costs.walking_to_flying;
    end
    if node1(3) ~= 0 && node2(3) == 0 % flying to walking
        cost = cost+ robot.costs.flying_to_walking;
    end

end

% Define a function to calculate the distance between two nodes
function distance = calculateDistance(node1, node2)
    distance = norm(node1-node2);
end

function [smooth_path, path_cost] = smooth_paths(path, path_cost, robot, rrt, map, iterations)

    q_goal = path(end,:);
    %iteration of tries
    iterations = 3;
    attempt = 0;
    while attempt < iterations
        smooth_path = path(1,:);
        skip_nxt = false;
        for i = 1:size(path, 1) - 2
            if skip_nxt
                skip_nxt = false;
                continue
            end

            one = path(i,:);
            other = path(i+2,:);
            %smoothed path  
            [valid, ~] = isValidEdgesmooth(one, other, rrt, map);
            if not(~valid)
                % If the desired waypoint is invalid, keep the current waypoint
                smooth_path = [smooth_path; other];
                skip_nxt = true;
            else
                smooth_path = [smooth_path; path(i+1,:)];
            end 
        end
        attempt = attempt + 1;
        path = smooth_path;
    end
    
    if not(isequal(smooth_path(end,:), q_goal))
    	smooth_path = [smooth_path; q_goal];
    end

    path_cost = 0;

    if length(smooth_path) > 1
        for i = 1:length(smooth_path)-1
            path_cost = path_cost + calculateCost(smooth_path(i,:), smooth_path(i+1,:), robot);
        end
    end
end