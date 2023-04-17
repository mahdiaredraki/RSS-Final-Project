function [waypoints, total_cost] = Path_planning_Astar (map, graph, robot)

    open = [];
    closed = [];
    
    %% put obstacles in closed
    
    idx = find (graph.grid(:,4) == -1);
    closed = [graph.grid(idx, 1) graph.grid(idx, 2) graph.grid(idx, 3)];
    
    closed_count = size(closed,1);
    
    %set the starting node as the first node
    x_start = map.start_position(1);
    y_start = map.start_position(2);
    z_start = map.start_position(3);
    
    x_node = x_start;
    y_node = y_start;
    z_node = z_start;
    
    x_goal = map.goal_position(1);
    y_goal = map.goal_position(2);
    z_goal = map.goal_position(3);
    
    open_count = 1;
    path_cost = 0;
    goal_distance = heuristic_function(x_node, y_node, z_node, x_goal, y_goal, z_goal, graph, robot);
    open(open_count,:) = [0, x_node, y_node, z_node, x_node, y_node, z_node,...
        path_cost,goal_distance,goal_distance];
    closed_count = closed_count+1;
    closed(closed_count, 1) = x_node;
    closed(closed_count, 2) = y_node;
    closed(closed_count, 3) = z_node;
    no_path = 1;
    
    
    while((x_node ~= x_goal || y_node ~= y_goal || z_node ~= z_goal) && no_path == 1)
     exp_array = expand_array(x_node, y_node, z_node, path_cost, closed, map, graph, robot);
     exp_count = size(exp_array,1);
     
     for i = 1:exp_count
        flag = 0;
        for j = 1:open_count
            if(exp_array(i, 1) == open(j, 2) && exp_array(i, 2) == open(j, 3) && exp_array(i, 3) == open(j, 4))
                open(j, 10) = min(open(j, 10), exp_array(i, 6)); 
                if open(j, 10)== exp_array(i, 6)
                    open(j, 5) = x_node;
                    open(j, 6) = y_node;
                    open(j, 7) = z_node;
                    open(j, 8) = exp_array(i, 4);
                    open(j, 9) = exp_array(i, 5);
                end
                flag = 1;
            end
        end
        if (flag == 0)
            open_count = open_count + 1;
            open(open_count,:) = [1, exp_array(i, 1), exp_array(i, 2), exp_array(i, 3), ...
                x_node, y_node, z_node, exp_array(i, 4), exp_array(i, 5), exp_array(i, 6)];
        end
     end
    
      index_min_node = min_fn(open, open_count, x_goal, y_goal, z_goal);
      if (index_min_node ~= -1)    

       x_node = open(index_min_node, 2);
       y_node = open(index_min_node, 3);
       z_node = open(index_min_node, 4);
       path_cost = open(index_min_node, 8);
      
      closed_count = closed_count + 1;
      closed(closed_count, 1) = x_node;
      closed(closed_count, 2) = y_node;
      closed(closed_count, 3) = z_node;
      open(index_min_node, 1) = 0;
      else 
          no_path=0;
      end
    end
    
    i = size(closed, 1);
    optimal_path = [];
    x_val = closed(i, 1);
    y_val = closed(i, 2);
    z_val = closed(i, 3);
    i = 1;
    optimal_path(i, 1) = x_val;
    optimal_path(i, 2) = y_val;
    optimal_path(i, 3) = z_val;
    i = i + 1; 
    
    if ((x_val == x_goal) && (y_val == y_goal) && (z_val == z_goal))
       inode = 0;

       idx = find(open(:,2) == x_val & open(:,3) == y_val & open(:,4) == z_val);
       parent_x = open(idx, 5);
       parent_y = open(idx, 6);
       parent_z = open(idx, 7);
       
       while((parent_x ~= x_start) || (parent_y ~= y_start) || (parent_z ~= z_start))
           optimal_path(i, 1) = parent_x;
           optimal_path(i, 2) = parent_y;
           optimal_path(i, 3) = parent_z;
    
           inode = find(open(:,2) == parent_x & open(:,3) == parent_y & open(:,4) == parent_z);
           parent_x = open(inode, 5);
           parent_y = open(inode, 6);
           parent_z = open(inode, 7);
    
           i = i + 1;
        end
        
        optimal_path(i,:) = [x_start y_start z_start];
        waypoints = optimal_path((end:-1:1),:);
        total_cost = 0;
        for i = 1:(length(waypoints) - 1) 
            [cost, mode] = cost_function(waypoints(i, 1), waypoints(i, 2), waypoints(i, 3),...
                waypoints(i + 1, 1), waypoints(i + 1, 2), waypoints(i +1 , 3), graph, robot);
            waypoints(i, 4) = mode;
            total_cost = total_cost + cost;

        end

    else
        pause(1);
        h=msgbox('Sorry, No path exists to the Target!','warn');
        uiwait(h,5);
    end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                               %
%             INTERNAL FUNCTIONS                %
%                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [cost, mode] = cost_function(x_1, y_1, z_1, x_2, y_2, z_2, graph, robot)
    
    cost = 0;
    mode = 0;

    format long
    x_1; y_1; z_1; x_2; y_2; z_2;

    idx1 = find(abs(graph.grid(:,1) - x_1) <= 1*10^-6 &...
        abs(graph.grid(:,2) - y_1) <= 1*10^-6 & abs(graph.grid(:,3) - z_1) <= 1*10^-6, 1);
    idx2 = find(abs(graph.grid(:,1) - x_2) <= 1*10^-6 &...
        abs(graph.grid(:,2) - y_2) <= 1*10^-6 & abs(graph.grid(:,3) - z_2) <= 1*10^-6, 1);
    
    node_value1 = graph.grid(idx1, 4);
    node_value2 = graph.grid(idx2, 4);
    dist = distance(x_1, y_1, z_1, x_2, y_2, z_2);
    switch node_value1
        case 1
            switch node_value2
                case 1
                    cost = dist*robot.costs.walking; % cost leg/leg
                    mode = 1;
                case 2
                    cost = dist*robot.costs.flying ; % cost leg/flying + To Do transition cost?
                    cost = cost + robot.costs.walking_to_flying;
                    mode = 3;
            end
        case 2
            switch node_value2
                case 1
                    cost = dist*robot.costs.flying; 
                    cost = cost + robot.costs.flying_to_walking;
                    mode = 4;
                case 2
                    cost = dist*robot.costs.flying;
                    mode = 2;
            end
    end 
end

function exp_array = expand_array(node_x, node_y, node_z, hn, closed, map, graph, robot)
 
    exp_array = [];
    exp_count = 1;
    c2 = size(closed, 1);

    format long
    node_x; node_y; node_z;

    idx1 = find(abs(graph.edges(:,1) - node_x) <= 1*10^-6 &...
        abs(graph.edges(:,2) - node_y) <= 1*10^-6 & abs(graph.edges(:,3) - node_z) <= 1*10^-6);
    
    idx2 = find(abs(graph.edges(:,4) - node_x) <= 1*10^-6 &...
        abs(graph.edges(:,5) - node_y) <= 1*10^-6 & abs(graph.edges(:,6) - node_z) <= 1*10^-6);

    neighbors = graph.edges(idx1,:);
    neighbors = [neighbors; graph.edges(idx2, 4), graph.edges(idx2, 5), graph.edges(idx2, 6),...
        graph.edges(idx2, 1), graph.edges(idx2, 2), graph.edges(idx2, 3)];
   

    for i = 1:length(neighbors(:,1))
        s_x = neighbors(i, 4);
        s_y = neighbors(i, 5);
        s_z = neighbors(i, 6);
        flag = 1;
        idx = find (closed(:,1) == s_x & closed(:,2) == s_y & closed (:,3) == s_z);
               
        if (idx ~= 0)
            flag = 0;
        end
        if (flag == 1)
            exp_array(exp_count, 1) = s_x;
            exp_array(exp_count, 2) = s_y;
            exp_array(exp_count, 3) = s_z;
            exp_array(exp_count, 4) = hn + cost_function(node_x, node_y, node_z, s_x, s_y, s_z, graph, robot);
            exp_array(exp_count, 5) = heuristic_function(s_x, s_y, s_z, map.goal_position(1), map.goal_position(2),...
                map.goal_position(3), graph, robot);
            exp_array(exp_count, 6) = exp_array(exp_count, 4) + exp_array(exp_count, 5);
            exp_count = exp_count + 1;
        end
    end
end   

function i_min = min_fn(open, open_count, x_goal, y_goal, z_goal)
    temp_array = [];
    k = 1;
    flag = 0;
    goal_index = 0;
    for j = 1:open_count
        if (open(j, 1) == 1)
            temp_array(k,:) = [open(j,:) j];
            if (open(j, 2) == x_goal && open(j,3) == y_goal && open(j,4) == z_goal)
                flag = 1;
                goal_index = j;
            end
            k = k + 1;
        end
    end
    if (flag == 1)
        i_min = goal_index;
    end
    %Send the index of the smallest node
    if size(temp_array ~= 0)
        [min_fn,temp_min] = min(temp_array(:,10));%Index of the smallest node in temp array
        i_min = temp_array(temp_min, 11);%Index of the smallest node in the open array
    else
        i_min = -1;%The temp_array is empty i.e No more paths are available.
    end
end

function cost  = heuristic_function(x_1, y_1, z_1, x_2, y_2, z_2, graph, robot)
    
    cost = 0;
    mode = 0;

    format long
    x_1; y_1; z_1; x_2; y_2; z_2;

    idx1 = find(abs(graph.grid(:,1) - x_1) <= 1*10^-6 &...
        abs(graph.grid(:,2) - y_1) <= 1*10^-6 & abs(graph.grid(:,3) - z_1) <= 1*10^-6);
    idx2 = find(abs(graph.grid(:,1) - x_2) <= 1*10^-6 &...
        abs(graph.grid(:,2) - y_2) <= 1*10^-6 & abs(graph.grid(:,3) - z_2) <= 1*10^-6);

    node_value1 = graph.grid(idx1, 4);
    node_value2 = graph.grid(idx2, 4);

    cost = robot.costs.walking + robot.costs.flying;  

end

function dist = distance(x1, y1, z1, x2, y2, z2)

dist=sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2 );
end