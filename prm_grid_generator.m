function graph = prm_grid_generator(map, prm)
    prm.radius = 1; %m
    prm.nodes = [];
    nearest_nodes = []; 
    edges_counter = 1;
    i = 1;
    prm.nodes(i,:) = [map.start_position(1), map.start_position(2), map.start_position(3), 1]; % Trick TO IMPROVE

    while i <= prm.number_nodes
        % Generate walking node
        new_node = [map.map_resolution*(round(rand(1)*(map.map_limits(1))/map.map_resolution)) map.map_resolution*(round(rand(1)*(map.map_limits(2))/map.map_resolution)) 0];
        if (i > prm.number_ground_nodes )
            % Generate flying node
            new_node(3) = map.map_resolution*(round(rand(1)*(map.map_limits(3))/map.map_resolution));
        end
       
        if(checkOccupancy(map.occupancy_map,new_node) == -1)    
            
            nearest_nodes = find_nearest_node(new_node, prm);
            nearest_nodes = sortrows(nearest_nodes, 4);
            
            node_value = 0;
            x = new_node(1);
            y = new_node(2);
            z = new_node(3);
    
            % walking
            if(z == 0 )
                node_value = 1;
            % flying_node
            else 
                node_value = 2;
            end
            
            prm.nodes(i,:) = [x, y, z, node_value];
            i = i + 1;
            
            for j = 1:length(nearest_nodes(:,1))
                if (free_segment(new_node, nearest_nodes(j,:), map))%TO DO ||...
                        %%already_in(new_node, nearest_nodes(j,:), prm))
                    prm.edges(edges_counter,:) = [new_node, nearest_nodes(j,:)];
                    edges_counter =  edges_counter + 1;
                end
            end
        end
    end

    %% add the start
    new_node = [map.start_position(1), map.start_position(2), map.start_position(3)];
    node_value = 1;
    
    nearest_nodes = find_nearest_node(new_node, prm);
    nearest_nodes = sortrows(nearest_nodes, 4);

    prm.nodes(i,:) = [new_node, node_value];
    i = i + 1;

    for j = 1:length(nearest_nodes(:,1))
        if (free_segment(new_node, nearest_nodes(j,:), map))%TO DO ||...
                %%already_in(new_node, nearest_nodes(j,:), prm))
            prm.edges(edges_counter,:) = [new_node, nearest_nodes(j,:)];
            edges_counter =  edges_counter + 1;
        end
    end

    %% add the goal
    new_node = [map.goal_position(1), map.goal_position(2), map.goal_position(3)];
    node_value = 1;
    
    nearest_nodes = find_nearest_node(new_node, prm);
    nearest_nodes = sortrows(nearest_nodes, 4);

    prm.nodes(i,:) = [new_node, node_value];
    i = i + 1;

    for j = 1:length(nearest_nodes(:,1))
        if (free_segment(new_node, nearest_nodes(j,:), map))%TO DO ||...
                %%already_in(new_node, nearest_nodes(j,:), prm))
            prm.edges(edges_counter,:) = [new_node, nearest_nodes(j,:)];
            edges_counter =  edges_counter + 1;
        end
    end
    
    graph.grid = [];
    graph.grid = prm.nodes;
    graph.edges = [];
    graph.edges = prm.edges(:,(1:6));
end

function result = free_segment(node_1, node_2, map)
    result = 1;
    x = linspace(node_1(1), node_2(1), 10)';
    y = linspace(node_1(2), node_2(2), 10)';
    z = linspace(node_1(3), node_2(3), 10)';
    
    for i = 1:length(x(:,1))
        node = [x(i), y(i), z(i)];
        if checkOccupancy(map.occupancy_map,node) == 1
            result = 0;
            break
        end
    end

end

function node = find_nearest_node(new_node, prm)
   
    idx = find (sqrt((prm.nodes(:,1) - new_node(1)).^2 + (prm.nodes(:,2) - new_node(2)).^2 ...
        + (prm.nodes(:,3) - new_node(3)).^2)  <= prm.radius);
    node = prm.nodes(idx,:);
    dist = sqrt((prm.nodes(idx,1) - new_node(1)).^2 + (prm.nodes(idx,2) - new_node(2)).^2 ...
        + (prm.nodes(idx,3) - new_node(3)).^2);
    node = [node(:,1:3), dist];

end




