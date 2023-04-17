function graph = uniform_grid_generator(map,uniform_resolution)

    % Initialize grid with zeros
    graph.grid = zeros (floor(map.map_limits(1)/uniform_resolution +1) *...
                        floor(map.map_limits(2)/uniform_resolution +1) *...
                        floor(map.map_limits(3)/uniform_resolution +1), 4);
    
    k = 1;
    for x = 0:uniform_resolution:map.map_limits(1)
        for y = 0:uniform_resolution:map.map_limits(2)
            for z = 0:uniform_resolution:map.map_limits(3)
                if checkOccupancy(map.occupancy_map,[x,y,z]) == 1
                    % obstacle
                    node_value = -1;
                else
                    if z == 0
                        % walking
                        node_value = 1;
                    else
                        % flying
                        node_value = 2;
                    end
                end
                   
                graph.grid(k,:) = [x y z node_value];
                k = k + 1;
            end
        end
    end
    
    % create edges table
    
    graph.edges = zeros(8 * length(graph.grid), 6);
    c = 1;
    for idx_1 = 1:length(graph.grid)
        x = graph.grid(idx_1, 1);
        y = graph.grid(idx_1, 2);
        z = graph.grid(idx_1, 3);

        for k = uniform_resolution:-uniform_resolution:-uniform_resolution
            for j = uniform_resolution:-uniform_resolution:-uniform_resolution
                for i = uniform_resolution:-uniform_resolution:-uniform_resolution

                    s_x = x + k;
                    s_y = y + j;
                    s_z = z + i;
                    
                    if( (s_x >= 0 && s_x <= map.map_limits(1)) && (s_y >= 0 &&...
                             s_y <= map.map_limits(2)) && (s_z >= 0 && s_z <= map.map_limits(3))) 
                        
                        if (s_x ~= x || s_y ~= y || s_z ~= z)
          
                            graph.edges(c,:) = [x, y, z, s_x, s_y, s_z];
                            c = c + 1;
                        
                        end
                    end
                end
            end
        end
    end
end