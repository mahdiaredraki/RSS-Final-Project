%% Running multiple iterations
Prob_g = [0 0 0.2 0.5 0.8]; %% % Sampling probabilities for ground and flight nodes for RRT
Batch_pp_algos = ["prm","uniform_grid","RRT","RRT","RRT"];

%% Generate Summary Tables
Environment_Counter=1;
RRT1 = 'RRT* with Prob_g = '+string(Prob_g(3));
RRT1_smooth = 'RRT* with smoothing with Prob_g = '+string(Prob_g(3));
RRT2 = 'RRT* with Prob_g = '+string(Prob_g(4));
RRT2_smooth = 'RRT* with smoothing with Prob_g = '+string(Prob_g(4));
RRT3 = 'RRT* with Prob_g = '+string(Prob_g(5));
RRT3_smooth = 'RRT* with smoothing with Prob_g = '+string(Prob_g(5));

varNames = {'Map','Astar with PRM','Astar with Uniform Grid',convertStringsToChars(RRT1),convertStringsToChars(RRT2),convertStringsToChars(RRT3),convertStringsToChars(RRT1_smooth),convertStringsToChars(RRT2_smooth),convertStringsToChars(RRT3_smooth)};
varTypes = {'string','string','string','string','string','string','string','string','string'};
SZ=[5 9];
Path_Cost_Table = table('Size',SZ,'VariableTypes',varTypes,'VariableNames',varNames);
Computation_Time_Table = table('Size',SZ,'VariableTypes',varTypes,'VariableNames',varNames);

clearvars -except Path_Cost_Table Computation_Time_Table Environment_Counter Prob_g Batch_pp_algos
clc

for current_map = 1:4
    for index_Batch_pp_algos = 1:size(Batch_pp_algos,2)
        close all
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %           USER DEFINED               %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Robot movement parameters
        robot.costs.walking_to_flying = 10;
        robot.costs.flying_to_walking = 5;
        robot.costs.walking = 1;
        robot.costs.flying = 5;

        %% Path Plannning Algorithm
        path_planning_algo = Batch_pp_algos(index_Batch_pp_algos); % "prm" or "uniform_grid" or "RRT"

        %% Path Planning Parameters
        switch path_planning_algo
            case "prm"
                if current_map == 5
                    prm.number_nodes = 10000; %600
                    prm.number_ground_nodes = prm.number_nodes/2; %300
                    prm.radius = 20; %m
                else
                    prm.number_nodes = 600;
                    prm.number_ground_nodes = prm.number_nodes/2;
                    prm.radius = 1; %m
                end
            case "uniform_grid"
                if current_map == 5
                    uniform_resolution = 1; % meter/node
                else
                    uniform_resolution = 0.5; % meter/node
                end
            case "RRT"
                rrt.groundNodeSamplingProb = Prob_g(index_Batch_pp_algos); % Sampling probabilities for ground and flight nodes
                rrt.goalNodeSamplingProb = 0.2; % Sampling probability for goal node
                smooth_flag = true;
                if current_map == 5
                    rrt.maxIterations = 100000; % Maximum number of iterations
                    rrt.stepSize = 5; % Step size for expanding the tree
                else
                    rrt.maxIterations = 10000; % Maximum number of iterations
                    rrt.stepSize = 0.5; % Step size for expanding the tree
                end  
        end

        %% Environment Parameter
        load_generated_map = "map_"+string(current_map); % ="NAME OF FILE TO LOAD", =[] to generate new map
        
        if isempty(load_generated_map)
            % Map parameters
            save_generated_map = "map_5"; % ="NAME OF SAVED FILE"
            map.map_limits = [50, 15, 2.5]; % [x, y, z] (meters) - set the dimension of the map
            map.num_ground_obstacles = 20; % number of randomly generated obstacles
            map.num_aerial_obstacles = 50;
            map.start_position = [0, 2, 0]; % [x, y, z] (meters) - starting position of the robot
            map.goal_position = [48, 13, 0]; % [x, y, z] (meters) - goal position of the robot
            map.map_resolution = 0.05; % meter/cell
        else
            % DON'T CHANGE THIS
            save_generated_map = [];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %          END USER DEFINED            %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %            BEGIN CODE                %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Generate Map
        Occupancy_map_generator

        start_time = tic;
        %% Graph and node generator
        switch path_planning_algo
            case "prm"
                disp("Generating PRM grid. This may take a few minutes..")
                graph = prm_grid_generator(map, prm);
            case "uniform_grid"
                disp("Generating uniform grid. This may take a few minutes..")
                graph = uniform_grid_generator(map, uniform_resolution);
            case "RRT"
                disp("Generating path with RRT*. This may take a few minutes..")
                [path_cost, comp_time, path_cost_smooth, comp_time_smooth] = RRT_path_planning(map, rrt, robot,smooth_flag,start_time);
        end

        if ~isequal(path_planning_algo, "RRT")
            %% Path planning using A_star
            [waypoints, path_cost] = Path_planning_Astar(map, graph, robot);
            %% Plot the map
            %% clean à la sauvage:
            idx_walking = find(graph.grid(:,4) == 1);
            idx_flying = find(graph.grid(:,4) == 2);
            walking_nodes = plot3(graph.grid(idx_walking, 1), graph.grid(idx_walking, 2), graph.grid(idx_walking, 3),'*', "Color", "black", "DisplayName","Walking Nodes");
            flying_nodes = plot3(graph.grid(idx_flying, 1), graph.grid(idx_flying, 2), graph.grid(idx_flying, 3), '*', "Color", "blue","DisplayName", "Flying Nodes");

            %% Plot the path
            plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3),'Linewidth', 2.5,"Color", [255, 165, 0]/255)
            plot3(map.start_position(1), map.start_position(2), map.start_position(3), '.', 'MarkerSize', 50, 'Color', 'blue')
            plot3(map.goal_position(1), map.goal_position(2), map.goal_position(2), '.', 'MarkerSize', 50, 'Color', 'green')
            comp_time = toc(start_time);
            Path_Cost_Table(Environment_Counter,index_Batch_pp_algos+1)={string(round(path_cost,2))};
            Computation_Time_Table(Environment_Counter,index_Batch_pp_algos+1)={string(round(comp_time,4))};
        else
            Path_Cost_Table(Environment_Counter,index_Batch_pp_algos+1)={string(round(path_cost,2))};
            Computation_Time_Table(Environment_Counter,index_Batch_pp_algos+1)={string(round(comp_time,4))};

            Path_Cost_Table(Environment_Counter,index_Batch_pp_algos+4)={string(round(path_cost_smooth,2))};
            Computation_Time_Table(Environment_Counter,index_Batch_pp_algos+4)={string(round(comp_time_smooth,4))};
        end
        Fig_Name = "Simulation Results/map"+ string(current_map)+"_algo_"+path_planning_algo+"_"+string(index_Batch_pp_algos);
        saveas(gcf,Fig_Name)
        Path_Cost_Table(Environment_Counter,1)={Environment_Counter};
        Computation_Time_Table(Environment_Counter,1)={Environment_Counter};
        
        if index_Batch_pp_algos == size(Batch_pp_algos,2)
            Environment_Counter=Environment_Counter+1;
        end
        
    end
end
