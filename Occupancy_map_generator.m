
addpath(['crameri_repo',filesep])

if ~isempty(load_generated_map)
    load(load_generated_map)
    obstacleNumber = map.obstacle.number+1;
else
    obstacleNumber = 1;
    % Create an occupancyMap3D object with a map resolution of 10 cells/meter.
    map.occupancy_map = occupancyMap3D(1/map.map_resolution);
    map.aerial_occupancy_map = occupancyMap3D(1/map.map_resolution);
    % Define a set of 3-D points as an observation from a pose [x y z qw qx qy qz].
    % This pose is for the sensor that observes these points and is centered on the origin.
    % Define two sets of points to insert multiple observations.
    pose = [ 0 0 0 1 0 0 0]; % [x y z qw qx qy qz]
end

% Check start and goal position do not exceed map limits
coordinate_axis = ["x", "y", "z"];
for i = 1:3
    if map.start_position(i) > map.map_limits(i)
        disp("WARNING! Start position in "+ coordinate_axis(i) +" direction exceeds map limit!")
        map.start_position(i) = map.map_limits(i);
    end
    if map.goal_position(i) > map.map_limits(i)
        disp("WARNING! Goal position in "+ coordinate_axis(i) +" direction exceeds map limit!")
        map.goal_position(i) = map.map_limits(i);
    end
end

%% Generate Aerial Obstacles
total_obstacles = map.num_ground_obstacles+map.num_aerial_obstacles;
while obstacleNumber <= map.num_aerial_obstacles
    % Generate random aerial obstacles
    radius= map.map_resolution*(round(rand()/2/map.map_resolution));   % The largest integer in the sample intervals for obtaining width, length and height
    x_pos = map.map_resolution*(round(rand()*(map.map_limits(1)-radius)/map.map_resolution));
    y_pos = map.map_resolution*(round(rand()*(map.map_limits(2)-radius)/map.map_resolution));
    z_pos = map.map_resolution*(round((rand()*(map.map_limits(3)-(radius*2))+radius/2)/map.map_resolution));
    
    % Make unit sphere
    [X,Y,Z] = sphere(100);
    % Scale to desire radius.
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    % Translate sphere to new location.
    X = X+x_pos;
    Y = Y+y_pos;
    Z = Z+z_pos;
    
    % Create points on sphere surface
    x = X(:);
    y = Y(:);
    z = Z(:);
    % Create bounding points inside sphere surface
    x_i = 0.999*X(:);
    y_i = 0.999*Y(:);
    z_i = 0.999*Z(:);
    % Create bounding points outside sphere surface
    x_o = 1.001*X(:);
    y_o = 1.001*Y(:);
    z_o = 1.001*Z(:);
    % Group sets of point to easily set occupancy level
    X_bound = [x_i,x_o];
    Y_bound = [y_i,y_o];
    Z_bound = [z_i,z_o];
    pts_surf = [x(:) y(:) z(:)];
    pts_bound = [X_bound(:) Y_bound(:) Z_bound(:)];
    
    % If start or goal position interects this obstacle, then try again
    % This ensures start and goal positions are obstacle free.
    modified_occupancy_map = copy(map.occupancy_map);

    % Set occupancy value of different regions
    setOccupancy(modified_occupancy_map,pts_surf,0);
    setOccupancy(modified_occupancy_map,pts_bound,1);
    
    if checkOccupancy(modified_occupancy_map,map.start_position) == 1 || checkOccupancy(modified_occupancy_map,map.goal_position) == 1
        disp("This aerial obstacle overlaps with start or goal position. Trying again!")
        modified_occupancy_map = [];
    else
        disp("Generated "+obstacleNumber+" out of "+map.num_aerial_obstacles+" aerial obstacles")
        obstacleNumber = obstacleNumber + 1;
        setOccupancy(map.occupancy_map,pts_surf,0);
        setOccupancy(map.occupancy_map,pts_bound,1);
        setOccupancy(map.aerial_occupancy_map,pts_surf,0);
        setOccupancy(map.aerial_occupancy_map,pts_bound,1);
    end
end

figure(1)
show(map.aerial_occupancy_map)
hold on
view(-6,33)
xlim([0 map.map_limits(1)])
ylim([0 map.map_limits(2)])
zlim([0 map.map_limits(3)])
pause(0.1)
%% Generate Ground Obstacles
while (obstacleNumber-map.num_aerial_obstacles) <= map.num_ground_obstacles
    % Generate random ground obstacles
    width= map.map_resolution*(round(rand()*map.map_limits(1)/5/map.map_resolution))+map.map_resolution;   % The largest integer in the sample intervals for obtaining width, length and height
    length = map.map_resolution*(round(rand()*map.map_limits(2)/1/map.map_resolution))+map.map_resolution;  % can be changed as necessary to create different occupancy maps.
    height = map.map_resolution*(round(rand()*map.map_limits(3)/3/map.map_resolution))+map.map_resolution;

    x_min = map.map_resolution*(round(rand()*(map.map_limits(1)-width)/map.map_resolution));
    y_min = map.map_resolution*(round(rand()*(map.map_limits(2)-length)/map.map_resolution));
    z_min = 0;

    x_max = x_min + width;
    y_max = y_min + length;
    z_max = z_min + height;

    [xObstacle,yObstacle,zObstacle] = meshgrid(x_min:map.map_resolution:x_max,y_min:map.map_resolution:y_max,z_min:map.map_resolution:z_max);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    % If start or goal position interects this obstacle, then try again
    % This ensures start and goal positions are obstacle free.
    modified_occupancy_map = copy(map.occupancy_map);
    setOccupancy(modified_occupancy_map,xyzObstacles,1)
    if checkOccupancy(modified_occupancy_map,map.start_position) == 1 || checkOccupancy(modified_occupancy_map,map.goal_position) == 1
        disp("This ground obstacle overlaps with start or goal position. Trying again!")
        modified_occupancy_map = [];
    else
        map.obstacle.x_min(obstacleNumber) = x_min;
        map.obstacle.x_max(obstacleNumber) = x_max;
        map.obstacle.y_min(obstacleNumber) = y_min;
        map.obstacle.y_max(obstacleNumber) = y_max;
        map.obstacle.z_min(obstacleNumber) = z_min;
        map.obstacle.z_max(obstacleNumber) = z_max;
        map.obstacle.width(obstacleNumber) = width;
        map.obstacle.length(obstacleNumber) = length;
        map.obstacle.height(obstacleNumber) = height;
        map.obstacle.number = obstacleNumber;
        disp("Generated "+(obstacleNumber-map.num_aerial_obstacles)+" out of "+map.num_ground_obstacles+" ground obstacles")
        obstacleNumber = obstacleNumber + 1;
        setOccupancy(map.occupancy_map,xyzObstacles,1)
    end
end

figure(1)
draw_all_obstacles(map.obstacle)
xlim([0 map.map_limits(1)])
ylim([0 map.map_limits(2)])
zlim([0 map.map_limits(3)])
pause(0.1)
% Save the generated map
if ~isempty(save_generated_map)
    save(save_generated_map,'map')
end

function draw_all_obstacles(obstacle)
    n_obs = obstacle.number;  
    obstalce_colors = crameri('batlow',n_obs);
    for obs = 1:n_obs
        x = [obstacle.x_min(obs), obstacle.x_max(obs)];
        y = [obstacle.y_min(obs), obstacle.y_max(obs)];
        z = [obstacle.z_min(obs), obstacle.z_max(obs)];

        % Draw all 6 faces
        face = cell(6,1);

        % Box positions, 1-2-5-6 is the front side
        %   7  --------- 6
        %     /       /
        %  8 --------- 5
        %   3  --------- 2
        %     /       /
        %  4 --------- 1
        %
        p1 = [x(2); y(1); z(1)];
        p2 = [x(2); y(2); z(1)];
        p3 = [x(1); y(2); z(1)];
        p4 = [x(1); y(1); z(1)];
        p5 = [x(2); y(1); z(2)];
        p6 = [x(2); y(2); z(2)];
        p7 = [x(1); y(2); z(2)];
        p8 = [x(1); y(1); z(2)];
        p_temp = [p1,p2,p3,p4,p5,p6,p7,p8];
        px = p_temp(1,:);
        py = p_temp(2,:);
        pz = p_temp(3,:);

        face{1} = [1,5,6,2]; % Front side
        face{2} = [2,6,7,3];
        face{3} = [1,2,3,4];
        face{4} = [5,6,7,8];
        face{5} = [4,8,7,3];
        face{6} = [1,5,8,4];

        for k = 1:6
            fill3(px(face{k}), py(face{k}), pz(face{k}), obstalce_colors(obs,:))
            hold on
        end

    end
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
end