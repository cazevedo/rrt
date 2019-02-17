clc
clear all
close all
colormap bone

%% INIT
% set here robot's starting state and goal state
x_new = [45,45];
x_goal = [8,16];
epsilon = 0.9 ;

% initialize map
hold on
map = init_map(x_new, x_goal);
imagesc(map)
set(gca,'YDir','normal')

%% RRT ALGORITHM
% initialize graph tree
node_index = 1;
source_node = [node_index];
target_node = [];
nodes_x(node_index) = x_new(1);
nodes_y(node_index) = x_new(2);
rrt_graph = graph(source_node,target_node);
rrt_plot = plot(rrt_graph, 'w','XData', nodes_y, 'YData', nodes_x,'NodeLabel',{});

disp('Press any key to start the algorithm')
pause
iterations = 1;
% check stopping condition (goal reached)
while (any(x_new ~= x_goal))
    iterations = iterations + 1;
    
    % select direction state
    x_rand = select_state(x_goal,epsilon,1);

    % select nearest neighbor to this current random state ('seuclidean', 'minkowski', or 'mahalanobis')
    for node = 1:node_index
        neighbors(node) = pdist([nodes_x(node),nodes_y(node);x_rand(1),x_rand(2)],'euclidean');
    end
    [dist, nearest_node] = min(neighbors);
    % state of the nearest neighbor
    x_near = [nodes_x(nearest_node), nodes_y(nearest_node)];

    % move towards x_rand position
    x_new = x_near + move(x_near,x_rand);

    % check if position is occupied
    if map(x_new(1), x_new(2)) ~= 1
        % check if the node already exists
        exists_node = false;
        for i=1:node_index
            if x_new(1) == nodes_x(node) && x_new(2) == nodes_y(node)
               exists_node = true;
               break
            end
        end

        if exists_node == false
            % add current state as a node to the graph tree
            node_index = node_index + 1;
            rrt_graph = addnode(rrt_graph,1);
            rrt_graph = addedge(rrt_graph,nearest_node,node_index);
            nodes_x(node_index) = x_new(1);
            nodes_y(node_index) = x_new(2);
        end
    end
    
    delete(rrt_plot)
    rrt_plot = plot(rrt_graph, 'w','XData', nodes_y, 'YData', nodes_x,'NodeLabel',{}, 'LineWidth', 0.5000, 'MarkerSize', 4);
    grid on
    pbaspect([1 1 1])
    xlim([1 50])
    ylim([1 50])
    pause(0.01)
end
hold off

% Use A* to retrieve the shortest path
spath = shortestpath(rrt_graph,1,node_index);
highlight(rrt_plot,spath,'NodeColor','k','EdgeColor','k');

%% AUXILIARY FUNCTIONS

function x = select_state(x_goal,epsilon,dist)
    if rand<epsilon
        if dist == 1
            % from a uniform distribution
            x = [randi([1,50]), randi([1,50])];
        elseif dist == 2
            x(1) = random('Normal',25,7.5);
            x(2) = random('Normal',25,7.5);
            for i=1:2
               if x(i) < 1
                  x(i) = 1;
               elseif x(i) >50
                   x(i) = 50;
               end
            end
        elseif dist == 3
            x(1) = random('Rayleigh',x_goal(1));
            x(2) = random('Rayleigh',x_goal(2));
            for i=1:2
               if x(i) < 1
                  x(i) = 1;
               elseif x(i) >50
                   x(i) = 50;
               end
            end
        end
    else
        x = x_goal;
    end
end

function angle = find_orientation(source,target)
    target(1) = target(1)-source(1);
    target(2) = target(2)-source(2);
    angle = atan2(target(1),target(2));
    if angle < 0
        angle = angle + 2*pi;
    end
end

function delta = move(source,target)
    angle = find_orientation(source,target);
    delta(1) = sin(angle);
    delta(2) = cos(angle);
    for i = 1:2
        if 0 < delta(i) && delta(i) < 0.3535
            delta(i) = 0;
        elseif 0.3535 <= delta(i) && delta(i) < 1
            delta(i) = 1;
        elseif -0.3535 < delta(i) && delta(i) < 0
            delta(i) = 0;
        elseif -1 < delta(i) && delta(i) <= -0.3535
            delta(i) = -1;
        end
    end
end

% function map = init_rand_map(map_source, map_target)
%     map = randi([0 1], 50,50);
%     
%     map(map_source(1),map_source(2)) = -1;
%     map(map_target(1),map_target(2)) = -2;
% end

function map = init_map(map_source, map_target)
    % free unnocuppied map
    map_x = 50;
    map_y = 50;
    for i = 1:map_x
        for j = 1:map_y
            map(i,j) = 0;
       end
    end
    
    % add obstacles
    for i = 10:25
        for j = 10:10
            map(i,j) = 1;
        end
    end

    for i = 25:25
        for j = 5:10
            map(i,j) = 1;
        end
    end
    
    for i = 10:10
        for j = 5:10
            map(i,j) = 1;
        end
    end
    
    for i = 30:45
        for j = 35:35
            map(i,j) = 1;
        end
    end
    
    for i = 30:45
        for j = 15:15
            map(i,j) = 1;
        end
    end
    
    % Walls bounding the map
    for i = 1:50
        for j = [1,50]
            map(i,j) = 1;
        end
    end
 
    % Walls bounding the map
    for i = [1,50]
        for j = 1:50
            map(i,j) = 1;
        end
    end
    
    map(map_source(1),map_source(2)) = -1;
    map(map_target(1),map_target(2)) = -2;
end
