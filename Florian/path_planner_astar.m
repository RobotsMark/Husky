function sub_goals = path_planner_astar(statevector, goal_position)



%%
%Initialization
%%%

width=15; %width of the map in meters
height=20; %height of the map in meters
resolution=10; %cells per meter
real_width=width*resolution;
real_height=height*resolution;



%%%
translation_x = 3; %translate the entire map by 10;
translation_y = 10;
translation_matrix=[translation_x, translation_y];
goal_position=goal_position+translation_matrix;

%%%



radius=round(resolution/2.8);


start_position=statevector(1:2)'+translation_matrix;
list_of_obstacles = statevector(4:end);


start_position=round(start_position*resolution)';
goal_position=round(goal_position*resolution);



list_of_obstacles=round(list_of_obstacles*resolution);


if(isempty(list_of_obstacles))
    disp("no obstacles");
    obstacle_x=[];
    obstacle_y=[];
    number_of_obstacles=0;
else
    disp("some obstacles");
    obstacle_x=list_of_obstacles(1:2:end)+translation_x*resolution; %maybe other way round
    obstacle_y=list_of_obstacles(2:2:end)+translation_y*resolution; %maybe other way round
    number_of_obstacles=size(obstacle_x, 1); %maybe other way round
end


grid = ones(width*resolution, height*resolution);

circle=[];
for i=-radius:radius
    for j=-radius:radius
        if sqrt(i*i+j*j)<=radius
            circle=[circle, [i;j]];
        end
    end
end %end for


try
    for i=1:number_of_obstacles
        for elm=circle
            if 0<obstacle_x(i)+elm(1)<= real_width && 0<obstacle_y(i)+elm(2) <= real_height
                grid(obstacle_x(i)+elm(1), obstacle_y(i)+elm(2))=0;
            end
        end
    end %end for

    for elm=circle
        if 0<start_position(1)+elm(1)<= real_width && 0<start_position(2)+elm(2) <= real_height
            grid(start_position(1)+elm(1), start_position(2)+elm(2))=2;
        end
    end
    for elm=circle
        if 0<goal_position(1)+elm(1)<= real_width && 0<goal_position(2)+elm(2) <= real_height
            grid(goal_position(1)+elm(1), goal_position(2)+elm(2))=3;
        end
    end

% 
% image(grid*20);
% colorbar;



%%
%Astar
%%%

upper_bound=resolution^2*width*height;

%The set of nodes already evaluated
closedSet=false(real_width, real_height);

%The set of currently discovered nodes that are not evaluated yet
openSet=false(real_width, real_height);
openSet(start_position(1), start_position(2))=true;

%For each node, which node it can most efficiently be reached from.
cameFrom=zeros(width*resolution, height*resolution,2);

%For each node, the cost of getting from the start node to that node
gScore=ones(real_width, real_height) * (upper_bound);
gScore(start_position(1), start_position(2))=0;

%for each node, the total cost of getting from the start node to the goal by passing by that node
fScore=ones(real_width, real_height) * (upper_bound);

fScore(start_position(1), start_position(2))=heuristic(start_position, goal_position);

%array to look up node in openSet with the lowest fScore[] value
%if node is in closedSet its value is set to upper_bound
fScore_lookup=ones(real_width, real_height) * (upper_bound);
fScore_lookup(start_position(1), start_position(2))=heuristic(start_position, goal_position);


path=[];
while any(any(openSet))

    [A, ind_1]= min(fScore_lookup); %= the node in openSet having the lowest fScore[] value
    [~, ind_2]=min(A);
    current=[ind_1(ind_2), ind_2];
    %disp(current)
    
    if current == goal_position
        disp("finished");
        path = reconstruct_path(cameFrom, start_position, goal_position);
        break;
    end

    openSet(current(1), current(2))=false;
    closedSet(current(1), current(2))=true;
    fScore_lookup(current(1), current(2))=upper_bound;

    
    for i=-1:1
        for j=-1:1
            neighbour = current + [i, j];
            if (0<neighbour(1)<=real_width && 0<neighbour(2)<=real_height && ~closedSet(neighbour(1), neighbour(2)) && ~(i==0 && j==0) && grid(neighbour(1), neighbour(2))>0)

                openSet(neighbour(1), neighbour(2))=true;


                %The distance from start to a neighbor
                %the "dist_between" function may vary as per the solution requirements.
                %%%tentative_gScore = gScore(current(1), current(2)) + norm(current - neighbour);
                tentative_gScore = gScore(current(1), current(2)) + distance_measure(current, neighbour);


                if tentative_gScore < gScore(neighbour(1), neighbour(2))
                    %This path is the best until now. Record it!

                    cameFrom(neighbour(1), neighbour(2), 1) = current(1);
                    cameFrom(neighbour(1), neighbour(2), 2) = current(2);
                    
                    gScore(neighbour(1), neighbour(2)) = tentative_gScore;
                    fScore(neighbour(1), neighbour(2)) = gScore(neighbour(1), neighbour(2)) + heuristic(neighbour, goal_position);
                    fScore_lookup(neighbour(1), neighbour(2))=fScore(neighbour(1), neighbour(2));
                end
            end
        end
    end



%end while    
end



%%   find the path

for i=1:size(path(), 1)
    x=path(i, 1);
    y=path(i, 2);
    grid(x, y)=5;
end %end for

figure(3)
image(grid*20)
colorbar;

path=flipud(path);


%% turn path into subgoals

sub_goals=path(resolution/2:resolution/2:end, :);
sub_goals=sub_goals/resolution-translation_matrix;
disp(sub_goals);

%%

catch
    disp("ERROR!");
end

end
%end function


