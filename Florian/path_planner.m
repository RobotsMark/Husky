function sub_goals = path_planner(statevector, goal_position)
%change number of nodes

%%%
%Path Planner
%RRT search
%
%%%


width=10; %width of the map in meters
height=10; %height of the map in meters
resolution=10; %cells per meter

%hard-coded example
% % % start=[0,0];
% % % goal=[9,9];
% % % x = [1.2; 2.3; 3.4; 4.5; 5.6; 1.2; 2.3; 3.4; 4.5; 5.6];
% % % y = [5.0; 4.0; 3.0; 2.0; 1.0; 7.0; 8.9; 5.5; 5.0; 4.0];
% % % number_of_obstacles=10;
%

%function version
start=statevector(1:2)';
goal=goal_position;
obstacles = statevector(4:end);


if(isempty(obstacles))
    disp("no obstacles");
    x=[];
    y=[];
    number_of_obstacles=0;
else
    disp("some obstacles");
    x=obstacles(1:2:end); %maybe other way round
    y=obstacles(2:2:end); %maybe other way round
    number_of_obstacles=size(x, 1); %maybe other way round
end
%

map = robotics.BinaryOccupancyGrid(width,height,resolution);

if(number_of_obstacles>0)
    setOccupancy(map, [x y], ones(number_of_obstacles,1));
end

inflate(map, 0.5);
%figure(1);
%show(map);
%%%change number of nodes
planner = robotics.PRM(map, 200);
planner.ConnectionDistance=3;

sub_goals = findpath(planner,start,goal);
figure(1);
show(planner);

sub_goals=sub_goals(2:end,:);
%ignore the first sub_goal %%dangerous
%sub_goals=sub_goals(3:end,:);
end
