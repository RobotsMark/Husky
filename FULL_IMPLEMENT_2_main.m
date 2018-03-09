clear all;
close all;
clc;

%% 
husky_id = 3; % Modify for your Husky
config = GetHuskyConfig(husky_id);
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); 

%% Initialise params
initialise_state = [0.0; 0.0; 0.0];
initialise_covariance = diag([0.01,0.01,0.01]);
% initialise_goal = [5.0,0.0,0.0]';

statevector = initialise_state;
covariance = initialise_covariance;
goal_position=[6,0];
update_subgoal=true;
phase = 1; 

FeaturesToPlot = [];
old_wheel_odometry = [];

while isempty(old_wheel_odometry)
    mailbox = mexmoos('FETCH');
    old_wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
    old_ml = old_wheel_odometry.m_l;
    old_mr = old_wheel_odometry.m_r;
    
end

%% initial graphics - plot true map
map_fig = figure;
hold on 
grid off; 
axis([-7 7 -7 7]);
% axis equal;
scatter(statevector(2),statevector(1),'b','x')
set(gcf,'doublebuffer','on');
hObsLine = line([0,0],[0,0]);
set(hObsLine,'linestyle',':');


target_fig = figure;

loop_counter = 1;
distance_from_origin = 0;
distance_to_goal = goal_position(1);
fail_count = 0;
%%
while true
    
% Fetch latest messages from mex-moos
mailbox = mexmoos('FETCH');
laser_scan = GetLaserScans(mailbox, config.laser_channel, true);
stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);

new_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
                              
while isempty(new_wheel_odometry) || isempty(laser_scan)
    mailbox = mexmoos('FETCH');    
    laser_scan = GetLaserScans(mailbox, config.laser_channel, true);
    new_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
    
    % Sometimes wheel odometry returns zeros (!!) - Fetch again when this happens                          
    if ~isempty(new_wheel_odometry)
        if and(and(distance_from_origin > 0, new_wheel_odometry.m_l == 0),new_wheel_odometry.m_l == 0 )
         new_wheel_odometry = [];
        end 
    end
end

% Get features from laser and wheel odom
new_ml = new_wheel_odometry.m_l;
new_mr = new_wheel_odometry.m_r;
observations = getPoleCoords(laser_scan);

% Get relative motion
rel_motion = CalculateControlVector( new_ml-old_ml, new_mr-old_mr);
old_ml = new_ml;
old_mr= new_mr;

% Update using SLAM
old_sv_length = length(statevector);
[statevector, covariance] = SLAMUpdate(rel_motion, observations, ...
    statevector, covariance);
new_sv_length = length(statevector);
distance_from_origin = norm(statevector(1:2));
distance_to_goal = norm(statevector(1:2)' - goal_position);
if abs(statevector(1))> 10 || abs(statevector(2))> 10
    disp("Statevector out of bounds");
end
%%

if and(phase ==1 || phase ==2,and(~isempty(stereo_images),mod(loop_counter,10)==0))
            figure(target_fig)
            hold on;
            imshow(stereo_images(end).left.rgb)
            [xtarget_rob, ytarget_rob] = dist_from_target(stereo_images(end).left.rgb, stereo_images(end).right.rgb,target_fig);
            hold off;
    
            if ~isempty(xtarget_rob)
                world_theta = statevector(3);
                xtarget_world = cos(world_theta)*xtarget_rob -sin(world_theta)*ytarget_rob;
                ytarget_world = sin(world_theta)*xtarget_rob + cos(world_theta)*ytarget_rob;
                xtarget_world = xtarget_world + statevector(1);
                ytarget_world = ytarget_world + statevector(2);
                
                if and(and(xtarget_world>5,xtarget_world <8),abs(ytarget_world)<5)
                    disp("Updating goal to target.")
                    goal_position = [xtarget_world,ytarget_world];
                    phase = 2;
                end
            end
end
        
%  When close to target, lock on and stop trying to refine it
if and(phase ==2, norm(statevector(1:2) - goal_position') < 2)
    disp("Entering phase 3.")
    phase = 3;
end


if phase~=5 % If not in final turning stage
   
   subgoals_position=path_planner_astar(statevector, goal_position);
   if isempty(subgoals_position) % Returns empty when reach goal
       
       if phase ==1
           fail_count = fail_count + 1;
           if fail_count>2
               phase = 4;
               goal_position = [0,0];
               disp("Couldn't find target. Returning home.")
           elseif fail_count ==1
               goal_position = [6,-1];
           elseif fail_count == 2
               goal_position = [6,1];
           end
           
           subgoals_position=path_planner_astar(statevector, goal_position);
            
       end
       
       if phase ==3 && distance_to_goal <0.1
            disp("Updating goal to start position.")
            goal_position = [0,0];
            subgoals_position=path_planner_astar(statevector, goal_position);
            phase = 4;
       elseif phase == 4 
            disp("Entering phase 5.")
            phase = 5;
       end     
   end
   
   if phase ~=5
       subgoal_position=subgoals_position(1,:);
       velocity_angle = get_velocity_and_angle_radiant(statevector(1:2)', ...
        subgoal_position, statevector(3));

       relative_angle=velocity_angle(2);
       distance=velocity_angle(3);
       clockwise=velocity_angle(4);

       epsilon_angle=pi/30;
       epsilon_distance=0.05;

       if(abs(relative_angle)>epsilon_angle)
            SendSpeedCommand(0, clockwise, config.control_channel);
       elseif(distance>epsilon_distance)
            SendSpeedCommand(1.0, 0, config.control_channel);
       else
            SendSpeedCommand(0, 0, config.control_channel);
       end
   end

else
    world_angle = statevector(3);
    while pi - abs(world_angle) > 0.09 % this is 5 degrees
        SendSpeedCommand(0, sign(world_angle), config.control_channel);
    end
    break;
end
    

% Plot new features
if new_sv_length > old_sv_length
   new_features = (new_sv_length - old_sv_length)/2;
  for i = 1:new_features
      xind = old_sv_length + 2*i - 1 ;
      yind = old_sv_length + 2*i;
      FeaturesToPlot = [FeaturesToPlot;statevector(xind),statevector(yind)];
  end
end

if length(statevector)>3
    figure(map_fig)
    hold off;
    scatter(statevector(5:2:end),statevector(4:2:end-1))
    axis([-7 7 -7 7]);
    hold on;
    scatter(statevector(2),statevector(1), 'r', 'x')
    h = PlotEllipse(statevector,covariance,1);
    if(~isempty(h))
        set(h,'color','r');
    end
    hold off;
end

clear laser_scan observations;
pause(0.1)
loop_counter = loop_counter + 1;
end

disp("Challenge complete!");