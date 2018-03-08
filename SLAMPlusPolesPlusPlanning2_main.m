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
goal_position=[5,0];
update_subgoal=true;

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
axis([-5 5 -5 5]);
% axis equal;
scatter(statevector(2),statevector(1),'b','x')
set(gcf,'doublebuffer','on');
hObsLine = line([0,0],[0,0]);
set(hObsLine,'linestyle',':');

%%
while true

% Fetch latest messages from mex-moos
mailbox = mexmoos('FETCH');
laser_scan = GetLaserScans(mailbox, config.laser_channel, true);
% stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);

new_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
while isempty(new_wheel_odometry) || isempty(laser_scan)
    mailbox = mexmoos('FETCH');    
    laser_scan = GetLaserScans(mailbox, config.laser_channel, true);
    new_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
%     new_ml = new_wheel_odometry.m_l;
%     new_mr = new_wheel_odometry.m_r;
end

if ~isempty(new_wheel_odometry)                         
    new_ml = new_wheel_odometry.m_l;
    new_mr = new_wheel_odometry.m_r;
%     new_time = new_wheel_odometry.timestamp;
%     timediff = new_time - old_time;
%     old_time = new_time;
%     disp( timediff );
else
    new_ml = old_ml;
    new_mr = old_mr;
%     disp("Misassignment of wheel odom");
end

% Get features from laser
if ~isempty(laser_scan)
    observations = getPoleCoords(laser_scan);
else 
    observations = [];
end


% Get relative motion
rel_motion = CalculateControlVector( new_ml-old_ml, new_mr-old_mr);
old_ml = new_ml;
old_mr= new_mr;

old_sv_length = length(statevector);

% Update using SLAM
[statevector, covariance] = SLAMUpdate(rel_motion, observations, ...
    statevector, covariance);



if(statevector(1)>5.5)
   subgoal_position=goal_position;
elseif(update_subgoal)
   subgoals_position=path_planner_astar(statevector, goal_position);
   if isempty(subgoals_position)
       goal_position = [0,0];
       subgoals_position=path_planner_astar(statevector, goal_position);
   end
   subgoal_position=subgoals_position(1,:);
   update_subgoal=true;
end
    
velocity_angle=get_velocity_and_angle_radiant(statevector(1:2)', ...
    subgoal_position, statevector(3));

relative_angle=velocity_angle(2);
distance=velocity_angle(3);
clockwise=velocity_angle(4);

epsilon_angle=pi/30;
epsilon_distance=0.05;

if(abs(relative_angle)>epsilon_angle)
    SendSpeedCommand(0, clockwise, config.control_channel);
elseif(distance>epsilon_distance)
    SendSpeedCommand(1, 0, config.control_channel);
else
    SendSpeedCommand(0, 0, config.control_channel);
    update_subgoal=true;
end


% figure(pole_fig)
% if ~isempty(observations)
%     obs_x = observations(1,:).*cos(observations(2,:));
%     obs_y = observations(1,:).*sin(observations(2,:));
%     theta = statevector(3);
%     obs_worldx = cos(theta)*obs_x -sin(theta)*obs_y;
%     obs_worldy = sin(theta)*obs_x + cos(theta)*obs_y;
%     
%     scatter(obs_worldy + statevector(2),obs_worldx + statevector(1))
%     axis([-5 5 -5 5]);
% end
% hold off

new_sv_length = length(statevector);

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
    axis([-5 5 -5 5]);
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

end