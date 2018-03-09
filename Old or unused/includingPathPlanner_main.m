%% 
husky_id = 3; % Modify for your Husky
config = GetHuskyConfig(husky_id);
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); 

%%
initialise_state = [0.0; 0.0; 0.0];
initialise_covariance = diag([0.01,0.01,0.01]);
% initialise_goal = [5.0,0.0,0.0]';

statevector = initialise_state;
covariance = initialise_covariance;
old_wheel_odometry = [];

while length(old_wheel_odometry)<1
    mailbox = mexmoos('FETCH');
    old_wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
end

old_ml = old_wheel_odometry.m_l;
old_mr = old_wheel_odometry.m_r;
    
% %initial graphics - plot true map
% figure(1) 
% hold on 
% grid off; 
% axis equal;
% scatter(statevector(1),statevector(2))
% set(gcf,'doublebuffer','on');
% hObsLine = line([0,0],[0,0]);
% set(hObsLine,'linestyle',':');

% statevector_store = zeros(2,1);
update_subgoal=true;

while true
% Fetch latest messages from mex-moos
mailbox = mexmoos('FETCH');
scan = GetLaserScans(mailbox, config.laser_channel, true);
% stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
new_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
if ~isempty(new_wheel_odometry)                         
    new_ml = new_wheel_odometry.m_l;
    new_mr = new_wheel_odometry.m_r;
else
    new_ml = old_ml;
    new_mr = old_mr;
end

% Get features from laser
if length(scan)>0
    observations = getPoleCoords(scan);
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

new_sv_length = length(statevector);

% % Plot new position
% scatter(statevector(1),statevector(2))

% % Plot new features
% if new_sv_length > old_sv_length
%    new_features = (new_sv_length - old_sv_length)/2;
%    
%   for i = 1:new_features
%       xind = old_sv_length + 2*i - 1 ;
%       yind = old_sv_length + 2*i;
%       scatter(statevector(xind),statevector(yind))
%   end
%   
% end

%     current_position=statevector(1:2);
%     current_orientation=statevector(3);

    %%%TODO: call goal function
    %if confident then turn target_information into x,y coordinates
%     target_information=get_target();

goal_position=[5,0];

if(statevector(1)>5.5)
    subgoal_position=goal_position;
elseif(update_subgoal)
   subgoals_position=path_planner(statevector, goal_position);
   subgoal_position=subgoals_position(1,:);
   update_subgoal=true;
end
    
    
% %%%TODO: Controller
%     %[absolute_angle, relative_angle, distance, clockwise]=get_velocity_and_angle_radiant(current_position, subgoal_position, current_orientation);
%     velocity_angle=get_velocity_and_angle_radiant(current_position, subgoal_position, current_orientation);
%     relative_angle=velocity_angle(2);
%     distance=velocity_angle(3);
%     clockwise=velocity_angle(4);
%     
%     
%     epsilon_angle=pi/30;
%     epsilon_distance=0.2;
% 
%     if(abs(relative_angle)>epsilon_angle)
%         SendSpeedCommand(0, clockwise, husky_config.control_channel);
%     elseif(distance>epsilon_distance)
%         SendSpeedCommand(1, 0, husky_config.control_channel);
%     else
%         SendSpeedCommand(0, 0, husky_config.control_channel);
%     end

pause(0.5)
disp("You're at the end of the loop");
end
