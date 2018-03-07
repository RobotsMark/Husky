
%%%MAIN PROGRAM
%
% 1. get obstacles
% 2. call SLAM
% 3. get goal
% 4. call path_planner
% 5. call control
%
%%%



%%%Initialization

husky_id = 3; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);



% Initialise mex-moos and register channels
% clear mexmoos % not needed b/c matlab will check if it has already been
                % initalised UNLESS you changed one of the five lines below
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); % Give mexmoos a chance to connect (important!)


%%%Initialize variable
stage1=true;
current_position=[0,0];
current_orientation=0;
goal_position=[0, 5];
observations=[1,1]; %%change!!!!
current_pose=[0;0;0];
P_new=[];

%true if we want to call path_planner to get a new sub_goal
update_subgoal=true;

goal_count=30;
goal_count_target=30; %how often do we update sub_goal

mailbox = mexmoos('FETCH');
wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
ml_0 = wheel_odometry.m_l;
mr_0 = wheel_odometry.m_r;
ml=ml_0;
mr=mr_0;


% Main loop
while stage1
    
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
    if (length(wheel_odometry) > 0)
        ml = wheel_odometry.m_l - ml_0;
        mr = wheel_odometry.m_r - mr_0;
    end
                                  
%%%%%%%%%%%%% Display sensor data    
% % %     % Display laser scan
% % %     subplot(1, 3, 1);
% % %     ShowLaserScan(scan);
% % %     
% % %     % Display stereo image
% % %     subplot(1, 3, 2);
% % %     ShowStereoImage(stereo_images)
% % %     
% % %     % Display undistorted stereo image
% % %     subplot(1, 3, 3);
% % %     ShowStereoImage(UndistortStereoImage(stereo_images, ...
% % %                                          config.camera_model));
% % %     
% % %     % Display wheel odometry
% % %     disp(wheel_odometry);
    
  



    %%%TODO: get obstacles
    observations=get_poles(scan);

    
    relative_motion=CalculateControlVector(ml, mr);
    ml_0=ml;
    mr_0=mr;
    

    %%%TODO: call SLAM
    [current_pose, P_new] = SLAMUpdate(relative_motion, observations, current_pose, P_new);
    current_position=current_pose(1:2);
    current_orientation=current_pose(3);

    %%%TODO: call goal function
    %if confident then turn target_information into x,y coordinates
    target_information=get_target();
    goal_position=[0,5];




    %%%TODO: call path planner
% % % %     if(current_position(2)>5.5)
% % % %         subgoal_position=goal_position;
% % % %     else
% % % %         if(goal_count>=goal_count_target)
% % % %             subgoal_position=path_planner(current_position, goal_position, observations);
% % % %             goal_count=0;
% % % %         else
% % % %             goal_count = goal_count +1;
% % % %         end
% % % %     end
    if(current_position(2)>5.5)
        subgoal_position=goal_position;
    elseif(update_subgoal)
       subgoals_position=path_planner(current_position, goal_position, observations);
       subgoal_position=subgoals_position(1,:);
       update_subgoal=false;
    end
    
    




    %%%TODO: Controller
    %[absolute_angle, relative_angle, distance, clockwise]=get_velocity_and_angle_radiant(current_position, subgoal_position, current_orientation);
    velocity_angle=get_velocity_and_angle_radiant(current_position, subgoal_position, current_orientation);
    relative_angle=velocity_angle(2);
    distance=velocity_angle(3);
    clockwise=velocity_angle(4);
    
    
    epsilon_angle=pi/30;
    epsilon_distance=0.2;

    if(abs(relative_angle)>epsilon_angle)
        SendSpeedCommand(0, clockwise, husky_config.control_channel);
    elseif(distance>epsilon_distance)
        SendSpeedCommand(1, 0, husky_config.control_channel);
    else
        SendSpeedCommand(0, 0, husky_config.control_channel);
    end



pause(0.1); % don't overload moos w/commands
end

    
%TODO: 
%find values for epsilon_angle, epsilon_distance, goal_count_target

%if past 5m line go straight to goal


%call planner if you've reached subgoal
%update if target found!


%turn target value into x,y coordinates


