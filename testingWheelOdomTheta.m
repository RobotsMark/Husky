
clear all;

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


mailbox = mexmoos('FETCH');
old_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
disp("Now Rotate")
pause(20)
disp("Stop Rotatng")                          
mailbox = mexmoos('FETCH');                              
new_wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  true);
                              
new_ml = new_wheel_odometry.m_l;
new_mr = new_wheel_odometry.m_r;
old_ml = old_wheel_odometry.m_l;
old_mr = old_wheel_odometry.m_r;
u = CalculateControlVector( new_ml-old_ml, new_mr-old_mr);

theta = u(3)*180/pi;
x_dist = u(1)