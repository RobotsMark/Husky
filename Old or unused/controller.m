% Example file demonstrating how to send movement commands to the Husky
%   Precondition: run all processes via Mission Control and press Y-A on
%   the remote control

% Add MRG helper functions
%addpath('mrg');

% Set the Husky ID
husky_id = 3;

% Get the Husky configuration, including the MOOS channel names.
husky_config = GetHuskyConfig(husky_id);

% Initialise mex-moos and register subscribers
clear mexmoos;
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', husky_config.host, 'MOOSNAME', client, 'SERVERPORT','9000');
mexmoos('REGISTER', husky_config.wheel_odometry_channel, 0.0);
pause(1.0); % give mexmoos a chance to connect (important!)

% First tell it not to move at all
SendSpeedCommand(0, 0, husky_config.control_channel)                                  
velocity = 0;
angle = 0.5;

test_time = 2*2*pi;

tic
while toc < 5
    mailbox = mexmoos('FETCH');
    SendSpeedCommand(velocity, angle, husky_config.control_channel);
    disp(toc);
    pause(0.1);
end

SendSpeedCommand(0, 0, husky_config.control_channel)                                  
mailbox = mexmoos('FETCH');
wheel_odometry = GetWheelOdometry(mailbox, husky_config.wheel_odometry_channel, true);
ml_0 = wheel_odometry.m_l;
mr_0 = wheel_odometry.m_r;
                                                                 
tic
while toc < test_time + 0.1
    mailbox = mexmoos('FETCH');
    
    if toc <= test_time
        SendSpeedCommand(velocity, angle, husky_config.control_channel);
    end
    
    wheel_odometry = GetWheelOdometry(mailbox, husky_config.wheel_odometry_channel, true);

    if (length(wheel_odometry) > 0)
        ml = wheel_odometry.m_l - ml_0;
        mr = wheel_odometry.m_r - mr_0; 
        disp(CalculateControlVector(ml, mr));
        disp(toc);
    end                              
                               
    pause(0.1); % don't overload moos w/commands
end