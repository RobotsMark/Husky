% Example glue-logic file from which you call your implementation of:
%  (1) Pole Detector
%  (2) Target Detector
%  (3) Planner
%  (4) Controller
%  -------------------
%  (5) SLAM [Note: full implementation is provided]

% Add MRG helper functions
% addpath('mrg'); % COMMENTED OUT

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

pause(20)
% Main loop
i = 1;

% scan = struct([]);
% stereo_images = struct([]);
% wheel_odometry = struct([]);

% scan = [];
% stereo_images = [];
% wheel_odometry = [];

%%
% while true
% Fetch latest messages from mex-moos
mailbox = mexmoos('FETCH');
scan = GetLaserScans(mailbox, config.laser_channel, false);
stereo_images = GetStereoImages(mailbox, config.stereo_channel, false);
wheel_odometry = GetWheelOdometry(mailbox, ...
                                  config.wheel_odometry_channel, ...
                                  false);

%     scan(i) = GetLaserScans(mailbox, config.laser_channel, true);
%     stereo_images(i) = GetStereoImages(mailbox, config.stereo_channel, true);
%     wheel_odometry(i) = GetWheelOdometry(mailbox, ...
%                                       config.wheel_odometry_channel, ...
%                                       true);
                                  
    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     % Display laser scan
%     subplot(1, 3, 1);
%     ShowLaserScan(scan);
% 
%     % Display stereo image
%     subplot(1, 3, 2);
%     ShowStereoImage(stereo_images)
% 
%     % Display undistorted stereo image
%     subplot(1, 3, 3);
%     ShowStereoImage(UndistortStereoImage(stereo_images, ...
%                                      config.camera_model));
                                 
%     savename = "full_scan" + i;
    % Display wheel odometry
    % disp(wheel_odometry);
%     save(savename, 'scan', 'stereo_images', 'wheel_odometry');
%     i = i + 1;
%     pause(1); % don't overload moos w/commands
% end
