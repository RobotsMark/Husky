husky_id = 3; % Modify for your Husky
config = GetHuskyConfig(husky_id);
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); % Give mexmoos a chance to connect (important!)

startingX = [0.0,0.0,0.0];
startingP = diag([1,1,(1*pi/180)^2]);

% Main loop
mailbox = mexmoos('FETCH');
scan = GetLaserScans(mailbox, config.laser_channel, true);

% Display laser scan
figure(1);
ShowLaserScan(scan);

%initial conditions:
xEst = startingX;
PEst = startingP;

[x_new, P_new] = SLAMUpdate(u, z_raw, x, P)
