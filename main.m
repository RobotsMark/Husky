% Use the laser scan to generate observations

initialise_state = [0.0, 0.0, 0.0]';
initialise_covariance = diag([0.01,0.01,0.01]);
initialise_goal = [5.0,0.0,0.0]';

statevector = initialise_state;
covariance = initialise_covariance;

[old_l_wheel, old_r_wheel, old_delta_theta] = getWheelOdom


goal = initialise_goal ;
confidence_thresh = 576678;
    
% loop
for 

% Get data
scan = getLaserScan;
images = getStereoImages;

% Get features from laser
observations = getPoleCoords(scan);

% Get relative motion
[new_l_wheel, new_r_wheel, new_delta_theta] = getWheelOdom;
[x_trav,y_trav, theta_change] = CalculateControlVector( delta_left, delta_right);
[old_l_wheel, old_r_wheel, old_delta_theta] = [new_l_wheel, new_r_wheel, new_delta_theta];

% Update using SLAM
[statevector, covariance] = SLAMUpdate(relative_motion, observations, ...
    statevector, covariance);

% Target detection
[rel_target_coords, confidence_value] = targetDetection();

if confidence_value > confidence_thresh
    world_target = transform(rel_target_coords, statevector);
    goal = world_target;
end

% Plan path
subgoal = pathPlanner(statevector,goal);    
sendCommands();


relative_motion = fromTimCode(wheel_odom);


end
