function husky_config = GetHuskyConfig( husky_id )
%
% husky_config = GetHuskyConfig( husky_id)
%
% Get Husky specific sensor IDs and channels
%
% INPUTS:
%   husky_id - The ID of the Husky as a number (e.g 4 for husky4)
%
% OUTPUTS:
%   husky_config - Struct of sensor details with the following fields:
%                  > camera_model - The camera model for undistorting 
%                                   images
%                  > laser_channel - The MOOS channel name for laser scans
%                  > stereo_channel - The MOOS channel name for stereo
%                                     images
%                  > wheel_odometry_channel - The MOOS channel name for 
%                                             wheel odometry
%                  > host - The IP address of the Husky LLC on its internal
%                           network
%                  > control_channel - The MOOS channel name for the 
%                                      control commands being sent to the 
%                                      Husky.
%
% Simon Chadwick
% February 2017
% Oxford Robotics Institute, Oxford University.

if ~isnumeric(husky_id)
    error('husky_id parameter must be a number');
end
    
switch husky_id
    case 1
        model_struct = load('full-size/BB2-14366971.mat'); % Load camera model % checked
%         husky_config.laser_channel = 'LMS1xx_11360134_laser2d';
    case 2
        model_struct = load('half-size/BB2-13161009.mat'); % Load camera model % checked
%         husky_config.laser_channel = 'LMS1xx_14280167_laser2d'; % checked
    case 3
        model_struct = load('full-size/BB2-11441506.mat'); % Load camera model % checked
%         husky_config.laser_channel = 'LMS1xx_14270167_laser2d'; % checked
    case 4
        model_struct = load('full-size/BB2-14366960.mat'); % Load camera model % checked
%         husky_config.laser_channel = 'LMS1xx_14320092_laser2d';
    otherwise
        error('Invalid Husky ID. ID = %d\n', husky_id);    
end
husky_config.laser_channel = 'LASER_SCANS';


husky_config.camera_model = model_struct.camera_model;
husky_config.wheel_odometry_channel = 'cdt_week_husky_encoder_state';
husky_config.stereo_channel = 'BUMBLEBEE2_IMAGES';
husky_config.host = '192.168.0.14';
husky_config.control_channel = 'husky_plan';

end

