function [z, x, z_correct, d, theta] = estimateDistance(lx, rx)
    % See figure at bottom for details http://www.dis.uniroma1.it/~iocchi/stereo/triang.html
    
    % Input 
    %   lx is the x co-ordinate of the point of interest from the left camera
    %   rx is the x co-ordinate of the point of interest from the right camera
    
    % Ouput
    %   z - forward coord of target from camera
    %   z_correct - forward coord from center of robot
    %   x - sideways coord of target from camera
    %   d - target distance from robot
    %   theta - angle from robot 
    
    b = 0.12; % base line (distance between cameras in meters)
    image_width = 512; % image width in pixels
    HFOV = 97; % horizontal field of view
    
    lx = lx - image_width/2;
    rx = rx - image_width/2;
    
    f = (image_width * 0.5) / tan((HFOV * 0.5 * pi)/180); % focal length in pixels
    d = lx - rx; % disparature 
    z = (f*b)/d; % z distance in meters
    x = lx*z/f;  % x distance in meters
    
    % height of camera
    huskey_height = 0.39;
    silver_pole_height = 0.5;
    h = huskey_height + silver_pole_height;
    
    % z correction
    if z > h
        z_correct = sqrt(z^2 - h^2);
    else
        z_correct = 0; 
    end
    
    % husky correction
    z_correct = z_correct + 0.544/2;
    
    
    % distance to target
    d = sqrt(z_correct^2 + x^2);
    
    % angle of target relative to camera facing
    if z_correct > 0 && x > 0 
        theta = atan(x/z_correct);
    elseif z > 0
        theta = atan(x/z);
    else
        theta = 0;
    end
end

