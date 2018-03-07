function [ u ] = CalculateControlVector( delta_left, delta_right, L, angular_slipping)
    if nargin < 4, angular_slipping = 1; end
    
%     0.535
    
    L = 0.56;
    % L = 0.15;
    
    % disp('ml', delta_left)
    % disp('mr', delta_right)
    
    R = (delta_right + delta_left) / 2;
    theta = (delta_left-delta_right) / ( 2* L / angular_slipping );
    x = R * cos(theta);
    y = R * sin(theta);

    u = [x y theta]';
    
%     x = u(1);
%     y = u(2);
%     theta = u(3);
%     d_center = 0.5 * (d_left  + d_right);
%     phi = (d_right - d_left) / (L * 2);
%     theta_prime = theta + phi;
%     x_prime = x + d_center * cos(theta);
%     y_prime = y + d_center * sin(theta);
%     u = [x_prime, y_prime, theta_prime]';
end
