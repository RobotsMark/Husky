function [x_rob, y_rob] = dist_from_target(left_image, right_image, target_fig)
% x_rob and y_rob are coords in centre of robot frame
[lx, ly, rx, ry] = locate_red_t(left_image, right_image,target_fig);

if ~(isempty(lx) || isempty(rx))
    [z, x_rob, y_rob, d, theta] = estimateDistance(lx, rx);
    
    
    
    
else
    z = [];
    x_rob = [];
    y_rob = [];
    d = [];
    theta = [];
end


end

