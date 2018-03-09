function [x_rob, y_rob] = dist_from_target(left_image, right_image, fig_handle)
% x_rob and y_rob are coords in centre of robot frame
[lx, ly, rx, ry] = locate_red_t(left_image, right_image,fig_handle);

if ~(isempty(lx) || isempty(rx))
    [~, y_rob, x_rob, d, theta] = estimateDistance(lx, rx);
    
    
    
    
else
    z = [];
    x_rob = [];
    y_rob = [];
    d = [];
    theta = [];
end


end

