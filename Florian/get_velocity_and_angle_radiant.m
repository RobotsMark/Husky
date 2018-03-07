function return_value=get_velocity_and_angle_radiant(position_robot, position_sub_goal, orientation_robot)
%angle of 0 means facing upwards

difference_x = position_sub_goal(1)-position_robot(1);
difference_y = position_sub_goal(2)-position_robot(2);

absolute_angle=atan2(difference_y,difference_x)*(-1)+pi/2;
if(absolute_angle>pi)
    absolute_angle=absolute_angle-2*pi;
end


relative_angle=-orientation_robot+absolute_angle;
if(relative_angle>pi)
    relative_angle=relative_angle-2*pi;
elseif(relative_angle<-pi)
    relative_angle=relative_angle+2*pi;
end
%if positive then turn clockwise
%if negative then turn anticlockwise
%relative_angle /in [-pi, pi]

distance = sqrt(difference_x^2+difference_y^2);

if (relative_angle==0)
    clockwise=0;
elseif (relative_angle>0)
    clockwise=1;
else
    clockwise=-1;
end

return_value=[absolute_angle, relative_angle, distance, clockwise]';

end
