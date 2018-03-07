function [ angle ] = AngleDifference( target, source )
%% ANGLEDIFFERENCE
%
% Computes the difference between angles and wraps it to [-pi, pi].
%

% Jeff Hawke
% February 2015
% Mobile Robotics Group, Oxford University.

%% Compute difference
target = wrapTo2Pi(target);
source = wrapTo2Pi(source);
angle = atan2(sin(target-source), cos(target-source));

end

