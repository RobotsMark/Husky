function [ h ] = circle(x,y,r)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,'r');
hold off
end

