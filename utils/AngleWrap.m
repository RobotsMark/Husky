function a = Anglemod(b)
%% ANGLEMOD
%
% Reduces an angle to the range [-pi,pi].
%

% Chi Hay Tong
% February 2015
% Mobile Robotics Group, Oxford University.

%% Perform modulus
m2pi = 2*pi;
a = b - (m2pi * round(b/m2pi));

end

