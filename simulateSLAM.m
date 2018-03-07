% Simulate 

close all; clear all;

initialise_state = [0,0,0]';
initialise_covariance = diag([0.01,0.01,0.01]);


statevector = initialise_state;
covariance = initialise_covariance;
%%

% Create random features
% pole_x = 10*rand(5,1);
% pole_y = 10*rand(5,1);

pole_x = [4.503238127334530;5.824703017362872;6.866378144963033;7.194327525959982;6.500407519253908]; 
pole_y = [7.269145509126521;3.738476658399816;5.815820832128658;1.161185127794003;0.576543612148722];

poles = [pole_x,pole_y];


% initial graphics - plot true map
figure(1); hold on; grid off; axis equal;
plot(statevector(1),statevector(2),'b*');hold on;
plot(poles(:,1),poles(:,2),'g*');hold on;
set(gcf,'doublebuffer','on');
hObsLine = line([0,0],[0,0]);
set(hObsLine,'linestyle',':');

%%
test_u = [0,1,45*pi/180;
    1,0,45*pi/180;
    0,-1,-45*pi/180;
    -1,0,-45*pi/180;
    0.5,0.5,45*pi/180];
% 
% test_u = [2,3,0;
%     -1,-1,0;
%     3,-3,0;
%     4,5,0;
%     1,-1,0];
% 

% test_u = [2,3,1*pi/180;
%     -1,-1,45*pi/180;
%     3,-3,-4*pi/180;
%     4,5,-10*pi/180;
%     1,-1,10*pi/180];

%%%%%%% The u_x and u_y here are in the local frame!!!%%%%%%
% test_u = [0,0,10*pi/180;
%     0,0,10*pi/180;
%     0,0,-10*pi/180;
%     4,2,-10*pi/180;
%     1,2,10*pi/180];

list_theta = [];
list_theta_diff = [];
list_bearingToPole = [];


list_x = [statevector(1)];
list_y = [statevector(2)];

for i = 1:5
rel_mot = [0;0;-statevector(3)];
worldchange = tcomp(rel_mot,test_u(i,:)');
% x_vehicle = statevector(1) + worldchange(1);
% y_vehicle = statevector(2) + worldchange(2);
% theta_vehicle = statevector(3) + test_u(i,3);

localChange = [worldchange(1);worldchange(2);test_u(i,3)];

x_vehicle = statevector(1) + test_u(i,1);
y_vehicle = statevector(2) + test_u(i,2);
theta_vehicle = statevector(3) + test_u(i,3);

x_pole = poles(i,1);
y_pole = poles(i,2);
diff = [x_pole - x_vehicle, y_pole - y_vehicle];
range = norm(diff);
theta = atan2(diff(2),diff(1));
% theta = atan(diff(2)/diff(1));
% 
% sign_x = sign(diff(1));
% sign_y = sign(diff(2));
% 
% if sign_x <0
%     if sign_y>0
%         theta = theta + pi/2;
%     else 
%         theta = theta + pi;
%     end
%         
% end


theta_diff = theta - theta_vehicle;

% This needs to be wrapped
bearingToPole = wrapToPi(theta_diff);
observations = [range,bearingToPole];
% 
list_theta = [list_theta,theta];
list_theta_diff = [list_theta_diff,theta_diff];
list_bearingToPole = [list_bearingToPole, bearingToPole];

[statevector, covariance] = SLAMUpdate(localChange, observations', ...
    statevector, covariance);

% [statevector, covariance] = SLAMUpdate(test_u(i,:)', observations', ...
%     statevector, covariance);
plot(statevector(1),statevector(2),'b*');hold on;
plot(statevector(2+2*i,:),statevector(3+2*i,:),'r.');hold on;
% 
list_x = [list_x, list_x(i) + test_u(i,1)];
list_y = [list_y, list_y(i) + test_u(i,2)];

end


