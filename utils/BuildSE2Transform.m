function [se2] = BuildSE2Transform(xyyaw)

% [se2] = BuildSE2Transform(xyyaw)

R = [ cos(xyyaw(3)), -sin(xyyaw(3));
      sin(xyyaw(3)), cos(xyyaw(3)) ];

se2 = zeros(3,3);
se2(1:2,1:2) = R;
tmp = xyyaw(1:2);
se2(1:2,3) = tmp(:);
se2(3,3) = 1;

end