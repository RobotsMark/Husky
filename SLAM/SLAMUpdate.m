function [x_new, P_new] = SLAMUpdate(u, z_raw, x, P)

% laser_scan - raw laser scan from GetLaserScans()
% z_raw: observations, each in the form [range, bearing]
% x - state vector
% P - covariance matrix for 'x'
% u - relative vehicle motion estimate
%    / motion estimate (from odometry or control inputs)

% TODO: actually, need to run this even if we detect no poles; just run
% prediction?

[x_pred, P_pred] = SLAMPrediction(u, x, P);

if ~isempty(z_raw)
    z = SLAMDataAssociations(x_pred, z_raw');
    [x_new, P_new] = SLAMMeasurement(z, x_pred, P_pred);
else
    x_new = x_pred;
    P_new = P_pred;
end


% if ~isempty(z_raw)
%     [x_pred, P_pred] = SLAMPrediction(u, x, P);
%     z = SLAMDataAssociations(x_pred, z_raw');
%     [x_new, P_new] = SLAMMeasurement(z, x_pred, P_pred);
% else
%     x_new = x;
%     P_new = P;
% end

end

