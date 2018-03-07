function [t_rel] = ComposeRelativePoses(poses)
% composes all the relative poses contained within 'poses' data structure
% (from GetRelativePoses function) into a single relative pose
    t_rel = [0 0 0]';
    if ~isempty(poses)
        for i = 1:length(poses)
            xyzrpy = poses(1).xyzrpy;
            t_rel = tcomp(t_rel, [xyzrpy(1) xyzrpy(2) xyzrpy(6)]');
        end
    end
end