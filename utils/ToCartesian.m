function points = ToCartesian(h_points)
points = h_points(1:end-1,:) ./ h_points(end,:);
end