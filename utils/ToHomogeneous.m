function h_points = ToHomogeneous(points)

h_points = [points; ones([1 size(points,2)])];

end