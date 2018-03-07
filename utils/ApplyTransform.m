function points = ApplyTransform(transform, points)
points = ToCartesian(...
  BuildSE2Transform(transform) * ToHomogeneous(points));
end

