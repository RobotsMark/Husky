function points = ApplyInverseTransform(transform, points)
points = ToCartesian(...
  BuildSE2Transform(transform) \ ToHomogeneous(points));
end

