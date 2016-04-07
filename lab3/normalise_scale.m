function normalised_points = normalise_scale(points)
% Normalises the scale of a set of points

normalised_points = zeros(size(points));
for i = 1:size(points,1)
    normalised_points(i,:) = points(i,:)./points(end,:);
end

end

