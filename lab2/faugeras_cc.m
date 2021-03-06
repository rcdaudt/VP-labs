function X = faugeras_cc(points,projections_sc)
% Camera calibration using Faugeras method
%   points - 4xN matrix with real word coordinates of points
%   projections - 3xN matrix with scaled projections
%   X - output X vector
% Daudt - 19/03/16

num_points = size(points,2);

assert(size(points,1) == 4);
assert(num_points >= 6);
assert(size(projections_sc,2) == num_points);

if min(projections_sc(3,:)) ~= 1 || max(projections_sc(3,:)) ~= 1
    for i = 1:3
        projections_sc(i,:) = projections_sc(i,:)./projections_sc(3,:);
    end
end

Q = zeros(2*num_points,11);
B = zeros(2*num_points,1);
for i = 1:num_points
    Q(2*i-1,:) = [points(1:3,i)' (-projections_sc(1,i)*points(1:3,i)') 0 0 0 1 0];
    Q(2*i,:) = [0 0 0 (-projections_sc(2,i)*points(1:3,i)') points(1:3,i)' 0 1];
    B(2*i-1) = projections_sc(1,i);
    B(2*i) = projections_sc(2,i);
end

X = Q\B;

end

