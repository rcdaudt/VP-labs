function A_hall= hall_cc(points,projections_sc)
% Camera calibration using Hall method
%   points - 4xN matrix with real word coordinates of points
%   projections - 3xN matrix with scaled projections
%   A_hall - output projection matrix
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
    IXui = projections_sc(1,i);
    IYui = projections_sc(2,i);
    Q(2*i-1,:) = [points(1,i) points(2,i) points(3,i) 1 0 0 0 0 -IXui*points(1,i) -IXui*points(2,i) -IXui*points(3,i)];
    Q(2*i,:) = [0 0 0 0 points(1,i) points(2,i) points(3,i) 1 -IYui*points(1,i) -IYui*points(2,i) -IYui*points(3,i)];
    B(2*i-1) = projections_sc(1,i);
    B(2*i) = projections_sc(2,i);
end
A = Q\B;
A_hall = [A(1) A(2) A(3) A(4);A(5) A(6) A(7) A(8);A(9) A(10) A(11) 1];

end

