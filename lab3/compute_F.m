function F = compute_F(p1,p2)
% Compute fundamental matrix based on projections on 2 different cameras

assert(size(p1,2) == size(p2,2))
assert(size(p1,2) > 7)
assert(size(p1,1) == size(p2,1))
assert(size(p1,1) == 3)

p1_n = normalise_scale(p1);
p2_n = normalise_scale(p2);

% Fv = [F11 F12 F13 F21 F22 F23 F31 F32]'
Q = zeros(size(p1,2),8);
b = -ones(size(p1,2),1);
for i = 1:size(p1,2)
    Q(i,:) = [p2_n(1,i)*p1_n(1,i) p2_n(1,i)*p1_n(2,i) p2_n(1,i) p2_n(2,i)*p1_n(1,i) p2_n(2,i)*p1_n(2,i) p2_n(2,i) p1_n(1,i) p1_n(2,i)];
end
Fv = Q\b;

F = [Fv(1) Fv(2) Fv(3);Fv(4) Fv(5) Fv(6);Fv(7) Fv(8) 1];

end

