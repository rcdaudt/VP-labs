function intrinsics = faugeras_compute_intrinsics(X)
% Compute camera intrinsics from Faugeras X vector
% Daudt - 19/03/16

assert(size(X,1) == 11);

T1 = X(1:3)';
T2 = X(4:6)';
T3 = X(7:9)';
C1 = X(10);
C2 = X(11);

u0 = T1*T2'/(norm(T2)^2);
v0 = T2*T3'/(norm(T2)^2);
au = norm(cross(T1',T2'))/(norm(T2)^2);
av = norm(cross(T2',T3'))/(norm(T2)^2);

intrinsics = [  au, 0, u0, 0;
                0, av, v0, 0;
                0, 0, 1, 0];

end

