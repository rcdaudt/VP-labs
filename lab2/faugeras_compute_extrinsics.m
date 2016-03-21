function extrinsics = faugeras_compute_extrinsics(X)
% Compute camera extrinsics from Faugeras X vector
% Daudt - 19/03/16

assert(size(X,1) == 11);

T1 = X(1:3)';
T2 = X(4:6)';
T3 = X(7:9)';
C1 = X(10);
C2 = X(11);

extrinsics = zeros(4);

extrinsics(1,1:3) = (norm(T2)/norm(cross(T1',T2')))*(T1-((T1*T2')/(norm(T2)^2))*T2); % r1
extrinsics(2,1:3) = (norm(T2)/norm(cross(T2',T3')))*(T3-((T2*T3')/(norm(T2)^2))*T2); % r2
extrinsics(3,1:3) = T2/norm(T2); % r3

extrinsics(1,4) = (norm(T2)/norm(cross(T1',T2')))*(C1-((T1*T2')/(norm(T2)^2))); % tx
extrinsics(2,4) = (norm(T2)/norm(cross(T2',T3')))*(C2-((T2*T3')/(norm(T2)^2))); % ty
extrinsics(3,4) = 1/norm(T2); % tz

extrinsics(4,4) = 1;


end

