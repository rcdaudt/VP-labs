function A = faugeras_compute_pm(X)
% Compute camera projection matrix from Faugeras X vector
% Daudt - 19/03/16

A = faugeras_compute_intrinsics(X)*faugeras_compute_extrinsics(X);
A = A/A(end:end);

end

