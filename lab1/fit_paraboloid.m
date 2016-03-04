function [dx,dy] = fit_paraboloid(M)
% Fit a conic over 3x3 matrix
% z = a*x*x + b*y*y + c*x*y + d*x + e*y + f

A = [];
b = [];

for x = 1:3
    for y = 1:3
        v = [x*x y*y x*y x y 1];
        A = [A;v];
        b = [b;M(x,y)];
    end
end

% Fit using MSE
fit = A\b; % fit = [a b c d e f]'

% Solve system to find maximum point
maximum = [2*fit(1) fit(3);fit(3) 2*fit(2)]\[-fit(4);-fit(5)]; % maxium = [x_max;y_max]

% Find dx and dy (difference between coordinates of current maximum pixel
% and estimated maximum position using paraboloid)
if fit(1)<0 && fit(2)<0
    dx = maximum(1) - 2;
    dx = max(min(dx,0.5),-0.5);
    dy = maximum(2) - 2;
    dy = max(min(dy,0.5),-0.5);
else
%     a = fit(1)
%     b = fit(2)
    dx = 0;
    dy = 0;
end

end

