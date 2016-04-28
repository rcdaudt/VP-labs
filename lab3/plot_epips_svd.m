function [F,epip1_diff_F,epip2_diff_F] = plot_epips_svd(p1_n,p2_n,epip1,epip2,type)
% Calculate and plot epipolar lines, epipoles, etc.
% Daudt

% Compute fundamental matrix with 8 noisy points
F = compute_F_svd(p1_n,p2_n);
[Un,Sn,Vn] = svd(F);
% Sn(end,end) = 0;
% F_8_n = Un*Sn*Vn';


% Calculate and plot epipolar lines and epipoles
epip1_F = normalise_scale(Vn(:,end));
epip1_diff_F = norm(epip1 - epip1_F);
x = [min(0,epip1(1)-10),max(256,epip1(1))+10];
figure;
hold on;
grid on;
for i = 1:size(p1_n,2)
    scatter(p1_n(1,i),p1_n(2,i),'b');
    lm = F'*p2_n(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 256 256 0 0],[0 0 256 256 0],'k');
scatter(epip1(1),epip1(2),'gh','LineWidth',2);
scatter(epip1_F(1),epip1_F(2),'kh','LineWidth',2);
axis('equal');
title(strcat('Camera 1 - SVD ',type));


epip2_F = normalise_scale(Un(:,end));
epip2_diff_F = norm(epip2 - epip2_F);
x = [min(0,epip2(1))-10,max(256,epip2(1))+10];
figure;
hold on;
grid on;
for i = 1:size(p1_n,2)
    scatter(p2_n(1,i),p2_n(2,i),'b');
    lm = F*p1_n(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 256 256 0 0],[0 0 256 256 0],'k');
scatter(epip2(1),epip2(2),'gh','LineWidth',2);
scatter(epip2_F(1),epip2_F(2),'kh','LineWidth',2);
axis('equal');
title(strcat('Camera 2 - SVD ',type));


end

