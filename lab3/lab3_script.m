% Visual Perception Lab 3 script
% Rodrigo Daudt
% 31/03/16

clear all
close all
clc

%% Part 1

display('Part 1');

%% Step 1

au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;
sx1 = 256; sy1 = 256;
display('S1: Camera 1 parameters defined');

%% Step 2

au2 = 90; av2 = 110; uo2 = 128; vo2 = 128;
ax = 0.1; by = pi/4; cz = 0.2;
tx = -1000; ty = 190; tz = 230;
sx2 = 256; sy2 = 256;
display('S2: Camera 2 parameters defined');

%% Step 3

% Notation: Camera 1 is W and camera 2 is C
% Intrinsic matrices
I1 = [au1 0 uo1; 0 av1 vo1; 0 0 1];
I2 = [au2 0 uo2; 0 av2 vo2; 0 0 1];
% Rotation matrices
Rx = [1 0 0;0 cos(ax) -sin(ax);0 sin(ax) cos(ax)];
Ry = [cos(by) 0 sin(by);0 1 0;-sin(by) 0 cos(by)];
Rz = [cos(cz) -sin(cz) 0;sin(cz) cos(cz) 0;0 0 1];
WRC = Rx*Ry*Rz;
CRW = WRC';
% Translation vectors
T = [tx;ty;tz];
% t_W = -WRC*t_C;
% Transformation matrices
WTC = [WRC, T;0 0 0 1];
CTW = inv(WTC);

display('S3: Marices calculated');

%% Step 4

t_cross = [0 -T(3) T(2);T(3) 0 -T(1);-T(2) T(1) 0];
F = inv(I2)'*WRC'*t_cross*inv(I1);
F = F/F(3,3);
display('S4: Fundamental matrix calculated');

%% Step 5

clear V;
V(:,1) = [100;-400;2000;1];
V(:,2) = [300;-400;3000;1];
V(:,3) = [500;-400;4000;1];
V(:,4) = [700;-400;2000;1];
V(:,5) = [900;-400;3000;1];
V(:,6) = [100;-50;4000;1];
V(:,7) = [300;-50;2000;1];
V(:,8) = [500;-50;3000;1];
V(:,9) = [700;-50;4000;1];
V(:,10) = [900;-50;2000;1];
V(:,11) = [100;50;3000;1];
V(:,12) = [300;50;4000;1];
V(:,13) = [500;50;2000;1];
V(:,14) = [700;50;3000;1];
V(:,15) = [900;50;4000;1];
V(:,16) = [100;400;2000;1];
V(:,17) = [300;400;3000;1];
V(:,18) = [500;400;4000;1];
V(:,19) = [700;400;2000;1];
V(:,20) = [900;400;3000;1];

morepoints = [(800*rand(1,30)+100);(800*rand(1,30)-400);(2000*rand(1,30)+200);ones(1,30)];
V50 = [V morepoints];

morepoints = [(800*rand(1,100)+100);(800*rand(1,100)-400);(2000*rand(1,100)+200);ones(1,100)];
V150 = [V50 morepoints];

display('S5: Object points defined')

%% Step 6

A1 = [I1 zeros(3,1)]*[eye(3) zeros(3,1);0 0 0 1];
A2 = [I2 zeros(3,1)]*CTW;

% 20 points
p1 = zeros(3,size(V,2));
p2 = zeros(3,size(V,2));
for i = 1:size(V,2)
    p1(:,i) = A1*V(:,i);
    p2(:,i) = A2*V(:,i);
end
p1 = normalise_scale(p1);
p2 = normalise_scale(p2);

% 50 points
p1_50 = zeros(3,size(V50,2));
p2_50 = zeros(3,size(V50,2));
for i = 1:size(V50,2)
    p1_50(:,i) = A1*V50(:,i);
    p2_50(:,i) = A2*V50(:,i);
end
p1_50 = normalise_scale(p1_50);
p2_50 = normalise_scale(p2_50);

% 150 points
p1_150 = zeros(3,size(V150,2));
p2_150 = zeros(3,size(V150,2));
for i = 1:size(V150,2)
    p1_150(:,i) = A1*V150(:,i);
    p2_150(:,i) = A2*V150(:,i);
end
p1_150 = normalise_scale(p1_150);
p2_150 = normalise_scale(p2_150);

display('S6: Projections calculated')


%% Step 7

figure(1);
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 1 - 20 points');
figure(2);
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 2 - 20 points');
for i = 1:size(V,2)
    figure(1)
    scatter(p1(1,i),p1(2,i));
    figure(2)
    scatter(p2(1,i),p2(2,i));
end

figure(3);
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 1 - 50 points');
figure(4);
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 2 - 50 points');
for i = 1:size(V50,2)
    figure(3)
    scatter(p1_50(1,i),p1_50(2,i));
    figure(4)
    scatter(p2_50(1,i),p2_50(2,i));
end

figure(5);
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 1 - 150 points');
figure(6);
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 2 - 150 points');
for i = 1:size(V150,2)
    figure(5)
    scatter(p1_150(1,i),p1_150(2,i));
    figure(6)
    scatter(p2_150(1,i),p2_150(2,i));
end

display('S7: Projections displayed')

%% Step 8

F_8 = compute_F(p1,p2)

display('S8: Fundamental matrix F_8 was calculated using 8 points')

%% Step 9

ratio = F_8./F

display('S9: Result F_8 compared to F')

%% Step 10


epip1 = normalise_scale(A1*[T;1]);
x = [min(0,epip1(1))-10,max(256,epip1(1))+10];
figure;
% subplot(1,2,1);
hold on;
grid on;
for i = 1:size(p1,2)
    scatter(p1(1,i),p1(2,i),'b');
    lm = F_8'*p2(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k');
scatter(epip1(1),epip1(2),'gh','LineWidth',2);
axis('equal');
title('Camera 1');


epip2 = normalise_scale(A2*[0;0;0;1]);
x = [min(0,epip2(1))-10,max(256,epip2(1))+10];
% subplot(1,2,2);
figure;
hold on;
grid on;
for i = 1:size(p1,2)
    scatter(p2(1,i),p2(2,i),'b');
    lm = F_8*p1(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k');
scatter(epip2(1),epip2(2),'gh','LineWidth',2);
axis('equal');
title('Camera 2');

display('S10: Epipolar lines displayed')

%% Step 11

% Add gaussian noise to projections
p1_n = p1 + [0.5*randn(2,size(V,2)); zeros(1,size(V,2))];
p2_n = p2 + [0.5*randn(2,size(V,2)); zeros(1,size(V,2))];

p1_50_n = p1_50 + [0.5*randn(2,size(V50,2)); zeros(1,size(V50,2))];
p2_50_n = p2_50 + [0.5*randn(2,size(V50,2)); zeros(1,size(V50,2))];

p1_150_n = p1_150 + [0.5*randn(2,size(V150,2)); zeros(1,size(V150,2))];
p2_150_n = p2_150 + [0.5*randn(2,size(V150,2)); zeros(1,size(V150,2))];

display('S11: Noise added to projections');

%% Step 12

% Calculate everything and compare to true F
[F_mse_20,epip1_diff_F_mse_20,epip2_diff_F_mse_20] = plot_epips(p1_n,p2_n,epip1,epip2,'(20 points, noise [-1,1])')
ratio_mse_20 = F_mse_20./F

[F_mse_50,epip1_diff_F_mse_50,epip2_diff_F_mse_50] = plot_epips(p1_50_n,p2_50_n,epip1,epip2,'(50 points, noise [-1,1])')
ratio_mse_50 = F_mse_50./F

[F_mse_150,epip1_diff_F_mse_150,epip2_diff_F_mse_150] = plot_epips(p1_150_n,p2_150_n,epip1,epip2,'(150 points, noise [-1,1])')
ratio_mse_150 = F_mse_150./F



display('S12: Fundamental matrix calculated with noisy points')

%% Step 13

% Add gaussian noise to projections
p1_n2 = p1 + [randn(2,size(V,2)); zeros(1,size(V,2))];
p2_n2 = p2 + [randn(2,size(V,2)); zeros(1,size(V,2))];

p1_50_n2 = p1_50 + [randn(2,size(V50,2)); zeros(1,size(V50,2))];
p2_50_n2 = p2_50 + [randn(2,size(V50,2)); zeros(1,size(V50,2))];

p1_150_n2 = p1_150 + [randn(2,size(V150,2)); zeros(1,size(V150,2))];
p2_150_n2 = p2_150 + [randn(2,size(V150,2)); zeros(1,size(V150,2))];


% Calculate everything and compare to true F
[F_mse2_20,epip1_diff_F_mse2_20,epip2_diff_F_mse2_20] = plot_epips(p1_n2,p2_n2,epip1,epip2,'(20 points, noise [-2,2])')
ratio_mse2_20 = F_mse2_20./F

[F_mse2_50,epip1_diff_F_mse2_50,epip2_diff_F_mse2_50] = plot_epips(p1_50_n2,p2_50_n2,epip1,epip2,'(50 points, noise [-2,2])')
ratio_mse2_50 = F_mse2_50./F

[F_mse2_150,epip1_diff_F_mse2_150,epip2_diff_F_mse2_150] = plot_epips(p1_150_n2,p2_150_n2,epip1,epip2,'(150 points, noise [-2,2])')
ratio_mse2_150 = F_mse2_150./F



display('S13: Fundamental matrix calculated with noisier points')

%% Part 2

display('Part 2');

%% Step 14

F_8_svd = compute_F_svd(p1,p2);

ratio_svd = F_8_svd./F_8

display('S14: Fundamental matrix calculated using SVD');

%% Step 15-1

% Calculate everything and compare to true F
[F_svd_20,epip1_diff_F_svd_20,epip2_diff_F_svd_20] = plot_epips_svd(p1_n,p2_n,epip1,epip2,'(20 points, noise [-1,1])')
ratio_svd_20 = F_svd_20./F

[F_svd_50,epip1_diff_F_svd_50,epip2_diff_F_svd_50] = plot_epips_svd(p1_50_n,p2_50_n,epip1,epip2,'(50 points, noise [-1,1])')
ratio_svd_50 = F_svd_50./F

[F_svd_150,epip1_diff_F_svd_150,epip2_diff_F_svd_150] = plot_epips_svd(p1_150_n,p2_150_n,epip1,epip2,'(150 points, noise [-1,1])')
ratio_svd_150 = F_svd_150./F


display('S15-1: Fundamental matrix calculated with noisy points and SVD')

%% Step 15-2

% Calculate everything and compare to true F
[F_svd2_20,epip1_diff_F_svd2_20,epip2_diff_F_svd2_20] = plot_epips_svd(p1_n2,p2_n2,epip1,epip2,'(20 points, noise [-2,2])')
ratio_svd2_20 = F_svd2_20./F

[F_svd2_50,epip1_diff_F_svd2_50,epip2_diff_F_svd2_50] = plot_epips_svd(p1_50_n2,p2_50_n2,epip1,epip2,'(50 points, noise [-2,2])')
ratio_svd2_50 = F_svd2_50./F

[F_svd2_150,epip1_diff_F_svd2_150,epip2_diff_F_svd2_150] = plot_epips_svd(p1_150_n2,p2_150_n2,epip1,epip2,'(150 points, noise [-2,2])')
ratio_svd2_150 = F_svd2_150./F



display('S15-2: Fundamental matrix calculated with noisier points and SVD')

%% Step 16


% 20 points
dist_ls = 0;
dist_svd = 0;
for i = 1:size(p1,2)
    % Least squares total distance
    lm = F_mse_20'*p2_n(:,i);
    dist_ls = dist_ls + abs(lm(1)*p1_n(1,i) + lm(2)*p1_n(2,i) + lm(3))/norm(lm(1:2));
    lm = F_mse_20*p1_n(:,i);
    dist_ls = dist_ls + abs(lm(1)*p2_n(1,i) + lm(2)*p2_n(2,i) + lm(3))/norm(lm(1:2));
    
    % SVD total distance
    lm = F_svd_20'*p2_n(:,i);
    dist_svd = dist_svd + abs(lm(1)*p1_n(1,i) + lm(2)*p1_n(2,i) + lm(3))/norm(lm(1:2));
    lm = F_svd_20*p1_n(:,i);
    dist_svd = dist_svd + abs(lm(1)*p2_n(1,i) + lm(2)*p2_n(2,i) + lm(3))/norm(lm(1:2));
end
mean_dist_ls_20 = dist_ls / (2 * size(p1,2))
mean_dist_svd_20 = dist_svd / (2 * size(p1,2))


% 50 points
dist_ls = 0;
dist_svd = 0;
for i = 1:size(p1_50,2)
    % Least squares total distance
    lm = F_mse_50'*p2_50_n(:,i);
    dist_ls = dist_ls + abs(lm(1)*p1_50_n(1,i) + lm(2)*p1_50_n(2,i) + lm(3))/norm(lm(1:2));
    lm = F_mse_50*p1_50_n(:,i);
    dist_ls = dist_ls + abs(lm(1)*p2_50_n(1,i) + lm(2)*p2_50_n(2,i) + lm(3))/norm(lm(1:2));
    
    % SVD total distance
    lm = F_svd_50'*p2_50_n(:,i);
    dist_svd = dist_svd + abs(lm(1)*p1_50_n(1,i) + lm(2)*p1_50_n(2,i) + lm(3))/norm(lm(1:2));
    lm = F_svd_50*p1_50_n(:,i);
    dist_svd = dist_svd + abs(lm(1)*p2_50_n(1,i) + lm(2)*p2_50_n(2,i) + lm(3))/norm(lm(1:2));
end
mean_dist_ls_50 = dist_ls / (2 * size(p1_50,2))
mean_dist_svd_50 = dist_svd / (2 * size(p1_50,2))


% 150 points
dist_ls = 0;
dist_svd = 0;
for i = 1:size(p1_150,2)
    % Least squares total distance
    lm = F_mse_150'*p2_150_n(:,i);
    dist_ls = dist_ls + abs(lm(1)*p1_150_n(1,i) + lm(2)*p1_150_n(2,i) + lm(3))/norm(lm(1:2));
    lm = F_mse_150*p1_150_n(:,i);
    dist_ls = dist_ls + abs(lm(1)*p2_150_n(1,i) + lm(2)*p2_150_n(2,i) + lm(3))/norm(lm(1:2));
    
    % SVD total distance
    lm = F_svd_150'*p2_150_n(:,i);
    dist_svd = dist_svd + abs(lm(1)*p1_150_n(1,i) + lm(2)*p1_150_n(2,i) + lm(3))/norm(lm(1:2));
    lm = F_svd_150*p1_150_n(:,i);
    dist_svd = dist_svd + abs(lm(1)*p2_150_n(1,i) + lm(2)*p2_150_n(2,i) + lm(3))/norm(lm(1:2));
end
mean_dist_ls_150 = dist_ls / (2 * size(p1_150,2))
mean_dist_svd_150 = dist_svd / (2 * size(p1_150,2))


display('S16: Mean distance between epipolar lines and points calculated for LS and SVD methods');