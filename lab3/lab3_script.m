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
display('S5: Object points defined')

%% Step 6

A1 = [I1 zeros(3,1)]*[eye(3) zeros(3,1);0 0 0 1];
A2 = [I2 zeros(3,1)]*CTW;
p1 = zeros(3,size(V,2));
p2 = zeros(3,size(V,2));
for i = 1:size(V,2)
    p1(:,i) = A1*V(:,i);
    p2(:,i) = A2*V(:,i);
end
p1 = normalise_scale(p1);
p2 = normalise_scale(p2);
display('S6: Projections calculated')


%% Step 7

figure(1);
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 1');
figure(2);
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k','LineWidth',3);
hold on;
grid on;
axis('equal');
title('Camera 2');
for i = 1:size(V,2)
    figure(1)
    scatter(p1(1,i),p1(2,i));
    figure(2)
    scatter(p2(1,i),p2(2,i));
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

display('S11: Noise added to projections');

%% Step 12

% Compute fundamental matrix with 8 noisy points
F_8_n = compute_F(p1_n,p2_n);
[Un,Sn,Vn] = svd(F_8_n);
Sn(end,end) = 0;
F_8_n = Un*Sn*Vn';

% Compare to true F
ratio_n = F_8_n./F


% Calculate and plot epipolar lines and epipoles
epip1 = normalise_scale(A1*[T;1]);
x = [min(0,epip1(1)-10),max(256,epip1(1))+10];
figure;
% subplot(1,2,1);
hold on;
grid on;
for i = 1:size(p1_n,2)
    scatter(p1_n(1,i),p1_n(2,i),'b');
    lm = F_8_n'*p2_n(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k');
scatter(epip1(1),epip1(2),'gh','LineWidth',2);
axis('equal');
title('Camera 1 (noisy [-1,1])');


epip2 = normalise_scale(A2*[0;0;0;1]);
x = [min(0,epip2(1))-10,max(256,epip2(1))+10];
% subplot(1,2,2);
figure;
hold on;
grid on;
for i = 1:size(p1_n,2)
    scatter(p2_n(1,i),p2_n(2,i),'b');
    lm = F_8_n*p1_n(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k');
scatter(epip2(1),epip2(2),'gh','LineWidth',2);
axis('equal');
title('Camera 2 (noisy [-1,1])');

display('S12: Fundamental matrix calculated with noisy points')

%% Step 13

% Add gaussian noise to projections
p1_n2 = p1 + [randn(2,size(V,2)); zeros(1,size(V,2))];
p2_n2 = p2 + [randn(2,size(V,2)); zeros(1,size(V,2))];

% Compute fundamental matrix with 8 noisy points
F_8_n2 = compute_F(p1_n2,p2_n2);
[Un,Sn,Vn] = svd(F_8_n2);
Sn(end,end) = 0;
F_8_n2 = Un*Sn*Vn';

% Compare to true F
ratio_n2 = F_8_n2./F


% Calculate and plot epipolar lines and epipoles
epip1 = normalise_scale(A1*[T;1]);
x = [min(0,epip1(1)-10),max(256,epip1(1))+10];
figure;
% subplot(1,2,1);
hold on;
grid on;
for i = 1:size(p1_n2,2)
    scatter(p1_n2(1,i),p1_n2(2,i),'b');
    lm = F_8_n2'*p2_n2(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k');
scatter(epip1(1),epip1(2),'gh','LineWidth',2);
axis('equal');
title('Camera 1 (noisy [-2,2])');


epip2 = normalise_scale(A2*[0;0;0;1]);
x = [min(0,epip2(1))-10,max(256,epip2(1))+10];
% subplot(1,2,2);
figure;
hold on;
grid on;
for i = 1:size(p1_n2,2)
    scatter(p2_n2(1,i),p2_n2(2,i),'b');
    lm = F_8_n2*p1_n2(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k');
scatter(epip2(1),epip2(2),'gh','LineWidth',2);
axis('equal');
title('Camera 2 (noisy [-2,2])');

display('S13: Fundamental matrix calculated with noisier points')

%% Part 2

display('Part 2');

%% Step 14

F_8_svd = compute_F_svd(p1,p2);

ratio_svd = F_8_svd./F_8

display('S14: Fundamental matrix calculated using SVD');

%% Step 15

% Compute fundamental matrix with 8 noisy points
F_8_n_svd = compute_F_svd(p1_n,p2_n);
[Un,Sn,Vn] = svd(F_8_n_svd);
Sn(end,end) = 0;
F_8_n_svd = Un*Sn*Vn';

% Compare to true F
ratio_n = F_8_n_svd./F


% Calculate and plot epipolar lines and epipoles
epip1 = normalise_scale(A1*[T;1]);
x = [min(0,epip1(1)-10),max(256,epip1(1))+10];
figure;
% subplot(1,2,1);
hold on;
grid on;
for i = 1:size(p1_n,2)
    scatter(p1_n(1,i),p1_n(2,i),'b');
    lm = F_8_n_svd'*p2_n(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k');
scatter(epip1(1),epip1(2),'gh','LineWidth',2);
axis('equal');
title('Camera 1 (noisy [-1,1] SVD)');


epip2 = normalise_scale(A2*[0;0;0;1]);
x = [min(0,epip2(1))-10,max(256,epip2(1))+10];
% subplot(1,2,2);
figure;
hold on;
grid on;
for i = 1:size(p1_n,2)
    scatter(p2_n(1,i),p2_n(2,i),'b');
    lm = F_8_n_svd*p1_n(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k');
scatter(epip2(1),epip2(2),'gh','LineWidth',2);
axis('equal');
title('Camera 2 (noisy [-1,1])');

display('S15-1: Fundamental matrix calculated with noisy points and SVD')

%% Step 15-2


% Compute fundamental matrix with 8 noisy points
F_8_n_svd2 = compute_F(p1_n2,p2_n2);
[Un,Sn,Vn] = svd(F_8_n_svd2);
Sn(end,end) = 0;
F_8_n_svd2 = Un*Sn*Vn';

% Compare to true F
ratio_n2 = F_8_n2./F


% Calculate and plot epipolar lines and epipoles
epip1 = normalise_scale(A1*[T;1]);
x = [min(0,epip1(1)-10),max(256,epip1(1))+10];
figure;
% subplot(1,2,1);
hold on;
grid on;
for i = 1:size(p1_n2,2)
    scatter(p1_n2(1,i),p1_n2(2,i),'b');
    lm = F_8_n_svd2'*p2_n2(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx1 sx1 0 0],[0 0 sy1 sy1 0],'k');
scatter(epip1(1),epip1(2),'gh','LineWidth',2);
axis('equal');
title('Camera 1 (noisy [-2,2] SVD)');


epip2 = normalise_scale(A2*[0;0;0;1]);
x = [min(0,epip2(1))-10,max(256,epip2(1))+10];
% subplot(1,2,2);
figure;
hold on;
grid on;
for i = 1:size(p1_n2,2)
    scatter(p2_n2(1,i),p2_n2(2,i),'b');
    lm = F_8_n_svd2*p1_n2(:,i);
    m = -lm(1)/lm(2);
    d = -lm(3)/lm(2);
    y = m*x + d;
    plot(x,y,'r');
end
plot([0 sx2 sx2 0 0],[0 0 sy2 sy2 0],'k');
scatter(epip2(1),epip2(2),'gh','LineWidth',2);
axis('equal');
title('Camera 2 (noisy [-2,2])');

display('S15-2: Fundamental matrix calculated with noisier points and SVD')





































