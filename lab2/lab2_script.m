% VP lab 2
% Daudt
% 10/03/16

clear all
close all
clc

%% Part 1

display('Part 1');

%% Step 1

au=557.0943; av=712.9824; u0=326.3819; v0=298.6679;
f=-80;
Tx=100; Ty=0; Tz=1500;
Phix=0.8*pi/2; Phiy=-1.8*pi/2; Phix1=pi/5;  % Euler XYX1
sx = 640; sy = 480;% Image size:640x480
display('Intrisnic and extrisnic parameters set');

%% Step 2

T_int = [au 0 u0 0;0 av v0 0;0 0 1 0];
R1 = [1 0 0 0;0 cos(Phix) -sin(Phix) 0;0 sin(Phix) cos(Phix) 0;0 0 0 1];
R2 = [cos(Phiy) 0 sin(Phiy) 0;0 1 0 0;-sin(Phiy) 0 cos(Phiy) 0;0 0 0 1];
R3 = [1 0 0 0;0 cos(Phix1) -sin(Phix1) 0;0 sin(Phix1) cos(Phix1) 0;0 0 0 1];
T_ext = R1*R2*R3;
R = T_ext(1:3,1:3); % For part 10
translation = [Tx;Ty;Tz];
T_ext(1:3,4) = translation;
T = T_int*T_ext;
T = T/T(end,end);
display('Intrisnic and extrisnic transformation matrices calculated');

%% Step 3

num_points = 6;
points = 960*rand(3,num_points)-480;
points(4,:) = ones(1,num_points);
display('Random points generated');
% scatter3(points(:,1),points(:,2),points(:,3));

%% Step 4

projections = T*points;
projections_sc = projections;
for i = 1:3
    projections_sc(i,:) = projections(i,:)./projections(3,:);
end
display('Projections calculated');

%% Step 5

figure;
scatter(projections(1,:),projections(2,:));
axis('equal');
grid on;
display('Projections displayed');

%% Step 6

A_hall = hall_cc(points,projections_sc);

%% Step 7

ratio_pure = A_hall./T

%% Step 8

projections_noisy = projections_sc;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy(:,i) = projections_noisy(:,i) + [(2*rand(2,1)-1);0];
    end
end

A_hall6 = hall_cc(points,projections_noisy);
ratio_noisy6 = A_hall6./T;

noisy_projections = A_hall6*points;
noisy_projections_sc = noisy_projections./[noisy_projections(3,:);noisy_projections(3,:);noisy_projections(3,:)];
proj_diff6 = projections_sc - noisy_projections_sc;
dist6 = sqrt(proj_diff6(1,:).^2+proj_diff6(2,:).^2);
mean_diff6 = mean(dist6)

%% Step 9 - 10 points

% Step 3
num_points = 10;
points10 = 960*rand(3,num_points)-480;
points10(4,:) = ones(1,num_points);

% Step 4
projections10 = T*points10;
projections_sc10 = projections10;
for i = 1:3
    projections_sc10(i,:) = projections10(i,:)./projections10(3,:);
end

% Step 8
projections_noisy10 = projections_sc10;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy10(:,i) = projections_noisy10(:,i) + [(2*rand(2,1)-1);0];
    end
end

A_hall10 = hall_cc(points10,projections_noisy10);
ratio_noisy10 = A_hall10./T;
noisy_projections = A_hall10*points10;
noisy_projections_sc10 = noisy_projections./[noisy_projections(3,:);noisy_projections(3,:);noisy_projections(3,:)];
proj_diff10 = projections_sc10 - noisy_projections_sc10;
dist10 = sqrt(proj_diff10(1,:).^2+proj_diff10(2,:).^2);
mean_diff10 = mean(dist10)

%% Step 9 - 50 points

% Step 3
num_points = 50;
points50 = 960*rand(3,num_points)-480;
points50(4,:) = ones(1,num_points);

% Step 4
projections50 = T*points50;
projections_sc50 = projections50;
for i = 1:3
    projections_sc50(i,:) = projections50(i,:)./projections50(3,:);
end

% Step 8
projections_noisy50 = projections_sc50;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy50(:,i) = projections_noisy50(:,i) + [(2*rand(2,1)-1);0];
    end
end

A_hall50 = hall_cc(points50,projections_noisy50);
ratio_noisy50 = A_hall50./T;
noisy_projections = A_hall50*points50;
noisy_projections_sc50 = noisy_projections./[noisy_projections(3,:);noisy_projections(3,:);noisy_projections(3,:)];
proj_diff50 = projections_sc50 - noisy_projections_sc50;
dist50 = sqrt(proj_diff50(1,:).^2+proj_diff50(2,:).^2);
mean_diff50 = mean(dist50)

%% Step 9 - 200 points

% Step 3
num_points = 200;
points200 = 960*rand(3,num_points)-480;
points200(4,:) = ones(1,num_points);

% Step 4
projections200 = T*points200;
projections_sc200 = projections200;
for i = 1:3
    projections_sc200(i,:) = projections200(i,:)./projections200(3,:);
end

% Step 8
projections_noisy200 = projections_sc200;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy200(:,i) = projections_noisy200(:,i) + [(2*rand(2,1)-1);0];
    end
end

A_hall200 = hall_cc(points200,projections_noisy200);
ratio_noisy50 = A_hall200./T;
noisy_projections = A_hall200*points200;
noisy_projections_sc200 = noisy_projections./[noisy_projections(3,:);noisy_projections(3,:);noisy_projections(3,:)];
proj_diff200 = projections_sc200 - noisy_projections_sc200;
dist200 = sqrt(proj_diff200(1,:).^2+proj_diff200(2,:).^2);
mean_diff200 = mean(dist200)


%% Part 2

display('Part 2');

%% Step 10

X = faugeras_cc(points,projections_sc);

intrinsics = faugeras_compute_intrinsics(X);
extrinsics = faugeras_compute_extrinsics(X);
faug_A = faugeras_compute_pm(X)

au_diff = au - intrinsics(1,1)
u0_diff = u0 - intrinsics(1,3)
av_diff = av - intrinsics(2,2)
v0_diff = v0 - intrinsics(2,3)

R_diff = extrinsics(1:3,1:3) - R
t_diff = extrinsics(1:3,4) - translation

%% Step 11 with [-1,1]

projections_noisy1 = projections_sc;
for i = 1:size(projections_sc,1)
    if rand() <= 0.95
        projections_noisy1(:,i) = projections_noisy1(:,i) + [(2*rand(2,1)-1);0];
    end
end

X1 = faugeras_cc(points,projections_noisy1);

intrinsics1 = faugeras_compute_intrinsics(X1);
extrinsics1 = faugeras_compute_extrinsics(X1);
faug_A1 = faugeras_compute_pm(X1)

au_diff1 = au - intrinsics1(1,1)
u0_diff1 = u0 - intrinsics1(1,3)
av_diff1 = av - intrinsics1(2,2)
v0_diff1 = v0 - intrinsics1(2,3)

R_diff1 = extrinsics1(1:3,1:3) - R
t_diff1 = extrinsics1(1:3,4) - translation

%% Step 11 with [-2,2]

projections_noisy2 = projections_sc;
for i = 1:size(projections_sc,1)
    if rand() <= 0.95
        projections_noisy2(:,i) = projections_noisy2(:,i) + [(4*rand(2,1)-2);0];
    end
end

X2 = faugeras_cc(points,projections_noisy2);

intrinsics2 = faugeras_compute_intrinsics(X2);
extrinsics2 = faugeras_compute_extrinsics(X2);
faug_A2 = faugeras_compute_pm(X2)

au_diff2 = au - intrinsics2(1,1)
u0_diff2 = u0 - intrinsics2(1,3)
av_diff2 = av - intrinsics2(2,2)
v0_diff2 = v0 - intrinsics2(2,3)

R_diff2 = extrinsics2(1:3,1:3) - R
t_diff2 = extrinsics2(1:3,4) - translation

%% Step 11 with [-3,3]

projections_noisy3 = projections_sc;
for i = 1:size(projections_sc,1)
    if rand() <= 0.95
        projections_noisy3(:,i) = projections_noisy3(:,i) + [(6*rand(2,1)-3);0];
    end
end

X3 = faugeras_cc(points,projections_noisy3);

intrinsics3 = faugeras_compute_intrinsics(X3);
extrinsics3 = faugeras_compute_extrinsics(X3);
faug_A3 = faugeras_compute_pm(X3)

au_diff3 = au - intrinsics3(1,1)
u0_diff3 = u0 - intrinsics3(1,3)
av_diff3 = av - intrinsics3(2,2)
v0_diff3 = v0 - intrinsics3(2,3)

R_diff3 = extrinsics3(1:3,1:3) - R
t_diff3 = extrinsics3(1:3,4) - translation

%% Part 3

display('Part 3');

%% Step 12

% scatter3(points(1,:),points(2,:),points(3,:),'rd','LineWidth',3);
% hold on;
%TODO: fix camera orientation
% plotCamera('Location',extrinsics(1:3,4)','Orientation',R,'AxesVisible',true);
cameraParams = cameraParameters;
showExtrinsics(cameraParams);
% for i = 1:size(points,2)
%     plot3([extrinsics(1,4) points(1,i)],[extrinsics(2,4) points(2,i)],[extrinsics(3,4) points(3,i)],'b');
% end








