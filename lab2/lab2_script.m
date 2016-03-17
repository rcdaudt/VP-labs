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
T_ext(1:3,4) = [Tx;Ty;Tz];
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
display('Projections displayed');

%% Step 6

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

%% Step 7

ratio_pure = A_hall./T

%% Step 8

projections_noisy = projections_sc;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy(:,i) = projections_noisy(:,i) + [(2*rand(2,1)-1);0];
    end
end


Q = zeros(2*num_points,11);
B = zeros(2*num_points,1);
for i = 1:num_points
    IXui = projections_noisy(1,i);
    IYui = projections_noisy(2,i);
    Q(2*i-1,:) = [points(1,i) points(2,i) points(3,i) 1 0 0 0 0 -IXui*points(1,i) -IXui*points(2,i) -IXui*points(3,i)];
    Q(2*i,:) = [0 0 0 0 points(1,i) points(2,i) points(3,i) 1 -IYui*points(1,i) -IYui*points(2,i) -IYui*points(3,i)];
    B(2*i-1) = projections_noisy(1,i);
    B(2*i) = projections_noisy(2,i);
end
A = Q\B;
A_hall6 = [A(1) A(2) A(3) A(4);A(5) A(6) A(7) A(8);A(9) A(10) A(11) 1];
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
projections = T*points10;
projections_sc10 = projections;
for i = 1:3
    projections_sc10(i,:) = projections(i,:)./projections(3,:);
end

% Step 8
projections_noisy = projections_sc10;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy(:,i) = projections_noisy(:,i) + [(2*rand(2,1)-1);0];
    end
end
Q = zeros(2*num_points,11);
B = zeros(2*num_points,1);
for i = 1:num_points
    IXui = projections_noisy(1,i);
    IYui = projections_noisy(2,i);
    Q(2*i-1,:) = [points10(1,i) points10(2,i) points10(3,i) 1 0 0 0 0 -IXui*points10(1,i) -IXui*points10(2,i) -IXui*points10(3,i)];
    Q(2*i,:) = [0 0 0 0 points10(1,i) points10(2,i) points10(3,i) 1 -IYui*points10(1,i) -IYui*points10(2,i) -IYui*points10(3,i)];
    B(2*i-1) = projections_noisy(1,i);
    B(2*i) = projections_noisy(2,i);
end
A = Q\B;
A_hall10 = [A(1) A(2) A(3) A(4);A(5) A(6) A(7) A(8);A(9) A(10) A(11) 1];
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
projections = T*points50;
projections_sc50 = projections;
for i = 1:3
    projections_sc50(i,:) = projections(i,:)./projections(3,:);
end

% Step 8
projections_noisy = projections_sc50;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy(:,i) = projections_noisy(:,i) + [(2*rand(2,1)-1);0];
    end
end
Q = zeros(2*num_points,11);
B = zeros(2*num_points,1);
for i = 1:num_points
    IXui = projections_noisy(1,i);
    IYui = projections_noisy(2,i);
    Q(2*i-1,:) = [points50(1,i) points50(2,i) points50(3,i) 1 0 0 0 0 -IXui*points50(1,i) -IXui*points50(2,i) -IXui*points50(3,i)];
    Q(2*i,:) = [0 0 0 0 points50(1,i) points50(2,i) points50(3,i) 1 -IYui*points50(1,i) -IYui*points50(2,i) -IYui*points50(3,i)];
    B(2*i-1) = projections_noisy(1,i);
    B(2*i) = projections_noisy(2,i);
end
A = Q\B;
A_hall50 = [A(1) A(2) A(3) A(4);A(5) A(6) A(7) A(8);A(9) A(10) A(11) 1];
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
projections = T*points200;
projections_sc200 = projections;
for i = 1:3
    projections_sc200(i,:) = projections(i,:)./projections(3,:);
end

% Step 8
projections_noisy = projections_sc200;
for i = 1:num_points
    if rand() <= 0.95
        projections_noisy(:,i) = projections_noisy(:,i) + [(2*rand(2,1)-1);0];
    end
end
Q = zeros(2*num_points,11);
B = zeros(2*num_points,1);
for i = 1:num_points
    IXui = projections_noisy(1,i);
    IYui = projections_noisy(2,i);
    Q(2*i-1,:) = [points200(1,i) points200(2,i) points200(3,i) 1 0 0 0 0 -IXui*points200(1,i) -IXui*points200(2,i) -IXui*points200(3,i)];
    Q(2*i,:) = [0 0 0 0 points200(1,i) points200(2,i) points200(3,i) 1 -IYui*points200(1,i) -IYui*points200(2,i) -IYui*points200(3,i)];
    B(2*i-1) = projections_noisy(1,i);
    B(2*i) = projections_noisy(2,i);
end
A = Q\B;
A_hall200 = [A(1) A(2) A(3) A(4);A(5) A(6) A(7) A(8);A(9) A(10) A(11) 1];
ratio_noisy50 = A_hall200./T;
noisy_projections = A_hall200*points200;
noisy_projections_sc200 = noisy_projections./[noisy_projections(3,:);noisy_projections(3,:);noisy_projections(3,:)];
proj_diff200 = projections_sc200 - noisy_projections_sc200;
dist200 = sqrt(proj_diff200(1,:).^2+proj_diff200(2,:).^2);
mean_diff200 = mean(dist200)


%% Part 2

display('Part 2');

%% Step 10

num_points = 6;
Q = zeros(2*num_points,11);
B = zeros(2*num_points,1);
for i = 1:num_points
    Q(2*i-1,:) = [points(1:3,i)' (-projections_sc(1,i)*points(1:3,i)') 0 0 0 1 0];
    Q(2*i,:) = [0 0 0 (-projections_sc(2,i)*points(1:3,i)') points(1:3,i)' 0 1];
    B(2*i-1) = projections_sc(1,i);
    B(2*i) = projections_sc(2,i);
end
X = Q\B;

T1 = X(1:3)';
T2 = X(4:6)';
T3 = X(7:9)';
C1 = X(10);
C2 = X(11);

u0_f = T1*T2'/(norm(T2)^2);
v0_f = T2*T3'/(norm(T2)^2);
au_f = norm(cross(T1',T2'))/(norm(T2)^2);
av_f = norm(cross(T2',T3'))/(norm(T2)^2);











