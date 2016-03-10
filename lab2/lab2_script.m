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



