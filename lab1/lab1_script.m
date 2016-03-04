% VP lab 1
clear all
close all
clc

%% Load images

cb1 = imread('chessboard03.png');
cb2 = imread('chessboard04.png');
cb3 = imread('chessboard05.png');
cb4 = imread('chessboard06.png');

%% Apply Harris corner detector

cb1_h = harris(cb1);
figure;
imshow(cb1);
hold on;
s = size(cb1);
for i = 1:length(cb1_h)
%     [x,y] = ind2sub(s,cb1_h(i));
    plot(cb1_h(:,2),cb1_h(:,1),'r+','LineWidth',2);
end

pause(0.2);

%%

cb2_h = harris(cb2);
figure;
imshow(cb2);
hold on;
s = size(cb2);
for i = 1:length(cb2_h)
%     [x,y] = ind2sub(s,cb2_h(i));
    plot(cb2_h(:,2),cb2_h(:,1),'r+','LineWidth',2);
end

pause(0.2);