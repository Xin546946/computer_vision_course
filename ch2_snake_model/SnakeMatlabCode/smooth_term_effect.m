clc;
clear;
close all;
%% initialize a contour
y = [192 233 251 205 169];
x = [203 166 207 248 210];
P = [x(:) y(:)];
P=InterpolateContourPoints2D(P,1000);
num_points = 1000;
%Get a non closed contour
P = P(1:num_points,:);
figure(100)
xlim([100,400])
ylim([100,400])
plot(P(:,1),P(:,2),'.');

%% define matrix A 
A = 2 * eye(num_points);
A = A + (-1) *circshift(eye(num_points),1);
A = A + (-1) * circshift(eye(num_points),-1);

%% define matrix B
B = 6 * eye(num_points);
B = B + (-4) *circshift(eye(num_points),-1);
B = B + circshift(eye(num_points),-2);
B = B + (-4) * circshift(eye(num_points),1);
B = B + circshift(eye(num_points),2);
%% Move the contour w.r.t. A
% alpha = 0.05;
% A_move_snake = inv((eye(num_points) - alpha * A) + 0.01);
% for i = 1:1000
% %   xlim([150,250])
% %   ylim([250,350])
%     pause(0.01);
%     P = A_move_snake * P;
%     plot(P(:,1),P(:,2),'.');
% end

%% Move the contour w.r.t. B
% beta = 100;
% B_move_snake = inv((eye(num_points) + beta * B));
% for i = 1:1000
%     xlim([100,400])
%     ylim([100,400])
%     pause(0.0001);
%     P = B_move_snake * P;
%     plot(P(:,1),P(:,2),'.');
% end

%% Move the contour w.r.t. A and B
alpha = 0.1;
beta = 0.1;
move_snake = inv((eye(num_points) - alpha * A + beta * B));
for i = 1:10000
   
    pause(0.0001);
    P = move_snake * P;
    plot(P(:,1),P(:,2),'.');
end