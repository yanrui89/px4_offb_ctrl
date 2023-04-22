% Written by Matthew Woo, (Started 20 May 2022)

%% Notes
% p is a [order + 1] column vector
% M is a [order+1 x order+1] matrix
% u, du, duu are [order + 1] row vector

%% Bspline Segment

% DO NOT follow https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve-open.html
% We are doing another method that should not be confused with multiple
% knots

% Following the notation from
% (1) https://link.springer.com/article/10.1007/s003710050206 
%   ("General matrix representations for B-splines")
% (2) (Real-Time Trajectory Replanning for MAVs using Uniform B-splines 
%   and a 3D Circular Buffer)

% According to (2) etc quintic bspline definition of u is
% s_t = (time - t(1)) / delta_t
% u(t) = s(t) − s(i)
% s(i) lies at the time of the control point(i)

% According to (2) etc quintic bspline uses a knot vector of
% [t(i−2), t(i−1), t(i), t(i+1), t(i+2), t(i+3)]

% p (control points) and t (times allocated to each control point) has a
% range of 0 to n (which is i = 0 to n)

% p(t) lies inside [t(i), t(i+1)), and they depend on k (order+1) control points

clc
close all
clear all

addpath('functions');

axis = 3;
order = 5;
timespan = [0,1];
kn_seg = 5; % Division of 1 span or 1 segment
numrange = 12; % range for evaluation
pva = 'pva'; % p for pos, pv for pos and vel, pva for pos, vel, acc

% for x=1:axis
%     r = rand(1,6);
%     ctrlpt(x,:) = numrange * r; % multiply the random [0-1] by the number range
% end
% 
% for x=1:order
%     ctrlpt = [ctrlpt(:,1) ctrlpt ctrlpt(:,end)];
% end

load('test_ctrlpt_3d.mat');

%% Main code

for x=1:axis
    
    [sstate, time] = get_uni_bspline_1d( ...
        order, timespan, ctrlpt(x,:), kn_seg, pva);
    
    if strcmp(pva,'pva')
        pos(x,:) = sstate(1,:);
        vel(x,:) = sstate(2,:);
        acc(x,:) = sstate(3,:);
    elseif strcmp(pva,'pv')
        pos(x,:) = sstate(1,:);
        vel(x,:) = sstate(2,:);
    elseif strcmp(pva,'p')
        pos(x,:) = sstate(1,:);
    else 
        error('pva not set');
    end
        
end

tt = time;

%% Plotting

figure(1)
subplot(3,1,1)
hold on
if axis == 2
    plot(pos(1,:),pos(2,:),'x');
    plot(pos(1,:),pos(2,:),'-');
    plot(ctrlpt(1,:),ctrlpt(2,:),'o');
    plot(ctrlpt(1,:),ctrlpt(2,:),'--');
elseif axis == 3
    plot3(pos(1,:),pos(2,:),pos(3,:),'x');
    plot3(pos(1,:),pos(2,:),pos(3,:),'-');
    plot3(ctrlpt(1,:),ctrlpt(2,:),ctrlpt(3,:),'o');
    plot3(ctrlpt(1,:),ctrlpt(2,:),ctrlpt(3,:),'--');   
end
hold off
grid on
view(3)
title(strcat('Position ', num2str(axis), 'D'));
xlabel('x/m');
ylabel('y/m');

subplot(3,1,2)
hold on
for x=1:axis
    plot(tt,pos(x,:),'x');
end
hold off
grid on
title('Position against time');
xlabel('t/s');
ylabel('x/m');

subplot(3,1,3)
hold on
for x=1:axis
    plot(tt,vel(x,:),'x');
end
hold off
grid on
title('Velocity against time');
xlabel('t/s');
ylabel('v/m/s');