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
order = 3;
timespan1 = [0,1];
kn_seg = 5; % Division of 1 span or 1 segment
numrange = 12; % range for evaluation
pva = 'pva'; % p for pos, pv for pos and vel, pva for pos, vel, acc

% Available points must be equal to or more than order+1
points = 7;

for x=1:axis
    r = rand(1,points);
    ctrlpt1(x,:) = numrange * r; % multiply the random [0-1] by the number range
end

for x=1:order
    ctrlpt1 = [ctrlpt1(:,1) ctrlpt1];
end

% load('test_conc_ctrlpt_3d.mat','ctrlpt1','ctrlpt2','timespan2');

n = length(ctrlpt1) - 1;
m = n + order + 1;
% Number of used control points for 1 segment
int = (m - order) - (order + 1) + 1; 
dt = (timespan1(2) - timespan1(1)) / int;

%% Main code

%% 1st curve
for x=1:axis
    
    [sstate, time] = get_uni_bspline_1d( ...
        order, timespan1, ctrlpt1(x,:), kn_seg, pva);
    
    if strcmp(pva,'pva')
        pos1(x,:) = sstate(1,:);
        vel1(x,:) = sstate(2,:);
        acc1(x,:) = sstate(3,:);
    elseif strcmp(pva,'pv')
        pos1(x,:) = sstate(1,:);
        vel1(x,:) = sstate(2,:);
    elseif strcmp(pva,'p')
        pos1(x,:) = sstate(1,:);
    else 
        error('pva not set');
    end
        
end

tt1 = time;

%% 2nd curve
overlap = 0;
cp_used = order + 2;
timespan2 = [timespan1(1) + (cp_used-order) * dt, timespan1(2) + (cp_used-order) * dt];
% overlap_size = length(ctrlpt1) - (overlap-order);
for x=1:axis
    r = rand(1,(points+order)-order-overlap);
    ctrlpt2(x,:) = numrange * r; % multiply the random [0-1] by the number range
end

ctrlpt2 = [ctrlpt1(:,cp_used-order+1:cp_used+overlap) ctrlpt2];


for x=1:axis
    
    [sstate, time] = get_uni_bspline_1d( ...
        order, timespan2, ctrlpt2(x,:), kn_seg, pva);
    
    if strcmp(pva,'pva')
        pos2(x,:) = sstate(1,:);
        vel2(x,:) = sstate(2,:);
        acc2(x,:) = sstate(3,:);
    elseif strcmp(pva,'pv')
        pos2(x,:) = sstate(1,:);
        vel2(x,:) = sstate(2,:);
    elseif strcmp(pva,'p')
        pos2(x,:) = sstate(1,:);
    else 
        error('pva not set');
    end
        
end

tt2 = time;

%% Plotting

ctrlpt = [ctrlpt1 ctrlpt2(:,order+1:end)];
figure(1)
subplot(3,1,1)
hold on
if axis == 2
    plot(pos1(1,:),pos1(2,:),'x');
    plot(pos1(1,:),pos1(2,:),'-');
    plot(pos2(1,:),pos2(2,:),'o');
    plot(pos2(1,:),pos2(2,:),'-');
    plot(ctrlpt(1,:),ctrlpt(2,:),'o');
    plot(ctrlpt(1,:),ctrlpt(2,:),'--');
    view(2)
elseif axis == 3
    plot3(pos1(1,:),pos1(2,:),pos1(3,:),'x');
    plot3(pos1(1,:),pos1(2,:),pos1(3,:),'-');
    plot3(pos2(1,:),pos2(2,:),pos2(3,:),'o');
    plot3(pos2(1,:),pos2(2,:),pos2(3,:),'-');
    plot3(ctrlpt(1,:),ctrlpt(2,:),ctrlpt(3,:),'o');
    plot3(ctrlpt(1,:),ctrlpt(2,:),ctrlpt(3,:),'--');
    view(3)
end
hold off
grid on
title(strcat('Position ', num2str(axis), 'D'));
xlabel('x/m');
ylabel('y/m');

subplot(3,1,2)
hold on
for x=1:axis
    plot(tt1,pos1(x,:),'x', ...
        'DisplayName',"axis"+num2str(x)+" index1",'MarkerSize',3);
    plot(tt2,pos2(x,:),'o', ...
        'DisplayName',"axis"+num2str(x)+" index2",'MarkerSize',3);
end
hold off
grid on
legend
title('Position against time');
xlabel('t/s');
ylabel('dist/m');

subplot(3,1,3)
hold on
for x=1:axis
    plot(tt1,vel1(x,:),'x', ...
        'DisplayName',"axis"+num2str(x)+" index1",'MarkerSize',3);
    plot(tt2,vel2(x,:),'o', ...
        'DisplayName',"axis"+num2str(x)+" index2",'MarkerSize',3);
end
hold off
grid on
legend
title('Velocity against time');
xlabel('t/s');
ylabel('v/m/s');