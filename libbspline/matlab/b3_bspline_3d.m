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

for x=1:axis
    r = rand(1,6);
    ctrlpt(x,:) = numrange * r; % multiply the random [0-1] by the number range
end

for x=1:order
    ctrlpt = [ctrlpt(:,1) ctrlpt ctrlpt(:,end)];
end

k = order + 1;
if strcmp(pva,'pva')
    u = zeros(1,k); du = zeros(1,k); ddu = zeros(1,k);
elseif strcmp(pva,'pv')
	u = zeros(1,k); du = zeros(1,k); 
elseif strcmp(pva,'p')
    u = zeros(1,k);
else 
    error('pva not set');
end
p = zeros(k,1);

M = get_m_matrix(order);

%% Bspline setup
k = order + 1;
n = length(ctrlpt) - 1;
m = n + order + 1;
n_k = m + 1; % Number of knots

% We establish the clamped path (where we start) or (where we end)
% If not clamped it will be inside the clamped path hence no repeating knots

range = [order + 1 : m - order]; % Range of index to evaluate accordingly (order to length of control points)
% Amount of knots that are not clamped m - 2 * order
% In C++ range = [order : m - order - 1]

knots = linspace(timespan(1), timespan(2), m - 2 * order + 1);
for i = 1:order
    knots = [knots(1) knots knots(end)];
end

%% Main code
dt = (timespan(2) - timespan(1)) / (numel(range)); % Length/Span of 1 knot
t = linspace(timespan(1), timespan(2), numel(range)+1);

for x=1:axis
    
    P = ctrlpt(x,:);
    tt = [];
    if strcmp(pva,'pva')
        single_pos = []; single_vel = []; single_acc = [];
    elseif strcmp(pva,'pv')
        single_pos = []; single_vel = [];
    elseif strcmp(pva,'p')
        single_pos = [];
    else 
        error('pva not set');
    end
    
    for l = 1:numel(range)

        % current idx to next idx
        idx = range(l) - order;
        nxt_idx = idx + 1; 
        
        if strcmp(pva,'pva')
            lpos = zeros(1, kn_seg-1); % [0,1)
            lvel = zeros(1, kn_seg-1); % [0,1)
            lacc = zeros(1, kn_seg-1); % [0,1)
        elseif strcmp(pva,'pv')
            lpos = zeros(1, kn_seg-1); % [0,1)
            lvel = zeros(1, kn_seg-1); % [0,1)
        elseif strcmp(pva,'p')
            lpos = zeros(1, kn_seg-1); % [0,1)
        else 
            error('pva not set');
        end
        
        lt = zeros(1, kn_seg-1); % [0,1)

        span = linspace(idx, nxt_idx, kn_seg); % Relative to the start time as 0 regardless of the time 
        actualspan = linspace(t(l), t(l+1), kn_seg); % Time in abs time (simulation time / given time)

        % numel(span)-1 becasue we are considering [0,1) hence not including 1
        for m = 1:numel(span)-1

            time = span(m); % current time in index form, of course we dont like to play with conversion
            u_t = (time - idx)/((idx+1) - idx); % using index is the same as using time since u_t is a factor

            % p have several conventions according to SO many papers but i
            % would use the convention under (1) and it is the correct one
            % etc if order is 5
            % (1) p = [P(idx-5) P(idx-4) P(idx-3) P(idx-2) P(idx-1) P(idx)]';

            % Make the u, du, ddu and p matrix
            for j = 1:k
                u(j) = u_t^(j-1);
                p(j) = P(idx + (j-1)); % we add a +1 here for matlab notation
                if j >= 2 && (strcmp(pva,'pv') || strcmp(pva,'pva'))
                    du(j) = (j-1) * u_t^(j-2);
                end
                if j >= 3 && strcmp(pva,'pva')
                    ddu(j) = (j-1) * (j-2) * u_t^(j-3);
                end
            end

            inv_dt = 1/dt;

            % Matrix multiplication to attain the pos, vel and acc
            if strcmp(pva,'pva')
                lpos(m) = u * M * p;
                lvel(m) = inv_dt * du * M * p;
                lacc(m) = inv_dt^2 * ddu * M * p;
            elseif strcmp(pva,'pv')
                lpos(m) = u * M * p;
                lvel(m) = inv_dt * du * M * p;
            elseif strcmp(pva,'p')
                lpos(m) = u * M * p;
            else 
                error('pva not set');
            end

            lt(m) = actualspan(m);
            
        end

        % Add the segment to the plot array
        if strcmp(pva,'pva')
            single_pos = [single_pos lpos];
            single_vel = [single_vel lvel];
            single_acc = [single_acc lacc];
        elseif strcmp(pva,'pv')
            single_pos = [single_pos lpos];
            single_vel = [single_vel lvel];
        elseif strcmp(pva,'p')
            single_pos = [single_pos lpos];
        else 
            error('pva not set');
        end
        
        tt = [tt lt];

    end
    
    if strcmp(pva,'pva')
        pos(x,:) = single_pos;
        vel(x,:) = single_vel;
        acc(x,:) = single_acc;
    elseif strcmp(pva,'pv')
        pos(x,:) = single_pos;
        vel(x,:) = single_vel;
    elseif strcmp(pva,'p')
        pos(x,:) = single_pos;
    else 
        error('pva not set');
    end
        
end

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