% Written by Matthew Woo, (Started 20 May 2022)

%% Bspline Segment

% Following the notation from
% (1) https://link.springer.com/article/10.1007/s003710050206 
% ("General matrix representations for B-splines")

% Following the notation from
% (2) (Real-Time Trajectory Replanning for MAVs using Uniform B-splines 
% and a 3D Circular Buffer)

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

order = 3;
timespan = [0,1];
r = rand(1,14);
numrange = 12; % range for evaluation
ctrlpt = numrange * r; % multiply the random [0-1] by the number range

k = order + 1;
u = zeros(1,k); du = zeros(1,k); ddu = zeros(1,k); dddu = zeros(1,k);
p = zeros(k,1);

M = get_m_matrix(order);

%% Bspline setup
k = order + 1;
n = numel(ctrlpt) - 1;
m = n + order + 1;
n_k = m + 1; % Number of knots

% We establish the clamped path (where we start) or (where we end)
% If not clamped it will be inside the clamped path hence no repeating knots

P = ctrlpt;
range = [order + 1 : m - order]; % Range of index to evaluate accordingly (order to length of control points)
% In C++ range = [order : m - order - 1]

knots = linspace(timespan(1), timespan(2), m - 2 * order + 1);
for i = 1:order
    knots = [knots(1) knots knots(end)];
end
