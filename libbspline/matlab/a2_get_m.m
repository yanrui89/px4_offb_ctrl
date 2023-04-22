% Written by Matthew Woo, (Started 20 May 2022)

%% Main Code

% Following the notation from
% https://link.springer.com/article/10.1007/s003710050206 
% ("General matrix representations for B-splines"
% See Theorem 1 of page 182

% Uniform bspline
% There are 2 ways, one is a shortcut, but the other has a real formulation
% View the portion on (section 3.2) on NURBs and (section 4.1)

% t(j) - t(j-1) = constant

% j = start knot point
% order = k-1 
% m(i,j) = (1/(k-1)) * C(k-1-i,k-1) * ...
%   sum(s=j to k-1) {pow(-1,s-j) * C(s-j,k) * pow(k-s-1,k-1-i)} 
% C(i,n) = factorial(n)/(factorial(i) * factorial(n-i));

clc
clear all
close all

addpath('functions');

order = 4;
k = order + 1;
% Via Single Function
M = zeros(k,k);
% Via Integrated Function
M0 = zeros(k,k);

for i = 1:numel(M)
    % notation is different for matlab
    if mod(i,k) == 0
        modv = k;
    else
        modv = mod(i,k);
    end
    idx = [ceil(i/k), modv];
    M(ceil(i/k), modv) = get_single_m_matrix(idx,order);
    fprintf("m(%d,%d) = %f\n",ceil(i/k),modv,M(ceil(i/k), modv));
end

M0 = get_m_matrix(order);

M
M0