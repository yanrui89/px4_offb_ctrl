function [state, tt] = get_uni_bspline_1d(order, timespan, ctrlpt, knotdiv, pva)
    %% Summary
    %   get_bspline_1d computes the 1D bspline with given control points and
    %   knotdiv
    %
    %   Inputs
    %       order : Order/degree of the bspline 
    %       timespan : timespan(2) represents end time and timespan(1)
    %           represents the start, we represent them in seconds
    %       ctrlpt : Control points vector
    %       knotdiv : Represents the divisions within 2 knots, like subdivisions
    %       pva : Represents whether the output consist of Position,
    %           Velocity and Acceleration
    %
    %   Outputs
    %       state :  A n*3 x 1 vector, n represents the number of states
    %           that we want to represent
    %       timespan : The vector of the time intervals
    %   
    %   Written by Matthew Woo, (Started 20 May 2022)
    
    %   p = degree
    %   n = cp - 1
    %   knots = m + 1
    %   basises is m = n + p + 1

    %% Using Bspline Segment code

    % Following the notation from
    % (1) https://link.springer.com/article/10.1007/s003710050206 
    %   ("General matrix representations for B-splines")
    % (2) (Real-Time Trajectory Replanning for MAVs using Uniform B-splines 
    %   and a 3D Circular Buffer)
    % (3) https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve.html

    %% Notes 
    % According to (2) etc quintic bspline definition of u is
    % s_t = (time - t(1)) / delta_t
    % u(t) = s(t) − s(i)
    % s(i) lies at the time of the control point(i)

    % According to (2) etc quintic bspline uses a knot vector of
    % [t(i−2), t(i−1), t(i), t(i+1), t(i+2), t(i+3)]

    % p (control points) and t (times allocated to each control point) has a
    % range of 0 to n (which is i = 0 to n)

    % p(t) lies inside [t(i), t(i+1)), and they depend on k (order+1) control points
    
    % Open B-splines are [knot(p), knot(m-p)]
    % * Clamping only happens for the knots not for the control points!
    % * Control points are not really attached to time

    %% Inputs
    
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

    P = ctrlpt;
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
            lpos = zeros(1, knotdiv-1); % [0,1)
            lvel = zeros(1, knotdiv-1); % [0,1)
            lacc = zeros(1, knotdiv-1); % [0,1)
        elseif strcmp(pva,'pv')
            lpos = zeros(1, knotdiv-1); % [0,1)
            lvel = zeros(1, knotdiv-1); % [0,1)
        elseif strcmp(pva,'p')
            lpos = zeros(1, knotdiv-1); % [0,1)
        else 
            error('pva not set');
        end

        lt = zeros(1, knotdiv-1); % [0,1)

        span = linspace(idx, nxt_idx, knotdiv); % Relative to the start time as 0 regardless of the time 
        actualspan = linspace(t(l), t(l+1), knotdiv); % Time in abs time (simulation time / given time)

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
        state(1,:) = single_pos;
        state(2,:) = single_vel;
        state(3,:) = single_acc;
    elseif strcmp(pva,'pv')
        state(1,:) = single_pos;
        state(2,:) = single_vel;
    elseif strcmp(pva,'p')
        state(1,:) = single_pos;
    else 
        error('pva not set');
    end
    
end