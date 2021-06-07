function [fit_flock, o, dfl, v, dfl1, dfl2, flock_ctr_x, flock_ctr_y] = fit_flocking(pos,...
    n_ts,n_rob, target_range, vmax, ts)
%performance metric: flocking
%calculate o(t): orientation between robots
%calculate dfl(t): distance between robots
o = zeros(n_ts,1);
dfl = zeros(n_ts,1);
dfl1 = zeros(n_ts,1);
dfl2 = zeros(n_ts,1);
v = zeros(n_ts,1);
flock_ctr_x = zeros(n_ts,1);
flock_ctr_y = zeros(n_ts,1);
for t = 1:n_ts
    for i = 1:n_rob 
        % find average flock positions
        flock_ctr_x(t) = flock_ctr_x(t) + pos(i).x(t);
        flock_ctr_y(t) = flock_ctr_y(t) + pos(i).y(t);
    end
    flock_ctr_x(t) = flock_ctr_x(t)/n_rob;
    flock_ctr_y(t) = flock_ctr_y(t)/n_rob;
    if t == 1   %first iteration: no previous value is available
        v(t) = 0;
    else
        v(t) = sqrt((flock_ctr_x(t-1)-flock_ctr_x(t))^2+...
            (flock_ctr_y(t-1)-flock_ctr_y(t))^2)/(vmax*ts);
    end
    
    for i = 1:n_rob     %for each robot
        dfl1(t) = dfl1(t) + sqrt((pos(i).x(t)-flock_ctr_x(t))^2+...
            (pos(i).y(t)-flock_ctr_y(t))^2);
        for j = (i+1):n_rob     %for each pair of robots
            o(t) = o(t) + abs(pos(i).heading(t)-pos(j).heading(t))/pi;
            %inter-robot distance of pair i,k
            delta_xj = sqrt((pos(i).x(t)-pos(j).x(t))^2+...
                (pos(i).y(t)-pos(j).y(t))^2);
            %target inter-robot distance of pair j,k
            Dfl = target_range(i,j);
            dfl2(t) = dfl2(t) + min(delta_xj/Dfl, 1/(1-Dfl+delta_xj)^2);
        end
    end
    o(t) = 1 - o(t)/(n_rob*(n_rob-1)/2);
    dfl1(t) = 1/(1 + dfl1(t)/n_rob);
    dfl2(t) = dfl2(t)/(n_rob*(n_rob-1)/2);
    dfl(t) = dfl1(t)*dfl2(t);
end
fit_flock = o.*dfl.*v;
