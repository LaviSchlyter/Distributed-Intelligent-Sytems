function [fit_loc] = fit_loc(pos,...
    est,n_ts, n_rob, vmax, g, ts)
%performance metric: localization
fit_loc = zeros(n_ts,n_rob);
for i = 1:n_rob
    for t = 1:n_ts
        fit_loc(t,i) = sqrt((pos(i).true_x(t) - pos(i).kal_wheel_x(t))^2+...
            (pos(i).true_y(t)-pos(i).kal_wheel_y(t))^2);
    end
end

% %performance metric: flocking
% %calculate o(t): orientation between robots
% %calculate dfl(t): distance between robots
% fit_flocking = zeros(n_ts,n_rob);
% o = zeros(n_ts,n_rob);
% dfl = zeros(n_ts,n_rob);
% dfl1 = zeros(n_ts,n_rob);
% dfl2 = zeros(n_ts,n_rob);
% v = zeros(n_ts,1);
% flock_ctr_x = zeros(n_ts,1);
% flock_ctr_y = zeros(n_ts,1);
% for t = 1:n_ts
%     for i = 1:n_rob 
%         % find average flock positions
%         flock_ctr_x(t) = flock_ctr_x(t) + mean(pos(i).true_x(t));
%         flock_ctr_y(t) = flock_ctr_y(t) + mean(pos(i).true_y(t));
%     end
%     flock_ctr_x(t) = flock_ctr_x(t)/n_rob;
%     flock_ctr_y(t) = flock_ctr_y(t)/n_rob;
%     if t == 1   %first iteration: no previous value is available
%         v(t,i) = 0;
%     else
%         v(t,i) = sqrt((flock_ctr_x(t-1)-flock_ctr_x(t))^2+...
%             (flock_ctr_y(t-1)-flock_ctr_y(t))^2)/(vmax*ts);
%     end
%     for i = 1:n_rob     %for each robot
%         dfl1(t,i) = dfl1(t,i) + sqrt((pos(i).true_x(t)-flock_ctr_x(t))^2+...
%             (pos(i).true_y(t)-flock_ctr_y(t))^2);
%         for j = (t+1):n_rob     %for each pair of robots
%             o(t) = o(t) + abs(pos(i).true_heading(t)-pos(j).true_heading(t))/pi;
%             %inter-robot distance of pair i,k
%             delta_xj = sqrt((pos(i).true_x(t)-pos(j).true_x(t))^2+...
%                 (pos(i).true_y(t)-pos(j).true_y(t))^2);
%             %target inter-robot distance of pair j,k
%             Dfl = sqrt((g(i,1)-g(j,1))^2 + (g(i,2)-g(j,2))^2);
%             dfl2(i) = dfl2(i) + min(delta_xj/Dfl, 1/(1-Dfl+delta_xj)^2);
%         end
%     end
%     o(t) = 1 - o(t)/(n_rob*(n_rob-1)/2);
%     dfl1(t) = 1/(1 + dfl1(t)/n_rob);
%     dfl2(t) = dfl2(t)/(n_rob*(n_rob-1)/2);
%     dfl(t) = dfl1(t) + dfl2(t);
% end
% fit_flocking = o.*dfl.*v;
% 
% %performance metric: formation
% %for this step, we need robot coordinates in local reference frame; origin
% %= leader = first robot.
% loc_pos_x = zeros(n_ts,n_rob);
% loc_pos_y = zeros(n_ts,n_rob);
% for i = 2:n_rob %leader (i = 1) keeps pos = 0
%     loc_pos_x(:,i) = pos(i).true_x(:) - pos(1).true_x(:);
%     loc_pos_y(:,i) = pos(i).true_y(:) - pos(1).true_y(:);
% end
% dfo = zeros(n_ts,1);
% for t = 1:n_ts
%     for i = 1:n_rob
%         dfo(t) = dfo(t) + sqrt((loc_pos_x(t,i)-g(i,1))^2+...
%             (loc_pos_y(t,i)-g(i,2))^2);
%     end
%     dfo(i) = 1 + dfo(i)/n_rob;
% end
% fit_formation = dfo.*v;
% end
% 
