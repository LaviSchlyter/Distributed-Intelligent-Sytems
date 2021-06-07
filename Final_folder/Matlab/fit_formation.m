function [fit_form, dfo, v, flock_ctr_x, flock_ctr_y, loc_pos_x, loc_pos_y]...
    = fit_formation(pos,n_ts,n_rob,n_teams, vmax, g, leader_ofeachrob, ts,...
    team, rob_idx, pos_rob)

%performance metric: formation
%for this step, we need robot coordinates in local reference frame; origin
%= leader = first robot.
dfo = zeros(n_ts,1);
loc_pos_x = zeros(n_ts,n_rob);
loc_pos_y = zeros(n_ts,n_rob);
flock_ctr_x = zeros(n_ts,n_teams);
flock_ctr_y = zeros(n_ts,n_teams);
v = zeros(n_ts,n_teams);
% find the position of each robot relative to its leader
for i = rob_idx
    if i == leader_ofeachrob(i) %leader of the formation
        loc_pos_x(1:n_ts,i) = zeros(n_ts,1);
        loc_pos_y(1:n_ts,i) = zeros(n_ts,1);
        sprintf("Rob %d is leader", i)

    else    %follower robot  
        loc_pos_x(1:n_ts,i) = pos(pos_rob(i)).x(1:n_ts) - ...
            pos(pos_rob(leader_ofeachrob(i))).x(1:n_ts);
        loc_pos_y(1:n_ts,i) = pos(pos_rob(i)).y(1:n_ts) - ...
            pos(pos_rob(leader_ofeachrob(i))).y(1:n_ts);
        sprintf("Rob %d has leader %d", i, leader_ofeachrob(i))
    end
end

for t = 1:n_ts
    for i = rob_idx
        % find average flock positions
        flock_ctr_x(t,team(i)) = flock_ctr_x(t, team(i)) + pos(pos_rob(i)).x(t);
        flock_ctr_y(t,team(i)) = flock_ctr_y(t, team(i)) + pos(pos_rob(i)).y(t);
        
        dfo(t) = dfo(t) + sqrt((loc_pos_x(t,i)-g(i,1))^2+...
        (loc_pos_y(t,i)-g(i,2))^2);
    end
%     ctr1 = [flock_ctr_x(t,1),flock_ctr_y(t,1)]
%     ctr2 = [flock_ctr_x(t,2),flock_ctr_y(t,2)]
    flock_ctr_x(t,:) = n_teams*flock_ctr_x(t,:)/n_rob;
    flock_ctr_y(t,:) = n_teams*flock_ctr_y(t,:)/n_rob;
    for k = 1:n_teams
        if t == 1   %first iteration: no previous value is available
            v(t,:) = 0;
        else
%             (flock_ctr_x(t-1,k) - flock_ctr_x(t,k))^2+...
%                 (flock_ctr_y(t-1,k)-flock_ctr_y(t,k))^2
            v(t,team(i)) = sqrt((flock_ctr_x(t,k) - flock_ctr_x(t-1,k))^2+...
                (flock_ctr_y(t,k)-flock_ctr_y(t-1,k))^2)/(vmax*ts);
        end
    end
    disp(v(t,:))
    v(t,:) = mean(abs(v(t,:)),2);
    

    dfo(t) = 1/(1 + dfo(t)/n_rob);
end
fit_form = dfo.*v;
end

