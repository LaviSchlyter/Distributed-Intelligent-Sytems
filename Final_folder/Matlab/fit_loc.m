function [fit_loc] = fit_loc(pos,...
    est,n_ts, n_rob)
%performance metric: localization
fit_loc = zeros(n_ts,n_rob);
for i = 1:n_rob
    for t = 1:n_ts
        fit_loc(t,i) = sqrt((pos(i).true_x(t) - pos(i).(est{1})(t))^2+...
            (pos(i).true_y(t)-pos(i).(est{2})(t))^2);
    end
end
