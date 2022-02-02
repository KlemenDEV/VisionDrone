%%%%% INTERPOLATIONS START
% interpolate estimate to GPS

% interpolate gt_pose to estimate_pose
estimate_pose_interp = gt_pose;
for i=1:1:size(gt_pose, 1)
    idx = size(estimate_pose, 1) * i / size(gt_pose, 1);
    estimate_pose_interp(i, 2:4) = estimate_pose(int32(round(idx)), 2:4);
end
estimate_pose = estimate_pose_interp;

% interpolate gt_vel to estimate_vel
estimate_vel_interp = gt_vel;
for i=1:1:size(gt_vel, 1)
    idx = size(estimate_vel, 1) * i / size(gt_vel, 1);
    estimate_vel_interp(i, 2:4) = estimate_vel(int32(round(idx)), 2:4);
end
estimate_vel = estimate_vel_interp;

% interpolate gt_vel_enu to estimate_vel
estimate_vel_enu_interp = gt_vel_enu;
for i=1:1:size(gt_vel_enu, 1)
    idx = size(estimate_vel_enu, 1) * i / size(gt_vel_enu, 1);
    estimate_vel_enu_interp(i, 2:4) = estimate_vel_enu(int32(round(idx)), 2:4);
end
estimate_vel_enu = estimate_vel_enu_interp;

%%%%% INTERPOLATIONS END

% 2D estimate error
err2d = sqrt((gt_pose(:,2) - estimate_pose(:,2)).^2 + (gt_pose(:,3) - estimate_pose(:,3)).^2);

% height error
errh = abs(gt_pose(:,4) - estimate_pose(:,4));

% distance acumulated error
d_gt_raw = diff(gt_pose(:, 2:3));
d_estimate_pose_raw = diff(estimate_pose(:, 2:3));
d_gt = cumsum(sqrt(sum(d_gt_raw.*d_gt_raw,2)));
d_estimate_pose = cumsum(sqrt(sum(d_estimate_pose_raw.*d_estimate_pose_raw,2)));
d_err = abs(d_gt - d_estimate_pose);

figure('units', 'normalized', 'outerposition', [0.1 0.1 0.8 0.8])
group = uitabgroup();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab1 = uitab(group, 'Title', 'General');
t = tiledlayout(tab1, 2, 3);
t.TileSpacing = 'tight';
t.Padding = 'tight';

gx = geoaxes(t);
gx.Layout.Tile = 1;
gx.Layout.TileSpan = [2 1];
key = 'pk.eyJ1Ijoia2xlbWVuNjMiLCJhIjoiY2twYTg5YjN5MHE0czJyb2c5Z2x0YjRmdSJ9.m5V1rMbGCykrJ2f0OMOuWA';
addCustomBasemap('mapbox', strcat('https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=', key));
[gtlat, gtlon, ~] = local2latlon(-gt_pose(:,3), gt_pose(:,2), 0, datum);
[estimate_poselat, estimate_poselon, ~] = local2latlon(-estimate_pose(:,3), estimate_pose(:,2), 0, datum);
hold (gx, 'on')
geobasemap('mapbox');
geoplot(gx, gtlat, gtlon);
geoplot(gx, estimate_poselat, estimate_poselon);
title(gx, "2D (map view)");
legend(gx, "GPS GT", "Estimate");
hold (gx, 'off')

ax = nexttile(t);
hold (ax, 'on')
plot(ax, gt_pose(1:end-1,1), d_gt);
plot(ax, gt_pose(1:end-1,1), d_estimate_pose);
title(ax, "Travelled distance over time");
xlabel(ax, "time / s");
ylabel(ax, "error / m");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t);
hold (ax, 'on')
plot(ax, d_err);
title(ax, "Travelled distance error over time");
xlabel(ax, "time / s");
ylabel(ax, "error / m");
hold (ax, 'off')

ax = nexttile(t);
plot(ax, gt_pose(:,1), errh);
title(ax, "Height error over time");
xlabel(ax, "time / s");
ylabel(ax, "error / m");

ax = nexttile(t);
plot(ax, gt_pose(:,1), err2d);
title(ax, "2D estimate error over time");
xlabel(ax, "time / s");
ylabel(ax, "error / m");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab2 = uitab(group, 'Title', '3D view');
ax = axes('Parent', tab2);
view(ax, 3)
hold (ax, 'on')
plot3(gt_pose(:,2), gt_pose(:,3), gt_pose(:,4), 'Parent', ax);
plot3(estimate_pose(:,2), estimate_pose(:,3), estimate_pose(:,4), 'Parent', ax);
title(ax, "3D");
hold (ax, 'off')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab5 = uitab(group, 'Title', 'Position in ENU frame');
t5 = tiledlayout(tab5, 2, 2);
t5.TileSpacing = 'tight';
t5.Padding = 'tight';

ax = nexttile(t5);
hold (ax, 'on')
plot(ax, gt_pose(:, 1), gt_pose(:, 2));
plot(ax, estimate_pose(:, 1), estimate_pose(:, 2));
title(ax, "X pose over time");
xlabel(ax, "time / s");
ylabel(ax, "m");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t5);
hold (ax, 'on')
plot(ax, gt_pose(:, 1), gt_pose(:, 3));
plot(ax, estimate_pose(:, 1), estimate_pose(:, 3));
title(ax, "Y pose over time");
xlabel(ax, "time / s");
ylabel(ax, "m");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

poserr_x = abs(gt_pose(:, 2) - estimate_pose(:, 2));
poserr_y = abs(gt_pose(:, 3) - estimate_pose(:, 3));

ax = nexttile(t5);
hold (ax, 'on')
plot(ax, gt_pose(:, 1), poserr_x);
title(ax, "X position error over time");
xlabel(ax, "time / s");
ylabel(ax, "m");
hold (ax, 'off')

ax = nexttile(t5);
hold (ax, 'on')
plot(ax, gt_pose(:, 1), poserr_y);
title(ax, "Y position error over time");
xlabel(ax, "time / s");
ylabel(ax, "m");
hold (ax, 'off')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab3 = uitab(group, 'Title', 'Velocity in ENU frame');
t2 = tiledlayout(tab3, 2, 2);
t2.TileSpacing = 'tight';
t2.Padding = 'tight';

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt_vel_enu(:, 1), -(gt_vel_enu(:, 3)));
plot(ax, estimate_vel_enu(:, 1), -(estimate_vel_enu(:, 2)));
title(ax, "X velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt_vel_enu(:, 1), (gt_vel_enu(:, 2)));
plot(ax, estimate_vel_enu(:, 1), -(estimate_vel_enu(:, 3)));
title(ax, "Y velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

enu_velerr_x = abs((gt_vel_enu(:, 3)) - (estimate_vel_enu(:, 2)));
enu_velerr_y = abs((gt_vel_enu(:, 2)) - (estimate_vel_enu(:, 3)));

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt_vel_enu(:, 1), enu_velerr_x);
title(ax, "X velocity error over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
hold (ax, 'off')

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt_vel_enu(:, 1), enu_velerr_y);
title(ax, "Y velocity error over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
hold (ax, 'off')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab4 = uitab(group, 'Title', 'Velocity in local frame');
t3 = tiledlayout(tab4, 2, 2);
t3.TileSpacing = 'tight';
t3.Padding = 'tight';

ax = nexttile(t3);
hold (ax, 'on')
plot(ax, gt_vel(:, 1), gt_vel(:, 3));
plot(ax, estimate_vel(:, 1), estimate_vel(:, 2));
title(ax, "X velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t3);
hold (ax, 'on')
plot(ax, gt_vel(:, 1), -(gt_vel(:, 2)));
plot(ax, estimate_vel(:, 1), (estimate_vel(:, 3)));
title(ax, "Y velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

velerr_x = abs((gt_vel(:, 3)) - (estimate_vel(:, 2)));
velerr_y = abs((gt_vel(:, 2)) - (estimate_vel(:, 3)));

ax = nexttile(t3);
hold (ax, 'on')
plot(ax, gt_vel(:, 1), velerr_x);
title(ax, "X velocity error over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
hold (ax, 'off')

ax = nexttile(t3);
hold (ax, 'on')
plot(ax, gt_vel(:, 1), velerr_y);
title(ax, "Y velocity error over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
hold (ax, 'off')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab6 = uitab(group, 'Title', 'Performance measures');

diff_maxptostart=sqrt(max(abs(gt_pose(:, 2)-gt_pose(1, 2)))^2 + max(abs(gt_pose(:, 3)-gt_pose(1, 3)))^2);

diff_gt_start_end=sqrt((gt_pose(end - 20, 2)-gt_pose(1, 2))^2 + (gt_pose(end - 20, 3)-gt_pose(1, 3))^2);
diff_estimate_start_end=sqrt((estimate_pose(end - 20, 2)-estimate_pose(1, 2))^2 + (estimate_pose(end - 20, 3)-estimate_pose(1, 3))^2);

data = {
    sprintf('Biggest distance from the starting point (from GT): %.3f m', diff_maxptostart)
    ""
    sprintf('Traveled distance (from GT): %.3f m', d_gt(end))
    sprintf('Total traveled distance error: %.3f m (%.3f %% traveled distance)', abs(d_gt(end) - d_estimate_pose(end)), 100 * (abs(d_gt(end) - d_estimate_pose(end))) / d_gt(end))
    ""
    sprintf('Average 2D error: %.3f m std=%.3f m', mean(err2d), std(err2d))
    sprintf('Relative average 2D error: %.3f %% std=%.3f %% of max. distance from st. pt.', mean(err2d / diff_maxptostart) * 100, std(err2d / diff_maxptostart) * 100)
    ""
    sprintf('Average height error: %.3f m std=%.3f m', mean(errh), std(errh))
    ""
    sprintf('Average ENU vx error: %.3f m/s std=%.3f m/s', mean(enu_velerr_x), std(enu_velerr_x))
    sprintf('Average ENU vy error: %.3f m/s std=%.3f m/s', mean(enu_velerr_y), std(enu_velerr_y))
    ""
    sprintf('Takeoff->land point 2D error: %.3f m', abs(diff_gt_start_end - diff_estimate_start_end))
    sprintf('Takeoff->land point 2D error: %.3f %% of max. dist. from st. pt.', abs(diff_gt_start_end - diff_estimate_start_end) / diff_maxptostart * 100)
};
uicontrol(tab6, 'Style', 'text', 'String', data, 'units', 'normalized', 'Position', [0.01 0.38 0.6 0.6], 'FontSize', 14, 'HorizontalAlignment', 'left', 'FontName', 'FixedWidth');

for k=1:length(data)
    fprintf("%s\n", string(data(k)));
end

tab7 = uitab(group, 'Title', 'Trajectory match');
ax = axes('Parent', tab7);
t.TileSpacing = 'tight';
t.Padding = 'tight';

% trajectory shape estimation

% rotate and scale trajectory to match GT
par = fminsearch(@(x) costf(x, gt_pose, estimate_pose), [0 1]);

rot = par(1);
k = par(2);

estimate_pose_fit = estimate_pose;
estimate_pose2 = cos(rot)*estimate_pose(:, 2) - sin(rot)*estimate_pose(:, 3);
estimate_pose3 = sin(rot)*estimate_pose(:, 2) + cos(rot)*estimate_pose(:, 3);
estimate_pose_fit(:, 2) = estimate_pose2;
estimate_pose_fit(:, 3) = estimate_pose3;

estimate_pose_fit(:, 2) = k * estimate_pose_fit(:, 2);
estimate_pose_fit(:, 3) = k * estimate_pose_fit(:, 3);

view(ax, 2)
hold (ax, 'on')
plot(gt_pose(:,2), gt_pose(:,3), 'Parent', ax);
plot(estimate_pose_fit(:,2), estimate_pose_fit(:,3), 'Parent', ax);
title(ax, "2D");
hold (ax, 'off')

% matched trajectory error
% 2D estimate error
err2d = sqrt((gt_pose(:,2) - estimate_pose_fit(:,2)).^2 + (gt_pose(:,3) - estimate_pose_fit(:,3)).^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tab8 = uitab(group, 'Title', 'Trajectory measures');

data = {
    sprintf('Biggest distance from the starting point (from GT): %.3f m', diff_maxptostart)
    ""
    sprintf('Average 2D error: %.3f m std=%.3f m', mean(err2d), std(err2d))
    sprintf('Relative average 2D error: %.3f %% std=%.3f %% of max. distance from st. pt.', mean(err2d / diff_maxptostart) * 100, std(err2d / diff_maxptostart) * 100)
    ""
    sprintf('Scale error: %.3f %%', abs(1-k) * 100)
    sprintf('Rotation error: %.3f rad, %.3f deg,', rot, rad2deg(rot))
};
uicontrol(tab8, 'Style', 'text', 'String', data, 'units', 'normalized', 'Position', [0.01 0.38 0.6 0.6], 'FontSize', 14, 'HorizontalAlignment', 'left', 'FontName', 'FixedWidth');

for k=1:length(data)
    fprintf("%s\n", string(data(k)));
end

% EOF