clear all;
close all;
clc;

[file, path] = uigetfile("*.mat", "Select estimate_pose file");
load(strcat(path, file));

% interpolate gt_pose to estimate_pose
gt_interp = interp1(gt_pose(:,1), gt_pose(:,2:4), estimate_pose(:, 1));
gt_pose = estimate_pose;
gt_pose(:,2:4) = gt_interp;

% interpolate gt_vel to estimate_vel
gt_vel_interp = interp1(gt_vel(:,1), gt_vel(:,2:4), estimate_vel(:, 1));
gt_vel = estimate_vel;
gt_vel(:,2:4) = gt_vel_interp;

% interpolate gt_vel_enu to estimate_vel
gt_vel_enu_interp = interp1(gt_vel_enu(:,1), gt_vel_enu(:,2:4), estimate_vel_enu(:, 1));
gt_vel_enu = estimate_vel_enu;
gt_vel_enu(:,2:4) = gt_vel_enu_interp;

% nan checks
estimate_pose(isnan(estimate_pose))=0;
estimate_vel(isnan(estimate_vel))=0;
estimate_vel_enu(isnan(estimate_vel_enu))=0;
gt_pose(isnan(gt_pose))=0;
gt_vel(isnan(gt_vel))=0;
gt_vel_enu(isnan(gt_vel_enu))=0;

%%%%
%rot = fminsearch(@(x) costf(x, gt(1:10, :), estimate_pose(1:10, :)), 0);
%estimate_pose2 = cos(rot)*estimate_pose(:, 2) - sin(rot)*estimate_pose(:, 3);
%estimate_pose3 = sin(rot)*estimate_pose(:, 2) + cos(rot)*estimate_pose(:, 3);
%estimate_pose(:, 2) = estimate_pose2;
%estimate_pose(:, 3) = estimate_pose3;
%%%%

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

figure('units','normalized', 'outerposition', [0 0.1 1 0.9])
group = uitabgroup();

tab1 = uitab(group, 'Title', 'General');
t = tiledlayout(tab1, 2, 3);
t.TileSpacing = 'tight';
t.Padding = 'tight';

gx = geoaxes(t);
gx.Layout.Tile = 1;
gx.Layout.TileSpan = [2 1];
key = 'pk.eyJ1Ijoia2xlbWVuNjMiLCJhIjoiY2twYTg5YjN5MHE0czJyb2c5Z2x0YjRmdSJ9.m5V1rMbGCykrJ2f0OMOuWA';
addCustomBasemap('mapbox', strcat('https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=', key));
[gtlat, gtlon, ~] = local2latlon(gt_pose(:,2), gt_pose(:,3), 0, datum);
[estimate_poselat, estimate_poselon, ~] = local2latlon(estimate_pose(:,2), estimate_pose(:,3), 0, datum);
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

tab2 = uitab(group, 'Title', '3D view');
ax = axes('Parent', tab2);
view(ax, 3)
hold (ax, 'on')
plot3(gt_pose(:,2), gt_pose(:,3), gt_pose(:,4), 'Parent', ax);
plot3(estimate_pose(:,2), estimate_pose(:,3), estimate_pose(:,4), 'Parent', ax);
title(ax, "3D");
hold (ax, 'off')

tab3 = uitab(group, 'Title', 'Velocity in ENU frame');
t2 = tiledlayout(tab3, 1, 2);
t2.TileSpacing = 'tight';
t2.Padding = 'tight';

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt_vel_enu(:, 1), -smooth(gt_vel_enu(:, 3), 5));
plot(ax, estimate_vel_enu(:, 1), smooth(estimate_vel_enu(:, 2), 100));
title(ax, "X velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt_vel_enu(:, 1), smooth(gt_vel_enu(:, 2), 5));
plot(ax, estimate_vel_enu(:, 1), smooth(estimate_vel_enu(:, 3), 100));
title(ax, "Y velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

tab4 = uitab(group, 'Title', 'Velocity in local frame');
t3 = tiledlayout(tab4, 1, 2);
t3.TileSpacing = 'tight';
t3.Padding = 'tight';

ax = nexttile(t3);
hold (ax, 'on')
plot(ax, gt_vel(:, 1), smooth(gt_vel(:, 3), 5));
plot(ax, estimate_vel(:, 1), smooth(estimate_vel(:, 2), 100));
title(ax, "X velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t3);
hold (ax, 'on')
plot(ax, gt_vel(:, 1), -smooth(gt_vel(:, 2), 5));
plot(ax, estimate_vel(:, 1), smooth(estimate_vel(:, 3), 100));
title(ax, "Y velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')
