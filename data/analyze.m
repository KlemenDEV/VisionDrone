clear all;
close all;
clc;

[file, path] = uigetfile("*.mat", "Select data file");
load(strcat(path, file));

gt_interp = interp1(gt(:,1), gt(:,2:4), data(:, 1));
gt = data;
gt(:,2:4) = gt_interp;

gt(isnan(gt))=0;
data(isnan(data))=0;

err2d = sqrt((gt(:,2) - data(:,2)).^2 + (gt(:,3) - data(:,3)).^2);

errh = abs(gt(:,4) - data(:,4));

d_gt_raw = diff(gt(:, 2:3));
d_data_raw = diff(data(:, 2:3));

d_gt = cumsum(sqrt(sum(d_gt_raw.*d_gt_raw,2)));
d_data = cumsum(sqrt(sum(d_data_raw.*d_data_raw,2)));
d_err = abs(d_gt - d_data);

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
[gtlat, gtlon, ~] = local2latlon(gt(:,2), gt(:,3), 0, datum);
[datalat, datalon, ~] = local2latlon(data(:,2), data(:,3), 0, datum);
hold (gx, 'on')
geobasemap('mapbox');
geoplot(gx, gtlat, gtlon);
geoplot(gx, datalat, datalon);
title(gx, "2D (map view)");
legend(gx, "GPS GT", "Estimate");
hold (gx, 'off')

ax = nexttile(t);
hold (ax, 'on')
plot(ax, gt(1:end-1,1), d_gt);
plot(ax, gt(1:end-1,1), d_data);
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
plot(ax, gt(:,1), errh);
title(ax, "Height error over time");
xlabel(ax, "time / s");
ylabel(ax, "error / m");

ax = nexttile(t);
plot(ax, gt(:,1), err2d);
title(ax, "2D estimate error over time");
xlabel(ax, "time / s");
ylabel(ax, "error / m");

tab2 = uitab(group, 'Title', '3D view');
ax = axes('Parent', tab2);
view(ax, 3)
hold (ax, 'on')
plot3(gt(:,2), gt(:,3), gt(:,4), 'Parent', ax);
plot3(data(:,2), data(:,3), data(:,4), 'Parent', ax);
title(ax, "3D");
hold (ax, 'off')

tab3 = uitab(group, 'Title', 'Velocity');
t2 = tiledlayout(tab3, 1, 2);
t2.TileSpacing = 'tight';
t2.Padding = 'tight';

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt(1:end-1, 1), smooth(filloutliers(d_gt_raw(:, 1)./diff(gt(:, 1)), 'previous'), 50) );
plot(ax, gt(1:end-1, 1), smooth(filloutliers(d_data_raw(:, 1)./diff(gt(:, 1)), 'previous'), 50) );
title(ax, "X velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')

ax = nexttile(t2);
hold (ax, 'on')
plot(ax, gt(1:end-1, 1), smooth(filloutliers(d_gt_raw(:, 2)./diff(gt(:, 1)), 'previous'), 50) );
plot(ax, gt(1:end-1, 1), smooth(filloutliers(d_data_raw(:, 2)./diff(gt(:, 1)), 'previous'), 50) );
title(ax, "X velocity over time");
xlabel(ax, "time / s");
ylabel(ax, "v / m/s");
legend(ax, "GPS GT", "Estimate");
hold (ax, 'off')
