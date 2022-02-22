clearvars;
close all;
clc;

[file, path] = uigetfile("*.mat", "Select estimate_pose file");
load(strcat(path, file));

%%%%% INTERPOLATIONS START
% interpolate estimate to GPS

% interpolate gt_pose to estimate_pose
estimate_pose_interp = gt_pose;
for i=1:1:size(gt_pose, 1)
    idx = size(estimate_pose, 1) * i / size(gt_pose, 1);
    estimate_pose_interp(i, 2:4) = estimate_pose(int32(round(idx)), 2:4);
end
estimate_pose = estimate_pose_interp;

gt_vacc_interp = gt_pose;
for i=1:1:size(gt_pose, 1)
    idx = size(gt_vacc, 1) * i / size(gt_pose, 1);
    gt_vacc_interp(i, 2) = gt_vacc(int32(round(idx)), 2);
end
gt_vacc = gt_vacc_interp;

%%%%% INTERPOLATIONS END

% height error
errh = abs(gt_pose(:,4) - estimate_pose(:,4));

f = figure(1);
f.Position = [0 0 650 500];
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

t = nexttile;
set(t, 'YGrid', 'on', 'XGrid', 'off')
hold on

x = gt_pose(:,1);
y1 = gt_pose(:,4) - (gt_vacc(:,2) .* 2 ./ 1000);
y2 = gt_pose(:,4) + (gt_vacc(:,2) .* 2 ./ 1000);
shade(x,y1,x,y2,'FillType',[1 2;2 1],'FillAlpha',0.15,'FillColor','black');

plot(gt_pose(:,1), gt_pose(:,4), 'LineWidth', 1.5, 'Color', 'black');
plot(gt_pose(:,1), estimate_pose(:,4), 'LineWidth', 1.5, 'Color', '#3d9db2');
plot(h_gnd(:,1), h_gnd(:,2), 'LineWidth', 1.5, 'Color', '#43bb4b');

hold off
title("Ocena višine");
xlabel("čas / s");
ylabel("h / m");
xlim([0 gt_pose(end, 1)])
legend("", "", "", "h_{rel}  glede na GPS", "h_{rel}  ocenjena", "h_{tla}  izmerjena");

t2 = nexttile;
set(t2, 'YGrid', 'on', 'XGrid', 'off')

hold on;
plot(gt_pose(:,1), errh, 'LineWidth', 1.5, 'Color', '#e9a06c');
hold off;
title("Napaka ocene h_{rel}");
xlabel("čas / s");
xlim([0 gt_pose(end, 1)])
ylabel("h_{err,rel} / m");

fprintf("Average error: %.3f m, standard deviation: %.3f m\n", mean(errh), std(errh));
