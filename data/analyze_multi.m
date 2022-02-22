clc;
close all;

[files, path] = uigetfile("*.mat", "Select estimate_pose file", 'Multiselect', 'on');

for k=1:length(files)
    files(k) = strcat(path, files(k));
end

results = containers.Map('KeyType', 'char', 'ValueType', 'any');

for k=1:length(files)
    file = string(files(k));
    
    fprintf("=====================================\n");
    fprintf("File: %s\n\n", file);

    load(file);
    run analyze_impl.m
    close all

    result.datum = datum;
	result.gt_pose = gt_pose;
    result.d_gt = d_gt;
    result.gt_vel_enu = gt_vel_enu;
    result.gt_vel = gt_vel;
    
	result.estimate_pose = estimate_pose;
	result.err2d = err2d;
	result.d_estimate_pose = d_estimate_pose;
	result.poserr_x = poserr_x;
	result.poserr_y = poserr_y;
	result.estimate_vel_enu = estimate_vel_enu;
	result.enu_velerr_x = enu_velerr_x;
	result.enu_velerr_y = enu_velerr_y;
	result.estimate_vel = estimate_vel;
	result.velerr_x = velerr_x;
	result.velerr_y = velerr_y;
	result.estimate_pose_fit = estimate_pose_fit;
    
    result.metrics = metrics;
    
    results(file) = result;

    clearvars -except files results
end

close all;

% data entries
mkey = keys(results);
mvalue = values(results);

f = figure(1);
f.Position = [0 0 750 450];
gx = geoaxes;
key = 'pk.eyJ1Ijoia2xlbWVuNjMiLCJhIjoiY2twYTg5YjN5MHE0czJyb2c5Z2x0YjRmdSJ9.m5V1rMbGCykrJ2f0OMOuWA';
addCustomBasemap('mapbox', strcat('https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=', key), 'Attribution', 'Vir ortofoto: MapBox');
hold (gx, 'on')
geobasemap('mapbox');
gx.Grid = 'off';
gx.LatitudeLabel.String = 'Zemplepisna širina';
gx.LongitudeLabel.String = 'Zemljepisna dolžina';
gx.Title.String = "";

names = cell(length(results) + 1, 1);
names{1} = "GPS";
for i = 1:length(results)
    names{i + 1} = localization_type(mkey{i});

    data = mvalue{i};
    if i == 1
        datum = data.datum;
        [gtlat, gtlon, ~] = local2latlon(data.gt_pose(:,2), data.gt_pose(:,3), 0, datum);
        geoplot(gx, gtlat, gtlon, ':', 'LineWidth', 2, 'Color', 'black');
        [lx, ly] = geolimits;
    end
    
    [estimate_poselat, estimate_poselon, ~] = local2latlon(data.estimate_pose(:,2), data.estimate_pose(:,3), 0, datum);
    geoplot(gx, estimate_poselat, estimate_poselon, 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
off = 0.0006;
lx(1) = lx(1) - off;
lx(2) = lx(2) + off;
ly(1) = ly(1) - off;
ly(2) = ly(2) + off;
geolimits(lx, ly);
legend(names);
hold (gx, 'off')

f = figure(2);
f.Position = [0 0 650 320];
ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
hold on

names = cell(length(results), 1);
hold on;
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    
    data = mvalue{i};
    plot(data.gt_pose(:,1), data.err2d, 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("2D napaka ocene lokacije skozi čas");
xlim([0 data.gt_pose(end, 1)])
xlabel("čas / s");
ylabel("p_{err} / m");
legend(names, 'Location','Best');
hold off;

f = figure(3);
f.Position = [0 0 700 320];
tiledlayout(1,2, 'Padding', 'none', 'TileSpacing', 'compact'); 
nexttile;

ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
hold on
names = cell(length(results) + 1, 1);
names{1} = "GPS";
hold on;
for i = 1:length(results)
    names{i + 1} = localization_type(mkey{i});
    
    data = mvalue{i};
    if i == 1
        plot(data.gt_pose(:,1), data.gt_pose(:,2), 'Color', 'black', 'LineWidth', 1.5);
    end
    
    plot(data.gt_pose(:,1), data.estimate_pose(:,2), 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("Ocena x_{ENU}");
xlabel("čas / s");
ylabel("x_{ENU} / m");
xlim([0 data.gt_pose(end, 1)])
legend(names, 'Location','Best');
hold off;

nexttile;
ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
hold on
names = cell(length(results) + 1, 1);
names{1} = "GPS";
hold on;
for i = 1:length(results)
    names{i + 1} = localization_type(mkey{i});

    data = mvalue{i};
    if i == 1
        plot(data.gt_pose(:,1), data.gt_pose(:,3), 'Color', 'black', 'LineWidth', 1.5);
    end

    plot(data.gt_pose(:,1), data.estimate_pose(:,3), 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("Ocena y_{ENU}");
xlabel("čas / s");
ylabel("y_{ENU} / m");
xlim([0 data.gt_pose(end, 1)])
legend(names, 'Location','Best');
hold off;

% figure(4)
% subplot(2,2,1)
% names = cell(length(results) + 1, 1);
% names{1} = "GPS";
% hold on;
% for i = 1:length(results)
%     names{i + 1} = localization_type(mkey{i});
% 
%     data = mvalue{i};
%     if i == 1
%         plot(data.gt_vel(:,1), data.gt_vel(:,2), 'Color', 'black', 'LineWidth', 1.5);
%     end
% 
%     plot(data.gt_vel(:,1), data.estimate_vel(:,2), 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
% end
% title("Lokalna hitrost x");
% xlabel("time / s");
% ylabel("v_x / m/s");
% legend(names);
% hold off;
% 
% subplot(2,2,2)
% names = cell(length(results) + 1, 1);
% names{1} = "GPS";
% hold on;
% for i = 1:length(results)
%     names{i + 1} = localization_type(mkey{i});
% 
%     data = mvalue{i};
%     if i == 1
%         plot(data.gt_vel(:,1), data.gt_vel(:,3), 'Color', 'black', 'LineWidth', 1.5);
%     end
% 
%     plot(data.gt_vel(:,1), data.estimate_vel(:,3), 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
% end
% title("Lokalna hitrost y");
% xlabel("time / s");
% ylabel("v_y / m/s");
% legend(names);
% hold off;
% 
% subplot(2,2,3)
% names = cell(length(results), 1);
% hold on;
% for i = 1:length(results)
%     names{i} = localization_type(mkey{i});
% 
%     data = mvalue{i};
%     plot(data.gt_vel(:,1), data.velerr_x, 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
% end
% title("Napaka ocene lokalne hitrosti x");
% xlabel("time / s");
% ylabel("v_{err,x} / m/s");
% legend(names);
% hold off;
% 
% subplot(2,2,4)
% names = cell(length(results), 1);
% hold on;
% for i = 1:length(results)
%     names{i} = localization_type(mkey{i});
% 
%     data = mvalue{i};
%     plot(data.gt_vel(:,1), data.velerr_y, 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
% end
% title("Napaka ocene lokalne hitrosti y");
% xlabel("time / s");
% ylabel("v_{err,y} / m/s");
% legend(names);
% hold off;

f = figure(5);
f.Position = [0 0 900 500];
tiledlayout(2,2, 'Padding', 'none', 'TileSpacing', 'compact'); 

nexttile;
ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
names = cell(length(results) + 1, 1);
names{1} = "GPS";
hold on;
for i = 1:length(results)
    names{i + 1} = localization_type(mkey{i});

    data = mvalue{i};
    if i == 1
        plot(data.gt_vel_enu(:,1), data.gt_vel_enu(:,2), 'Color', 'black', 'LineWidth', 1.5);
    end

    plot(data.gt_vel_enu(:,1), data.estimate_vel_enu(:,2), 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("Ocena hitrosti v_{ENU,x}");
xlabel("čas / s");
ylabel("v_{ENU,x} / m/s");
legend(names);
xlim([0 data.gt_pose(end, 1)])
hold off;

nexttile;
ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
names = cell(length(results) + 1, 1);
names{1} = "GPS";
hold on;
for i = 1:length(results)
    names{i + 1} = localization_type(mkey{i});

    data = mvalue{i};
    if i == 1
        plot(data.gt_vel_enu(:,1), data.gt_vel_enu(:,3), 'Color', 'black', 'LineWidth', 1.5);
    end

    plot(data.gt_vel_enu(:,1), data.estimate_vel_enu(:,3), 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("Ocena hitrosti v_{ENU,y}");
xlabel("čas / s");
ylabel("v_{ENU,y} / m/s");
xlim([0 data.gt_pose(end, 1)])
legend(names);
hold off;

nexttile;
ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
names = cell(length(results), 1);
hold on;
for i = 1:length(results)
    names{i} = localization_type(mkey{i});

    data = mvalue{i};
    plot(data.gt_vel_enu(:,1), data.enu_velerr_x, 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("Napaka ocene hitrosti v_{ENU,x}");
xlabel("čas / s");
ylabel("v_{err,ENU,x} / m/s");
xlim([0 data.gt_pose(end, 1)])
legend(names);
hold off;

nexttile;
ax = gca;
grid minor
set(ax, 'YGrid', 'on', 'XGrid', 'off')
xlim([0 data.gt_pose(end, 1)])
names = cell(length(results), 1);
hold on;
for i = 1:length(results)
    names{i} = localization_type(mkey{i});

    data = mvalue{i};
    plot(data.gt_vel_enu(:,1), data.enu_velerr_y, 'Color', localization_color(mkey{i}), 'LineWidth', 1.5);
end
title("Napaka ocene hitrosti v_{ENU,y}");
xlabel("čas / s");
ylabel("v_{err,ENU,y} / m/s");
legend(names);
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Izpis tabele za latex %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf("\\hline\n");
fprintf("\\multicolumn{1}{|l||}{\\backslashbox{Metrika}{Metoda}} ");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    fprintf("& %s ", names{i});
end
fprintf("\\\\\n");

fprintf("\\hline\\hline\n");

fprintf("$p_{err}$~~~~~~~~~~[$m$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(1)),'.',','));
end
fprintf("\\\\\n");

fprintf("$\\sigma_{p_{err}}$~~~~~~~~~[$m$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(2)),'.',','));
end
fprintf("\\\\\n");

fprintf("$p_{err,rel}$~~~~~~~[$\\%%$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(3)),'.',','));
end
fprintf("\\\\\n");

fprintf("\\hline\n");
fprintf("$D_{err}$~~~~~~~~~[$m$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(4)),'.',','));
end
fprintf("\\\\\n");

fprintf("$D_{err,rel}$~~~~~~[$\\%%$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(5)),'.',','));
end
fprintf("\\\\\n");

fprintf("\\hline\n");
fprintf("$\\overline{v}_{err,x}$~~~~~~~~~[$m/s$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(6)),'.',','));
end
fprintf("\\\\\n");
fprintf("$\\sigma_{v_{err,x}}$~~~~~~~~[$m/s$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(7)),'.',','));
end
fprintf("\\\\\n");

fprintf("\\hline\n");
fprintf("$\\overline{v}_{err,y}$~~~~~~~~~[$m/s$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(8)),'.',','));
end
fprintf("\\\\\n");
fprintf("$\\sigma_{v_{err,y}}$~~~~~~~~[$m/s$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(9)),'.',','));
end
fprintf("\\\\\n");

fprintf("\\hline\n");
fprintf("$D12_{err}$~~~~~~~[$m$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(10)),'.',','));
end
fprintf("\\\\\n");

fprintf("$D12_{err,rel}$~~~~[$\\%%$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(11)),'.',','));
end
fprintf("\\\\\n");

fprintf("\\hline\n");
fprintf("$T_{err}$~~~~~~~~~~~[$\\%%$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(12)),'.',','));
end
fprintf("\\\\\n");

fprintf("$k_{err}$~~~~~~~~~~~[$\\%%$] \n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(13)),'.',','));
end
fprintf("\\\\\n");

fprintf("$\\varphi_{err}$~~~~~~~~~~~[$rad$]\n");
for i = 1:length(results)
    names{i} = localization_type(mkey{i});
    metrics = mvalue{i}.metrics;
    fprintf("%s", strrep(sprintf("& %.2f ", metrics(14)),'.',','));
end
fprintf("\\\\\n");

fprintf("\\hline\n");
