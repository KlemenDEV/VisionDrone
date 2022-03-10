clc;
close all;

[files, path] = uigetfile("*.mat", "Select estimate_pose file", 'Multiselect', 'on');

for k=1:length(files)
    files(k) = strcat(path, files(k));
end

results = containers.Map('KeyType', 'char', 'ValueType', 'any');

for k=1:length(files)
    file = string(files(k));

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
clc;

% data entries
mkey = keys(results);
mvalue = values(results);

key = 'pk.eyJ1Ijoia2xlbWVuNjMiLCJhIjoiY2twYTg5YjN5MHE0czJyb2c5Z2x0YjRmdSJ9.m5V1rMbGCykrJ2f0OMOuWA';
addCustomBasemap('mapbox', strcat('https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=', key), 'Attribution', 'Vir ortofoto: MapBox');

f = figure(1);
f.Position = [0 0 750 450];
gx = geoaxes;
hold (gx, 'on')
geobasemap('mapbox');
gx.Grid = 'off';
gx.LatitudeLabel.String = 'Zemplepisna širina';
gx.LongitudeLabel.String = 'Zemljepisna dolžina';
gx.Title.String = "";
for i = 1:length(results)
    data = mvalue{i};
    datum = data.datum;
	[gtlat, gtlon, ~] = local2latlon(data.gt_pose(1:end-1,2), data.gt_pose(1:end-1,3), 0, datum);
	geoplot(gx, gtlat, gtlon, 'LineWidth', 1.5);
	[lx, ly] = geolimits;
end
off = 0.0001;
lx(1) = lx(1) - off;
lx(2) = lx(2) + off;
ly(1) = ly(1) - off;
ly(2) = ly(2) + off;
geolimits(lx, ly);
hold (gx, 'off')

f = figure(2);
f.Position = [0 0 750 450];
gx = geoaxes;
hold (gx, 'on')
geobasemap('mapbox');
gx.Grid = 'off';
gx.LatitudeLabel.String = 'Zemplepisna širina';
gx.LongitudeLabel.String = 'Zemljepisna dolžina';
gx.Title.String = "";
for i = 1:length(results)
    data = mvalue{i};
    [estimate_poselat, estimate_poselon, ~] = local2latlon(data.estimate_pose(1:end-10,2), data.estimate_pose(1:end-10,3), 0, datum);
    geoplot(gx, estimate_poselat, estimate_poselon, 'LineWidth', 1.5);
    [lx, ly] = geolimits;
end
off = 0.0001;
lx(1) = lx(1) - off;
lx(2) = lx(2) + off;
ly(1) = ly(1) - off;
ly(2) = ly(2) + off;
geolimits(lx, ly);
hold (gx, 'off')
