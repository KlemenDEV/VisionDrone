function err = costf(rot, gt, data)

data2 = cos(rot)*data(:, 2) - sin(rot)*data(:, 3);
data3 = sin(rot)*data(:, 2) + cos(rot)*data(:, 3);

data(:, 2) = data2;
data(:, 2) = data3;

err2d = sqrt((gt(:,2) - data(:,2)).^2 + (gt(:,3) - data(:,3)).^2);

err = sum(err2d);
