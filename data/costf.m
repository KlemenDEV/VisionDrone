function err = costf(par, gt, data)

data2 = cos(par(1))*data(:, 2) - sin(par(1))*data(:, 3);
data3 = sin(par(1))*data(:, 2) + cos(par(1))*data(:, 3);

data2 = par(2)*data2;
data3 = par(2)*data3;

err2d = sqrt((gt(:,2) - data2).^2 + (gt(:,3) - data3).^2);

err = sum(err2d);

