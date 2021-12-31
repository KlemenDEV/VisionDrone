function err = costf2(k, gt, data)

data2 = k*data(:, 2);
data3 = k*data(:, 3);

err2d = sqrt((gt(:,2) - data2).^2 + (gt(:,3) - data3).^2);

err = sum(err2d);

