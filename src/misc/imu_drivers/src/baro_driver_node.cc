#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <time.h>

#include "bmp280.h"
#include "i2c.h"

struct I2cDevice dev;

static uint32_t compute_meas_time(const struct bmp280_config *conf) {
    uint32_t measurement_time[] = {5500, 7500, 11500, 19500, 37500};
    uint32_t standby_time[] = {500, 62500, 125000, 250000, 500000, 1000000, 2000000, 4000000};
    return measurement_time[conf->os_pres] + standby_time[conf->odr]; // us
}

static void delay_ms(uint32_t period_ms) {
    int res;
    struct timespec ts;
    ts.tv_sec = period_ms / 1000;
    ts.tv_nsec = (period_ms % 1000) * 1000000;
    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);
}

static int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    int res = i2c_writen_reg(&dev, reg_addr, reg_data, length);
    if (res > 0)
        res = 0;
    return res;
}

static int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    int res = i2c_readn_reg(&dev, reg_addr, reg_data, length);
    if (res > 0)
        res = 0;
    return res;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "baro_driver_node");
    ros::NodeHandle nh;

    ros::Publisher baro_pub = nh.advertise<sensor_msgs::FluidPressure>("/baro/data", 10);

    dev.filename = (char *) "/dev/i2c-1";
    dev.addr = BMP280_I2C_ADDR_PRIM;

    if (i2c_start(&dev)) {
        ROS_ERROR("failed to start i2c device");
        return -1;
    }

    struct bmp280_dev bmp;
    struct bmp280_config conf;
    bmp.intf = BMP280_I2C_INTF;
    bmp.delay_ms = delay_ms;
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    int8_t rslt = bmp280_init(&bmp);
    ROS_INFO("bmp280_init status: %d", rslt);

    rslt = bmp280_get_config(&conf, &bmp);
    ROS_INFO("bmp280_get_config status: %d", rslt);

    conf.filter = BMP280_FILTER_COEFF_2;
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_2X;
    conf.odr = BMP280_ODR_62_5_MS;

    rslt = bmp280_set_config(&conf, &bmp);
    ROS_INFO("bmp280_set_config status: %d", rslt);

    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    ROS_INFO("bmp280_set_power_mode status: %d", rslt);

    double sleep_time_ms = compute_meas_time(&conf) / 1000.0;
    ros::Rate rate(1000.0 / sleep_time_ms);
    while (ros::ok()) {
        double pressure_comp, temperat_comp;
        struct bmp280_uncomp_data ucomp_data;
        bmp280_get_uncomp_data(&ucomp_data, &bmp);
        bmp280_get_comp_pres_double(&pressure_comp, ucomp_data.uncomp_press, &bmp);
        bmp280_get_comp_temp_double(&temperat_comp, ucomp_data.uncomp_temp, &bmp);

        sensor_msgs::FluidPressure pressure;
        pressure.header.stamp = ros::Time::now();
        pressure.fluid_pressure = pressure_comp;
        baro_pub.publish(pressure);
        ros::spinOnce();
        rate.sleep();
    }

    i2c_stop(&dev);

    ros::shutdown();

    return 0;
}