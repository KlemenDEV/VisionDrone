#pragma once

#define UBX_HEADER { 0xB5, 0x62 }
#define UBX_CLASS_NAV 0x01

struct checksum_t {
    uint8_t ck_a;
    uint8_t ck_b;
};

// UBX messages
#define UBX_NAV_POSLLH_ID 0x02
struct PACKED ubx_nav_posllh {
        APM_REQUIRED uint32_t time; // GPS msToW
        APM_REQUIRED int32_t longitude;
        APM_REQUIRED int32_t latitude;
        int32_t altitude_ellipsoid;
        APM_REQUIRED int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
};

#define UBX_NAV_STATUS_ID 0x03
struct PACKED ubx_nav_status {
        uint32_t time; // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime; // milliseconds
};

#define UBX_NAV_SOLUTION_ID 0x06
struct PACKED ubx_nav_solution {
        uint32_t time;
        int32_t time_nsec;
        uint16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
};

#define UBX_NAV_VELEND_ID 0x12
struct PACKED ubx_nav_velned {
        uint32_t time; // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
};