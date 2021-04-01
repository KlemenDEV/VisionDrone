#pragma once

#define UBX_HEADER { 0xB5, 0x62 }
#define UBX_CLASS_NAV 0x01

#define PACKED __attribute__((__packed__))
#define __APM_REQUIRED

#define DEBUG

// Protocol specification
// https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf

struct checksum_t {
    uint8_t ck_a;
    uint8_t ck_b;
};

// UBX messages
#define UBX_NAV_POSLLH_ID 0x02
struct PACKED ubx_nav_posllh {
    __APM_REQUIRED uint32_t time; // GPS msToW
    __APM_REQUIRED int32_t longitude;
    __APM_REQUIRED int32_t latitude;
    int32_t altitude_ellipsoid;
    __APM_REQUIRED int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
};

#define UBX_NAV_STATUS_ID 0x03
struct PACKED ubx_nav_status {
    uint32_t time; // GPS msToW
    __APM_REQUIRED uint8_t fix_type;
    __APM_REQUIRED uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;
};

#define UBX_NAV_SOLUTION_ID 0x06
struct PACKED ubx_nav_solution {
    __APM_REQUIRED uint32_t time; // GPS msToW
    int32_t time_nsec;
    __APM_REQUIRED uint16_t week;
    __APM_REQUIRED uint8_t fix_type;
    __APM_REQUIRED uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    __APM_REQUIRED uint16_t position_DOP; // https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
    uint8_t res;
    __APM_REQUIRED uint8_t satellites;
    uint32_t res2;
};

#define UBX_NAV_VELNED_ID 0x12
struct PACKED ubx_nav_velned {
    __APM_REQUIRED uint32_t time; // GPS msToW
    __APM_REQUIRED int32_t ned_north;
    __APM_REQUIRED int32_t ned_east;
    __APM_REQUIRED int32_t ned_down;
    uint32_t speed_3d;
    __APM_REQUIRED uint32_t speed_2d;
    __APM_REQUIRED int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
};

/**
 * Calculate checksum using 8-bit Fletcher's algorithm.
 *
 * @param data
 * @param size
 * @return checksum
 */
static inline checksum_t fletcher_16_checksum(uint8_t *data, size_t size) {
    checksum_t checksum;

    checksum.ck_a = 0;
    checksum.ck_b = 0;

    for (size_t i = 0; i < size; i++) {
        checksum.ck_a += data[i];
        checksum.ck_b += checksum.ck_a;
    }

    return checksum;
}

static inline std::vector <uint8_t>
generateUBXMessage(uint8_t msgClass, uint8_t msgID, uint8_t *data, uint16_t length) {
    std::vector <uint8_t> result;

    result.push_back(msgClass);
    result.push_back(msgID);

    result.push_back(length & 0xFF);
    result.push_back((length >> 8) & 0xFF);

    // add data if present
    if (length > 0) {
        result.insert(result.end(), data, data + length);
    }

    // add 16-bit fletcher checksum at the end of the message
    checksum_t checksum = fletcher_16_checksum(result.data(), result.size());
    result.push_back(checksum.ck_a);
    result.push_back(checksum.ck_b);

    // after data checksum is calculated, insert header at the start of the message
    result.insert(result.begin(), UBX_HEADER);

#ifdef DEBUG
    printf("UBX MSG> ");
    for (uint8_t i : result)
        printf("%02x ", i);
    printf("\n");
#endif

    return result;
}

// Message generator test sample code
/*uint8_t data[] = {0xF0, 0x00};
std::vector<uint8_t> msg = generateUBXMessage(0x06, 0x01, data, 2);
printf("\n");
for (uint8_t i : msg) {
    printf("%02x ", i);
}
printf("\n");*/
