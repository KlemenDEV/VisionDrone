package com.intel.realsense.librealsense;

public enum Option {
    BACKLIGHT_COMPENSATION(0),
    BRIGHTNESS(1),
    CONTRAST(2),
    EXPOSURE(3),
    GAIN(4),
    GAMMA(5),
    HUE(6),
    SATURATION(7),
    SHARPNESS(8),
    WHITE_BALANCE(9),
    ENABLE_AUTO_EXPOSURE(10),
    ENABLE_AUTO_WHITE_BALANCE(11),
    VISUAL_PRESET(12),
    LASER_POWER(13),
    ACCURACY(14),
    MOTION_RANGE(15),
    FILTER_OPTION(16),
    CONFIDENCE_THRESHOLD(17),
    EMITTER_ENABLED(18),
    FRAMES_QUEUE_SIZE(19),
    TOTAL_FRAME_DROPS(20),
    AUTO_EXPOSURE_MODE(21),
    POWER_LINE_FREQUENCY(22),
    ASIC_TEMPERATURE(23),
    ERROR_POLLING_ENABLED(24),
    PROJECTOR_TEMPERATURE(25),
    OUTPUT_TRIGGER_ENABLED(26),
    MOTION_MODULE_TEMPERATURE(27),
    DEPTH_UNITS(28),
    ENABLE_MOTION_CORRECTION(29),
    AUTO_EXPOSURE_PRIORITY(30),
    COLOR_SCHEME(31),
    HISTOGRAM_EQUALIZATION_ENABLED(32),
    MIN_DISTANCE(33),
    MAX_DISTANCE(34),
    TEXTURE_SOURCE(35),
    FILTER_MAGNITUDE(36),
    FILTER_SMOOTH_ALPHA(37),
    FILTER_SMOOTH_DELTA(38),
    HOLES_FILL(39),
    STEREO_BASELINE(40),
    AUTO_EXPOSURE_CONVERGE_STEP(41),
    INTER_CAM_SYNC_MODE(42),
    STREAM_FILTER(43),
    STREAM_FORMAT_FILTER(44),
    STREAM_INDEX_FILTER(45),
    EMITTER_ON_OFF(46),
    ZERO_ORDER_POINT_X(47),
    ZERO_ORDER_POINT_Y(48),
    LLD_TEMPERATURE(49),
    MC_TEMPERATURE(50),
    MA_TEMPERATURE(51),
    HARDWARE_PRESET(52),
    GLOBAL_TIME_ENABLED(53),
    APD_TEMPERATURE(54),
    ENABLE_MAPPING(55),
    ENABLE_RELOCALIZATION(56),
    ENABLE_POSE_JUMPING(57),
    ENABLE_DYNAMIC_CALIBRATION(58),
    DEPTH_OFFSET(59),
    LED_POWER(60),
    ZERO_ORDER_ENABLED(61), // Deprecated
    ENABLE_MAP_PRESERVATION(62),
    FREEFALL_DETECTION_ENABLED(63),
    AVALANCHE_PHOTO_DIODE(64),
    POST_PROCESSING_SHARPENING(65),
    PRE_PROCESSING_SHARPENING(66),
    NOISE_FILTERING(67),
    INVALIDATION_BYPASS(68),
    AMBIENT_LIGHT(69), // Deprecated - Use DIGITAL_GAIN instead
    DIGITAL_GAIN(69),
    SENSOR_MODE(70),
    EMITTER_ALWAYS_ON(71),
    THERMAL_COMPENSATION(72),
    TRIGGER_CAMERA_ACCURACY_HEALTH(73),
    RESET_CAMERA_ACCURACY_HEALTH(74),
    HOST_PERFORMANCE(75),
    HDR_ENABLED(76),
    SEQUENCE_NAME(77),
    SEQUENCE_SIZE(78),
    SEQUENCE_ID(79),
    HUMIDITY_TEMPERATURE(80),
    ENABLE_MAX_USABLE_RANGE(81),
    ALTERNATE_IR(82),
    NOISE_ESTIMATION(83),
    ENABLE_IR_REFLECTIVITY(84),
    AUTO_EXPOSURE_LIMIT(85),
    AUTO_GAIN_LIMIT(86);
    private final int mValue;

    private Option(int value) { mValue = value; }
    public int value() { return mValue; }
}
