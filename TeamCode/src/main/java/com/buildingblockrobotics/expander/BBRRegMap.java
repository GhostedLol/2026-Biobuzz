// BBRRegMap.java — BBR Digital Expander I2C register map constants.
//
// GENERATED FILE — DO NOT EDIT. Regenerated from the protocol definition
// whenever the register map changes.
//
// Protocol contract 1.2. Multi-byte values are little-endian.
// What each register means: https://expander.buildingblockrobotics.com/

package com.buildingblockrobotics.expander;

public final class BBRRegMap {
    private BBRRegMap() {}

    // Protocol and device identity
    public static final int PROTOCOL_MAJOR_VALUE = 1;
    public static final int PROTOCOL_MINOR_VALUE = 2;
    public static final int DEVICE_ID_VALUE = 0xB2;
    public static final int I2C_ADDR_DEFAULT = 0x38; // 7-bit
    public static final int I2C_ADDR_MIN = 0x38;
    public static final int I2C_ADDR_MAX = 0x3B;

    // Registers
    public static final int REG_DEVICE_ID = 0x00; // u8, R
    public static final int REG_FW_VERSION_MAJOR = 0x01; // u8, R
    public static final int REG_FW_VERSION_MINOR = 0x02; // u8, R
    public static final int REG_FW_VERSION_PATCH = 0x03; // u8, R
    public static final int REG_PROTOCOL_MAJOR = 0x04; // u8, R
    public static final int REG_PROTOCOL_MINOR = 0x05; // u8, R
    public static final int REG_HW_VARIANT = 0x06; // u8, R
    public static final int REG_CAPABILITIES = 0x07; // u8, R
    public static final int REG_STATUS = 0x08; // u8, R
    public static final int REG_LAST_INVALID_REG = 0x09; // u8, R
    public static final int REG_COMMAND_ARG = 0x0A; // u8, W
    public static final int REG_COMMAND_TOKEN = 0x0B; // u8, RW
    public static final int REG_COMMAND = 0x0C; // u8, W
    public static final int REG_COMMAND_STATUS = 0x0D; // u8, R
    public static final int REG_COMMAND_RESULT = 0x0E; // u8, R
    public static final int REG_COMMAND_ECHO = 0x0F; // u8, R
    public static final int REG_ENC0_COUNT = 0x10; // i32, R
    public static final int REG_ENC1_COUNT = 0x14; // i32, R
    public static final int REG_ENC2_COUNT = 0x18; // i32, R
    public static final int REG_ENC3_COUNT = 0x1C; // i32, R
    public static final int REG_ENC0_VELOCITY = 0x20; // i32, R
    public static final int REG_ENC1_VELOCITY = 0x24; // i32, R
    public static final int REG_ENC2_VELOCITY = 0x28; // i32, R
    public static final int REG_ENC3_VELOCITY = 0x2C; // i32, R
    public static final int REG_S0_CLASS = 0x30; // u8, R
    public static final int REG_S1_CLASS = 0x31; // u8, R
    public static final int REG_S2_CLASS = 0x32; // u8, R
    public static final int REG_S3_CLASS = 0x33; // u8, R
    public static final int REG_SENSOR_PRESENT_MASK = 0x34; // u8, R
    public static final int REG_CLASS_CONFIDENCE = 0x35; // u8[4], R
    public static final int REG_CLASS_CONFIDENCE_COUNT = 4;
    public static final int REG_SENSOR_STATUS = 0x39; // u8[4], R
    public static final int REG_SENSOR_STATUS_COUNT = 4;
    public static final int REG_TELEMETRY_CRC16 = 0x3D; // u16, R
    public static final int REG_TELEMETRY_TIMESTAMP = 0x68; // u32, R
    public static final int REG_SENSOR_TYPE = 0x6C; // u8[4], R
    public static final int REG_SENSOR_TYPE_COUNT = 4;
    public static final int REG_CFG_SENSOR_SELECT = 0x70; // u8, RW
    public static final int REG_CFG_CLASS_SELECT = 0x71; // u8, RW
    public static final int REG_CFG_GAIN = 0x72; // u8, RW
    public static final int REG_CFG_INTEGRATION = 0x73; // u8, RW
    public static final int REG_CFG_LED_CURRENT = 0x74; // u8, RW
    public static final int REG_ENC_INVERT_MASK = 0x75; // u8, RW
    public static final int REG_VEL_INTERVAL_MS = 0x76; // u8, RW
    public static final int REG_IMU_AXIS_UP = 0x77; // u8, RW
    public static final int REG_IMU_YAW_OFFSET = 0x78; // i16, RW
    public static final int REG_IR_COEFF_R = 0x7A; // u8, RW
    public static final int REG_IR_COEFF_G = 0x7B; // u8, RW
    public static final int REG_IR_COEFF_B = 0x7C; // u8, RW
    public static final int REG_CFG_MIN_SUM = 0x7D; // u16, RW
    public static final int REG_CFG_MEAS_RATE = 0x7F; // u8, RW
    public static final int REG_CHANNEL_MODE = 0x9A; // u8, RW
    public static final int REG_PWM_WRAP_MASK = 0x9B; // u8, RW
    public static final int REG_ENC_PIN_STATE = 0x9C; // u8, R
    public static final int REG_CFG_GENERATION = 0x94; // u32, R
    public static final int REG_CFG_SCHEMA_VER = 0x98; // u8, R
    public static final int REG_CFG_ACTIVE_SLOT = 0x99; // u8, R
    public static final int REG_QUAT_W = 0xA0; // i16, R
    public static final int REG_QUAT_X = 0xA2; // i16, R
    public static final int REG_QUAT_Y = 0xA4; // i16, R
    public static final int REG_QUAT_Z = 0xA6; // i16, R
    public static final int REG_YAW = 0xA8; // i16, R
    public static final int REG_PITCH = 0xAA; // i16, R
    public static final int REG_ROLL = 0xAC; // i16, R
    public static final int REG_GYRO_X = 0xAE; // i16, R
    public static final int REG_GYRO_Y = 0xB0; // i16, R
    public static final int REG_GYRO_Z = 0xB2; // i16, R
    public static final int REG_ACCEL_X = 0xB4; // i16, R
    public static final int REG_ACCEL_Y = 0xB6; // i16, R
    public static final int REG_ACCEL_Z = 0xB8; // i16, R
    public static final int REG_IMU_CALIB = 0xBA; // u8, R
    public static final int REG_IMU_STATUS = 0xBB; // u8, R
    public static final int REG_IMU_TIMESTAMP = 0xBC; // u32, R
    public static final int REG_DOUT_SELECT = 0xC0; // u8, RW
    public static final int REG_DOUT_STATE = 0xC1; // u8, R
    public static final int REG_DOUT_LATCHED = 0xC2; // u8, R
    public static final int REG_LOC_STATUS = 0xE0; // u8, R
    public static final int REG_LOC_FLAGS = 0xE1; // u8, R
    public static final int REG_LOC_VEL_X = 0xE2; // i16, R
    public static final int REG_LOC_VEL_Y = 0xE4; // i16, R
    public static final int REG_LOC_VEL_H = 0xE6; // i16, R
    public static final int REG_LOC_X = 0xE8; // i16, R
    public static final int REG_LOC_Y = 0xEA; // i16, R
    public static final int REG_LOC_H = 0xEC; // i16, R
    public static final int REG_LOC_TIMESTAMP = 0xF0; // u32, R
    public static final int REG_LOC_CRC16 = 0xF4; // u16, R
    public static final int REG_RESET_CAUSE = 0xF6; // u8, R
    public static final int REG_RESETS_SINCE_POWER_ON = 0xF7; // u8, R
    public static final int REG_I2C_RECOVERY_COUNT = 0xF8; // u16, R
    public static final int REG_I2C_ARB_LOST_COUNT = 0xFA; // u16, R

    // Raw sensor channel block: base + stride*port + field offset
    public static final int RAW_BASE = 0x40;
    public static final int RAW_STRIDE = 10;
    public static final int RAW_COUNT = 4;
    public static final int RAW_RED = 0; // u16 (color overlay)
    public static final int RAW_GREEN = 2; // u16 (color overlay)
    public static final int RAW_BLUE = 4; // u16 (color overlay)
    public static final int RAW_IR = 6; // u16 (color overlay)
    public static final int RAW_PROXIMITY = 8; // u16 (color overlay)
    public static final int RAW_DISTANCE_MM = 0; // u16 (distance overlay)
    public static final int RAW_SIGNAL_RATE = 2; // u16 (distance overlay)
    public static final int RAW_AMBIENT_RATE = 4; // u16 (distance overlay)
    public static final int RAW_RANGE_STATUS = 6; // u8 (distance overlay)

    // Config data window (color / distance / localizer / pose overlays)
    public static final int CFGWIN_BASE = 0x80;
    public static final int CFGWIN_SIZE = 20;
    public static final int CFGWIN_ENABLED = 0x80; // u8
    public static final int CFGWIN_FLAGS = 0x81; // u8
    public static final int CFGWIN_R_MIN = 0x82; // u16
    public static final int CFGWIN_R_MAX = 0x84; // u16
    public static final int CFGWIN_G_MIN = 0x86; // u16
    public static final int CFGWIN_G_MAX = 0x88; // u16
    public static final int CFGWIN_B_MIN = 0x8A; // u16
    public static final int CFGWIN_B_MAX = 0x8C; // u16
    public static final int CFGWIN_PROX_MIN = 0x8E; // u16
    public static final int CFGWIN_PROX_MAX = 0x90; // u16
    public static final int CFGWIN_LABEL = 0x92; // u8
    public static final int CFGWIN_DIST_MIN = 0x82; // u16 (distance overlay)
    public static final int CFGWIN_DIST_MAX = 0x84; // u16 (distance overlay)
    public static final int CFGWIN_HYSTERESIS = 0x86; // u16 (distance overlay)
    public static final int CFGWIN_MIN_SIGNAL_RATE = 0x88; // u16 (distance overlay)
    public static final int LOCWIN_TPM_X = 0x80; // u32 (localizer overlay)
    public static final int LOCWIN_TPM_Y = 0x84; // u32 (localizer overlay)
    public static final int LOCWIN_TCP_X = 0x88; // i16 (localizer overlay)
    public static final int LOCWIN_TCP_Y = 0x8A; // i16 (localizer overlay)
    public static final int LOCWIN_IMU_SCALAR = 0x8C; // u16 (localizer overlay)
    public static final int LOCWIN_PORT_X = 0x8E; // u8 (localizer overlay)
    public static final int LOCWIN_PORT_Y = 0x8F; // u8 (localizer overlay)
    public static final int LOCWIN_VEL_INTERVAL_MS = 0x90; // u8 (localizer overlay)
    public static final int LOCWIN_POSE_X = 0x80; // i16 (pose overlay)
    public static final int LOCWIN_POSE_Y = 0x82; // i16 (pose overlay)
    public static final int LOCWIN_POSE_H = 0x84; // i16 (pose overlay)
    public static final int PWMWIN_PWM0_MIN_US = 0x80; // u16 (pwm overlay)
    public static final int PWMWIN_PWM0_MAX_US = 0x82; // u16 (pwm overlay)
    public static final int PWMWIN_PWM1_MIN_US = 0x84; // u16 (pwm overlay)
    public static final int PWMWIN_PWM1_MAX_US = 0x86; // u16 (pwm overlay)
    public static final int PWMWIN_PWM2_MIN_US = 0x88; // u16 (pwm overlay)
    public static final int PWMWIN_PWM2_MAX_US = 0x8A; // u16 (pwm overlay)
    public static final int PWMWIN_PWM3_MIN_US = 0x8C; // u16 (pwm overlay)
    public static final int PWMWIN_PWM3_MAX_US = 0x8E; // u16 (pwm overlay)

    // Digital output config window
    public static final int DOUTWIN_BASE = 0xD0;
    public static final int DOUTWIN_SIZE = 16;
    public static final int DOUTWIN_SRC_TYPE = 0xD0; // u8
    public static final int DOUTWIN_SRC_INDEX = 0xD1; // u8
    public static final int DOUTWIN_CLASS_INDEX = 0xD2; // u8
    public static final int DOUTWIN_MODE = 0xD3; // u8
    public static final int DOUTWIN_DEBOUNCE_ASSERT = 0xD4; // u8
    public static final int DOUTWIN_DEBOUNCE_RELEASE = 0xD5; // u8
    public static final int DOUTWIN_PULSE_MS = 0xD6; // u8
    public static final int DOUTWIN_THRESH_MIN = 0xD8; // i32
    public static final int DOUTWIN_THRESH_MAX = 0xDC; // i32

    // Commands (write opcode to REG_COMMAND; see driver sequence in the md)
    public static final int CMD_RESET_ENCODERS = 0x01; // sync, arg: channel mask
    public static final int CMD_CLEAR_FAULTS = 0x02; // sync
    public static final int CMD_CFG_LOAD = 0x10; // sync
    public static final int CMD_CFG_STORE = 0x11; // sync
    public static final int CMD_CFG_SAVE_FLASH = 0x12; // async
    public static final int CMD_CFG_LOAD_DEFAULTS = 0x13; // sync
    public static final int CMD_SENSOR_CAPTURE = 0x20; // async
    public static final int CMD_IMU_RESET_HEADING = 0x30; // sync
    public static final int CMD_CALIBRATE_GYRO = 0x31; // async
    public static final int CMD_LOC_RESET = 0x32; // async
    public static final int CMD_LOC_SET_POSE = 0x33; // sync
    public static final int CMD_LOC_PARAMS_LOAD = 0x34; // sync
    public static final int CMD_LOC_PARAMS_STORE = 0x35; // sync
    public static final int CMD_PWM_PARAMS_LOAD = 0x36; // sync
    public static final int CMD_PWM_PARAMS_STORE = 0x37; // sync
    public static final int CMD_DOUT_LOAD = 0x40; // sync
    public static final int CMD_DOUT_STORE = 0x41; // sync
    public static final int CMD_DOUT_CLEAR_LATCH = 0x42; // sync, arg: output mask
    public static final int CMD_REBOOT_BOOTLOADER = 0xF0; // noreturn, arg: 0x5A required
    public static final int CMD_CFG_SAVE_FLASH_MAX_MS = 500;
    public static final int CMD_SENSOR_CAPTURE_MAX_MS = 5000;
    public static final int CMD_CALIBRATE_GYRO_MAX_MS = 2000;
    public static final int CMD_LOC_RESET_MAX_MS = 3000;
    public static final int CMD_SYNC_MAX_US = 100; // all sync commands complete in under this

    // Command status (REG_COMMAND_STATUS)
    public static final int CMDSTAT_IDLE = 0;
    public static final int CMDSTAT_BUSY = 1;
    public static final int CMDSTAT_DONE = 2;
    public static final int CMDSTAT_ERROR = 3;

    // Error codes (REG_COMMAND_RESULT)
    public static final int OK = 0x00;
    public static final int ERR_UNKNOWN_COMMAND = 0x01;
    public static final int ERR_BAD_ARG = 0x02;
    public static final int ERR_NOT_SUPPORTED = 0x03;
    public static final int ERR_TOKEN_REUSE = 0x04;
    public static final int ERR_FLASH_WRITE = 0x10;
    public static final int ERR_FLASH_VERIFY = 0x11;
    public static final int ERR_FLASH_RATE_LIMITED = 0x12;
    public static final int ERR_SCHEMA_UNSUPPORTED = 0x13;
    public static final int ERR_SENSOR_ABSENT = 0x20;
    public static final int ERR_SENSOR_NAK = 0x21;
    public static final int ERR_SENSOR_TIMEOUT = 0x22;
    public static final int ERR_SENSOR_SATURATED = 0x23;
    public static final int ERR_INSUFFICIENT_SIGNAL = 0x24;
    public static final int ERR_CFG_INVALID_SLOT = 0x30;
    public static final int ERR_IMU_NOT_STATIONARY = 0x40;
    public static final int ERR_LOC_BAD_PARAMS = 0x41;

    // STATUS bits
    public static final int STATUS_READY = 1 << 0; // bit 0
    public static final int STATUS_CFG_DIRTY = 1 << 1; // bit 1
    public static final int STATUS_FLASH_ERROR = 1 << 2; // bit 2
    public static final int STATUS_SENSOR_BUS_ERROR = 1 << 3; // bit 3
    public static final int STATUS_IMU_PRESENT = 1 << 4; // bit 4
    public static final int STATUS_IMU_CALIBRATED = 1 << 5; // bit 5
    public static final int STATUS_CMD_OVERRUN = 1 << 6; // bit 6
    public static final int STATUS_CFG_DEFAULTED = 1 << 7; // bit 7

    // CAP bits
    public static final int CAP_ENCODERS = 1 << 0; // bit 0
    public static final int CAP_COLOR = 1 << 1; // bit 1
    public static final int CAP_IMU = 1 << 2; // bit 2
    public static final int CAP_DOUT = 1 << 3; // bit 3

    // SENSOR_STATUS bits
    public static final int SSTAT_VALID = 1 << 0; // bit 0
    public static final int SSTAT_STALE = 1 << 1; // bit 1
    public static final int SSTAT_QUALITY_FAIL = 1 << 2; // bit 2
    public static final int SSTAT_OVERFLOW = 1 << 3; // bit 3
    public static final int SSTAT_BUS_ERROR = 1 << 4; // bit 4
    public static final int SSTAT_INSUFFICIENT_SIGNAL = 1 << 5; // bit 5

    // IMU_STATUS bits
    public static final int ISTAT_PRESENT = 1 << 0; // bit 0
    public static final int ISTAT_FUSION_VALID = 1 << 1; // bit 1
    public static final int ISTAT_SATURATED = 1 << 2; // bit 2
    public static final int ISTAT_BIAS_VALID = 1 << 3; // bit 3
    public static final int ISTAT_STATIONARY = 1 << 4; // bit 4
    public static final int ISTAT_VIBRATION = 1 << 5; // bit 5
    public static final int ISTAT_FAULT = 1 << 6; // bit 6
    public static final int ISTAT_UNEXPECTED = 1 << 7; // bit 7

    // CLASS_FLAGS bits
    public static final int CLASSF_PROX_GATE = 1 << 0; // bit 0

    // DOUT_MODE_FLAGS bits
    public static final int DOUT_MODE_ACTIVE_LOW = 1 << 0; // bit 0

    // LOC_FLAGS bits
    public static final int LOCF_POSE_CLIPPED = 1 << 0; // bit 0
    public static final int LOCF_GYRO_SATURATED_EVER = 1 << 1; // bit 1
    public static final int LOCF_PORT_CONFLICT = 1 << 2; // bit 2

    // SENSOR_TYPE values
    public static final int STYPE_EMPTY = 0;
    public static final int STYPE_COLOR = 1;
    public static final int STYPE_DISTANCE = 2;
    public static final int STYPE_UNKNOWN = 0xFF;

    // RESET_CAUSE values
    public static final int RESET_UNKNOWN = 0;
    public static final int RESET_POWER_ON = 1;
    public static final int RESET_RUN_PIN = 2;
    public static final int RESET_WATCHDOG = 3;
    public static final int RESET_SOFTWARE = 4;
    public static final int RESET_DEBUG = 5;

    // HW_VARIANT values
    public static final int VARIANT_BASE = 1;
    public static final int VARIANT_ODOMETRY = 2;

    // GAIN values
    public static final int GAIN_1X = 0;
    public static final int GAIN_3X = 1;
    public static final int GAIN_6X = 2;
    public static final int GAIN_9X = 3;
    public static final int GAIN_18X = 4;

    // INTEGRATION values
    public static final int INTEG_400MS_20BIT = 0;
    public static final int INTEG_200MS_19BIT = 1;
    public static final int INTEG_100MS_18BIT = 2;
    public static final int INTEG_50MS_17BIT = 3;
    public static final int INTEG_25MS_16BIT = 4;
    public static final int INTEG_3MS125_13BIT = 5;

    // MEAS_RATE values
    public static final int MRATE_25MS = 0;
    public static final int MRATE_50MS = 1;
    public static final int MRATE_100MS = 2;
    public static final int MRATE_200MS = 3;
    public static final int MRATE_500MS = 4;
    public static final int MRATE_1000MS = 5;
    public static final int MRATE_2000MS = 6;

    // LED_CURRENT values
    public static final int LED_2MA5 = 0;
    public static final int LED_5MA = 1;
    public static final int LED_10MA = 2;
    public static final int LED_25MA = 3;
    public static final int LED_50MA = 4;
    public static final int LED_75MA = 5;
    public static final int LED_100MA = 6;
    public static final int LED_125MA = 7;

    // IMU_AXIS_UP values
    public static final int AXIS_POS_X = 0;
    public static final int AXIS_NEG_X = 1;
    public static final int AXIS_POS_Y = 2;
    public static final int AXIS_NEG_Y = 3;
    public static final int AXIS_POS_Z = 4;
    public static final int AXIS_NEG_Z = 5;

    // IMU_CALIB values
    public static final int CALIB_NONE = 0;
    public static final int CALIB_COARSE = 1;
    public static final int CALIB_GOOD = 2;
    public static final int CALIB_EXCELLENT = 3;

    // DOUT_SRC values
    public static final int DOUT_SRC_DISABLED = 0;
    public static final int DOUT_SRC_SENSOR_CLASS = 1;
    public static final int DOUT_SRC_ENCODER = 2;
    public static final int DOUT_SRC_IMU_HEADING = 3;

    // DOUT_MODE values
    public static final int DOUT_MODE_LEVEL = 0;
    public static final int DOUT_MODE_LATCHED = 1;
    public static final int DOUT_MODE_PULSE = 2;

    // LOC_STATUS values
    public static final int LOC_NOT_READY = 0;
    public static final int LOC_WARMING_UP_IMU = 1;
    public static final int LOC_CALIBRATING_IMU = 2;
    public static final int LOC_RUNNING = 3;
    public static final int LOC_FAULT_NO_IMU = 4;

    // Snapshot regions (latched on pointer entry; see md)
    public static final int SNAP_TELEMETRY_START = 0x10;
    public static final int SNAP_TELEMETRY_END = 0x6F; // inclusive
    public static final int SNAP_PERSISTENCE_START = 0x94;
    public static final int SNAP_PERSISTENCE_END = 0x99; // inclusive
    public static final int SNAP_IMU_START = 0xA0;
    public static final int SNAP_IMU_END = 0xBF; // inclusive
    public static final int SNAP_LOCALIZER_START = 0xE0;
    public static final int SNAP_LOCALIZER_END = 0xF5; // inclusive
    public static final int SNAP_DIAGNOSTICS_START = 0xF6;
    public static final int SNAP_DIAGNOSTICS_END = 0xFB; // inclusive

    // Flash persistence
    public static final int FLASH_MAGIC = 0x43524242; // "BBRC" little-endian
    public static final int FLASH_SCHEMA_VER = 3;
    public static final int FLASH_HEADER_SIZE = 20;
    public static final int FLASH_PAYLOAD_SIZE = 706; // at the current schema
    public static final int FLASH_PAYLOAD_SIZE_V1 = 671; // at schema 1
    public static final int FLASH_PAYLOAD_SIZE_V2 = 688; // at schema 2
    public static final int FLASH_SECTOR_SIZE = 4096;
    public static final int FLASH_SLOT_COUNT = 2;
    public static final int FLASH_SAVE_RATE_LIMIT_MS = 1000;

    // Constants and scale factors
    public static final int NUM_SENSOR_PORTS = 4;
    public static final int NUM_ENCODERS = 4;
    public static final int NUM_DOUTS = 4;
    public static final int NUM_CLASS_SLOTS = 7;
    public static final int CLASS_SLOT_FIRST = 1;
    public static final int CLASS_SLOT_LAST = 7;
    public static final int CLASS_NO_MATCH = 0;
    public static final int SENSOR_SELECT_ALL = 0xFF;
    public static final int ACTIVE_SLOT_DEFAULTS = 0xFF;
    public static final int TOKEN_NONE = 0x00;
    public static final int CHROMA_SCALE = 1000;
    public static final int PROX_MAX_VALUE = 2047;
    public static final int DISTANCE_INVALID = 0xFFFF;
    public static final int QUAT_SCALE = 16384;
    public static final int ANGLE_SCALE_CDEG = 100;
    public static final int GYRO_SCALE_LSB_PER_DPS = 16;
    public static final int ACCEL_SCALE_LSB_PER_MPS2 = 100;
    public static final int YAW_MIN_CDEG = -18000;
    public static final int YAW_MAX_CDEG = 17999;
    public static final int PITCH_MIN_CDEG = -9000;
    public static final int PITCH_MAX_CDEG = 9000;
    public static final int REBOOT_BOOTLOADER_ARG = 0x5A;
    public static final int LOC_HEADING_SCALE = 5000;
    public static final int LOC_HEADING_VEL_SCALE = 600;
    public static final int LOC_IMU_SCALAR_SCALE = 10000;
    public static final int LOC_IMU_SCALAR_MIN = 8000;
    public static final int LOC_IMU_SCALAR_MAX = 12000;
    public static final int LOC_TPM_MIN = 1000;
    public static final int LOC_TPM_MAX = 10000000;
    public static final int LOC_PORT_UNASSIGNED = 0xFF;
    public static final int PWM_WIDTH_MAX_US = 65535;
    public static final int PWM_NO_SIGNAL_TIMEOUT_MS = 100;

    // Power-on defaults
    public static final int DEFAULT_VEL_INTERVAL_MS = 25;
    public static final int DEFAULT_CFG_MIN_SUM = 300;
    public static final int DEFAULT_CFG_INTEGRATION = 2;
    public static final int DEFAULT_DEBOUNCE_ASSERT = 2;
    public static final int DEFAULT_DEBOUNCE_RELEASE = 2;
    public static final int DEFAULT_IR_COEFF = 0;
    public static final int DEFAULT_CAPTURE_CHROMA_TOLERANCE = 50;
    public static final int DEFAULT_LOC_VEL_INTERVAL_MS = 25;
    public static final int DEFAULT_LOC_IMU_SCALAR = 10000;
}
