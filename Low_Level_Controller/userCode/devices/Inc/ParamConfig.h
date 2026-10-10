#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

// 系统
#define CONTROL_FREQUENCY 100       // 更新频率
#define MAX_SERIAL_LENGTH 128
#define SENSOR_NUM 4
#define SERVO_NUM 7
#define PROPELLER_NUM 8
#define LED_PERIOD 500
#define ROLL_SEN_DIST 14.0          // 计算roll方向误差角的水压计间距，用于从深度误差计算角度误差，单位cm
#define PITCH_SEN_DIST 33.65        // 计算pitch方向误差角的水压计间距，用于从深度误差计算角度误差，单位cm
#define I2C_TO 10                   // I2C超时
#define I2C_STATE_WAIT_TIMEOUT_MS 2
#define PRESSURE_SENSOR_FAIL_LIMIT 3
#define PRESSURE_SENSOR_RECOVER_INTERVAL_MS 500
#define PRESSURE_SENSOR_RECOVER_SUCCESS_LIMIT 3
#define UART6_HEAD_0 0xAA           
#define UART6_HEAD_1 0x55
#define UART6_MIN_FRAME_SIZE 7
#define UART6_PARSE_BUF_SIZE 256
#define TELEMETRY_ADDR 0xF0
#define TELEMETRY_CMD_STATE 0x01
#define TELEMETRY_PWM_COUNT 12
#define TELEMETRY_PWM_PACKED_SIZE 15
#define TELEMETRY_PAYLOAD_SIZE 33

/*
 * 设备类型枚举
 */
typedef enum {
    IM, // imu
    PS, // pressure sensor
    PR, // propeller
    SE, // servo
    LE  // led
} DEVICE_TYPE_E;

typedef enum{
    V30,
    V31,
    V32,
    V33,
    V40
} VERSION_E;

#define PI 3.14159265358979323846

// IMU

// 推进�?

#endif
