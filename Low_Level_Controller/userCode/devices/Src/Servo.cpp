//
// Created by admin on 2023/12/5.
//
#include <cstring>
#include "Servo.h"

static constexpr uint8_t CMD_SET_SERVO_PWM = 0x03;
static constexpr uint8_t ARM_FINGER_INDEX = 4;
// Finger servo safety limits. Tune these after checking the real mechanism
// travel; current_pwm[6] is clamped here before writing to the PWM expander.
static constexpr int32_t ARM_FINGER_MIN_PWM = 500;
static constexpr int32_t ARM_FINGER_MAX_PWM = 2500;
extern volatile uint16_t usart6_frame_size;

static int32_t ClampServoPWM(uint8_t index, int32_t pwm)
{
    if (index == ARM_FINGER_INDEX)
    {
        return clamp(pwm, ARM_FINGER_MIN_PWM, ARM_FINGER_MAX_PWM);
    }
    return clamp(pwm, (int32_t)500, (int32_t)2500);
}

int32_t ID_V30[SERVO_NUM] = {8, 9, 10, 11, 12, 13, 14};
int32_t ID_V31[SERVO_NUM] = {8, 9, 10, 11, 12, 13, 14};
int32_t ID_V32[SERVO_NUM] = {8, 9, 11, 10, 12, 13, 14};
int32_t ID_V33[SERVO_NUM] = {8, 9, 10, 11, 12, 13, 14};
int32_t ID_V40[SERVO_NUM] = {8, 9, 10, 11, 12, 13, 14};

void Servo::Init()
{
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        current_pwm[i] = 1500;
    }
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, current_pwm[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, current_pwm[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, current_pwm[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, current_pwm[1]);
}

void Servo_I2C::Init()
{
    std::memcpy(ID, ID_V40, sizeof(ID));
    // TCA_SetChannel(4);
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        current_pwm[i] = 1550;
        PCA_Setpwm(i + 8, 0, floor(current_pwm[i] * 4096 / 20000 + 0.5f));
    }
		current_pwm[1] = 1400;
		current_pwm[2] = 1510;
		current_pwm[3] = 1500; //关节舵机
		current_pwm[4] = 1500; //手指舵机
		//current_pwm[5] = 1500;
		//current_pwm[6] = 1500;	
    PCA_Setpwm(1 + 8, 0, floor(current_pwm[1] * 4096 / 20000 + 0.5f));
}

void Servo::Receive()
{
    data_extract(RxBuffer6, current_pwm, SERVO_NUM);
}

void Servo_I2C::Receive()
{
    uint8_t code = RxBuffer6[1];
    if (code == CMD_SET_SERVO_PWM)
    {
        CommandSetServoPWM();
        return;
    }

    data_extract(RxBuffer6, current_pwm, SERVO_NUM);
}

void Servo::Handle()
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, current_pwm[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, current_pwm[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, current_pwm[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, current_pwm[1]);
}

void Servo_I2C::Handle()
{
    // TCA_SetChannel(4);
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        current_pwm[i] = ClampServoPWM(i, current_pwm[i]);
    }
    // for (int i = 0; i < SERVO_NUM; ++i)
    // {
    //     PCA_Setpwm(ID[i], 0, floor(current_pwm[i] * 4096 / 20000 + 0.5f));
    // }
    // i2c_state = I2C_TaskState::PROPELLER;

    if (!I2C2_WaitState(I2C_TaskState::SERVO, I2C_STATE_WAIT_TIMEOUT_MS)){
        I2C2_Recover();
        return;
    }
    SER_I2C_IT_Handle();
}

void Servo::data_extract(uint8_t *rx, int32_t *current_pwm, int32_t num)
{
    if (strncmp((char *)rx, "MOT:", 4) == 0)
    {

        char *data_str = (char *)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num)
        {
            current_pwm[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}

void Servo_I2C::data_extract(uint8_t *rx, int32_t *current_pwm, int32_t num)
{
    // 示例：假设 PWM 命令格式是 "MOT:1000,2000,1500,1800,1600,1400\n"
    if (strncmp((char *)rx, "MOT:", 4) == 0)
    {
        char *data_str = (char *)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num)
        {
            current_pwm[i] = ClampServoPWM(i, atoi(token));
            token = strtok(NULL, ",");
            i++;
        }
    }
}

void Servo_I2C::CommandSetServoPWM()
{
    if (usart6_frame_size < 5)
    {
        return;
    }

    uint8_t index = RxBuffer6[2];
    if (index >= SERVO_NUM)
    {
        return;
    }

    int32_t pwm = static_cast<int16_t>(RxBuffer6[3] | (RxBuffer6[4] << 8));
    current_pwm[index] = ClampServoPWM(index, pwm);
}

void Servo_I2C::SER_I2C_IT_Handle(){
    i2c_it_tx[0] = LED0_ON_L + 4*8;
    for (int i = 0; i < SERVO_NUM; i++){
        int16_t pwm = static_cast<int16_t>(ClampServoPWM(i, current_pwm[i]));
        int16_t duty_count = (pwm * 4096 + 10000) / 20000;
        i2c_it_tx[4*i+1] = 0x00;
        i2c_it_tx[4*i+2] = 0x00;
        i2c_it_tx[4*i+3] = duty_count & 0xFF;
        i2c_it_tx[4*i+4] = duty_count >> 8;
    }
    HAL_I2C_Master_Transmit_IT(&hi2c2, PCA9685_ADDR, i2c_it_tx, 1+4*SERVO_NUM);
}
