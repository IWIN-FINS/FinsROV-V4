//
// Created by admin on 2023/10/30.
//
#include "Propeller.h"
#include "Sensor.h"
#include "IMU.h"
#include <algorithm>
#include "math.h"

std::array<int8_t, 8> Sign = {1, -1, 1, 1, -1, -1, 1, -1};
std::array<uint8_t, 4> InID = {1, 2, 6, 5};
std::array<uint8_t, 4> OutID = {0, 3, 7, 4};
int16_t InitPWM = 1610;
std::array<int16_t, 8> Compensation = {50, 50, 50, 50, 50, 50, 50, 50};
std::array<int16_t, 4> FloatPWM = {InitPWM, InitPWM, InitPWM, InitPWM};

PID_Regulator_t DepthPID(4, 0.001, 60, 15, 7.5, 7.5, 30);
PID_Regulator_t RollInPID(0.1, 0, 40, 5, 2.5, 2.5, 10);
PID_Regulator_t PitchInPID(0.4, 0, 20, 5, 2.5, 2.5, 10);
PID_Regulator_t YawInPID(0.4, 0, 30, 5, 2.5, 2.5, 10);
PID_Regulator_t RollOutPID(6, 0.04, 0, 5, 2.5, 2.5, 10);
PID_Regulator_t PitchOutPID(6, 0.04, 0, 5, 2.5, 2.5, 10);
PID_Regulator_t YawOutPID(6, 0.04, 0, 5, 2.5, 2.5, 20);

float rotate_speed = 0.0;
float depth_speed = 0.0;
float yaw_thrust_offset = 4.5;

float J[3] = {0.2f, 0.3f, 0.3f};          // roll, pitch, yaw
float B[3] = {0.02f, 0.03f, 0.03f};         // roll, pitch, yaw
float D[3] = {0.001f, 0.002f, 0.002f};      // roll, pitch, yaw
float omega_o[3] = {30.0f, 30.0f, 40.0f}; // roll, pitch, yaw

static CommandHandler propeller_command[256] = {nullptr};

void Propeller_I2C::Init()
{
    propeller_command[0x00] = &Propeller_I2C::CommandSetDepthValue;
    propeller_command[0x01] = &Propeller_I2C::CommandSetDepthIncrement;
    propeller_command[0x02] = &Propeller_I2C::CommandSetDepthForce;
    propeller_command[0x10] = &Propeller_I2C::CommandSetFloatCloseloop;
    propeller_command[0x20] = &Propeller_I2C::CommandSetYawCloseloop;
    propeller_command[0x21] = &Propeller_I2C::CommandSetYawValue;
    propeller_command[0x22] = &Propeller_I2C::CommandSetYawIncrement;
    propeller_command[0x23] = &Propeller_I2C::CommandSetYawRate;
    propeller_command[0x30] = &Propeller_I2C::CommandSetPlanarVelocity;
    propeller_command[0x31] = &Propeller_I2C::CommandSetCombinedMotion;

    flag_on = false;
    flag_PWM_output = false;

    for (int i=0; i<3; i++){
        ff[i].init(J[i], B[i], D[i]);
        eso[i].init(J[i], B[i], D[i], omega_o[i]);
    }

    DepthPID_.PIDInfo = DepthPID;
    RollInPID_.PIDInfo = RollInPID;
    PitchInPID_.PIDInfo = PitchInPID;
    YawInPID_.PIDInfo = YawInPID;
    RollOutPID_.PIDInfo = RollOutPID;
    PitchOutPID_.PIDInfo = PitchOutPID;
    YawOutPID_.PIDInfo = YawOutPID;

    TCA_SetChannel(4);
    HAL_Delay(5);

    // 瀵偓閸氼垵绻涚紒顓☆嚢閸?    // uint8_t mode = PCA_Read(PCA9685_MODE1);
    // mode = (mode & 0x7F) | 0x20;
    uint8_t mode = 0x00;
    PCA_Write(PCA9685_MODE1, mode);

    PCA_Setfreq(50); // Hz
    // for (int i = 0; i < PROPELLER_NUM; ++i)
    // {
    //     current_pwm[i] = InitPWM;
    //     PCA_Setpwm(i, 0, floor(current_pwm[i] * 4096 / 20000 + 0.5f));
    // }

    a_coef = expf(-1.0f / (thr_tau * CONTROL_FREQUENCY));
    b_coef = thr_K * (1.0f - a_coef);

    for (int i = 0; i < PROPELLER_NUM; ++i) {
        T_hat[i] = 0.0f;
    }

    i2c_it_data = 0x01 << 4;
    pro_i2c_state = PRO_I2C_TaskState::TCA_SET;
};

// 閺嶈宓佹稉鎻掑經閹稿洣鎶ょ拋鍓х枂PWM閸欏倹鏆?
void Propeller_I2C::Receive()
{
    uint8_t code = RxBuffer6[1];
    if (propeller_command[code] != nullptr) 
    {
        (this->*propeller_command[code])();
    }
}

void Propeller_I2C::Handle()
{
    if (!I2C2_WaitState(I2C_TaskState::PROPELLER, I2C_STATE_WAIT_TIMEOUT_MS)){
        I2C2_Recover();
        return;
    }
    if (pro_i2c_state == PRO_I2C_TaskState::TCA_SET)
        PRO_I2C_IT_Handle();
    for (int i=0; i<PROPELLER_NUM; i++){
        current_pwm[i] = InitPWM;
    }
    if (flag_on){
        ComputeThrustTorque();
        AllocateThrust();
        Thrust2PWM();
    }
    uint32_t wait_start = HAL_GetTick();
    while(pro_i2c_state != PRO_I2C_TaskState::SET_PWM){
        if ((HAL_GetTick() - wait_start) >= I2C_STATE_WAIT_TIMEOUT_MS){
            I2C2_Recover();
            return;
        }
        __NOP();
    }
    PRO_I2C_IT_Handle();
}

void Propeller_I2C::ComputeThrustTorque(){
    target.depth += depth_speed / CONTROL_FREQUENCY;
    target.ang[2] += rotate_speed / CONTROL_FREQUENCY;
    target.ang[2] = NormalizeAngle(target.ang[2]);
    if (pressure_sensor.IsDepthValid()){
        target.thrust[2] = DepthPID_.PIDCalc(target.depth, pressure_sensor.data_depth) + yaw_thrust_offset;
    }
    else{
        target.thrust[2] = 0.0f;
        target.ang_vel[0] = 0.0f;
        target.ang_vel[1] = 0.0f;
    }
    
    if (pressure_sensor.ps_state == PS_HandleState::READ_PRES_THEN_REQU_TEMP)
    {
        yaw_diff = NormalizeAngle(imu.attitude.ang[2] - target.ang[2]);
        if (pressure_sensor.IsDepthValid()){
            pitch_diff = NormalizeAngle(pressure_sensor.data_pitch / PITCH_SEN_DIST - target.ang[1]);
            roll_diff = NormalizeAngle(pressure_sensor.data_roll / ROLL_SEN_DIST - target.ang[0]);
            target.ang_vel[0] = RollOutPID_.PIDCalc(target.ang[0], pressure_sensor.data_roll / ROLL_SEN_DIST);
            target.ang_vel[1] = PitchOutPID_.PIDCalc(target.ang[1], pressure_sensor.data_pitch / PITCH_SEN_DIST);
        }
        target.ang_vel[2] = YawOutPID_.PIDCalc(0.0f, yaw_diff);
    }

    torque_comp.PID[0] = RollInPID_.PIDCalc(target.ang_vel[0], imu.attitude.ang_vel[0]);
    torque_comp.PID[1] = PitchInPID_.PIDCalc(target.ang_vel[1], imu.attitude.ang_vel[1]);
    torque_comp.PID[2] = YawInPID_.PIDCalc(target.ang_vel[2], imu.attitude.ang_vel[2]);
    
    for (int i=0; i<3; i++){
        torque_comp.FF[i] = ff[i].computeTorque(target.ang_vel[i]);
        // ESO
        eso[i].update(imu.attitude.ang_vel[i], torque_act[i]);
        torque_comp.ESO[i] = eso[i].getDisturbTorque();
        // 
        target.torque[i] = torque_comp.PID[i] + torque_comp.FF[i] - 0.5 * torque_comp.ESO[i];
			  //target.torque[i] = torque_comp.PID[i] + torque_comp.FF[i];
        // if (i == 0) {
        //     target.torque[i] += 0.025 * target.thrust[1];
        // }
        // else if (i == 1) {
        //     target.torque[i] += 0.035 * target.thrust[0] + 0.02 * target.thrust[2];
        // }
    }
}

void Propeller_I2C::AllocateThrust()
{
    torque_act[0] = torque_act[1] = torque_act[2] = 0.0f;
    for (int i=0; i<PROPELLER_NUM; i++){
        float thrust = thrust_alloc_factor[i][0] * target.thrust[0];
        thrust += thrust_alloc_factor[i][1] * target.thrust[1];
        thrust += thrust_alloc_factor[i][2] * target.thrust[2];
        thrust += thrust_alloc_factor[i][3] * target.torque[0];
        thrust += thrust_alloc_factor[i][4] * target.torque[1];
        thrust += thrust_alloc_factor[i][5] * target.torque[2];

        // 娑撯偓闂冭埖绮搁崥搴に夐崑?        // float u_prev = target.thrust_single[i];
        // float T_pred = a_coef * T_hat[i] + b_coef * u_prev;
        // float e = thrust - T_pred;
        // float thrust_cmd = u_prev + imc_lambda * e / b_coef;

        // 闂勬劕绠?        // thrust_cmd = clamp(thrust_cmd, u_prev-MAX_DELTA_THRUST, u_prev+MAX_DELTA_THRUST);
        float thrust_cmd = clamp(thrust, target.thrust_single[i]-MAX_DELTA_THRUST, target.thrust_single[i]+MAX_DELTA_THRUST);

        target.thrust_single[i] = thrust_cmd;
        // T_hat[i] = a_coef * T_hat[i] + b_coef * thrust_cmd;

        // 鐠侊紕鐣荤€圭偤妾崝娑氱叐
        torque_act[0] += thrust_calc_factor[3][i] * thrust_cmd;
        torque_act[1] += thrust_calc_factor[4][i] * thrust_cmd;
        torque_act[2] += thrust_calc_factor[5][i] * thrust_cmd;
    }
}

void Propeller_I2C::Thrust2PWM(){
    for (int i=0; i<PROPELLER_NUM; i++){
        int16_t delta_pwm = (int16_t)(target.thrust_single[i] * FORCE2PWM_FACTOR * (-Sign[i]));
        if (delta_pwm > 0) current_pwm[i] = InitPWM + delta_pwm + Compensation[i];
        else if (delta_pwm < 0) current_pwm[i] = InitPWM + delta_pwm - Compensation[i];
    }
}

void Propeller_I2C::SetPWM(){
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        int16_t pwm = current_pwm[i];
        pwm = clamp(pwm, (int16_t)1000, (int16_t)2000);
        int16_t duty_count = (pwm * 4096 + 10000) / 20000;
        PCA_Setpwm(i, 0, duty_count);
    }
}

void Propeller_I2C::PRO_I2C_IT_Handle(){
    switch (pro_i2c_state)
    {
    case PRO_I2C_TaskState::TCA_SET:
        HAL_I2C_Master_Transmit_IT(&hi2c2, (TCA9548A_ADDR << 1), &i2c_it_data, 1);
        break;
    
    case PRO_I2C_TaskState::SET_PWM:
        i2c_it_tx[0] = LED0_ON_L;
        for (int i = 0; i < PROPELLER_NUM; i++){
            int16_t pwm = current_pwm[i];
            pwm = clamp(pwm, (int16_t)1100, (int16_t)1900);
            int16_t duty_count = (pwm * 4096 + 10000) / 20000;
            i2c_it_tx[4*i+1] = 0x00;
            i2c_it_tx[4*i+2] = 0x00;
            i2c_it_tx[4*i+3] = duty_count & 0xFF;
            i2c_it_tx[4*i+4] = duty_count >> 8;
        }
        HAL_I2C_Master_Transmit_IT(&hi2c2, PCA9685_ADDR, i2c_it_tx, 1+4*PROPELLER_NUM);
        break;
				
    default:
        break;
    }
}

float Propeller_I2C::deg2rad(float degree){
    return degree * PI / 180;
}

float Propeller_I2C::rad2deg(float rad){
    return rad * 180 / PI;
}

float Propeller_I2C::NormalizeAngle(float angle)
{
    angle = std::fmod(angle + PI, 2 * PI);
    if (angle < 0) angle += 2 * PI;
    return angle - PI;
}

void Propeller_I2C::CommandSetFloatCloseloop(){
  	bool enable = (RxBuffer6[2] != 0);
		
		if (enable && !flag_on)
    {
        flag_on = true;

        target.depth = (pressure_sensor.data_depth - 1.0f > 30.0f)
                       ? (pressure_sensor.data_depth - 1.0f)
                       : 20.0f;

        target.ang[2] = imu.attitude.ang[2];

        depth_speed = 0;
        rotate_speed = 0;
        flag_depth = false;
        flag_rotate = false;
    }
    else if (!enable && flag_on)
    {
        flag_on = false;

        target.depth = 30.0f;
        depth_speed = 0;
        rotate_speed = 0;
        flag_depth = false;
        flag_rotate = false;

        target.thrust[0] = 0;
        target.thrust[1] = 0;
        target.thrust[2] = 0;
    }
}

void Propeller_I2C::CommandSetDepthValue(){
    if (!flag_on) return;
    target.depth = (int16_t)(RxBuffer6[2] | (RxBuffer6[3] << 8)) / 10.0f;
}

void Propeller_I2C::CommandSetDepthIncrement(){
    if (!flag_on) return;
    target.depth += (int16_t)(RxBuffer6[2] | (RxBuffer6[3] << 8)) / 10.0f;
}

void Propeller_I2C::CommandSetDepthForce(){
    if (!flag_on) return;
    int16_t data = (int16_t)(RxBuffer6[2] | (RxBuffer6[3] << 8));
    data = clamp(data, (int16_t)-99, (int16_t)99);
    if (data != 0){
        depth_speed = MAX_DEPTH_SPEED * data / 100.0f; // 閺堢喐婀滃ǎ鍗炲閺傜懓鎮?        flag_depth = true;
    }
    else{
        flag_depth = false;
        depth_speed = 0;
    }
}

void Propeller_I2C::CommandSetPlanarVelocity(){
    if (!flag_on) return;
    int8_t data_x = (int8_t)RxBuffer6[2];
    int8_t data_y = (int8_t)RxBuffer6[3];
    data_x = clamp(data_x, (int8_t)-99, (int8_t)99);
    data_y = clamp(data_y, (int8_t)-99, (int8_t)99);
    target.thrust[1] = (int16_t)std::round((int)MAX_LATERAL_FORCE * (data_x / 100.0));
    target.thrust[0] = (int16_t)std::round((int)MAX_LONGITUDINAL_FORCE * (data_y / 100.0));
}

void Propeller_I2C::CommandSetCombinedMotion(){
    if (!flag_on) return;

    int8_t data_x = (int8_t)RxBuffer6[2];
    int8_t data_y = (int8_t)RxBuffer6[3];
    int16_t yaw_data = (int16_t)(RxBuffer6[4] | (RxBuffer6[5] << 8));
    int16_t depth_data = (int16_t)(RxBuffer6[6] | (RxBuffer6[7] << 8));

    data_x = clamp(data_x, (int8_t)-99, (int8_t)99);
    data_y = clamp(data_y, (int8_t)-99, (int8_t)99);
    yaw_data = clamp(yaw_data, (int16_t)-99, (int16_t)99);
    depth_data = clamp(depth_data, (int16_t)-99, (int16_t)99);

    target.thrust[1] = (int16_t)std::round((int)MAX_LATERAL_FORCE * (data_x / 100.0));
    target.thrust[0] = (int16_t)std::round((int)MAX_LONGITUDINAL_FORCE * (data_y / 100.0));

    if (yaw_data != 0){
        rotate_speed = MAX_ROTATE_SPEED * yaw_data / 100.0f;
        flag_rotate = true;
    }
    else{
        flag_rotate = false;
        rotate_speed = 0;
    }

    if (depth_data != 0){
        depth_speed = MAX_DEPTH_SPEED * depth_data / 100.0f;
        flag_depth = true;
    }
    else{
        flag_depth = false;
        depth_speed = 0;
    }
}

void Propeller_I2C::CommandSetYawValue(){
    if (!flag_on) return;
    int16_t deg = (int16_t)(RxBuffer6[2] | (RxBuffer6[3] << 8));
    target.ang[2] = deg2rad((float)deg);
}

void Propeller_I2C::CommandSetHorizontalStop(){
    if (!flag_on) return;
    target.thrust[0] = target.thrust[1] = 0;
}

void Propeller_I2C::CommandSetYawIncrement(){
    if (!flag_on) return;
    int16_t deg = (int16_t)(RxBuffer6[2] | (RxBuffer6[3] << 8));
    target.ang[2] += deg2rad((float)deg);
}

void Propeller_I2C::CommandSetYawRate(){
    if (!flag_on) return;
    int16_t data = (int16_t)(RxBuffer6[2] | (RxBuffer6[3] << 8));
    data = clamp(data, (int16_t)-99, (int16_t)99);
    if (data != 0){
        rotate_speed = MAX_ROTATE_SPEED * data / 100.0f; // 閺堢喐婀滅憴鎺椻偓鐔峰(瀵冨/缁?
        flag_rotate = true;
    }
    else{
        flag_rotate = false;
        rotate_speed = 0;
    }
}

void Propeller_I2C::CommandSetYawCloseloop(){
}

void Propeller_I2C::CommandTest(){
    if (!flag_on)
    {
        uint8_t* data = reinterpret_cast<uint8_t*>(RxBuffer6) + 2;
        for (int i = 0; i < 8; ++i) {
            int16_t test_pwm = (int16_t)(data[0] |(data[1] << 8));
            current_pwm[i] = (int16_t)(test_pwm);
            data += 2;
        }
    }
}
