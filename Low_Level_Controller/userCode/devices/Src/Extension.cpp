//
// Created by admin on 2023/10/29.
//
#include "Extension.h"
#include "Sensor.h"
#include "Propeller.h"
//婵縿鍊栭弸鍐╃閸撲焦鏆忛柡澶堝劚椤︹晠鎮堕崬?c闁圭鏅涢惈宥夊级閸栵紕绀凾CA闁挎稑顦绌歸m闁圭鏅涢惈宥夊级閸栵紕绀凱CA�?//i2c闁圭鏅涢惈宥夊级閹稿骸澶嶉柛锔光偓鎾诲级缁?c闁告瑱缍囩槐娆愮▔婵犲啯鐓欑紒妤婂厸缁ㄢ晠骞?闁哄秴婀遍崵搴ㄦ晬鐏炴垝�?SDA-SCL-VCC-GND-闁告瑧顒茬槐?//pwm闁圭鏅涢惈宥夊级閹稿骸澶嶉柛锔规殣2c闁圭鏅涢惈宥夊级鐠恒劍�?闁告瑥鍢茶ぐ?
I2C_TaskState i2c_state = I2C_TaskState::PressureSensor;

// 鐎殿喒鍋撻梺顐ｆ皑椤?channel 閻犱警鍨冲▓�?C
bool TCA_SetChannel(uint8_t channel)
{
    uint8_t data;
    data = 0x01 << channel;
    if (HAL_I2C_Master_Transmit(&hi2c2, (TCA9548A_ADDR << 1), &data, 1, I2C_TO) != HAL_OK)
    {
        return false;
    }
    return true;
}

bool I2C2_WaitState(I2C_TaskState target, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (i2c_state != target)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return false;
        }
        __NOP();
    }
    return true;
}

void I2C2_Recover()
{
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);

    pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TCA_SET_0;
    pressure_sensor.ps_state = PS_HandleState::READ_TEMP_THEN_REQU_PRES;
    propeller_i2c.pro_i2c_state = PRO_I2C_TaskState::TCA_SET;
    i2c_state = I2C_TaskState::PressureSensor;
}

uint8_t PCA_Read(uint8_t startAddress)
{
    // Read one PCA9685 register.
    uint8_t tx[1];
    uint8_t buffer[1];
    tx[0] = startAddress;
    if (HAL_I2C_Master_Transmit(&hi2c2,PCA9685_ADDR,tx,1,I2C_TO) != HAL_OK)
    {
        return 0;
    }
    if (HAL_I2C_Master_Receive(&hi2c2,PCA9685_ADDR,buffer,1,I2C_TO) != HAL_OK)
    {
        return 0;
    }
    return buffer[0];
}

void PCA_Write(uint8_t startAddress, uint8_t buffer)
{
    //閻犱礁澧介悿鍡楊嚕閳ь剚鎱ㄧ€ｎ亜鏅搁柡浣哄瀹撲線鎯冮崟顐ゆ閻庢稒锚濞呮帡宕烽弶鎸庣祷
    uint8_t tx[2];
    tx[0] = startAddress;
    tx[1] = buffer;
    HAL_I2C_Master_Transmit(&hi2c2,PCA9685_ADDR, tx,2,I2C_TO);
}

// 閻犱礁澧介悿鍝朩M濡増鍨瑰鍏肩�?freq
void PCA_Setfreq(float freq)
{
    uint8_t prescale,oldmode,newmode;
    double prescaleval;
    freq *= 1.016; 			//閻庡湱鍋ゅ顖涙媴鐠恒劍鏆忛弶鈺佹川閳诲吋绋夐鐐垫憼闁革负鍔屾禍绋款啅椤曗偓濞撳墎鎲版导杈殘閻擃偄顫㈤敐鍥厙闁?0.83
    // prescaleval = 25000000;
    prescaleval = 25000000.0/(4096.0*freq);
    //prescaleval /= freq;
    prescaleval -= 1;
    prescale = floor(prescaleval + 0.5f);
    oldmode = PCA_Read(PCA9685_MODE1);
    newmode = (oldmode&0x7F) | 0x10;
    PCA_Write(PCA9685_MODE1, newmode);
    PCA_Write(PCA9685_PRESCALE, prescale);
    PCA_Write(PCA9685_MODE1, oldmode);
    HAL_Delay(2);
    PCA_Write(PCA9685_MODE1, oldmode | 0xA1);
}

//  @brief 閻犱礁澧介悿鍡欌偓鐢垫嚀缁ㄥ弶娼忛幘鍐叉瘔鐎殿喗娲濋崜濂告儍閸戠洭M�?//  @param num PWM閺夊牊鎸搁崵顓烆嚕閺団€冲0 ~ 15, 0 ~ 7 閻庣數鎳撶花鏌ュ箳閵娿劎绠婚柛锝庣厜�?8 ~ 11 閻庣數鎳撶花鏌ユ嚋閸偅�?//  @param on PWM濞戞挸锕ゅ畷宀€鎷嬮埄鍐╂闁? ~ 4096, 濞戞挴鍋撻柤鍓插墮瑜?on = 0
//  @param off PWM濞戞挸顑夊椋庢媼閳╁啯娈堕�? ~ 4096, 闁告濮烽埞鏍掗弬鎹愮�?off/4096
void PCA_Setpwm(uint8_t num, uint32_t on, uint32_t off) 
{
    // uint8_t data[5];
    // data[0] = LED0_ON_L + 4 * num;
    // data[1] = on & 0xFF;
    // data[2] = on >> 8;
    // data[3] = off & 0xFF;
    // data[4] = off >> 8;
    // HAL_I2C_Master_Transmit(&hi2c2, PCA9685_ADDR, data, 5, I2C_TO);
    PCA_Write(LED0_ON_L+4*num, on);
    PCA_Write(LED0_ON_H+4*num, on>>8);
    PCA_Write(LED0_OFF_L+4*num, off);
    PCA_Write(LED0_OFF_H+4*num, off>>8);
}

void PCA_Setpwm_IT(uint8_t num, uint32_t on, uint32_t off)
{
    uint8_t data[5];
    data[0] = LED0_ON_L + 4 * num;
    data[1] = on & 0xFF;
    data[2] = on >> 8;
    data[3] = off & 0xFF;
    data[4] = off >> 8;
    HAL_I2C_Master_Transmit_IT(&hi2c2, PCA9685_ADDR, data, 5);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        NOP_Delay(2000);
        switch (i2c_state)
        {
        case I2C_TaskState::PressureSensor:
            switch (pressure_sensor.ps_i2c_state)
            {
            case PS_I2C_TaskState::TCA_SET_0:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_READ_0;
                break;
            case PS_I2C_TaskState::TCA_SET_1:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_READ_1;
                break;
            case PS_I2C_TaskState::TCA_SET_2:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_READ_2;
                break;
            case PS_I2C_TaskState::TCA_SET_3:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_READ_3;
                break;
            case PS_I2C_TaskState::TRANSMIT_READ_0:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::RECEIVE_0;
                break;
            case PS_I2C_TaskState::TRANSMIT_READ_1:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::RECEIVE_1;
                break;
            case PS_I2C_TaskState::TRANSMIT_READ_2:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::RECEIVE_2;
                break;
            case PS_I2C_TaskState::TRANSMIT_READ_3:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::RECEIVE_3;
                break;
            case PS_I2C_TaskState::TRANSMIT_CONV_0:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TCA_SET_1;
                break;
            case PS_I2C_TaskState::TRANSMIT_CONV_1:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TCA_SET_2;
                break;
            case PS_I2C_TaskState::TRANSMIT_CONV_2:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TCA_SET_3;
                break;
            case PS_I2C_TaskState::TRANSMIT_CONV_3:
                pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TCA_SET_0;
                break;
            default:
                break;
            }

            if (pressure_sensor.ps_i2c_state == PS_I2C_TaskState::TCA_SET_0)
            {
                i2c_state = I2C_TaskState::PROPELLER;
            }
            else
            {
                pressure_sensor.PS_I2C_IT_Handle();
            }
            break;

        case I2C_TaskState::PROPELLER:
            if (propeller_i2c.pro_i2c_state == PRO_I2C_TaskState::TCA_SET)
            {
                propeller_i2c.pro_i2c_state = PRO_I2C_TaskState::SET_PWM;
                NOP_Delay(2000);
            }
            else
            {
                propeller_i2c.pro_i2c_state = PRO_I2C_TaskState::TCA_SET;
                i2c_state = I2C_TaskState::SERVO;
            }
            break;

        case I2C_TaskState::SERVO:
            i2c_state = I2C_TaskState::PressureSensor;
            break;
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2){
        switch (i2c_state)
        {
        case I2C_TaskState::PressureSensor:
            switch (pressure_sensor.ps_i2c_state) 
            {
                case PS_I2C_TaskState::RECEIVE_0:
                    pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_CONV_0;
                    break;

                case PS_I2C_TaskState::RECEIVE_1:
                    pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_CONV_1;
                    break;

                case PS_I2C_TaskState::RECEIVE_2:
                    pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_CONV_2;
                    break;

                case PS_I2C_TaskState::RECEIVE_3:
                    pressure_sensor.ps_i2c_state = PS_I2C_TaskState::TRANSMIT_CONV_3;
                    break;

                default:
                    break;
            }
            pressure_sensor.PS_I2C_IT_Handle();
            break;
        
        default:
            break;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        pressure_sensor.MarkI2CError();
        I2C2_Recover();
    }
}
