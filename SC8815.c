/*---------------------------------------------------------------------------/
/  SC8815 library V0.1 (C)Sghz，2021
/----------------------------------------------------------------------------/
/ SC8815The library is free software developed in accordance with the license policy under the following conditions.
/
/Copyright (C) 2021, Sghz, all rights reserved.
/
/ 1. Redistribution of the source code must retain the above copyright notice, this condition and the following disclaimer.
/
/ This software is provided "as is" by the copyright owner and contributors, and any guarantees related to this software are not liable.
/ The copyright owner or contributor is not responsible for any damage caused by the use of this software.
/---------------------------------------------------------------------------*/

//Include header file
#include "SC8815.h"

/****************************************
* @brief    Example Demo function to initialize SC8815
*****************************************/
void SC8815_Init_Demo(void)
{
    SC8815_BatteryConfigTypeDef SC8815_BatteryConfigStruct = { 0 };
    SC8815_HardwareInitTypeDef SC8815_HardwareInitStruct = { 0 };
    SC8815_InterruptStatusTypeDef SC8815_InterruptMaskInitStruct = { 0 };

    /****starting SC8815...****/
    //->Set PSTOP to high
     //->Set CE to low
     //SoftwareDelay(5); //Necessary start delay

     //Configure SC8815 battery parameter options
    SC8815_BatteryConfigStruct.IRCOMP = SCBAT_IRCOMP_20mR;
    SC8815_BatteryConfigStruct.VBAT_SEL = SCBAT_VBAT_SEL_Internal;
    SC8815_BatteryConfigStruct.CSEL = SCBAT_CSEL_4S;
    SC8815_BatteryConfigStruct.VCELL = SCBAT_VCELL_4v25;
    SC8815_BatteryConfig(&SC8815_BatteryConfigStruct);

   //Configuring SC8815 hardware parameter options
    SC8815_HardwareInitStruct.IBAT_RATIO = SCHWI_IBAT_RATIO_6x;
    SC8815_HardwareInitStruct.IBUS_RATIO = SCHWI_IBUS_RATIO_3x;
    SC8815_HardwareInitStruct.VBAT_RATIO = SCHWI_VBAT_RATIO_12_5x;
    SC8815_HardwareInitStruct.VBUS_RATIO = SCHWI_VBUS_RATIO_12_5x;
    SC8815_HardwareInitStruct.VINREG_Ratio = SCHWI_VINREG_RATIO_100x;
    SC8815_HardwareInitStruct.SW_FREQ = SCHWI_FREQ_300KHz_2;
    SC8815_HardwareInitStruct.DeadTime = SCHWI_DT_40ns;
    SC8815_HardwareInitStruct.ICHAR = SCHWI_ICHAR_IBAT;
    SC8815_HardwareInitStruct.TRICKLE = SCHWI_TRICKLE_Enable;
    SC8815_HardwareInitStruct.TERM = SCHWI_TERM_Enable;
    SC8815_HardwareInitStruct.FB_Mode = SCHWI_FB_Internal;
    SC8815_HardwareInitStruct.TRICKLE_SET = SCHWI_TRICKLE_SET_70;
    SC8815_HardwareInitStruct.OVP = SCHWI_OVP_Enable;
    SC8815_HardwareInitStruct.DITHER = SCHWI_DITHER_Disable;
    SC8815_HardwareInitStruct.SLEW_SET = SCHWI_SLEW_1mV_us;
    SC8815_HardwareInitStruct.ADC_SCAN = SCHWI_ADC_Disable;
    SC8815_HardwareInitStruct.ILIM_BW = SCHWI_ILIM_BW_1_25KHz;
    SC8815_HardwareInitStruct.LOOP = SCHWI_LOOP_Normal;
    SC8815_HardwareInitStruct.ShortFoldBack = SCHWI_SFB_Enable;
    SC8815_HardwareInitStruct.EOC = SCHWI_EOC_1_25;
    SC8815_HardwareInitStruct.PFM = SCHWI_PFM_Disable;
    SC8815_HardwareInit(&SC8815_HardwareInitStruct);

   //Configuring SC8815 interrupt mask option
    SC8815_InterruptMaskInitStruct.AC_OK = sENABLE;
    SC8815_InterruptMaskInitStruct.INDET = sENABLE;
    SC8815_InterruptMaskInitStruct.VBUS_SHORT = sENABLE;
    SC8815_InterruptMaskInitStruct.OTP = sENABLE;
    SC8815_InterruptMaskInitStruct.EOC = sENABLE;
    SC8815_ConfigInterruptMask(&SC8815_InterruptMaskInitStruct);
    /***Now you can set the PSTOP pin to low to start SC8815 power conversion****/



   /*** Example 1, set to charge mode, battery and VBUS current limit 2A, VINREG set to 12V ****/
    SC8815_SetBatteryCurrLimit(2000);
    SC8815_SetBusCurrentLimit(2000);
    SC8815_VINREG_SetVoltage(12000);
    SC8815_OTG_Disable();


    /*** Example 2, set to reverse discharge mode, battery and VBUS current limit 3A, output voltage set to 12V ****/
    //SC8815_SetBatteryCurrLimit(2000);
    //SC8815_SetBusCurrentLimit(2000);
    //SC8815_SetOutputVoltage(12000);
    //SC8815_OTG_Enable();


    /*** Example 3, read SC8815 ADC data ****/
    //uint16_t VbusVolt = SC8815_Read_VBUS_Voltage();
    //uint16_t VbusCurr = SC8815_Read_VBUS_Current();
    //uint16_t BattVolt = SC8815_Read_BATT_Voltage();
    //uint16_t BattCurr = SC8815_Read_BATT_Current();


   /*** Example 4, read SC8815 interrupt status ****/
    SC8815_ReadInterrupStatus(&SC8815_InterruptMaskInitStruct);     //MCU calls this function after receiving the SC8815 interrupt to read the SC8815 interrupt (after reading the interrupt status, the interrupt status bit will be cleared)
    if (SC8815_InterruptMaskInitStruct.AC_OK == 1)                  //Check if the AC_OK interrupt is triggered
    {
        // AC_OK interrupt handling code
    }
    else if (SC8815_InterruptMaskInitStruct.EOC == 1)
    {
        // EOC interrupt handling code
    }
}

/****************************************
* @brief Configuring SC8815 battery parameters
* @param SC8815_BatteryConfigStruct Pointer to SC8815 battery configuration structure
* @note Before configuring, start SC8815 and delay 5ms to ensure that the device is ready. You also need SC8815 PSTOP pin to be high to perform this hardware configuration
*****************************************/
void SC8815_BatteryConfig(SC8815_BatteryConfigTypeDef *SC8815_BatteryConfigStruct)
{
    uint8_t tmp;
    tmp = SC8815_BatteryConfigStruct->IRCOMP;
    tmp |= SC8815_BatteryConfigStruct->VBAT_SEL;
    tmp |= SC8815_BatteryConfigStruct->CSEL;
    tmp |= SC8815_BatteryConfigStruct->VCELL;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_VBAT_SET, tmp);
}
/****************************************
* @brief Initializing SC8815 hardware configuration
* @param SC8815_HardwareInitStruct Pointer to SC8815 hardware initialization structure
* @note Before configuring, start SC8815 and delay 5ms to ensure that the device is ready. You also need SC8815 PSTOP pin to be high to perform this hardware configuration
*****************************************/
void SC8815_HardwareInit(SC8815_HardwareInitTypeDef *SC8815_HardwareInitStruct)
{
    uint8_t tmp;

    //Ratio configuration
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0xE0; //Read the reserved bits in the register (these reserved bits cannot be moved)
    tmp |= SC8815_HardwareInitStruct->IBAT_RATIO;           //Loading configuration parameters
    tmp |= SC8815_HardwareInitStruct->IBUS_RATIO;
    tmp |= SC8815_HardwareInitStruct->VBAT_RATIO;
    tmp |= SC8815_HardwareInitStruct->VBUS_RATIO;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_RATIO, tmp);    //Write back to register

    //Hardware configuration 0
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x60;
    tmp |= SC8815_HardwareInitStruct->VINREG_Ratio;
    tmp |= SC8815_HardwareInitStruct->SW_FREQ;
    tmp |= SC8815_HardwareInitStruct->DeadTime;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, tmp);

    //Hardware configuration 1
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x03;
    tmp |= SC8815_HardwareInitStruct->ICHAR;
    tmp |= SC8815_HardwareInitStruct->TRICKLE;
    tmp |= SC8815_HardwareInitStruct->TERM;
    tmp |= SC8815_HardwareInitStruct->FB_Mode;
    tmp |= SC8815_HardwareInitStruct->TRICKLE_SET;
    tmp |= SC8815_HardwareInitStruct->OVP;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL1_SET, tmp);

    //Hardware configuration 2
    tmp = (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL2_SET) & 0xF0) | 0x08;
    tmp |= SC8815_HardwareInitStruct->DITHER;
    tmp |= SC8815_HardwareInitStruct->SLEW_SET;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL2_SET, tmp);

    //Hardware configuration 3 (no reserved bits)
    tmp = SC8815_HardwareInitStruct->ADC_SCAN;
    tmp |= SC8815_HardwareInitStruct->ILIM_BW;
    tmp |= SC8815_HardwareInitStruct->LOOP;
    tmp |= SC8815_HardwareInitStruct->ShortFoldBack;
    tmp |= SC8815_HardwareInitStruct->EOC;
    tmp |= SC8815_HardwareInitStruct->PFM;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, tmp);

    //Delay required after the configuration is complete
    SoftwareDelay(5);
}
/****************************************
* @brief Configure SC8815 interrupt mask (interrupt enable or disable)
* @param InterruptStatusStruct Pointer to SC8815 interrupt status configuration structure
*****************************************/
void SC8815_ConfigInterruptMask(SC8815_InterruptStatusTypeDef *InterruptStatusStruct)
{
    uint8_t tmp;
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_MASK) & 0x91;
    tmp |= InterruptStatusStruct->AC_OK ? 0x40 : 0x00;
    tmp |= InterruptStatusStruct->INDET ? 0x20 : 0x00;
    tmp |= InterruptStatusStruct->VBUS_SHORT ? 0x08 : 0x00;
    tmp |= InterruptStatusStruct->OTP ? 0x04 : 0x00;
    tmp |= InterruptStatusStruct->EOC ? 0x02 : 0x00;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_MASK, tmp);
}

/****************************************
* @brief read the interrupt status of SC8815
* @param InterruptStatusStruct Pointer to SC8815 interrupt status structure
*****************************************/
void SC8815_ReadInterrupStatus(SC8815_InterruptStatusTypeDef *InterruptStatusStruct)
{
    uint8_t tmp;
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_MASK);     //Read status register
    InterruptStatusStruct->AC_OK = (tmp & 0x40) >> 6;   //Disassemble every bit
    InterruptStatusStruct->INDET = (tmp & 0x20) >> 5;
    InterruptStatusStruct->VBUS_SHORT = (tmp & 0x08) >> 3;
    InterruptStatusStruct->OTP = (tmp & 0x04) >> 2;
    InterruptStatusStruct->EOC = (tmp & 0x02) >> 1;
}

/****************************************
* @brief reads the measurement result of the VBUS voltage by the built-in ADC of SC8815
* @return VBUS voltage in mV
*****************************************/
uint16_t SC8815_Read_VBUS_Voltage(void)
{
    uint8_t tmp1, tmp2;
    double RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x01) == 1) ? 5 : 12.5; //Get the ratio of VBUS voltage
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUS_FB_VALUE);           //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUS_FB_VALUE2) >> 6;     //Read ADC value register 2

    //Return VBUS voltage value
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * RATIO_Value) * 2;
}
/****************************************
* @brief reads the measurement result of the VBUS current by the built-in ADC of SC8815
* @return VBUS current in mA (SC8815 has no current direction distinction)
*****************************************/
uint16_t SC8815_Read_VBUS_Current(void)
{
    uint8_t tmp1, tmp2;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x0C) == 4) ? 6 : 3;    //IBUS ratio
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBUS_VALUE);              //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBUS_VALUE2) >> 6;        //Read ADC value register 2

    //Return VBUS current value
    return ((uint16_t)(50 * RATIO_Value) * (4 * tmp1 + tmp2 + 1)) / (3 * SCHW_VBUS_RSHUNT);
}
/****************************************
* @brief    读取 SC8815 内置 ADC 对电池电压的测量结果
* @return   单位为 mV 的电池电压
*****************************************/
uint16_t SC8815_Read_BATT_Voltage(void)
{
    uint8_t tmp1, tmp2;
    double RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x02) == 2) ? 5 : 12.5; //取得电池电压的比率
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBAT_FB_VALUE);           //读取 ADC 值寄存器1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBAT_FB_VALUE2) >> 6;     //读取 ADC 值寄存器2

    //返回电池电压值
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * RATIO_Value) * 2;
}
/****************************************
* @brief    读取 SC8815 内置 ADC 对电池电流的测量结果
* @return   单位为 mA 的电池电流 (SC8815 无电流方向区分)
*****************************************/
uint16_t SC8815_Read_BATT_Current(void)
{
    uint8_t tmp1, tmp2;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x10) == 16) ? 12 : 6;  //取得 IBAT 的比率
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBAT_VALUE);              //读取 ADC 值寄存器1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBAT_VALUE2) >> 6;        //读取 ADC 值寄存器2

    //返回 IBAT 电流值
    return (uint16_t)((50 * RATIO_Value) * (4 * tmp1 + tmp2 + 1)) / (3 * SCHW_BATT_RSHUNT);
}
/****************************************
* @brief    读取 SC8815 内置 ADC 对 ADIN_PIN 电压的测量结果
* @return   单位为 mV 的 ADIN_PIN 电压
*****************************************/
uint16_t SC8815_Read_ADIN_Voltage(void)
{
    uint8_t tmp1,tmp2;

    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_ADIN_VALUE);          //读取 ADC 值寄存器1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_ADIN_VALUE2) >> 6;    //读取 ADC 值寄存器2

    //返回 ADIN_PIN 电压值
    return (uint16_t)(4 * tmp1 + tmp2 + 1) * 2;
}

/****************************************
* @brief    设置 SC8815 在 OTG 反向输出时的输出电压
* @param    NewVolt 新的单位为 mV 的输出电压设定值
* @note     不得输入超出最大可设定值的电压值, 如最大输出 1024mV,  输入 1145mV 将导致计算结果溢出错误
*****************************************/
void SC8815_SetOutputVoltage(uint16_t NewVolt)
{
    uint16_t tmp1,tmp2;
    double RATIO_Value;

    //判断 VBUS 电压反馈的模式
    if (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x10)
    {
        RATIO_Value = (double)SCHW_FB_RUP / SCHW_FB_RDOWM + 1.0;    //计算输出电压比率
        tmp1 = (NewVolt / RATIO_Value) / 2;                         //计算对应的参考电压

        //得到 VBUSREF 寄存器 2 的值
        for (tmp2 = 0; tmp2 < 3; tmp2++)
        {
            if (((tmp1 - tmp2 - 1) % 4) == 0)
            {
                break;
            }
        }

        //得到 VBUSREF 寄存器 1 的值
        tmp1 = (tmp1 - tmp2 - 1) / 4;

        //写入到 SC8815 VBUSREF_E_SET 寄存器
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET, (uint8_t)tmp1);
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET2, (uint8_t)tmp2);
    }
    else
    {
        RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x01) == 1) ? 5 : 12.5; //取得 VBUS 电压的比率
        tmp1 = NewVolt / RATIO_Value / 2;   //计算对应的参考电压

        //得到 VBUSREF 寄存器 2 的值
        for (tmp2 = 0; tmp2 < 3; tmp2++)
        {
            if (((tmp1 - tmp2 - 1) % 4) == 0)
            {
                break;
            }
        }

        //得到 VBUSREF 寄存器 1 的值
        tmp1 = (tmp1 - tmp2 - 1) / 4;

        //写入到 SC8815 VBUSREF_I_SET 寄存器
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_I_SET, (uint8_t)tmp1);
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_I_SET2, (uint8_t)tmp2);
    }
}
/****************************************
* @brief    设置 SC8815 VBUS 路径上的限流值,双向通用
* @param    NewILIM 新的单位为 mA 的输出限流设定值
* @note     最小的限流值不应低于 300mA, 不得输入超出最大可设定值的电流值
*****************************************/
void SC8815_SetBusCurrentLimit(uint16_t NewILIM)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x0C) == 4) ? 6 : 3;    //取得 IBUS 的比率
    tmp = (16 * (NewILIM) * (SCHW_VBUS_RSHUNT)) / (625 * RATIO_Value) - 1;              //计算 LIM 设置值
    I2C_WriteRegByte(SC8815_ADDR, SCREG_IBUS_LIM_SET, tmp);                             //写入到 SC8815 寄存器
}
/****************************************
* @brief    设置 SC8815 电池路径上的限流值,双向通用
* @param    NewILIM 新的单位为 mA 的电池限流设定值
* @note     最小的限流值不应低于 300mA, 不得输入超出最大可设定值的电流值
*****************************************/
void SC8815_SetBatteryCurrLimit(uint16_t NewILIM)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x10) == 16) ? 12 : 6; //取得 IBAT 的比率
    tmp = (16 * (NewILIM) * (SCHW_VBUS_RSHUNT)) / (625 * RATIO_Value) - 1;             //计算 LIM 设置值
    I2C_WriteRegByte(SC8815_ADDR, SCREG_IBAT_LIM_SET, tmp);                            //写入到 SC8815 寄存器
}
/****************************************
* @brief    设置 SC8815 VINREG 电压值 (类似 MPPT)
* @param    NewVolt 新的单位为 mV 的 VINREG 电压设定值
* @note     不得输入超出最大可设定值的电压值
*****************************************/
void SC8815_VINREG_SetVoltage(uint16_t NewVolt)
{
    uint16_t RATIO_Value;
    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x10) == 16) ? 40 : 100;    //取得 VINREG 的比率
    I2C_WriteRegByte(SC8815_ADDR, SCREG_VINREG_SET, (NewVolt / RATIO_Value) - 1);               //写入到 SC8815 寄存器
}

/****************************************
* @brief    获取 SC8815 在 OTG 反向输出时的输出电压设定值
* @return   单位为 mV 的输出电压设定值
*****************************************/
uint16_t SC8815_GetOutputVoltage(void)
{
    uint16_t tmp1, tmp2;
    double RATIO_Value;

    //判断 VBUS 电压反馈的模式
    if (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x10)
    {
        //读取 VBUSREF_E_SET 寄存器
        tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET);
        tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET2);

        //计算输出电压比率
        RATIO_Value = (double)SCHW_FB_RUP / SCHW_FB_RDOWM + 1.0;
    }
    else
    {
        //读取 VBUSREF_E_SET 寄存器
        tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET);
        tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET2);

        //取得VBUS电压的比率
        RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x01) == 1) ? 5 : 12.5;
    }

    //返回对应的输出电压
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * RATIO_Value) * 2;
}
/****************************************
* @brief    获取 SC8815 VBUS 路径上的限流设定值
* @return   单位为 mA 的 VBUS 路径限流设定值
*****************************************/
uint16_t SC8815_GetBusCurrentLimit(void)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x0C) == 4) ? 6 : 3; //取得 IBUS 的比率
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBUS_LIM_SET);                          //取得 IBUS 限流寄存器值

    //返回IBUS限流值
    return (uint16_t)((uint32_t)(625 * (RATIO_Value) * (tmp + 1)) / (16 * (SCHW_VBUS_RSHUNT)));
}
/****************************************
* @brief    获取 SC8815 电池路径上的限流设定值
* @return   单位为 mA 的电池路径限流设定值
*****************************************/
uint16_t SC8815_GetBatteryCurrLimit(void)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x10) == 16) ? 12 : 6; //取得 IBAT 的比率
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBAT_LIM_SET);                            //取得 IBAT 限流寄存器值

    //返回IBAT限流值
    return (uint16_t)((uint32_t)(625 * (RATIO_Value) * (tmp + 1)) / (16 * (SCHW_BATT_RSHUNT)));
}
/****************************************
* @brief    获取 SC8815 VINREG 电压设定值
* @return   单位为 mV 的 VINREG 电压设定值
*****************************************/
uint16_t SC8815_VINREG_GetVoltage(void)
{
    uint8_t tmp;
    uint16_t RATIO_Value;
    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x10) == 16) ? 40 : 100; //取得 VINREG 的比率
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_VINREG_SET);                                    //取得 VINREG 寄存器值
    return tmp * RATIO_Value;
}

/****************************************
* @brief    打开 OTG 反向放电模式
*****************************************/
void SC8815_OTG_Enable(void)
{
    //设置 EN_OTG 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) | 0x80);
}
/****************************************
* @brief    关闭 OTG 反向放电模式, SC8815 将处于充电模式
*****************************************/
void SC8815_OTG_Disable(void)
{
    //清除 EN_OTG 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x7F);
}
/****************************************
* @brief    设置 VINREG 的增益为 40x
*****************************************/
void SC8815_VINREG_SetRatio_40x(void)
{
    //设置 VINREG_RATIO 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) | 0x10);
}
/****************************************
* @brief    设置 VINREG 的增益为 100x
*****************************************/
void SC8815_VINREG_SetRatio_100x(void)
{
    //清除 VINREG_RATIO 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0xEF);
}
/****************************************
* @brief    打开 OTG 反向放电模式中的 VBUS 过压保护功能
*****************************************/
void SC8815_OVP_Enable(void)
{
    //清除 DIS_OVP 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL1_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0xFB);
}
/****************************************
* @brief    关闭 OTG 反向放电模式中的 VBUS 过压保护功能
*****************************************/
void SC8815_OVP_Disable(void)
{
    //设置 DIS_OVP 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL1_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) | 0x04);
}
/****************************************
* @brief    打开 PGATE 引脚功能, 输出低电平打开 PMOS
*****************************************/
void SC8815_PGATE_Enable(void)
{
    //设置 EN_PGATE 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x80);
}
/****************************************
* @brief    关闭 PGATE 引脚功能, 输出高电平关闭 PMOS
*****************************************/
void SC8815_PGATE_Disable(void)
{
    //清除 EN_PGATE 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x7F);
}
/****************************************
* @brief    打开 GPO 引脚功能, 输出低电平
*****************************************/
void SC8815_GPO_Enable(void)
{
    //设置 GPO_CTRL 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x40);
}
/****************************************
* @brief    关闭 GPO 引脚功能, 输出高阻状态
*****************************************/
void SC8815_GPO_Disable(void)
{
    //清除 GPO_CTRL 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xBF);
}
/****************************************
* @brief    打开 ADC 扫描, 此时可以读取 ADC 数据
*****************************************/
void SC8815_ADC_Enable(void)
{
    //设置 AD_START 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x20);
}
/****************************************
* @brief    关闭 ADC 扫描, 节约 1-2mA 的耗电
*****************************************/
void SC8815_ADC_Disable(void)
{
    //清除 AD_START 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xDF);
}
/****************************************
* @brief    打开 OTG 反向放电模式中的 VBUS 和 VBAT 短路保护功能
*****************************************/
void SC8815_SFB_Enable(void)
{
    //清除 DIS_ShortFoldBack 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xFB);
}
/****************************************
* @brief    关闭 OTG 反向放电模式中的 VBUS 和 VBAT 短路保护功能
*****************************************/
void SC8815_SFB_Disable(void)
{
    //设置 DIS_ShortFoldBack 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x04);
}
/****************************************
* @brief    打开 OTG 模式中轻载条件下的 PFM 模式
*****************************************/
void SC8815_PFM_Enable(void)
{
    //设置 EN_PFM 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x01);
}
/****************************************
* @brief    关闭 OTG 模式中轻载条件下的 PFM 模式
*****************************************/
void SC8815_PFM_Disable(void)
{
    //清除 EN_PFM 位
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xFE);
}

/****************************************
* @brief    检查 OTG 是否处于打开状态
* @return   OTG 功能的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_OTG_IsEnable(void)
{
    //返回 OTG 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x80) ? 1 : 0;
}
/****************************************
* @brief    获取 VINREG 的增益
* @return   VINREG 的增益 (40 或 100)
*****************************************/
uint8_t SC8815_VINREG_GetRatio(void)
{
    //返回 VINREG 的增益
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x10) ? 40 : 100;
}
/****************************************
* @brief    检查 OTG 模式中 OVP(过压保护) 功能是否处于打开状态
* @return   OVP 功能的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_OVP_IsEnable(void)
{
    //返回 OVP 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x04) ? 0 : 1;
}
/****************************************
* @brief    检查 PGATE 引脚功能是否处于打开状态
* @return   PGATE 引脚功能的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_PGATE_IsEnable(void)
{
    //返回 PGATE 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x80) ? 1 : 0;
}
/****************************************
* @brief    检查 GPO 引脚功能是否处于打开状态
* @return   GPO 引脚功能的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_GPO_IsEnable(void)
{
    //返回 GPO 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x40) ? 1 : 0;
}
/****************************************
* @brief    检查 ADC 扫描是否处于打开状态
* @return   ADC 扫描的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_ADC_IsEnable(void)
{
    //返回 ADC 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x20) ? 1 : 0;
}
/****************************************
* @brief    检查 OTG 模式中 短路保护 功能是否处于打开状态
* @return   短路保护功能的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_SFB_IsEnable(void)
{
    //返回 短路保护功能 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x04) ? 0 : 1;
}
/****************************************
* @brief    检查 OTG 模式中的 PFM 模式是否处于打开状态
* @return   PFM 模式的状态 (1b 或 0b)
*****************************************/
uint8_t SC8815_PFM_IsEnable(void)
{
    //返回 PFM 的状态
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x01) ? 1 : 0;
}

/*****END OF FILE****/
