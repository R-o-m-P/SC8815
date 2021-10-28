## SC8815 General firmware library, which can realize all functions of SC8815.

    The library is written in the style of the STM32 firmware library. It has passed the preliminary test and can successfully initialize the SC8815 and set the parameters, but the functions of all functions are not guaranteed.

#### 1. The library will provide the following useful functions:
* Initialize the demo function of SC8815
* SC8815 hardware configuration initialization function
* SC8815 read interrupt status function
* SC8815 reads the built-in ADC conversion result function
* SC8815 set parameter value function
* SC8815 Get parameter setting value function
* SC8815 set hardware configuration function
* SC8815 get hardware configuration status function

#### 2. To use this library, you need to provide the following external functions:
```c
void I2C_WriteRegByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t ByteData); //Write a byte to the device register through I2C
uint8_t I2C_ReadRegByte(uint8_t SlaveAddress, uint8_t RegAddress); //Read a byte from the device register through I2C
void SoftwareDelay(uint8_t ms); //Software delay in milliseconds
```

#### 3. Four hardware parameter macros are defined in the header file of the library. You need to set according to your hardware parameters:
* SCHW_VBUS_RSHUNT: the sense resistance value on the VBUS current path, in mOhm (milliohm)
* SCHW_BATT_RSHUNT: the sense resistance value on the battery current path, the unit is mOhm (milliohm)

If in your application, the way of VBUS voltage feedback during OTG is that the VBUS pin feedbacks inside the chip, you only need to set the above two hardware parameter macros.

If the VBUS voltage feedback mode during OTG is the feedback of the external voltage divider on the FB pin, the following two hardware parameter macros need to be set.
* SCHW_FB_RUP: The resistance value between FB and VBUS, the unit is Ohm (ohm)
* SCHW_FB_RDOWM: the resistance value between FB and GND, the unit is Ohm (ohm)

When setting the OTG output voltage, the library will automatically calculate the SC8815 register parameters according to the feedback method used. You only need to set the hardware parameter macro.

#### 4. After completing a few simple configurations, you can start using this library.

    It is recommended to modify the initialization parameters you need from the demo function, and then call the demo function to initialize.

#### 5. Things needing attention:
* Each parameter in the hardware initialization structure must use the specified macro definition value.
* The library only operates on I2C, and the control pins (CE, PSTOP) of SC8815 need to be manually operated by you.
* The library will not perform any active operations. The interrupt processing of SC8815 is to call SC8815_ReadInterrupStatus() to read the interrupt status after the interrupt occurs.
