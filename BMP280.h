#ifndef BMP280_H
#define BMP280_H


#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"


// device has default bus address of 0x76
#define ADDR _u(0x76)

// hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

#define REG_CHIP_ID _u(0xD0)
#define WHO_AM_I _u(0x58)

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 24

class BMP280
{
    private:
        // Private Variables or Functions

        // Temperature calibration parameters
        static uint16_t dig_t1;
        static int16_t dig_t2;
        static int16_t dig_t3;

        // Pressure calibration parameters
        static uint16_t dig_p1;
        static int16_t dig_p2;
        static int16_t dig_p3;
        static int16_t dig_p4;
        static int16_t dig_p5;
        static int16_t dig_p6;
        static int16_t dig_p7;
        static int16_t dig_p8;
        static int16_t dig_p9;

        // Initialization data
        // uint8_t ;

        // I2C initialization parameters
        i2c_inst_t* inst;
        int scl;
        int sda;

        int32_t raw_temperature;
        int32_t raw_pressure;

        int32_t convert(int32_t temp); // private

        void get_calib_params();


    public:
        bool init();
        
        void reset();

        void get_raw();

        float get_temperature(); // get_temp

        int32_t get_pressure(); // get_press, remove temp

        BMP280(i2c_inst_t *inst, int sda, int scl);
        ~BMP280();

};




#endif