#include <stdio.h>

#include "BMP280.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"


BMP280::BMP280(/*i2c_inst_t *inst, int sda, int scl*/)
{
    // this->inst = inst;
    // this->sda = sda;
    // this->scl = scl;

    i2c_init(this->inst, 100*1000); // 100 Khz.

    gpio_set_function(this->sda, GPIO_FUNC_I2C);
    gpio_set_function(this->scl, GPIO_FUNC_I2C);
    gpio_pull_up(this->sda);
    gpio_pull_up(this->scl);

}

BMP280::~BMP280()
{
}

        // Temperature calibration parameters
        uint16_t BMP280::dig_t1 = 0;
        int16_t BMP280::dig_t2 = 0;
        int16_t BMP280::dig_t3 = 0;

        // Pressure calibration parameters
        uint16_t BMP280::dig_p1 = 0;
        int16_t BMP280::dig_p2 = 0;
        int16_t BMP280::dig_p3 = 0;
        int16_t BMP280::dig_p4 = 0;
        int16_t BMP280::dig_p5 = 0;
        int16_t BMP280::dig_p6 = 0;
        int16_t BMP280::dig_p7 = 0;
        int16_t BMP280::dig_p8 = 0;
        int16_t BMP280::dig_p9 = 0;

bool BMP280::init()
{
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // t_sb = 0.5ms sampling time, x16 filter
    const uint8_t reg_config_val = ((0x00 << 5) | (0x05 << 2)) & 0xFC;

    // send register number followed by its corresponding value
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    i2c_write_blocking(inst, ADDR, buf, 2, false);

    // osrs_t x2, osrs_p x16, normal mode operation
    const uint8_t reg_ctrl_meas_val = (0x02 << 5) | (0x05 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(inst, ADDR, buf, 2, false);

    get_calib_params();
    uint8_t cBuf[1];
    uint8_t chip_val[1];

    cBuf[0] = REG_CHIP_ID;
    chip_val[0] = {0x00};
    i2c_write_blocking(inst, ADDR, cBuf, 1, false);
    i2c_read_blocking(inst, ADDR, chip_val, 1, false);

    if( chip_val[0] == WHO_AM_I )
    {
        return true;
    }
    else
    {
        printf("Check Connections!\n");
        reset();
        return false;
    }

}

void BMP280::get_raw()
{
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes

    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    i2c_write_blocking(inst, ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(inst, ADDR, buf, 6, false);  // false - finished with bus

    // store the 20 bit read in a 32 bit signed integer for conversion
    this->raw_pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    this->raw_temperature = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

void BMP280::reset()
{
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(inst, ADDR, buf, 2, false);
}

int32_t BMP280::convert(int32_t temp)
{
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)dig_t1 << 1))) * ((int32_t)dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)dig_t1)) * ((temp >> 4) - ((int32_t)dig_t1))) >> 12) * ((int32_t)dig_t3)) >> 14;
    return var1 + var2;
    
}

float BMP280::get_temperature() {
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = this->convert(this->raw_temperature);
    return ((t_fine * 5 + 128) >> 8)/100.f;
}

int32_t BMP280::get_pressure() {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = this->convert(this->raw_temperature);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_p6);
    var2 += ((var1 * ((int32_t)dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_p4) << 16);
    var1 = (((dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - this->raw_pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + dig_p7) >> 4));
    return converted;
}


void BMP280::get_calib_params() {
    // raw temp and pressure values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, and 9 pressure params, each with a LSB
    // and MSB register, so we read from 24 registers

    uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking(inst, ADDR, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    i2c_read_blocking(inst, ADDR, buf, NUM_CALIB_PARAMS, false);  // false, we're done reading

    // store these in a struct for later use
    dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

    
}