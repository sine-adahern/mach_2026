#ifndef BMP388I2C_H
#define BMP388I2C_H

#include <Arduino.h>
#include <Wire.h>

class BMP388I2C {
public:
    explicit BMP388I2C(uint8_t addr = 0x76);

    bool begin();

    float readTemperature();
    float readPressure();

private:

    struct CalibData {
        float par_t1;
        float par_t2;
        float par_t3;

        float par_p1;
        float par_p2;
        float par_p3;
        float par_p4;
        float par_p5;
        float par_p6;
        float par_p7;
        float par_p8;
        float par_p9;
        float par_p10;
        float par_p11;

        float t_lin;
    };

    uint8_t _addr;
    CalibData calib;

    void write8(uint8_t reg, uint8_t value);
    uint8_t read8(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len);

    uint32_t readRawTemp();
    uint32_t readRawPress();

    void readCalibrationData();

    float compensateTemperature(uint32_t rawTemp);
    float compensatePressure(uint32_t rawPress);
};

#endif