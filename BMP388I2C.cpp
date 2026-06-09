#include "BMP388I2C.h"

#define BMP388_CHIP_ID 0x00

#define BMP388_TEMP_DATA 0x07
#define BMP388_PRESS_DATA 0x04

#define BMP388_PWR_CTRL 0x1B
#define BMP388_OSR      0x1C
#define BMP388_ODR      0x1D
#define BMP388_CONFIG   0x1F

#define BMP388_CALIB_DATA 0x31

#define BMP388_TEMP_OFFSET 12.0f

BMP388I2C::BMP388I2C(uint8_t addr) {
    _addr = addr;
}

bool BMP388I2C::begin() {

    Wire.begin();

    if (read8(BMP388_CHIP_ID) != 0x50) {
        return false;
    }

    // enable pressure + temperature normal mode
    write8(BMP388_PWR_CTRL, 0x37);

    // no oversampling
    write8(BMP388_OSR, 0x00);

    // no filter
    write8(BMP388_CONFIG, 0x00);

    readCalibrationData();

    return true;
}

void BMP388I2C::write8(uint8_t reg, uint8_t value) {

    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t BMP388I2C::read8(uint8_t reg) {

    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_addr, (uint8_t)1);

    return Wire.read();
}

void BMP388I2C::readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) {

    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_addr, len);

    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = Wire.read();
    }
}

uint32_t BMP388I2C::readRawTemp() {

    uint8_t data[3];

    readBytes(BMP388_TEMP_DATA, data, 3);

    return (uint32_t)data[0] |
          ((uint32_t)data[1] << 8) |
          ((uint32_t)data[2] << 16);
}

uint32_t BMP388I2C::readRawPress() {

    uint8_t data[3];

    readBytes(BMP388_PRESS_DATA, data, 3);

    return (uint32_t)data[0] |
          ((uint32_t)data[1] << 8) |
          ((uint32_t)data[2] << 16);
}

void BMP388I2C::readCalibrationData() {

    uint8_t d[21];
    readBytes(BMP388_CALIB_DATA, d, 21);

    uint16_t par_t1 = (d[1] << 8) | d[0];
    uint16_t par_t2 = (d[3] << 8) | d[2];
    int8_t   par_t3 = d[4];

    int16_t par_p1 = (d[6] << 8) | d[5];
    int16_t par_p2 = (d[8] << 8) | d[7];
    int8_t  par_p3 = d[9];
    int8_t  par_p4 = d[10];

    uint16_t par_p5 = (d[12] << 8) | d[11];
    uint16_t par_p6 = (d[14] << 8) | d[13];

    int8_t  par_p7  = d[15];
    int8_t  par_p8  = d[16];
    int16_t par_p9  = (d[18] << 8) | d[17];
    int8_t  par_p10 = d[19];
    int8_t  par_p11 = d[20];

    // Temp
    calib.par_t1 = (float)par_t1 * 256.0f;          
    calib.par_t2 = (float)par_t2 / 1073741824.0f;   
    calib.par_t3 = (float)par_t3 / 281474976710656.0f; 

    // Pressure
    calib.par_p1  = ((float)par_p1  - 16384.0f) / 1048576.0f;    
    calib.par_p2  = ((float)par_p2  - 16384.0f) / 536870912.0f; 
    calib.par_p3  = (float)par_p3  / 4294967296.0f;              
    calib.par_p4  = (float)par_p4  / 137438953472.0f;           
    calib.par_p5  = (float)par_p5  * 8.0f;                       
    calib.par_p6  = (float)par_p6  / 64.0f;                      
    calib.par_p7  = (float)par_p7  / 256.0f;                     
    calib.par_p8  = (float)par_p8  / 32768.0f;                   
    calib.par_p9  = (float)par_p9  / 281474976710656.0f;         
    calib.par_p10 = (float)par_p10 / 281474976710656.0f;         
    calib.par_p11 = (float)par_p11 / 36893488147419103232.0f;    
}

float BMP388I2C::compensateTemperature(uint32_t rawTemp) {

    float partial1 =
        ((float)rawTemp - calib.par_t1);

    float partial2 =
        partial1 * calib.par_t2;

    calib.t_lin =
        partial2 +
        (partial1 * partial1) * calib.par_t3;

    return calib.t_lin;
}

float BMP388I2C::compensatePressure(uint32_t rawPress) {

    float pd1, pd2, pd3, pd4;
    float po1, po2;

    pd1 = calib.par_p6 * calib.t_lin;
    pd2 = calib.par_p7 * calib.t_lin * calib.t_lin;
    pd3 = calib.par_p8 * calib.t_lin * calib.t_lin * calib.t_lin;

    po1 = calib.par_p5 + pd1 + pd2 + pd3;

    pd1 = calib.par_p2 * calib.t_lin;
    pd2 = calib.par_p3 * calib.t_lin * calib.t_lin;
    pd3 = calib.par_p4 * calib.t_lin * calib.t_lin * calib.t_lin;

    po2 =
        rawPress *
        (calib.par_p1 + pd1 + pd2 + pd3);

    pd1 = rawPress * rawPress;

    pd2 = calib.par_p9 + calib.par_p10 * calib.t_lin;

    pd3 = pd1 * pd2;

    pd4 = pd3 +
        (rawPress * rawPress * rawPress) *
        calib.par_p11;

    return po1 + po2 + pd4;
}

float BMP388I2C::readTemperature() {

    uint32_t rawTemp = readRawTemp();

    return compensateTemperature(rawTemp) - BMP388_TEMP_OFFSET;
}

float BMP388I2C::readPressure() {

    uint32_t rawTemp = readRawTemp();
    uint32_t rawPress = readRawPress();

    compensateTemperature(rawTemp);
    rawTemp -= BMP388_TEMP_OFFSET;

    return compensatePressure(rawPress);
}