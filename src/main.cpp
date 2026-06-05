#include "BMP388I2C.h"

BMP388I2C bmp;

void setup() {

    Serial.begin(9600);

    if (!bmp.begin()) {

        Serial.println("BMP388 not found");

        while(1);
    }
}

void loop() {

    Serial.print("Temp: ");
    Serial.println(bmp.readTemperature());

    Serial.print("Pressure: ");
    Serial.println(bmp.readPressure());

    delay(500);
}