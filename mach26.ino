#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include "BMP388I2C.h"

BMP388I2C bmp;

int counter = 0;
String gps;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial1.begin(9600);
  while (!Serial1);

  if (!bmp.begin()) {
    Serial.println("BMP388 not found");
    while(1);
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {

  Serial.print("Sending packet: ");
  Serial.println(counter);
  
  while(Serial1.available() > 0){
    gps = Serial1.readStringUntil('\n');
  }

  // send packet
  LoRa.beginPacket();
  LoRa.println(String(bmp.readPressure()) + "," + String(bmp.readTemperature()) + "," + String(gps) );
  LoRa.endPacket();

  counter++;

  delay(500);
}