 #include <Wire.h>
 #include <BH1750.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BME280.h>

// Variables and Declarations
BH1750 lightMeter;
Adafruit_BME280 bme;

float batteryLevel;
float bh1750Factor = 1.2;  //Multiplier for lux sensor
float luxRaw;
float lux;
float temp;
float batt;

void readLux() {
  luxRaw = lightMeter.readLightLevel();
  lux = luxRaw * bh1750Factor;
  Serial.print("Lux: ");
  Serial.print(lux);
  Serial.println("lx");
}


void readBME() {
  temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
}


void readBatt() {
 batt = map(analogRead(36), 2827, 4350, 0, 100);
 Serial.print("% Battery: ");
 Serial.print(batt);
 Serial.println("%");
 Serial.println();
  }
