#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

/*
    Needed libs:
  -Adafruit BMP3XX Library (by Adafruit)
  -Adafruit GPS Library (by Adafruit)
  -RadioHead (by Mike McCauley)
  -dependencies of those three (already installed automatically)
  
*/

#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define RF95_FREQ   433.1  // MHz !!CHANGE BEFORE LAUNCH!!

#define BMP_CS 10
#define VBATPIN A7

#define PRESSURE_SEA 998 // hPa !!CHANGE BEFORE LAUNCH!!

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_BMP3XX bmp;

Adafruit_GPS GPS(&Serial1);
#define GPSECHO false

int counter = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  SPI.begin(); 
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);

  if (!bmp.begin_SPI(BMP_CS)) {
    Serial.println("Cant find bmp");
    while(1);
  }
  Serial.println("bmp is running");

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("rf doesnt initialize");
    while(1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Frequenty cannot be set");
    while(1);
  }

  rf95.setTxPower(20, false); //power as low as needed (2 - 20 input) (interferes with other sources)
  GPS.begin(9600);

}

void loop() {
  float temp = bmp.temperature;
  float pressure = bmp.pressure / 100.0;   //hPa
  float altitudeBMP = 44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  float measuredvbat = analogRead(VBATPIN);
  float altitudeGPS = GPS.altitude;
  float longitude = GPS.longitude;
  float latitude = GPS.latitude;
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  char message[64];
  sprintf(message, "#%.1f;%.2f;%.2f;%.2f;%.6f;%.6f;%i;%i", temp, pressure, altitudeBMP, altitudeGPS, longitude, latitude, measuredvbat, counter);

  rf95.send((uint8_t*)message, strlen(message));
  rf95.waitPacketSent();
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  counter++;
  digitalWrite(LED_BUILTIN, HIGH);
}
