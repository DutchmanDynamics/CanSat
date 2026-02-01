#include "sbus.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

//Pins
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define RF95_FREQ   433.1  // MHz !!CHANGE BEFORE LAUNCH!!

#define BMP_CS 10

//intervals
#define RADIO_INTERVAL 1000
#define SBUS_INTERVAL 7

//variables (CHANGE BEFORE LAUNCH !!!!!!!!!)
#define PRESSURE_SEA 1010.2
#define DESTINATION_LONGITUDE 5356.345128341576 
#define DESTINATION_LATITUDE 4123.189453712354

//constants
#define ARCMIN_TO_METER 1114.561423
#define TURN_SPEED 1.5 //radians/s, change during tests

//object definition
bfs::SbusRx sbus_rx(&Serial1);

bfs::SbusTx sbus_tx(&Serial1);

bfs::SbusData data;

Adafruit_GPS GPS(&Serial0);
#define GPSECHO false

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BMP3XX bmp;

//variables (changed within the program)
unsigned long lastRadio = 0;
unsigned long lastSbus = 0;

int lastHeight = 0;
int status = 0;

//setup

void setup() {
  SPI.begin(); 
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);

  if (!bmp.begin_SPI(BMP_CS)) {
    while(1);
  }
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    while(1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    while(1);
  }

  rf95.setTxPower(4, false); //power as low as needed (2 - 20 input) (interferes with other sources)
  GPS.begin(9600);
  sbus_rx.Begin();
  sbus_tx.Begin();

  for (int i = 5; i < data.NUM_CH; i++) {
    data.ch[i] = 1024;
  }
  data.ch[2] = 172; //throttle, low before arming
  data.ch[4] = 172; //aux, low
}


//Functions

void sendRadio(float height, float pressure, float temp, float longitude, float latitude, int status) {
  char message[128];
  sprintf(message, "%.2f;%.2f;%.1f;%.7f;%.7f;%i", height, pressure, temp, longitude, latitude, status);

  rf95.send((uint8_t*)message, strlen(message));
  rf95.waitPacketSent();
}

float calcAngle(float x1, float x2, float y1, float y2) {
  return std::atan((y2-y1)/(x2-x1));
}

float calcDistance(float x1, float x2, float y1, float y2) {
  float dx = (x2 - x1) * ARCMIN_TO_METER;
  float dy = (y2 - y1) * ARCMIN_TO_METER;
  return sqrt(dx*dx + dy*dy);
}

void write() {
  sbus_tx.data(data);
  sbus_tx.Write();
}

void turn(float speed) {
  data.ch[3] = map(speed * 1000, TURN_SPEED*-1000, TURN_SPEED * 1000, 172, 1811);
  write();
}
void moveForward() {
  data.ch[1] = 1200;
  data.ch[2] = 1100;
  write();
}

void stop() {
  for (int i = 0; i < data.NUM_CH; i++) {
    data.ch[i] = 1024;
  }
  data.ch[4] = 1811;
  write();
}

void sbusArm() {
  data.ch[2] = 172;
  data.ch[4] = 1811;
  write();
  delay(SBUS_INTERVAL);
}

void sbusDisarm() {
  data.ch[2] = 172;
  data.ch[4] = 172;
  write();
  delay(SBUS_INTERVAL);
}

float handleDomain(float angle) {
  while (angle > PI)  angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

//Modes

void standbyMode(int height) {
  if (height > 50) {
    status = 1;
    rf95.setTxPower(20, false);
  }
  delay(1000);
}

void launchedSetup(int height) {
  if (height - lastHeight < 0) {
    status = 2;
  }
  lastHeight = height;
  delay(100);
}

void fallingMode(int height) {
  if (height <100) {
    status = 3;
    sbusArm();
  }
  delay(100);
}

void openProps() {
  //smth smth mechanism
}

void handleAngleDiff() {
  for (int i = 0; i <100; i++) {
    moveForward();
    delay(SBUS_INTERVAL);
  }

  stop();
  float movedLong = GPS.longitude();
  float movedLat = GPS.latitude();
  float droneAngle = calcAngle(latitude, movedLat, longitude, movedLong);
  float neededAngle = calcAngle(movedLat, DESTINATION_LATITUDE, movedLong, DESTINATION_LONGITUDE);
  float angleDiff = handleDomain(droneAngle - neededAngle);
  float anglespeed = angleDiff * TURN_SPEED; 
  anglespeed = constrain(anglespeed, -1 *TURN_SPEED, TURN_SPEED);
  if (fabs(angleDiff) < 0.01) {
    anglespeed = 0; //stop
    status = 5;
  }
  turn(anglespeed);
}

void moveMode() {
  moveForward();
  if (calcDistance(latitude, DESTINATION_LATITUDE, longitude, DESTINATION_LONGITUDE) < 5) {
    status = 6;
    stop();
    sbusDisarm();
  }
  delay(SBUS_INTERVAL);
}

void landedMode() {
  //beeper starts automatically after disarm
  delay(100);
}

void brokenMode() {
  while(1);
}
//Loop

void loop() {
  float temp = bmp.temperature;
  float pressure = bmp.pressure / 100.0;   //hPa
  float height = 44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  float longitude = GPS.longitude();
  float latitude = GPS.latitude();
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
  }
  unsigned long now = millis();
  
  switch (status) {
    case 0:
      standbyMode(height);
      break;
    case 1:
      launchedSetup(height);
      break;
    case 2:
      fallingMode(height);
      break;
    case 3:
      openProps();
      break;
    case 4:
      handleAngleDiff();
      break;
    case 5:
      moveMode();
      break;
    case 6:
      landedMode();
      break;
    default:
      brokenMode();
  }
  if (now - lastRadio >= RADIO_INTERVAL) {
    sendRadio(height, pressure, temp, latitude, longitude, status);
  }
}
