#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>

//pin definition
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

#define BMP_CS 10

#define RADIO_INTERVAL 1000
#define MSP_INTERVAL 20

#define msp &Serial1

//msp commands
constexpr uint8_t RC_CMD = 200;
constexpr uint8_t GPS_GET = 106;
constexpr uint8_t GYRO_GET = 102;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;

//constants (CHANGE BEFORE LAUNCH !!!!!!!!!)
#define PRESSURE_SEA 1010.2
#define RF95_FREQ   433.1

//object definition
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_BMP3XX bmp;

//variables
int status = 0;
unsigned long lastRadio = 0;
unsigned long lastMSP = 0;
int16_t data[8];
bool landing = false;

//msp reading var
uint8_t payload[16];
enum MSPType {
  IDLE,
  DOLLAR,
  M,
  ARROW,
  SIZE,
  CMD,
  PAYLOAD,
  CHECKSUM
};

MSPType type = IDLE;
uint8_t dataSize = 0;
uint8_t cmd = 0;
uint8_t checksum = 0;
uint8_t arrayptr = 0;

//gps reading
uint8_t fix, numSat;
float latitude, longitude;
int16_t altitudeGPS;

//gyro reading
float accX, accY, accZ;

//mission state reading
uint8_t navState;

//battery voltage 
float battVolt;

//setup
void setup() {

  //spi
  SPI.begin();

  //bmp init 
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);

  if (!bmp.begin_SPI(BMP_CS)) {
    while(1);
  }

  //radio init
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

  rf95.setTxPower(20, false); 

  //msp init
  msp.begin(115200)

  //msp data array init
  for (int i = 0; i < 16; i++) {
    data[i] = 1500;
  }
  data[2] = 1000;

  //watchdog setup
  Watchdog.enable(2000);
}

//MSP functions
void mspWrite(uint8_t cmd, uint8_t* payload, uint8_t size) {
  uint8_t checksum = 0;
  msp.write('$');
  msp.write('M');
  msp.write('<');
  
  msp.write(size);
  checksum ^= size;

  msp.write(cmd);
  checksum ^= cmd;

  for (uint8_t i = 0; i < size; i++) {
      msp.write(payload[i]);
      checksum ^= payload[i];
  }
  msp.write(checksum);


}
void parsePacket(uint8_t cmd) {
    switch (cmd) {
    case GPS_GET:
    fix  = payload[0];
    numSat = payload[1];

    latitude = *(int32_t*)&payload[2]/1e7;
    longitude = *(int32_t*)&payload[6]/1e7;
    altitudeGPS = (*(int16_t*)&payload[10]);
    break;
    case GYRO_GET:
    accX  = *(int16_t*)&payload[6]/1670.13;
    accY  = *(int16_t*)&payload[8]/1670.13;
    accZ  = *(int16_t*)&payload[10]/1670.13;
    break;
    case NAV_STAT:
    navState = payload[0];
    break;
    case BATT_GET:
    battVolt= payload[0]/10.0f;

  }
}
void parseMSP(uint8_t readChar) {

  switch (type) {
  case IDLE: type = (readChar == '$') ? DOLLAR : IDLE; break;
  case DOLLAR: type = (readChar == 'M') ? M : IDLE; break;
  case M: type = (readChar == '>') ? ARROW : IDLE; break;
  case ARROW: dataSize = readChar; checksum ^= readChar; type = SIZE; break;
  case SIZE: cmd = readChar; checksum ^= readChar; type = CMD; arrayptr = 0; break;
  case CMD: 
    if (arrayptr < dataSize) {
      payload[arrayptr] = readChar;
      arrayptr++;
      checksum ^= readChar;
    } else {
      type = CHECKSUM;
    }
  break;
  case CHECKSUM: checksum == readChar ? parsePacket(cmd);checksum = 0;type = IDLE; 
  }


}


void mspReadGPS() {
  mspWrite(GPS_GET, nullptr, 0);
  delay(10);
  while (msp.available()) {
    parseMSP(msp.read());
  }
}

void mspReadGyro() {
  mspWrite(GYRO_GET, nullptr, 0);
  delay(10);
  while (msp.available()) {
    parseMSP(msp.read());
  }
}

void mspReadNavStat() {
  mspWrite(NAV_STAT, nullptr, 0);
  delay(10);
  while (msp.available()) {
    parseMSP(msp.read());
  }
}

void mspReadVoltage() {
  mspWrite(BATT_GET, nullptr, 0);
  delay(10);
  while (msp.available()) {
    parseMSP(msp.read());
  }
}

bool missionCompleted() {
  mspWrite(NAV_STAT, nullptr, 0);
  while (msp.available()) {
    parseMSP(msp.read());
  }
  return (navState == 0);
}
//modes

void standbyMode(float h, float a) {
  if (h > 50 || a > 3) { //3 m/s^2
      status = 1;
  }
}

void launchedMode(float h, float a) {
  if (h < 300 && a < 0 ) {
    data[4] = 2000; //arm the fc 
    status = 2;
  }
}

void rtwpMode() {
  data[5] = 2000; //ch 6, set to navigate mission
  if (missionCompleted()) {
    status = 3;
  }
}

void landingMode() {
  landing = true;
}

void sendRadio(float height, float pressure, float temp, float altitudeGPS, float latitude, float longitude, int fix, int numSat, float battVolt, int status) {
  char message[128];
  sprintf(message, "%.2f;%.2f;%.1f;%.1f;%.7f;%.7f;%i;%i;%.2f;%i", height, pressure, temp, altitudeGPS, latitude, longitude, fix, numSat, battVolt, status);

  rf95.send((uint8_t*)message, strlen(message));
  rf95.waitPacketSent();
}



void loop() {
  // put your main code here, to run repeatedly:
  //needed sensor reading
  float temp = bmp.temperature;
  float pressure = bmp.pressure / 100.0;   //hPa
  float height = 44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  mspReadGPS();
  mspReadGyro();
  mspReadNavStat();
  mspReadVoltage();

  unsigned long now = millis();

  //mode detection
  switch (status) {
    case 0: standbyMode(height, accY); break;
    case 1: launchedMode(height, accY); break;
    case 2: rtwpMode(); break;
    case 3: landingMode(); break;
    default: 
    while(1) {
      delay(100);
    }
    break;
  }
  //sending data
  
  if (now - lastMSP >= MSP_INTERVAL && landing == false) {
    mspWrite(RC_CMD, (uint8_t*)data, 16);
    lastMSP = now;
  }

  if (now - lastRadio >= RADIO_INTERVAL) {
    sendRadio(height, pressure, temp, altitudeGPS, latitude, longitude, fix, numSat, battVolt, status);
    lastRadio = now;
  }

  Watchdog.reset();

}
