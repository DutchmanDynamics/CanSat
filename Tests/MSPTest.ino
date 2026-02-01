#define MSP_INTERVAL 20
#define SERIAL_INTERVAL 1000
#define msp &Serial1
//vars
unsigned long lastMSP = 0;
unsigned long lastSerial = 0;
int16_t data[8];
bool landing = false
//msp commands
constexpr uint8_t RC_CMD = 200;
constexpr uint8_t GPS_GET = 106;
constexpr uint8_t GYRO_GET = 102;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;
 
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

void setup() {
  // put your setup code here, to run once
  Serial.begin(115200);
  Serial.println("MSP INIT:")
  if(!msp.begin(115200) {
    Serial.println("MSP INIT ERROR");
    return;
  }
  //msp data array init
  for (uint8_t i = 0; i < 16; i++) {
    data[i] = 1500;
  }
  data[2] = 1000;
}

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
  case CHECKSUM:checksum == readChar ? parsePacket(cmd) : while(1);checksum = 0;type = IDLE; 
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

void loop() {
  // put your main code here, to run repeatedly:
  mspReadGPS();
  Serial.println("GPS OK");
  mspReadGyro();
  Serial.println("GYRO OK");
  mspReadNavStat();
  Serial.println("NAVSTAT OK");
  mspReadVoltage();
  Serial.println("BATTVOLT OK");
  unsigned long now = millis();

  if (now - lastMSP >= MSP_INTERVAL && landing == false) {
    mspWrite(RC_CMD, (uint8_t*)data, 16);
    lastMSP = now;
  }
  if (now - lastSerial >= SERIAL_INTERVAL) {
  Serial.println("RCWRITE OK");
  Serial.println("RESULTS:");
  Serial.print(fix);
  Serial.print(";");
  Serial.print(numSat);
  Serial.print(";");
  Serial.print(latitude);
  Serial.print(";");
  Serial.print(longitude);
  Serial.print(";");
  Serial.print(altitudeGPS);
  Serial.print(";");
  Serial.print(accY);
  Serial.print(";");
  Serial.print(navState);
  Serial.print(";");
  Serial.print(battVolt);
  lastSerial = now;
  }
}
