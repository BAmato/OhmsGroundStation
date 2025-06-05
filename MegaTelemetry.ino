#include <Wire.h>
#include <Adafruit_GPS.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Serial ports
// Serial  : USB debug
// Serial1 : GPS (RX1 = 19, TX1 = 18)
// Serial2 : XBee (TX2 = 16, RX2 = 17)

// GPS object
Adafruit_GPS GPS(&Serial1);

// MPU6050 object
MPU6050 mpu;

// DMP related
bool dmpReady = false;           // set true if DMP init succeeded
uint8_t fifoBuffer[64];
uint16_t packetSize = 0;

// Constants
#define GPS_PPS_PIN 2                 // PPS input pin
#define LOOP_INTERVAL_MS 100          // main loop period
#define ACCEL_SCALE     16384.0f      // LSB/g for ±2g range

// time keeping
volatile uint32_t currentTime = 0;    // ms since GPS epoch at last PPS
volatile uint32_t lastPpsMillis = 0;  // millis() captured on last PPS

// forward declarations
static uint32_t gpsToEpochMillis(int year, int month, int day,
                                 int hour, int minute, int second, int ms);
void updateTimeFromGPS();
void ppsISR();

void setup() {
  // initialize serial ports
  Serial.begin(115200);
  Serial.println("Mega Telemetry Starting");

  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // XBee

  // I2C for MPU6050
  Wire.begin();

  // set up GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // RMC+GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);     // 5 Hz update
  GPS.sendCommand(PGCMD_ANTENNA);

  // setup PPS interrupt
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), ppsISR, RISING);

  // setup MPU6050 DMP
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("MPU6050 ready");
  } else {
    Serial.print("MPU6050 DMP failed: ");
    Serial.println(devStatus);
  }
}

void loop() {
  // feed GPS characters
  while (Serial1.available()) {
    GPS.read();
  }

  // parse new NMEA sentence if available
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (GPS.fix) {
        updateTimeFromGPS();
      }
    }
  }

  // timed telemetry update
  static uint32_t lastLoop = 0;
  if (millis() - lastLoop >= LOOP_INTERVAL_MS) {
    lastLoop += LOOP_INTERVAL_MS;

    // compute current timestamp using PPS base and millis offset
    uint32_t timestamp;
    noInterrupts();
    timestamp = currentTime + (millis() - lastPpsMillis);
    interrupts();

    // get orientation and acceleration
    float roll = 0, pitch = 0, yaw = 0;
    float accX = 0, accY = 0, accZ = 0;
    if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      VectorFloat gravity;
      VectorInt16 aa;
      float ypr[3];
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw   = ypr[0] * 180.0 / M_PI;
      pitch = ypr[1] * 180.0 / M_PI;
      roll  = ypr[2] * 180.0 / M_PI;
      mpu.dmpGetAccel(&aa, fifoBuffer);
      accX = aa.x / ACCEL_SCALE;
      accY = aa.y / ACCEL_SCALE;
      accZ = aa.z / ACCEL_SCALE;
    }

    // latest GPS values
    float lat = GPS.latitudeDegrees;
    float lon = GPS.longitudeDegrees;
    float alt = GPS.altitude;

    // build telemetry string
    char packet[128];
    snprintf(packet, sizeof(packet),
             "%lu,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f",
             timestamp, lat, lon, alt, roll, pitch, yaw, accX, accY, accZ);

    // send over XBee and debug port
    Serial2.println(packet);
    Serial.println(packet);
  }
}

// --- Helper Functions -----------------------------------------------------

void ppsISR() {
  currentTime += 1000;             // advance by 1 s
  lastPpsMillis = millis();
}

void updateTimeFromGPS() {
  uint32_t epoch = gpsToEpochMillis(GPS.year, GPS.month, GPS.day,
                                    GPS.hour, GPS.minute, GPS.seconds,
                                    GPS.milliseconds);
  noInterrupts();
  currentTime = epoch;
  lastPpsMillis = millis() - GPS.milliseconds;
  interrupts();
}

static uint32_t daysFromCivil(int y, int m, int d) {
  if (m <= 2) {
    y -= 1;
    m += 12;
  }
  int era = y / 400;
  unsigned yoe = y - era * 400;
  unsigned doy = (153 * (m - 3) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const unsigned GPS_EPOCH_DAYS = 723125U;  // days from civil 0 to 1980-01-06
  return era * 146097U + doe - GPS_EPOCH_DAYS;
}

static uint32_t gpsToEpochMillis(int year, int month, int day,
                                 int hour, int minute, int second, int ms) {
  int fullYear = year + 2000;
  uint32_t days = daysFromCivil(fullYear, month, day);
  uint32_t secs = days * 86400UL + hour * 3600UL + minute * 60UL + second;
  return secs * 1000UL + ms;
}
