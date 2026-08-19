#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// OLED
#define OLED_SDA 25
#define OLED_SCL 26

// BME280
#define BME_SDA 21
#define BME_SCL 22

// MPU6050
#define MPU_SDA 13
#define MPU_SCL 14

// GPS
#define GPS_RX 16
#define GPS_TX 17

// SD CARD
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 32

// START / STOP BUTTON
#define BUTTON_PIN 27

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

// ============================================================
// MPU6050
// ============================================================

#define MPU_ADDRESS 0x68

#define MPU_WHO_AM_I 0x75
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B

// ============================================================
// TIMING
// ============================================================

const unsigned long SENSOR_INTERVAL = 250;      // 4 Hz
const unsigned long RECORD_INTERVAL = 1000;     // 1 Hz
const unsigned long HEALTH_INTERVAL = 2000;     // 2 seconds
const unsigned long SCREEN_INTERVAL = 3000;      // 3 seconds
const unsigned long BUTTON_DEBOUNCE = 60;

// ============================================================
// BME280 STARTUP CALIBRATION
// ============================================================

const unsigned long BME_CALIBRATION_TIME = 10000; // 10 seconds

// Fallback pressure if absolutely no valid BME readings
// are obtained during the calibration period.
const float DEFAULT_REFERENCE_PRESSURE = 1013.25;

// ============================================================
// OBJECTS
// ============================================================

TwoWire OLEDWire = TwoWire(0);
TwoWire BMEWire = TwoWire(1);

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &OLEDWire,
-1
);

Adafruit_BME280 bme;

TinyGPSPlus gps;

HardwareSerial GPSserial(2);

// ============================================================
// MODULE STATUS
// ============================================================

bool oledOK = false;
bool mpuOK = false;
bool bmeOK = false;
bool sdOK = false;
bool gpsOK = false;

// ============================================================
// RECORDING STATE
// ============================================================

bool recording = false;
bool flightComplete = false;
bool calibratingBME = false;

// ============================================================
// FILE
// ============================================================

File flightFile;

char currentFileName[32];

// ============================================================
// SCREEN
// ============================================================

// 0 = Module Status
// 1 = Recording Status
// 2 = Flight Data

int currentScreen = 0;

// ============================================================
// TIMERS
// ============================================================

unsigned long lastSensorRead = 0;
unsigned long lastRecord = 0;
unsigned long lastHealthCheck = 0;
unsigned long lastScreenChange = 0;

unsigned long recordingStartTime = 0;

// ============================================================
// RECORDING STATISTICS
// ============================================================

unsigned long sampleCount = 0;
unsigned long successfulWrites = 0;
unsigned long failedWrites = 0;

// ============================================================
// BUTTON
// ============================================================

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

unsigned long lastButtonChange = 0;

// ============================================================
// SENSOR VALUES
// ============================================================

// BME280
float temperatureC = NAN;
float pressureHpa = NAN;
float baroAltitudeFt = NAN;

// Starting/reference pressure
float referencePressureHpa = NAN;

// GPS
double gpsLatitude = NAN;
double gpsLongitude = NAN;
double gpsAltitudeFt = NAN;
double gpsSpeedKt = NAN;
double gpsCourse = NAN;

uint32_t gpsSatellites = 0;

// MPU6050
float accelX = NAN;
float accelY = NAN;
float accelZ = NAN;

float gyroX = NAN;
float gyroY = NAN;
float gyroZ = NAN;

float totalG = NAN;

// ============================================================
// LAST SUCCESSFUL COMMUNICATION
// ============================================================

unsigned long lastGPSData = 0;
unsigned long lastMPUCommunication = 0;
unsigned long lastBMECommunication = 0;
unsigned long lastSuccessfulSDWrite = 0;

// ============================================================
// MPU6050 SOFTWARE I2C
// ============================================================

void mpuI2CDelay()
{
  delayMicroseconds(5);
}

void mpuSDAHigh()
{
  pinMode(MPU_SDA, INPUT_PULLUP);
}

void mpuSDALow()
{
  pinMode(MPU_SDA, OUTPUT);
  digitalWrite(MPU_SDA, LOW);
}

void mpuSCLHigh()
{
  pinMode(MPU_SCL, INPUT_PULLUP);
}

void mpuSCLLow()
{
  pinMode(MPU_SCL, OUTPUT);
  digitalWrite(MPU_SCL, LOW);
}

bool mpuReadSDA()
{
  return digitalRead(MPU_SDA);
}

void mpuI2CStart()
{
  mpuSDAHigh();
  mpuSCLHigh();

  mpuI2CDelay();

  mpuSDALow();

  mpuI2CDelay();

  mpuSCLLow();

  mpuI2CDelay();
}

void mpuI2CStop()
{
  mpuSDALow();

  mpuI2CDelay();

  mpuSCLHigh();

  mpuI2CDelay();

  mpuSDAHigh();

  mpuI2CDelay();
}

bool mpuI2CWriteByte(uint8_t data)
{
  for (int i = 7; i >= 0; i--)
  {
    if (data & (1 << i))
      mpuSDAHigh();
    else
      mpuSDALow();

    mpuSCLHigh();
    mpuI2CDelay();

    mpuSCLLow();
    mpuI2CDelay();
  }

  mpuSDAHigh();

  mpuSCLHigh();
  mpuI2CDelay();

  bool ack = !mpuReadSDA();

  mpuSCLLow();
  mpuI2CDelay();

  return ack;
}

uint8_t mpuI2CReadByte(bool ack)
{
  uint8_t data = 0;

  mpuSDAHigh();

  for (int i = 7; i >= 0; i--)
  {
    mpuSCLHigh();
    mpuI2CDelay();

    if (mpuReadSDA())
      data |= (1 << i);

    mpuSCLLow();
    mpuI2CDelay();
  }

  if (ack)
    mpuSDALow();
  else
    mpuSDAHigh();

  mpuSCLHigh();
  mpuI2CDelay();

  mpuSCLLow();
  mpuI2CDelay();

  mpuSDAHigh();

  return data;
}

bool mpuWriteRegister(uint8_t reg, uint8_t value)
{
  mpuI2CStart();

  if (!mpuI2CWriteByte((MPU_ADDRESS << 1) | 0))
  {
    mpuI2CStop();
    return false;
  }

  if (!mpuI2CWriteByte(reg))
  {
    mpuI2CStop();
    return false;
  }

  if (!mpuI2CWriteByte(value))
  {
    mpuI2CStop();
    return false;
  }

  mpuI2CStop();

  return true;
}

bool mpuReadRegister(uint8_t reg, uint8_t &value)
{
  mpuI2CStart();

  if (!mpuI2CWriteByte((MPU_ADDRESS << 1) | 0))
  {
    mpuI2CStop();
    return false;
  }

  if (!mpuI2CWriteByte(reg))
  {
    mpuI2CStop();
    return false;
  }

  mpuI2CStart();

  if (!mpuI2CWriteByte((MPU_ADDRESS << 1) | 1))
  {
    mpuI2CStop();
    return false;
  }

  value = mpuI2CReadByte(false);

  mpuI2CStop();

  return true;
}

// ============================================================
// READ MPU6050
// ============================================================

bool readMPU6050()
{
  uint8_t data[14];

  mpuI2CStart();

  if (!mpuI2CWriteByte((MPU_ADDRESS << 1) | 0))
  {
    mpuI2CStop();
    return false;
  }

  if (!mpuI2CWriteByte(MPU_ACCEL_XOUT_H))
  {
    mpuI2CStop();
    return false;
  }

  mpuI2CStart();

  if (!mpuI2CWriteByte((MPU_ADDRESS << 1) | 1))
  {
    mpuI2CStop();
    return false;
  }

  for (int i = 0; i < 14; i++)
  {
    data[i] = mpuI2CReadByte(i < 13);
  }

  mpuI2CStop();

  int16_t rawAx =
    ((int16_t)data[0] << 8) | data[1];

  int16_t rawAy =
    ((int16_t)data[2] << 8) | data[3];

  int16_t rawAz =
    ((int16_t)data[4] << 8) | data[5];

  int16_t rawGx =
    ((int16_t)data[8] << 8) | data[9];

  int16_t rawGy =
    ((int16_t)data[10] << 8) | data[11];

  int16_t rawGz =
    ((int16_t)data[12] << 8) | data[13];

  accelX = rawAx / 16384.0;
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  gyroX = rawGx / 131.0;
  gyroY = rawGy / 131.0;
  gyroZ = rawGz / 131.0;

  totalG = sqrt(
    accelX * accelX +
    accelY * accelY +
    accelZ * accelZ
  );

  return true;
}

// ============================================================
// CHECK MPU6050
// ============================================================

bool checkMPU6050()
{
  uint8_t whoAmI = 0;

  if (!mpuReadRegister(MPU_WHO_AM_I, whoAmI))
    return false;

  if (whoAmI != 0x68 && whoAmI != 0x69)
    return false;

  return true;
}

// ============================================================
// CHECK BME280
// ============================================================

bool checkBME280()
{
  uint8_t chipID = bme.sensorID();

  if (
    chipID == 0x60 ||
    chipID == 0x58 ||
    chipID == 0x56 ||
    chipID == 0x57
  )
  {
    return true;
  }

  return false;
}

// ============================================================
// CHECK SD
// ============================================================

bool checkSDStatus()
{
  if (recording)
  {
    if (successfulWrites == 0)
      return false;

    if (
      millis() - lastSuccessfulSDWrite <=
      (RECORD_INTERVAL * 3)
    )
    {
      return true;
    }

    return false;
  }

  if (SD.cardType() == CARD_NONE)
    return false;

  return true;
}

// ============================================================
// CHECK GPS
// ============================================================

bool checkGPSStatus()
{
  if (lastGPSData == 0)
    return false;

  if (millis() - lastGPSData <= 5000)
    return true;

  return false;
}

// ============================================================
// MODULE HEALTH CHECK
// ============================================================

void checkModuleHealth()
{
  // MPU6050
  if (checkMPU6050())
  {
    mpuOK = true;
    lastMPUCommunication = millis();
  }
  else
  {
    mpuOK = false;

    accelX = NAN;
    accelY = NAN;
    accelZ = NAN;

    gyroX = NAN;
    gyroY = NAN;
    gyroZ = NAN;

    totalG = NAN;
  }

  // BME280
  if (checkBME280())
  {
    bmeOK = true;
    lastBMECommunication = millis();
  }
  else
  {
    bmeOK = false;

    temperatureC = NAN;
    pressureHpa = NAN;
    baroAltitudeFt = NAN;
  }

  // GPS
  gpsOK = checkGPSStatus();

  // SD
  sdOK = checkSDStatus();

  Serial.println();
  Serial.println("MODULE HEALTH CHECK");

  Serial.print("OLED: ");
  Serial.println(oledOK ? "OK" : "FAIL");

  Serial.print("MPU6050: ");
  Serial.println(mpuOK ? "OK" : "FAIL");

  Serial.print("BME280: ");
  Serial.println(bmeOK ? "OK" : "FAIL");

  Serial.print("GPS: ");
  Serial.println(gpsOK ? "OK" : "WAIT");

  Serial.print("SD: ");
  Serial.println(sdOK ? "OK" : "FAIL");

  Serial.print("Samples: ");
  Serial.println(sampleCount);
}

// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  // ----------------------------------------------------------
  // BME280
  // ----------------------------------------------------------

  if (bmeOK)
  {
    float newTemperature =
      bme.readTemperature();

    float newPressure =
      bme.readPressure() / 100.0;

    if (
      !isnan(newTemperature) &&
      !isnan(newPressure) &&
      newPressure > 300.0 &&
      newPressure < 1100.0
    )
    {
      temperatureC = newTemperature;
      pressureHpa = newPressure;

      if (!isnan(referencePressureHpa))
      {
        float altitudeMeters =
          44330.0 *
          (
            1.0 -
            pow(
              pressureHpa / referencePressureHpa,
0.19029495
            )
          );

        baroAltitudeFt =
          altitudeMeters * 3.28084;
      }

      lastBMECommunication = millis();
    }
    else
    {
      bmeOK = false;

      temperatureC = NAN;
      pressureHpa = NAN;
      baroAltitudeFt = NAN;
    }
  }

  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  if (mpuOK)
  {
    if (readMPU6050())
    {
      lastMPUCommunication = millis();
    }
    else
    {
      mpuOK = false;

      accelX = NAN;
      accelY = NAN;
      accelZ = NAN;

      gyroX = NAN;
      gyroY = NAN;
      gyroZ = NAN;

      totalG = NAN;
    }
  }

  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  if (gps.location.isValid())
  {
    gpsLatitude = gps.location.lat();
    gpsLongitude = gps.location.lng();

    lastGPSData = millis();
  }

  if (gps.altitude.isValid())
    gpsAltitudeFt = gps.altitude.feet();

  if (gps.speed.isValid())
    gpsSpeedKt = gps.speed.knots();

  if (gps.course.isValid())
    gpsCourse = gps.course.deg();

  if (gps.satellites.isValid())
    gpsSatellites = gps.satellites.value();

  gpsOK = checkGPSStatus();
}

// ============================================================
// BME280 STARTING-PRESSURE CALIBRATION
// ============================================================

void calibrateBME280()
{
  Serial.println();
  Serial.println("==============================");
  Serial.println("BME280 STARTING CALIBRATION");
  Serial.println("==============================");

  calibratingBME = true;

  double pressureSum = 0.0;
  unsigned long validSamples = 0;

  unsigned long calibrationStart =
    millis();

  unsigned long lastCalibrationRead = 0;

  while (
    millis() - calibrationStart <
    BME_CALIBRATION_TIME
  )
  {
    // --------------------------------------------------------
    // Continue decoding GPS during calibration
    // --------------------------------------------------------

    while (GPSserial.available())
    {
      char c = GPSserial.read();
      gps.encode(c);
    }

    // --------------------------------------------------------
    // Try a BME280 reading every 250 ms
    // --------------------------------------------------------

    if (
      millis() - lastCalibrationRead >=
      SENSOR_INTERVAL
    )
    {
      lastCalibrationRead = millis();

      bool gotValidReading = false;

      // Try to communicate with the BME280.
      if (checkBME280())
      {
        bmeOK = true;

        float pressure =
          bme.readPressure() / 100.0;

        if (
          !isnan(pressure) &&
          pressure > 300.0 &&
          pressure < 1100.0
        )
        {
          pressureSum += pressure;
          validSamples++;

          pressureHpa = pressure;

          lastBMECommunication = millis();

          gotValidReading = true;
        }
      }

      // If the BME failed this particular attempt,
      // do NOT stop calibration.
      if (!gotValidReading)
      {
        bmeOK = false;

        Serial.println(
          "BME280 calibration reading unavailable"
        );
      }

      // ------------------------------------------------------
      // Progress screen
      // ------------------------------------------------------

      unsigned long elapsed =
        millis() - calibrationStart;

      unsigned long seconds =
        elapsed / 1000;

      display.clearDisplay();

      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);

      display.setCursor(0, 0);
      display.println("CALIBRATING ALT");

      display.setCursor(0, 14);
      display.print("TIME: ");
      display.print(seconds);
      display.println(" / 10");

      display.setCursor(0, 28);
      display.print("VALID: ");
      display.println(validSamples);

      display.setCursor(0, 42);

      if (gotValidReading)
        display.println("BME: OK");
      else
        display.println("BME: WAIT");

      display.setCursor(0, 54);
      display.println("KEEP STATIONARY");

      display.display();
    }

    delay(1);
  }

  calibratingBME = false;

  // ==========================================================
  // DETERMINE REFERENCE PRESSURE
  // ==========================================================

  if (validSamples > 0)
  {
    // Use the average of every valid reading obtained.
    referencePressureHpa =
      pressureSum / validSamples;

    Serial.println();
    Serial.print("VALID BME SAMPLES: ");
    Serial.println(validSamples);

    Serial.print("AVERAGE REFERENCE PRESSURE: ");
    Serial.print(referencePressureHpa, 3);
    Serial.println(" hPa");
  }
  else
  {
    // --------------------------------------------------------
    // BME COMPLETELY UNAVAILABLE
    // --------------------------------------------------------
    // The recorder MUST still start.
    // Use standard atmospheric pressure as fallback.
    // --------------------------------------------------------

    referencePressureHpa =
      DEFAULT_REFERENCE_PRESSURE;

    Serial.println();
    Serial.println(
      "NO VALID BME280 READINGS"
    );

    Serial.print(
      "USING FALLBACK REFERENCE: "
    );

    Serial.print(
      DEFAULT_REFERENCE_PRESSURE,
2
    );

    Serial.println(" hPa");
  }

  // At the starting reference point,
  // relative barometric altitude is defined as 0.
  baroAltitudeFt = 0.0;

  Serial.print("REFERENCE PRESSURE: ");
  Serial.print(referencePressureHpa, 3);
  Serial.println(" hPa");

  Serial.println("ALTITUDE REFERENCE SET");
  Serial.println("==============================");
}

// ============================================================
// CREATE NEW FLIGHT FILE
// ============================================================

bool createNewFlightFile()
{
  for (int i = 1; i <= 999; i++)
  {
    snprintf(
      currentFileName,
      sizeof(currentFileName),
      "/FLIGHT%03d.CSV",
      i
    );

    if (!SD.exists(currentFileName))
    {
      flightFile =
        SD.open(
          currentFileName,
          FILE_WRITE
        );

      if (!flightFile)
        return false;

      flightFile.println(
        "TIME_S,GPS_LAT,GPS_LON,GPS_ALT_FT,GPS_SAT,GPS_SPEED_KT,GPS_COURSE,BME_TEMP_C,PRESSURE_HPA,BARO_ALT_FT,ACCEL_X_G,ACCEL_Y_G,ACCEL_Z_G,TOTAL_G,GYRO_X_DPS,GYRO_Y_DPS,GYRO_Z_DPS"
      );

      flightFile.flush();

      return true;
    }
  }

  return false;
}

// ============================================================
// START SCREEN
// ============================================================

void showStartingScreen()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("STARTING FLIGHT");

  display.setCursor(0, 20);
  display.println("PREPARING...");

  display.setCursor(0, 36);
  display.println("PLEASE WAIT");

  display.display();
}

// ============================================================
// STOP SCREEN
// ============================================================

void showStoppingScreen()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("STOPPING FLIGHT");

  display.setCursor(0, 20);
  display.println("SAVING DATA...");

  display.setCursor(0, 36);
  display.println("CLOSING FILE...");

  display.setCursor(0, 52);
  display.println("PLEASE WAIT");

  display.display();
}

// ============================================================
// FINAL SCREEN
// ============================================================

void showFinalScreen()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("FLIGHT COMPLETE");

  display.setCursor(0, 20);
  display.println("DATA SAVED");

  display.setCursor(0, 36);
  display.println("FILE CLOSED");

  display.setCursor(0, 52);
  display.println("SAFE TO POWER OFF");

  display.display();
}

// ============================================================
// MODULE STATUS SCREEN
// ============================================================

void showModuleStatus()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("MODULE STATUS");

  display.setCursor(0, 12);
  display.print("OLED: ");
  display.println(oledOK ? "OK" : "FAIL");

  display.setCursor(0, 24);
  display.print("MPU6050: ");
  display.println(mpuOK ? "OK" : "FAIL");

  display.setCursor(0, 36);
  display.print("BME280: ");
  display.println(bmeOK ? "OK" : "FAIL");

  display.setCursor(0, 48);
  display.print("SD: ");
  display.println(sdOK ? "OK" : "FAIL");

  display.setCursor(68, 12);
  display.print("GPS: ");
  display.println(gpsOK ? "OK" : "WAIT");

  display.display();
}

// ============================================================
// RECORDING TIME
// ============================================================

void printRecordingTime()
{
  unsigned long elapsed =
    (millis() - recordingStartTime) / 1000;

  unsigned long hours =
    elapsed / 3600;

  unsigned long minutes =
    (elapsed % 3600) / 60;

  unsigned long seconds =
    elapsed % 60;

  if (hours < 10)
    display.print("0");

  display.print(hours);
  display.print(":");

  if (minutes < 10)
    display.print("0");

  display.print(minutes);
  display.print(":");

  if (seconds < 10)
    display.print("0");

  display.print(seconds);
}

// ============================================================
// RECORDING STATUS SCREEN
// ============================================================

void showRecordingStatus()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("RECORDING STATUS");

  display.setCursor(0, 14);
  display.print("RECORDING: ");

  if (recording)
    display.println("YES");
  else
    display.println("NO");

  display.setCursor(0, 28);
  display.print("TIME: ");

  if (recording)
    printRecordingTime();
  else
    display.println("00:00:00");

  display.setCursor(0, 42);
  display.print("SD: ");

  if (sdOK)
    display.println("SAVING");
  else
    display.println("ERROR");

  display.setCursor(0, 56);
  display.print("SAMPLES: ");
  display.print(sampleCount);

  display.display();
}

// ============================================================
// FLIGHT DATA SCREEN
// ============================================================

void showFlightData()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("FLIGHT DATA");

  display.setCursor(0, 12);
  display.print("ALT: ");

  if (!isnan(gpsAltitudeFt))
  {
    display.print(gpsAltitudeFt, 0);
    display.println(" FT");
  }
  else
  {
    display.println("--- FT");
  }

  display.setCursor(0, 24);
  display.print("SPD: ");

  if (!isnan(gpsSpeedKt))
  {
    display.print(gpsSpeedKt, 0);
    display.println(" KT");
  }
  else
  {
    display.println("--- KT");
  }

  display.setCursor(0, 36);
  display.print("GPS: ");
  display.print(gpsSatellites);
  display.println(" SAT");

  display.setCursor(0, 48);
  display.print("G: ");

  if (!isnan(totalG))
    display.println(totalG, 2);
  else
    display.println("---");

  display.display();
}

// ============================================================
// SHOW CURRENT SCREEN
// ============================================================

void showCurrentScreen()
{
  if (currentScreen == 0)
    showModuleStatus();

  else if (currentScreen == 1)
    showRecordingStatus();

  else
    showFlightData();
}

// ============================================================
// WRITE FLIGHT DATA
// ============================================================

bool writeFlightData()
{
  if (!recording)
    return false;

  if (!flightFile)
    return false;

  unsigned long elapsedMs =
    millis() - recordingStartTime;

  float timeSeconds =
    elapsedMs / 1000.0;

  // TIME
  flightFile.print(timeSeconds, 2);
  flightFile.print(",");

  // GPS LATITUDE
  if (!isnan(gpsLatitude))
    flightFile.print(gpsLatitude, 6);
  flightFile.print(",");

  // GPS LONGITUDE
  if (!isnan(gpsLongitude))
    flightFile.print(gpsLongitude, 6);
  flightFile.print(",");

  // GPS ALTITUDE
  if (!isnan(gpsAltitudeFt))
    flightFile.print(gpsAltitudeFt, 2);
  flightFile.print(",");

  // GPS SATELLITES
  if (gps.satellites.isValid())
    flightFile.print(gpsSatellites);
  flightFile.print(",");

  // GPS SPEED
  if (!isnan(gpsSpeedKt))
    flightFile.print(gpsSpeedKt, 2);
  flightFile.print(",");

  // GPS COURSE
  if (!isnan(gpsCourse))
    flightFile.print(gpsCourse, 2);
  flightFile.print(",");

  // TEMPERATURE
  if (!isnan(temperatureC))
    flightFile.print(temperatureC, 2);
  flightFile.print(",");

  // RAW PRESSURE
  if (!isnan(pressureHpa))
    flightFile.print(pressureHpa, 2);
  flightFile.print(",");

  // RELATIVE BAROMETRIC ALTITUDE
  if (!isnan(baroAltitudeFt))
    flightFile.print(baroAltitudeFt, 2);
  flightFile.print(",");

  // ACCELERATION
  if (!isnan(accelX))
    flightFile.print(accelX, 3);
  flightFile.print(",");

  if (!isnan(accelY))
    flightFile.print(accelY, 3);
  flightFile.print(",");

  if (!isnan(accelZ))
    flightFile.print(accelZ, 3);
  flightFile.print(",");

  // TOTAL G
  if (!isnan(totalG))
    flightFile.print(totalG, 3);
  flightFile.print(",");

  // GYROSCOPE
  if (!isnan(gyroX))
    flightFile.print(gyroX, 3);
  flightFile.print(",");

  if (!isnan(gyroY))
    flightFile.print(gyroY, 3);
  flightFile.print(",");

  if (!isnan(gyroZ))
    flightFile.print(gyroZ, 3);

  flightFile.println();

  // Ensure the new data is written to the SD card.
  flightFile.flush();

  if (!flightFile)
  {
    failedWrites++;
    sdOK = false;

    return false;
  }

  successfulWrites++;

  lastSuccessfulSDWrite =
    millis();

  sdOK = true;

  sampleCount++;

  return true;
}

// ============================================================
// START RECORDING
// ============================================================

void startRecording()
{
  if (recording || flightComplete)
    return;

  Serial.println();
  Serial.println("==============================");
  Serial.println("STARTING FLIGHT");
  Serial.println("==============================");

  showStartingScreen();

  delay(500);

  // ----------------------------------------------------------
  // SD CARD MUST BE AVAILABLE
  // ----------------------------------------------------------

  if (SD.cardType() == CARD_NONE)
  {
    sdOK = false;

    Serial.println("SD CARD NOT AVAILABLE");

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("START FAILED");

    display.setCursor(0, 20);
    display.println("SD CARD ERROR");

    display.setCursor(0, 40);
    display.println("CHECK SD CARD");

    display.display();

    delay(2000);

    showModuleStatus();

    return;
  }

  sdOK = true;

  // ----------------------------------------------------------
  // BME280 CALIBRATION
  // ----------------------------------------------------------
  //
  // IMPORTANT:
  // This function NEVER prevents recording from starting.
  //
  // If readings are available:
  //     average them.
  //
  // If no readings are available:
  //     use 1013.25 hPa.
  //
  // ----------------------------------------------------------

  calibrateBME280();

  // ----------------------------------------------------------
  // CREATE FLIGHT FILE
  // ----------------------------------------------------------

  if (!createNewFlightFile())
  {
    sdOK = false;

    Serial.println("FILE CREATION FAILED");

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("START FAILED");

    display.setCursor(0, 20);
    display.println("FILE ERROR");

    display.setCursor(0, 40);
    display.println("CHECK SD CARD");

    display.display();

    delay(2000);

    showModuleStatus();

    return;
  }

  // ----------------------------------------------------------
  // RESET RECORDING STATISTICS
  // ----------------------------------------------------------

  sampleCount = 0;
  successfulWrites = 0;
  failedWrites = 0;

  lastSuccessfulSDWrite =
    millis();

  // ----------------------------------------------------------
  // START RECORDING
  // ----------------------------------------------------------

  recordingStartTime =
    millis();

  recording = true;

  currentScreen = 1;

  lastScreenChange =
    millis();

  lastRecord =
    millis();

  Serial.println();
  Serial.println("==============================");
  Serial.println("RECORDING STARTED");
  Serial.println("==============================");

  Serial.print("FILE: ");
  Serial.println(currentFileName);

  Serial.print("REFERENCE PRESSURE: ");
  Serial.print(referencePressureHpa, 3);
  Serial.println(" hPa");

  if (bmeOK)
    Serial.println("BME STATUS: AVAILABLE");
  else
    Serial.println("BME STATUS: UNAVAILABLE");

  delay(500);

  showRecordingStatus();
}

// ============================================================
// STOP RECORDING
// ============================================================

void stopRecording()
{
  if (!recording)
    return;

  Serial.println();
  Serial.println("==============================");
  Serial.println("STOPPING FLIGHT");
  Serial.println("==============================");

  showStoppingScreen();

  recording = false;

  // ----------------------------------------------------------
  // FINAL SENSOR READ
  // ----------------------------------------------------------

  readSensors();

  // ----------------------------------------------------------
  // FINAL CSV SAMPLE
  // ----------------------------------------------------------

  if (flightFile)
  {
    unsigned long elapsedMs =
      millis() - recordingStartTime;

    float timeSeconds =
      elapsedMs / 1000.0;

    flightFile.print(timeSeconds, 2);
    flightFile.print(",");

    if (!isnan(gpsLatitude))
      flightFile.print(gpsLatitude, 6);
    flightFile.print(",");

    if (!isnan(gpsLongitude))
      flightFile.print(gpsLongitude, 6);
    flightFile.print(",");

    if (!isnan(gpsAltitudeFt))
      flightFile.print(gpsAltitudeFt, 2);
    flightFile.print(",");

    if (gps.satellites.isValid())
      flightFile.print(gpsSatellites);
    flightFile.print(",");

    if (!isnan(gpsSpeedKt))
      flightFile.print(gpsSpeedKt, 2);
    flightFile.print(",");

    if (!isnan(gpsCourse))
      flightFile.print(gpsCourse, 2);
    flightFile.print(",");

    if (!isnan(temperatureC))
      flightFile.print(temperatureC, 2);
    flightFile.print(",");

    if (!isnan(pressureHpa))
      flightFile.print(pressureHpa, 2);
    flightFile.print(",");

    if (!isnan(baroAltitudeFt))
      flightFile.print(baroAltitudeFt, 2);
    flightFile.print(",");

    if (!isnan(accelX))
      flightFile.print(accelX, 3);
    flightFile.print(",");

    if (!isnan(accelY))
      flightFile.print(accelY, 3);
    flightFile.print(",");

    if (!isnan(accelZ))
      flightFile.print(accelZ, 3);
    flightFile.print(",");

    if (!isnan(totalG))
      flightFile.print(totalG, 3);
    flightFile.print(",");

    if (!isnan(gyroX))
      flightFile.print(gyroX, 3);
    flightFile.print(",");

    if (!isnan(gyroY))
      flightFile.print(gyroY, 3);
    flightFile.print(",");

    if (!isnan(gyroZ))
      flightFile.print(gyroZ, 3);

    flightFile.println();

    flightFile.flush();

    delay(300);

    flightFile.close();

    delay(300);
  }

  Serial.println("CSV FILE CLOSED");
  Serial.println("DATA SAVED");
  Serial.println("FLIGHT COMPLETE");
  Serial.println("SAFE TO POWER OFF");

  flightComplete = true;

  showFinalScreen();

  // ----------------------------------------------------------
  // KEEP FINAL SCREEN ON UNTIL POWER IS REMOVED
  // ----------------------------------------------------------

  while (true)
  {
    delay(1000);
  }
}

// ============================================================
// BUTTON
// ============================================================

void checkButton()
{
  if (flightComplete)
    return;

  bool reading =
    digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading)
  {
    lastButtonChange =
      millis();
  }

  if (
    millis() - lastButtonChange >
    BUTTON_DEBOUNCE
  )
  {
    if (reading != stableButtonState)
    {
      stableButtonState =
        reading;

      if (stableButtonState == LOW)
      {
        if (!recording)
          startRecording();
        else
          stopRecording();
      }
    }
  }

  lastButtonReading =
    reading;
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("FLIGHT RECORDER");
  Serial.println("==============================");

  // ----------------------------------------------------------
  // BUTTON
  // ----------------------------------------------------------

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );

  Serial.println("BUTTON: GPIO 27");
  Serial.println("BUTTON GND: GND");

  // ----------------------------------------------------------
  // OLED I2C
  // ----------------------------------------------------------

  OLEDWire.begin(
    OLED_SDA,
    OLED_SCL
  );

  // ----------------------------------------------------------
  // BME280 I2C
  // ----------------------------------------------------------

  BMEWire.begin(
    BME_SDA,
    BME_SCL
  );

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  Serial.println("Starting OLED...");

  if (
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDRESS
    )
  )
  {
    oledOK = true;

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("FLIGHT RECORDER");

    display.setCursor(0, 20);
    display.println("INITIALISING...");

    display.display();

    Serial.println("OLED: OK");
  }
  else
  {
    oledOK = false;

    Serial.println("OLED: FAIL");
  }

  delay(500);

  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  pinMode(
    MPU_SDA,
    INPUT_PULLUP
  );

  pinMode(
    MPU_SCL,
    INPUT_PULLUP
  );

  delay(20);

  uint8_t whoAmI = 0;

  if (
    mpuReadRegister(
      MPU_WHO_AM_I,
      whoAmI
    )
  )
  {
    if (
      whoAmI == 0x68 ||
      whoAmI == 0x69
    )
    {
      mpuOK = true;

      mpuWriteRegister(
        MPU_PWR_MGMT_1,
        0x00
      );

      Serial.println("MPU6050: OK");
    }
    else
    {
      mpuOK = false;

      Serial.println("MPU6050: FAIL");
    }
  }
  else
  {
    mpuOK = false;

    Serial.println("MPU6050: FAIL");
  }

  // ----------------------------------------------------------
  // BME280
  // ----------------------------------------------------------

  if (
    bme.begin(
      0x76,
      &BMEWire
    )
  )
  {
    bmeOK = true;

    Serial.println("BME280: OK");
  }
  else if (
    bme.begin(
      0x77,
      &BMEWire
    )
  )
  {
    bmeOK = true;

    Serial.println("BME280: OK");
  }
  else
  {
    bmeOK = false;

    Serial.println("BME280: FAIL");
  }

  // ----------------------------------------------------------
  // SD CARD
  // ----------------------------------------------------------

  SPI.begin(
    SD_SCK,
    SD_MISO,
    SD_MOSI,
    SD_CS
  );

  if (
    SD.begin(
      SD_CS,
      SPI
    )
  )
  {
    if (
      SD.cardType() != CARD_NONE
    )
    {
      sdOK = true;

      Serial.println("SD CARD: OK");
    }
    else
    {
      sdOK = false;

      Serial.println("SD CARD: FAIL");
    }
  }
  else
  {
    sdOK = false;

    Serial.println("SD CARD: FAIL");
  }

  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  GPSserial.begin(
    9600,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  gpsOK = false;

  Serial.println(
    "GPS: WAITING FOR VALID FIX"
  );

  // ----------------------------------------------------------
  // INITIAL DISPLAY
  // ----------------------------------------------------------

  currentScreen = 0;

  showModuleStatus();

  lastScreenChange =
    millis();

  lastHealthCheck =
    millis();

  Serial.println();
  Serial.println("==============================");
  Serial.println("SYSTEM READY");
  Serial.println("RECORDING: NO");
  Serial.println("PRESS BUTTON TO START");
  Serial.println("==============================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  while (GPSserial.available())
  {
    char c =
      GPSserial.read();

    gps.encode(c);
  }

  if (gps.location.isValid())
  {
    lastGPSData =
      millis();
  }

  gpsOK =
    checkGPSStatus();

  // ----------------------------------------------------------
  // BUTTON
  // ----------------------------------------------------------

  checkButton();

  // ----------------------------------------------------------
  // SENSOR READS
  // ----------------------------------------------------------

  if (
    millis() - lastSensorRead >=
    SENSOR_INTERVAL
  )
  {
    lastSensorRead =
      millis();

    readSensors();
  }

  // ----------------------------------------------------------
  // HEALTH CHECK EVERY 2 SECONDS
  // ----------------------------------------------------------

  if (
    millis() - lastHealthCheck >=
    HEALTH_INTERVAL
  )
  {
    lastHealthCheck =
      millis();

    checkModuleHealth();

    if (
      !flightComplete &&
      currentScreen == 0 &&
      !calibratingBME
    )
    {
      showModuleStatus();
    }
  }

  // ----------------------------------------------------------
  // RECORD EVERY SECOND
  // ----------------------------------------------------------

  if (
    recording &&
    millis() - lastRecord >=
    RECORD_INTERVAL
  )
  {
    lastRecord =
      millis();

    if (writeFlightData())
    {
      Serial.println("DATA SAVED");
    }
    else
    {
      Serial.println("SD WRITE FAILED");
    }
  }

  // ----------------------------------------------------------
  // CYCLE THREE NORMAL SCREENS
  // ----------------------------------------------------------

  if (
    !flightComplete &&
    !calibratingBME &&
    millis() - lastScreenChange >=
    SCREEN_INTERVAL
  )
  {
    lastScreenChange =
      millis();

    currentScreen++;

    if (currentScreen > 2)
      currentScreen = 0;

    showCurrentScreen();
  }
}
