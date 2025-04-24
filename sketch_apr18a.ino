#include <Wire.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_BME680.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_MICS.h"
 
#define CALIBRATION_TIME   2    // Default calibration time is three minutes
#define ADC_PIN            20  // Analog pin connected to the sensor's analog output
#define POWER_PIN          12    // Digital pin to power the sensor
DFRobot_MICS_ADC mics(ADC_PIN, POWER_PIN);
//#define MICS5524_PIN 4  // ESP32 analog pin

Adafruit_SCD30 scd30;
Adafruit_BME680 bme;
Adafruit_ADS1115 ads; 

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init SCD30
  if (!scd30.begin()) {
    Serial.println("Couldn't find SCD30");
    while (1);
  }

  // Init BME680
  if (!bme.begin()) {
    Serial.println("Couldn't find BME680");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // Temperature in °C, duration in ms

  // Init ADS1115
  ads.begin();

  //init mics2254
   // Attempt to initialize the sensor
  while (!mics.begin()) {
    Serial.println("No devices found! Please check the connections.");
    delay(1000);
  }
// Check the current power state of the sensor
  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println("Sensor woken up successfully!");
  } else {
    Serial.println("Sensor is already in wake-up mode.");
  }
 
  // Wait for the sensor to complete its warm-up time
  while (!mics.warmUpTime(CALIBRATION_TIME)) {
    Serial.println("Please wait until the warm-up time is over!");
    delay(1000);
  }
  Serial.println("mics Sensor is ready for use.");

  Serial.println("Sensor initialization complete.\n");
}

void loop() {
  Serial.println("........ Sensor Readings ........");

  // SCD30
  if (scd30.dataReady()) {
    if (scd30.read()) {
      Serial.print("SCD30 CO2 (ppm): ");
      Serial.println(scd30.CO2, 2);
      Serial.print("SCD30 Temp (°C): ");
      Serial.println(scd30.temperature, 2);
      Serial.print("SCD30 Humidity (%): ");
      Serial.println(scd30.relative_humidity, 2);
    }
  }

  // BME680
  if (bme.performReading()) {
    Serial.print("BME680 Temp (°C): ");
    Serial.println(bme.temperature, 2);
    Serial.print("BME680 Pressure (hPa): ");
    Serial.println(bme.pressure / 100.0, 2);
    Serial.print("BME680 Humidity (%): ");
    Serial.println(bme.humidity, 2);
    Serial.print("BME680 Gas Resistance (KΩ): ");
    Serial.println(bme.gas_resistance / 1000.0, 2);
  }

  // MICS5524 
  int micsValue = analogRead(ADC_PIN);
  Serial.print("MICS5524 (Raw ADC): ");
  Serial.println(micsValue);

 // Read gas data from the sensor
  float coConcentration = mics.getGasData(CO);
  float ch4Concentration = mics.getGasData(CH4);
  float c2h5ohConcentration = mics.getGasData(C2H5OH);
  float h2Concentration = mics.getGasData(H2);
  float nh3Concentration = mics.getGasData(NH3);
  float no2Concentration = mics.getGasData(NO2);
 
  // Print the gas concentrations to the serial monitor
  Serial.print("CO (Carbon Monoxide): ");
  Serial.print(coConcentration, 1);
  Serial.println(" PPM");
 
  Serial.print("CH4 (Methane): ");
  Serial.print(ch4Concentration, 1);
  Serial.println(" PPM");
 
  Serial.print("C2H5OH (Ethanol): ");
  Serial.print(c2h5ohConcentration, 1);
  Serial.println(" PPM");
 
  Serial.print("H2 (Hydrogen): ");
  Serial.print(h2Concentration, 1);
  Serial.println(" PPM");
 
  Serial.print("NH3 (Ammonia): ");
  Serial.print(nh3Concentration, 1);
  Serial.println(" PPM");
 
  Serial.print("NO2 (Nitrogen Dioxide): ");
  Serial.print(no2Concentration, 1);
  Serial.println(" PPM");
  Serial.println();
  // TGS2611 via ADS1115
  int16_t tgsValue = ads.readADC_SingleEnded(0);
  Serial.print("TGS2611 via ADS1115 (Raw ADC): ");
  Serial.println(tgsValue);

  Serial.println("............................\n");

  delay(2000);
}
