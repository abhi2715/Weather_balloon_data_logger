#include <DHT.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // Include the Wire library for I2C communication
#include <Adafruit_BMP085.h> // Include the BMP180 library

#define ONE_WIRE_BUS 5    // DS18B20 data pin
#define MQ135_PIN A3      // MQ135 analog pin  
#define SD_CS_PIN 6       // SD card CS pin
#define DHT_PIN 4         // Digital pin for DHT11
#define UV_PIN A0
#define DHTTYPE DHT11     // DHT 11
#define SD_STATUS_LED 10  // New LED pin for SD card status

#define RL_VALUE 10000.0  // Replace with the measured RL value in ohms
#define RZERO_CO2 76.63    // Calibrated RZERO value for MQ135 for CO2
#define ATMOSPHERIC_CO2 400 // PPM of CO2 in the atmosphere
#define CALIBRATION_CO2 400 // PPM of CO2 during calibration

// Conversion factors for estimating other gases based on CO2 readings
#define NH3_CO2_FACTOR 0.2   // Conversion factor for NH3
#define C6H6_CO2_FACTOR 0.5  // Conversion factor for C6H6
#define C2H5OH_CO2_FACTOR 0.4 // Conversion factor for C2H5OH
#define CO_CO2_FACTOR 0.7     // Conversion factor for CO
#define SMOKE_CO2_FACTOR 0.6  // Conversion factor for Smoke

const int LOG_INTERVAL = 2000; // Logging interval in milliseconds
unsigned long previousMillis = 0;
File dataFile;

// Temperature sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Setup DHT11
DHT dht(DHT_PIN, DHTTYPE);

// Define LED pins
const int LED_TEMPERATURE = 8; // Change to pin 8
const int LED_AIR_QUALITY = 2;
const int LED_HUMIDITY = 3;
const int LED_UV = 7;
const int LED_PRESSURE = 9; // New LED pin for pressure

// Initialize the BMP180 sensor
Adafruit_BMP085 bmp;
bool bmpSensorWorking = false;

void setup() {
  Serial.begin(9600);
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(LED_TEMPERATURE, OUTPUT);
  pinMode(LED_AIR_QUALITY, OUTPUT);
  pinMode(LED_HUMIDITY, OUTPUT);
  pinMode(LED_UV, OUTPUT);
  pinMode(LED_PRESSURE, OUTPUT); // Set LED pin for pressure
  pinMode(SD_STATUS_LED, OUTPUT); // Set LED pin for SD card status

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD initialization failed!");
    while (1) {} // Halt the program if SD card initialization fails
  }
  Serial.println("SD initialization done.");

  // Start temperature sensor
  sensors.begin();

  // Start DHT11 sensor
  dht.begin();

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 or BMP085 sensor!");
  } else {
    bmpSensorWorking = true;
  }

  Wire.begin(); // Start I2C communication
}

void loop() {
  unsigned long currentMillis = millis();

  // Log data at regular intervals
  if (currentMillis - previousMillis >= LOG_INTERVAL) {
    previousMillis = currentMillis;

    // Read temperature from DS18B20 sensor
    sensors.requestTemperatures();
    float temperature = sensors.getTempCByIndex(0);

    // Read air quality from MQ135 sensor
    int airQuality = analogRead(MQ135_PIN);
    
    // Convert sensor reading to resistance (adjust based on your circuit)
    float resistance = (float)(1023 - airQuality) * RL_VALUE / airQuality;

    // Calculate Carbon Concentration (PPM of CO2)
    float carbonConcentration = ATMOSPHERIC_CO2 * pow((resistance / RZERO_CO2), -0.35);

    // Use CO2 readings to estimate other gases
    float nh3Concentration = carbonConcentration * NH3_CO2_FACTOR;
    float c6h6Concentration = carbonConcentration * C6H6_CO2_FACTOR;
    float c2h5ohConcentration = carbonConcentration * C2H5OH_CO2_FACTOR;
    float coConcentration = carbonConcentration * CO_CO2_FACTOR;
    float smokeConcentration = carbonConcentration * SMOKE_CO2_FACTOR;

    // Open the file in append mode
    dataFile = SD.open("data.csv", FILE_WRITE);

    // Write to the file
    if (dataFile) {
      // Write sensor data to the file
      dataFile.print(carbonConcentration);
      dataFile.print(",");
      dataFile.print(nh3Concentration);
      dataFile.print(",");
      dataFile.print(c6h6Concentration);
      dataFile.print(",");
      dataFile.print(c2h5ohConcentration);
      dataFile.print(",");
      dataFile.print(coConcentration);
      dataFile.print(",");
      dataFile.println(smokeConcentration);

      // Flush the data to ensure it's written immediately
      dataFile.flush();

      // Close the file
      dataFile.close();
    } else {
      Serial.println("Error opening file.");
      digitalWrite(SD_STATUS_LED, LOW); // Turn off SD status LED if SD card error occurs
      delay(1000); // Delay to avoid flickering
    }

    // Turn on LEDs for visual feedback
    digitalWrite(LED_TEMPERATURE, HIGH);
    delay(500); // Delay for LED_TEMPERATURE
    digitalWrite(LED_AIR_QUALITY, HIGH);
    delay(500); // Delay for LED_AIR_QUALITY
    digitalWrite(LED_HUMIDITY, HIGH);
    delay(500); // Delay for LED_HUMIDITY
    digitalWrite(LED_UV, HIGH);
    delay(500); // Delay for LED_UV
    digitalWrite(LED_PRESSURE, HIGH);
    delay(500); // Delay for LED_PRESSURE

    // Turn off LEDs
    digitalWrite(LED_TEMPERATURE, LOW);
    digitalWrite(LED_AIR_QUALITY, LOW);
    digitalWrite(LED_HUMIDITY, LOW);
    digitalWrite(LED_UV, LOW);
    digitalWrite(LED_PRESSURE, LOW);
    delay(1000);

    // Turn on SD status LED
    digitalWrite(SD_STATUS_LED, HIGH);

    // Delay for logging interval
    delay(LOG_INTERVAL);
  }
}