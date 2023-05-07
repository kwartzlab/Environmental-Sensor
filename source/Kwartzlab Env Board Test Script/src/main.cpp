/* Test script to test the Kwartzlab Environmental Sensor Board */

// Neo Pixel
#define STATUS_LED 0
#define TOTAL_LEDS 1

// GPIO Pins
#define RELAY1 1
#define RELAY2 2
#define GPIO3 3
#define GPIO4 4
#define GPIO5 5
#define I2C_SDA 6
#define I2C_SCL 7
#define GPIO8 8
#define PROGRAM_BUTTON 9
#define GPIO10 10

// I2C Addresses
#define SEGMENT_DISPLAY_1 0x71
#define SEGMENT_DISPLAY_2 0x72
#define BME688_SENSOR 0x76
#define SHTC3_SENSOR 0x70
#define SPS30_SENSOR 0x69
#define ZIO_LOUDNESS_SENSOR 0x38

#define BIAS_LEVEL 400

// Includes
#include <Wire.h>
#include <i2cdetect.h>

#include <Ticker.h>
Ticker checkBME688Sensor;
Ticker checkSPS30Sensor;
Ticker checkSHTC3Sensor;
Ticker checkZioSensor;
Ticker toggleGPIO;

#include <SparkFun_Alphanumeric_Display.h>
HT16K33 segmentDisplay;

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel statusLed(TOTAL_LEDS, STATUS_LED, NEO_GRB + NEO_KHZ800);

#include <bsec2.h>
Bsec2 bme688Sensor;

#include <sps30.h>

#include <Adafruit_SHTC3.h>
Adafruit_SHTC3 shtc3Sensor = Adafruit_SHTC3();

#include <zio.h>
ZIO zioSensor;

//  Variables
uint32_t chipId = 0;
uint32_t currentScreen = 0;

// I2C Device Status
bool statusLed_output = false;
bool displayEnabled = false;
bool bme688SensorEnabled = false;
bool shtc3SensorEnabled = false;
bool sps30SensorEnabled = false;
bool zioLoudnessSensorEnabled = false;

bool gpioOutput = false;

void IRAM_ATTR buttonPress() {
  int buttonVal = digitalRead(PROGRAM_BUTTON);
  Serial.print("Button changed (");
  Serial.print(buttonVal);
  Serial.println(")");
}

void scanI2CBus() {
  Serial.println("Scanning I2C address range from 0x03 to 0x77\n");
  i2cdetect();
}

void checkBsecStatus(Bsec2 bsec) {
    if (bsec.status < BSEC_OK) {
      Serial.println("BSEC error code : " + String(bsec.status));
    } else if (bsec.status > BSEC_OK) {
      Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK) {
      Serial.println("BME68X error code : " + String(bsec.sensor.status));
    } else if (bsec.sensor.status > BME68X_OK) {
      Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

float sensor_bme688_iaq;
float sensor_bme688_co2;
float sensor_bme688_voc;
/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void bme688SensorDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  Serial.println(">>> Data input...");

  if (!outputs.nOutputs) {
      return;
  }

  Serial.println("BSEC outputs:\ntimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData output  = outputs.output[i];
    switch (output.sensor_id) {
      case BSEC_OUTPUT_IAQ:
        sensor_bme688_iaq = output.signal;
        Serial.println("iaq = " + String(output.signal));
        Serial.println("iaq accuracy = " + String((int) output.accuracy));
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        Serial.println("temperature = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        Serial.println("pressure = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
        Serial.println("humidity = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_GAS:
        Serial.println("gas resistance = " + String(output.signal));
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        Serial.println("stabilization status = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        Serial.println("run in status = " + String(output.signal));
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        sensor_bme688_voc = output.signal;
        Serial.println("breath voc equivalent = " + String(output.signal));
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        sensor_bme688_co2 = output.signal;
        Serial.println("co2 equivalent = " + String(output.signal));
      default:
        break;
    }
  }
}

void checkBME688SensorTick() {
  // BME688
  if (!bme688Sensor.run()) {
    checkBsecStatus(bme688Sensor);
  }
}

float sensor_sps30_pm25;
float sensor_sps30_particle_size;
void checkSPS30SensorTick() {
  // SPS30
  struct sps30_measurement m;
  char serial[SPS30_MAX_SERIAL_LEN];
  uint16_t data_ready;
  int16_t ret;

  do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.print("error reading data-ready flag: ");
      Serial.println(ret);
    } else if (!data_ready)
      Serial.print("data not ready, no new measurement available\n");
    else
      break;
    delay(100); /* retry in 100ms */
  } while (1);

  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("error reading measurement\n");
  } else {
    sensor_sps30_pm25 = m.mc_2p5;
    sensor_sps30_particle_size = m.typical_particle_size;

    Serial.print("PM  1.0: ");
    Serial.println(m.mc_1p0);
    Serial.print("PM  2.5: ");
    Serial.println(m.mc_2p5);
    Serial.print("PM  4.0: ");
    Serial.println(m.mc_4p0);
    Serial.print("PM 10.0: ");
    Serial.println(m.mc_10p0);

    Serial.print("NC  0.5: ");
    Serial.println(m.nc_0p5);
    Serial.print("NC  1.0: ");
    Serial.println(m.nc_1p0);
    Serial.print("NC  2.5: ");
    Serial.println(m.nc_2p5);
    Serial.print("NC  4.0: ");
    Serial.println(m.nc_4p0);
    Serial.print("NC 10.0: ");
    Serial.println(m.nc_10p0);

    Serial.print("Typical partical size: ");
    Serial.println(m.typical_particle_size);
  }
}


float sensor_shtc3_temperature;
float sensor_shtc3_humidity;
void checkSHTC3SensorTick() {
  sensors_event_t humidity, temp;
  shtc3Sensor.getEvent(&humidity, &temp);
  sensor_shtc3_temperature = temp.temperature;
  sensor_shtc3_humidity = humidity.relative_humidity;
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
}

float sensor_zio_db;
void checkZioSensorTick() {
  zioSensor.ledOn();
  delay(100);

  int sumSquare;
  for (int i = 0; i < 100; i++) {
    int capture_ac = zioSensor.getValue() - BIAS_LEVEL;
    sumSquare += capture_ac * capture_ac;
  }
  int rms = sqrt(sumSquare / 100);

  sensor_zio_db = rms;
  Serial.print("ADC_Value:  ");
  Serial.println(rms,DEC);
  zioSensor.ledOff();
}

void toggleGPIOTick() {
  Serial.print("Setting GPIO to ");
  Serial.println(gpioOutput ? "HIGH" : "LOW");

  digitalWrite(3, gpioOutput ? HIGH : LOW);
  digitalWrite(4, gpioOutput ? HIGH : LOW);
  digitalWrite(5, gpioOutput ? HIGH : LOW);
  digitalWrite(8, gpioOutput ? HIGH : LOW);
  digitalWrite(10, gpioOutput ? HIGH : LOW);
  if (gpioOutput)
    gpioOutput = false;
  else
    gpioOutput = true;
}

void showDisplayMessage(String message) {
  if (message.length() <= 8) {
    segmentDisplay.print(message);
    delay(1000);
  } else {
    for (int i = 0; i < message.length() - 7; i++) {
      segmentDisplay.print(message.substring(i, i + 8));
      delay(400);
      if (i == 0)
        delay(1000);
    }
  }
}

void setup() {
  // Setup Serial
  Serial.begin(115200);
  while (!Serial) delay(2000);

  // Setup Relay GPIO Pins
  pinMode(RELAY1, OUTPUT); // Relay 1
  pinMode(RELAY2, OUTPUT); // Relay 2

  // Setup Generic GPIO Pins
  pinMode(GPIO3, OUTPUT); // GPIO3
  pinMode(GPIO4, OUTPUT); // GPIO4
  pinMode(GPIO5, OUTPUT); // GPIO5
  pinMode(GPIO8, OUTPUT); // GPIO6
  pinMode(GPIO10, OUTPUT); // GPIO10

  delay(5000);

  // Setup Input GPIO Pin
  pinMode(PROGRAM_BUTTON, INPUT_PULLUP); // Program Button

  // Setup interrupt
  attachInterrupt(9, buttonPress, CHANGE);

  // Setup I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Setup NeoPixel & Clear
  statusLed.begin();
  statusLed.clear();
  statusLed.show();
  statusLed_output = true; // There really isn't anything to check for...

  // Setup SPS30
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;

  sensirion_i2c_init();

  if (sps30_probe() == 0) {
    sps30SensorEnabled = true;

    ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
    if (ret) {
      Serial.print("error setting the auto-clean interval: ");
      Serial.println(ret);
    }

    ret = sps30_start_measurement();
    if (ret < 0) {
      Serial.print("error starting measurement\n");
    }
  }

  // Setup Alphanumeric Display
  if (segmentDisplay.begin(SEGMENT_DISPLAY_1, SEGMENT_DISPLAY_2)) {
    displayEnabled = true;
    segmentDisplay.setBrightness(5);
  } else {
    statusLed.setPixelColor(0, statusLed.Color(255, 0, 0));
    statusLed.show();
    if (digitalRead(PROGRAM_BUTTON))
      exit(1);
    else {
      statusLed.clear();
      statusLed.show();
    }
  }

  // Setup BME688 Sensor
  if (bme688Sensor.begin(BME688_SENSOR, Wire)) {
    bme688SensorEnabled = true;

    bsecSensor sensorList[] = {
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_STABILIZATION_STATUS,
      BSEC_OUTPUT_RUN_IN_STATUS,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT
    };

    if (!bme688Sensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP)) {
      bme688SensorEnabled = false;
      checkBsecStatus(bme688Sensor);
    }

    //if (bme688SensorEnabled)
    bme688Sensor.attachCallback(bme688SensorDataCallback);
  } else {
    Serial.println("Failed to setup the BME688 Sensor");
  }

  // Setup SHTC3 Sensor
  if (shtc3Sensor.begin()) {
    shtc3SensorEnabled = true;
  }

  // Setup Zio Sensor
  if (zioSensor.begin(ZIO_DEFAULT_ADDRESS, Wire)) {
    zioLoudnessSensorEnabled = true;
  }

  // Display Build info
  Serial.println("Kwartzlab Environmental Sensor Test App");
  Serial.println("Revision 1, Built 04/05/23");
  Serial.println("Created By: FireLabs - www.firelabs.ca");
  segmentDisplay.print("ENV TEST");
  delay(1000);
  segmentDisplay.print("REV 1");
  delay(1000);
  segmentDisplay.print("04/05/23");
  delay(1000);
  segmentDisplay.print("FIRELABS");
  delay(1000);

  delay(5000);
  // Test LED
  segmentDisplay.print("LED RED");
  Serial.println("Testing Status Led - RED");
  statusLed.setPixelColor(0, statusLed.Color(255, 0, 0));
  statusLed.show();
  delay(1000);
  segmentDisplay.print("LED GRN");
  Serial.println("Testing Status Led - GREEN");
  statusLed.setPixelColor(0, statusLed.Color(0, 255, 0));
  statusLed.show();
  delay(1000);
  segmentDisplay.print("LED BLUE");
  Serial.println("Testing Status Led - BLUE");
  statusLed.setPixelColor(0, statusLed.Color(0, 0, 255));
  statusLed.show();
  delay(1000);
  segmentDisplay.print("LED WHT");
  Serial.println("Testing Status Led - WHITE");
  statusLed.setPixelColor(0, statusLed.Color(255, 255, 255));
  statusLed.show();
  delay(1000);
  statusLed.clear();
  statusLed.show();
  segmentDisplay.print("DONE");

  delay(5000);
  // Test Relays
  segmentDisplay.print("RELAY *O");
  Serial.println("Testing Left Relay - On");
  digitalWrite(RELAY1, HIGH);
  delay(1000);
  segmentDisplay.print("RELAY OO");
  Serial.println("Testing Left Relay - Off");
  digitalWrite(RELAY1, LOW);
  delay(1000);
  segmentDisplay.print("RELAY O*");
  Serial.println("Testing Right Relay - On");
  digitalWrite(RELAY2, HIGH);
  delay(1000);
  segmentDisplay.print("RELAY OO");
  Serial.println("Testing Right Relay - Off");
  digitalWrite(RELAY2, LOW);
  delay(1000);
  segmentDisplay.print("RELAY **");
  Serial.println("Testing Both Relays - On");
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  delay(1000);
  segmentDisplay.print("RELAY OO");
  Serial.println("Testing Both Relays - Off");
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  delay(1000);
  segmentDisplay.print("DONE");

  delay(5000);


  // Show ESP32 Info
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);

  // Enable Timers
  checkBME688Sensor.attach(.1, checkBME688SensorTick);
  checkSPS30Sensor.attach(15, checkSPS30SensorTick);
  checkSHTC3Sensor.attach(15, checkSHTC3SensorTick);
  checkZioSensor.attach(15, checkZioSensorTick);
  toggleGPIO.attach(2, toggleGPIOTick);

  // Show Status Report
  Serial.println("Status Report:");
  Serial.print("Status LED: "); Serial.println(statusLed_output ? "true" : "false");
  Serial.print("Display Enabled: "); Serial.println(displayEnabled ? "true" : "false");
  Serial.print("BME688: "); Serial.println(bme688SensorEnabled ? "true" : "false");
  Serial.print("SHTC3: "); Serial.println(shtc3SensorEnabled ? "true" : "false");
  Serial.print("SPS30: "); Serial.println(sps30SensorEnabled ? "true" : "false");
  Serial.print("Zio Sensor: "); Serial.println(zioLoudnessSensorEnabled ? "true" : "false");
}

void loop() {
  char value[32];
  switch (currentScreen) {
    case 1:
      snprintf(value, 32, "TEMPERATURE %.0fc", sensor_shtc3_temperature);
      showDisplayMessage(value);
      break;
    case 2:
      snprintf(value, 32, "HUMIDITY %.0f%%", sensor_shtc3_humidity);
      showDisplayMessage(value);
      break;
    case 3:
      snprintf(value, 32, "NOISE %.0fdb", sensor_zio_db);
      showDisplayMessage(value);
      break;
    case 4:
      snprintf(value, 32, "AIR QUALITY %.0f", sensor_bme688_iaq);
      showDisplayMessage(value);
      break;
    case 5:
      snprintf(value, 32, "CO2 %.0f", sensor_bme688_co2);
      showDisplayMessage(value);
      break;
    case 6:
      snprintf(value, 32, "VOC %.0f", sensor_bme688_voc);
      showDisplayMessage(value);
      break;
    case 7:
      snprintf(value, 32, "PM2.5%.0f", sensor_sps30_pm25);
      showDisplayMessage(value);
      break;
    case 8:
      snprintf(value, 32, "PARTICLE SIZE %.0f", sensor_sps30_particle_size);
      showDisplayMessage(value);
      break;
    default:
      segmentDisplay.clear();
      currentScreen = 0;
      break;
  }

  currentScreen++;
  delay(2000);
}
