/* Test script to test the Kwartzlab Environmental Sensor Board */

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <i2cdetect.h>

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

uint32_t chipId = 0;

Adafruit_NeoPixel pixels(TOTAL_LEDS, STATUS_LED, NEO_GRB + NEO_KHZ800);

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

void setup() {
  // Setup I2C
  Wire.begin(I2C_SDA, I2C_SCL)

  // Setup NeoPixel
  pixels.begin();

  // Setup Relay GPIO Pins
  pinMode(RELAY1, OUTPUT); // Relay 1
  pinMode(RELAY2, OUTPUT); // Relay 2

  // Setup Generic GPIO Pins
  pinMode(GPIO3, OUTPUT); // GPIO3
  pinMode(GPIO4, OUTPUT); // GPIO4
  pinMode(GPIO5, OUTPUT); // GPIO5
  pinMode(GPIO8, OUTPUT); // GPIO6
  pinMode(GPIO10, OUTPUT); // GPIO10

  // Setup Input GPIO Pin
  pinMode(PROGRAM_BUTTON, INPUT_PULLUP ); // Program Button

  // Setup interrupt
  attachInterrupt(9, buttonPress, CHANGE);

  // Setup Serial
  Serial.begin(9600);

  // Show ESP32 Info
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);
}

void loop() {
  Serial.println("Setting GPIO Pins High");

  // Set NeoPixel to Green
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 80, 0));
  pixels.show();

  // Set all GPIO pins to high
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(10, HIGH);
  delay(1000);

  Serial.println("Setting GPIO Pins Low");

  // Set NeoPixel to Red
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(80, 0, 0));
  pixels.show();

  // Set all GPIO pins to low
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(8, LOW);
  digitalWrite(10, LOW);
  delay(1000);
}
