#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Create instances for the two I2C buses
TwoWire I2Ctwo = TwoWire(1);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Pin definitions
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  // Start serial communication
  Serial.begin(115200);
  
  // Initialize I2C bus with custom pins
  I2Ctwo.begin(21, 22);

  // Initialize the sensor on the second I2C bus
  if (!lox.begin(0x29, true, &I2Ctwo)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  // Measure the distance
  lox.rangingTest(&measure, false); // Pass in 'true' to get debug data printout!

  // Check for errors
  if (measure.RangeStatus != 4) { // If there is no error
    Serial.print(F("Distance: "));
    Serial.print(measure.RangeMilliMeter);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("Out of range"));
  }

  // Delay between measurements
  delay(100);
}
