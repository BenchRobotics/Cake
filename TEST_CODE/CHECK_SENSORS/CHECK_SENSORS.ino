#include <Wire.h>

// Create instances for the two I2C buses
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

void scanI2CBus(TwoWire &wire, int busNumber) {
  byte error, address;
  int nDevices;

  Serial.print("Scanning I2C Bus ");
  Serial.print(busNumber);
  Serial.println("...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup() {
  Serial.begin(115200);

  // Initialize the I2C buses
  Serial.println("Initializing I2C Buses...");

  I2Cone.begin(21, 22); //5,23 //  SDA, SCL for sensor2 and MPU6050

  Serial.println("I2C Bus 2 initialized");

  I2Ctwo.begin(18, 19); // SDA, SCL 18,19
  Serial.println("I2C Bus 3 initialized");

  delay(1000); // Wait for the serial monitor to open

  //scanI2CBus(I2Cone, 2);
  //scanI2CBus(I2Ctwo, 3);
}

void loop() {
  // Nothing to do here
  scanI2CBus(I2Cone, 0);
  scanI2CBus(I2Ctwo, 1);
  delay(1000);
}
