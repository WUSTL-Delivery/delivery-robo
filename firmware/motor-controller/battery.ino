// Battery monitor using the INA219
//
// The reading is cached at UPDATE_INTERVAL so the 'b' command replies without touching I2C
// and the serial command parser is never stalled.

#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

unsigned long lastUpdate = 0;
float lastVoltage = 0.0;

void batteryUpdate() {
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();
    lastVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);
  }
}

void batteryInit() {
  ina219.begin();
}

// Latest battery voltage (updated by batteryUpdate(); also answers the 'b' command)
float readVoltage() {
  return lastVoltage;
}

// Optional: Calibration function
// Call this if your readings are off
float calibrateVoltage(float rawVoltage) {
  // Measure actual voltage with multimeter
  // Adjust this multiplier if readings are consistently off
  const float CALIBRATION_FACTOR = 1.0;  // Adjust as needed
  return rawVoltage * CALIBRATION_FACTOR;
}
