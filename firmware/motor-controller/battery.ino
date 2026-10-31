// ina219 motor driver

#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

unsigned long lastUpdate = 0;

void batteryUpdate() {
  // Update display at regular intervals
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();
    
    float voltage = readVoltage();
    displayVoltage(voltage);
    checkLowBattery(voltage);
  }
}

void batteryInit() {
  ina219.begin()
}

// Read voltage with averaging for stability
float readVoltage() {
  return ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);
}

// Display voltage on LCD
void displayVoltage(float voltage) {
  lcd.setCursor(0, 0);
  lcd.print("Battery Voltage:");
  
  lcd.setCursor(0, 1);
  lcd.print("   ");  // Clear previous value
  lcd.setCursor(0, 1);
  if(((voltage/Battery_MAX)*100) > 1){
    lcd.print(100);
    lcd.print(" %    ");
  }else{
    lcd.print((voltage/Battery_MAX)*100, 2);  // Show 2 decimal places
    lcd.print(" %    ");    // Extra spaces to clear old characters
  }
  // Show battery status high,med,low
  lcd.setCursor(10, 1);
  if ((voltage/Battery_MAX) >= HIGH_VOLTAGE) { //Above or at 0.7
    lcd.print("HIGH");
  } else if ((voltage/Battery_MAX)>= MED_VOLTAGE) { //Above or at 0.35
    lcd.print("MEDIUM");
  } else if ((voltage/Battery_MAX) < MED_VOLTAGE){ //Below 0.35
    lcd.print("LOW!");
  }
}

// Check for low battery and activate LED warning
void checkLowBattery(float voltage) {
  if (voltage < MED_VOLTAGE) {
    // Blink LED for critical battery
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// Optional: Calibration function
// Call this if your readings are off
float calibrateVoltage(float rawVoltage) {
  // Measure actual voltage with multimeter
  // Adjust this multiplier if readings are consistently off
  const float CALIBRATION_FACTOR = 1.0;  // Adjust as needed
  return rawVoltage * CALIBRATION_FACTOR;
}
