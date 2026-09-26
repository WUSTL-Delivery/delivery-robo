// Battery Voltage Monitor with I2C LCD
// Reads battery voltage through voltage divider (2x 10K resistors)
// Displays on 16x2 LCD and alerts if voltage is low
//
// Sampling is non-blocking: one analogRead per loop() pass, SAMPLE_DELAY ms apart, so the
// serial command parser is never stalled. (The previous readVoltage() delay()ed 100 ms
// every 500 ms, holding up motor commands ~20% of the time.)

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD (address, columns, rows)
LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

unsigned long lastUpdate = 0;
unsigned long lastSampleTime = 0;
long sampleSum = 0;
int sampleCount = 0;
float lastVoltage = 0.0;

void batteryUpdate() {
  unsigned long now = millis();

  // Collect SAMPLES readings, one per pass, SAMPLE_DELAY ms apart
  if (sampleCount < SAMPLES) {
    if (now - lastSampleTime >= SAMPLE_DELAY) {
      sampleSum += analogRead(ANALOG_PIN);
      sampleCount++;
      lastSampleTime = now;
    }
    return;
  }

  // Average is ready; publish it to the display at the regular interval
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;

    float average = sampleSum / (float)SAMPLES;
    // Arduino ADC: 0-1023 represents 0-5V; multiply by the divider ratio for battery volts
    lastVoltage = (average * 5.0 / 1023.0) * VOLTAGE_DIVIDER;
    sampleSum = 0;
    sampleCount = 0;

    displayVoltage(lastVoltage);
    checkLowBattery(lastVoltage);
  }
}

void batteryInit() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("Battery Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(2000);
  lcd.clear();
}

// Latest averaged battery voltage (updated by batteryUpdate(); also answers the 'b' command)
float readVoltage() {
  return lastVoltage;
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
