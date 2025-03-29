/////////////////////////////////
//        DMM SOFTWARE
/////////////////////////////////
//Created By: Peter Kocour
//            Alex Ritter
//            Luc Prisby
//            Caleb Jahncke
//            James Lovell
//            Ethan Dwyer
//Created on: 03/27/2025
//Last Modified on: 3/27/2025


// Include Necessary Libraries:
#include <LiquidCrystal.h>
#include "INA219.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>


// Set up variables:
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // LCD Pins
const int mode_read_pin = A3;  // Pin A3 for analog input
int mode_read = 0;      // Variable to store the analog value


// Initialize LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Initialize INA219 Current ADC
INA219 INA(0x40);
// Create an instance for ADS1115
Adafruit_ADS1115 ads;


void setup() {
  //---Set up LDC---//
  lcd.begin(16, 2);

  //---Set up INA219 Current ADC---//
  INA.setMaxCurrentShunt(5, 0.05);

  //---Set up ADS1115 Voltage ADC---//
  ads.begin(0x48);
  ads.setGain(GAIN_ONE); // Set gain to avoid clipping (allows input up to ±6.144V)

  delay(1000);
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);

  // Read the analog value from pin A3
  mode_read = analogRead(mode_read_pin);
  
  // Print the value to the Serial Monitor
  //Serial.print("Analog value from A3: ");
  //Serial.println(mode_read);
  //lcd.print(mode_read);

  // Small delay for stability
  //delay(500);

  if (mode_read > 220 && mode_read < 250) {
    
    float current=0;
    current=INA.getCurrent();

    // Display's Voltage Reading on LCD
    lcd.clear();
    lcd.print("OSHE DMM  (mA/A)");
    lcd.setCursor(0, 1);
    lcd.print("Current: ");
    lcd.print(current);
    lcd.print(" A");

    // Wait 0.5 seconds before next reading
    delay(500);

  }
  
  //---2 VOLT MODE---//
  if (mode_read > 720 && mode_read < 760) {

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) {
        delay(10);
    }

    // Read ADC result
    int16_t rawValue = ads.getLastConversionResults();
    float voltage = ads.computeVolts(rawValue);

    // Display's Voltage Reading on LCD
    lcd.clear();
    lcd.print("OSHE DMM    (2V)");
    lcd.setCursor(0, 1);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.print(" V");

    // Wait 0.5 seconds before next reading
    delay(500);
  }

//---20 VOLT MODE---//
  if (mode_read > 500 && mode_read < 540) {

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) {
        delay(10);
    }

    // Read ADC result
    int16_t rawValue = ads.getLastConversionResults();
    float voltage = (ads.computeVolts(rawValue) * 10.06); // * 10 puts voltage back into current range

    // Display's Voltage Reading on LCD
    lcd.clear();
    lcd.print("OSHE DMM   (20V)");
    lcd.setCursor(0, 1);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.print(" V");

    // Wait 0.5 seconds before next reading
    delay(500);
  }

  if (mode_read > 350 && mode_read < 390) {

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) {
        delay(10);
    }

    // Read ADC result
    int16_t rawValue = ads.getLastConversionResults();
    float voltage = (ads.computeVolts(rawValue) * 99.06); // * 101 puts voltage back into current range

    // Display's Voltage Reading on LCD
    lcd.clear();
    lcd.print("OSHE DMM  (200V)");
    lcd.setCursor(0, 1);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.print(" V");

    // Wait 0.5 seconds before next reading
    delay(500);
  }

  delay(100);
}
