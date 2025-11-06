/////////////////////////////////
//        DMM SOFTWARE
/////////////////////////////////
//Created By: Evan Howard
//            Luc Prisby
//            Caleb Jahncke
//            James Lovell
//Created on: 10/31/2025
//Last Modified on: 11/03/2025


// Include Necessary Libraries:
#include <LiquidCrystal.h>
#include "INA219.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Setup for the resistance measurement
#define NUM_REF_RESISTORS 8
#define NUM_SELECT_PINS   3
#define MAX_ANALOG_VALUE  1023
#define SWITCH_RESISTANCE 4.5

// Reference Resistor values      x0    x1   x2     x3     x4       x5       x6       x7
float rRef[NUM_REF_RESISTORS] = {49.5, 100, 1000, 10000, 100000, 1000000, 4990000, 10000000};

// Multiplexer select pins              A  B  C
const byte rSelPins[NUM_SELECT_PINS] = {6, 7, 8};
const byte enableMux = 9;
 
// Set up variables:
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // LCD Pins
const int mode_read_pin = A3;  // Pin A3 for analog input
int mode_read = 0;      // Variable to store the analog value

const int contPin = 10;


// Initialize LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Initialize INA219 Current ADC
INA219 INA(0x40);
// Create an instance for ADS1115
Adafruit_ADS1115 ads;


void setup() 
{
  //---Set up LDC---//
  lcd.begin(16, 2);

  //---Set up INA219 Current ADC---//
  INA.setMaxCurrentShunt(5, 0.05);

  //---Set up ADS1115 Voltage ADC---//
  ads.begin(0x48);
  ads.setGain(GAIN_ONE); // Set gain to avoid clipping (allows input up to ±6.144V)

  //---Set up resistance pins---//
  pinMode(enableMux, OUTPUT);
  digitalWrite(enableMux, HIGH);      // disable all switches
  
  for (int i = 0; i < NUM_SELECT_PINS; i++)
  {
    pinMode(rSelPins[i], OUTPUT);     // Mux select pins configured as outputs
    digitalWrite(rSelPins[i], HIGH);  // select the highest Rref
  }

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
  
  //---2 VOLT DC MODE---//
  if (mode_read > 60 && mode_read < 140) 
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) 
    {
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

  //---20 VOLT DC MODE---//
  if (mode_read > 160 && mode_read < 240) 
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) 
    {
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

  //---200 VOLT DC MODE---//
  if (mode_read > 260 && mode_read < 340) 
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) 
    {
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

  //---2 VOLT AC MODE---//
  if (mode_read > 360 && mode_read < 440)
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) 
    {
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

  //---20 VOLT AC MODE---//
  if (mode_read > 460 && mode_read < 540)
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) 
    {
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

  //---200 VOLT AC MODE---//
  if (mode_read > 560 && mode_read < 640)
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    // Force a new differential reading (AIN0 - AIN1)
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); // false means one shot mode, forces a full measurement and waits for math

    // Wait for conversion to complete
    while (!ads.conversionComplete()) 
    {
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

  //--- DC Current Mode ---//
  if (mode_read > 660 && mode_read < 740) 
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);
    
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

  //---AC CURRENT MODE---//
  if (mode_read > 760 && mode_read < 840)
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    delay(500);
  }

  //---RESISTANCE MEASUREMENT---//
  if (mode_read > 860 && mode_read < 940)
  {
    //---Pin for continuity---/
    pinMode(contPin, LOW);

    lcd.clear();
    lcd.print("OSHE DMM   (RES)");
    lcd.setCursor(0, 1);

    int cOut;
    float delta, deltaBest1 = MAX_ANALOG_VALUE, deltaBest2 = MAX_ANALOG_VALUE;
    float rBest1 = -1, rBest2 = -1, rR, rX;
    char unit = 0, fStr[16];

    for (byte count = 0; count < NUM_REF_RESISTORS; count++)
    {
      // Set the Mux select pins to switch in one Rref at a time.
      // count=0: Rref0 (49.9 ohms), count=1: Rref1 (100 ohms), etc...
      digitalWrite(rSelPins[0], count & 1); // C: least significant bit
      digitalWrite(rSelPins[1], count & 2); // B:
      digitalWrite(rSelPins[2], count & 4); // A: most significant bit
    
      digitalWrite(enableMux, LOW);       // enable the selected reference resistor
      delay(count + 1);                   // delay 1ms for Rref0, 2ms for Ref1, etc...
      cOut = analogRead(A0);              // convert analog voltage Vx to a digital value
      digitalWrite(enableMux, HIGH);      // disable the selected reference resistor
      delay(NUM_REF_RESISTORS - count);   // delay 8ms for Rref0, 7ms for Ref1, etc...

      // Work only with valid digitized values
      if (cOut < MAX_ANALOG_VALUE)
      {
        // Identify the Rref value being used and compute Rx based on formula #2.
        // Note how Mux's internal switch resistance is added to Rref. 
        rR = rRef[count] + SWITCH_RESISTANCE; 
        rX = (rR * cOut) / (MAX_ANALOG_VALUE - cOut);

        // Compute the delta and track the top two best delta and Rx values
        delta = (MAX_ANALOG_VALUE / 2.0 - cOut);
        if (fabs(delta) < fabs(deltaBest1))
        {
          deltaBest2 = deltaBest1;
          rBest2 = rBest1;
          deltaBest1 = delta;
          rBest1 = rX;
        }
        else if (fabs(deltaBest2) > fabs(delta))
        {
          deltaBest2 = delta;
          rBest2 = rX;
        }
      }
    }

    // Make sure there are at least two good samples to work with
    if (rBest1 >= 0 && rBest2 >= 0)
    {
      // Check to see if need to interpolate between the two data points.
      // Refer to the documentation for details regarding this.
      if (deltaBest1 * deltaBest2 < 0)
      {
        rX = rBest1 - deltaBest1 * (rBest2 - rBest1) / (deltaBest2 - deltaBest1); // Yes
      }
      else
      {
        rX = rBest1;  // No. Just use the best value
      }

      // Convert the scaled float result to string and extract the units
      unit = ScaleToMetricUnits(&rX, fStr);
    }

    //DisplayResultsOnLEDScreen(4, 40, unit, fStr);

    lcd.print("Resistance: ");
    lcd.print(fStr);
    lcd.print(unit);
    delay(1000); // Wait for a second

    delay(250);
  }

  //---CONTINUITY TESTING---//
  if (mode_read > 960 && mode_read < 1040)
  {
    //---Pin for continuity---/
    pinMode(contPin, INPUT_PULLUP);

    lcd.clear();
    lcd.print("OSHE DMM  (CONT)");
    lcd.setCursor(0, 1);

    if(digitalRead(contPin) == LOW)
    {
        lcd.print("THERE IS CONT!!");
    } 
    else
    {
        lcd.print("----------------");
    }

    delay(500);
  }

  delay(100);
}

// function to scale units for resistance measurement
char ScaleToMetricUnits(float *prVal, char fStr[])
{
  char unit;

  if (*prVal < 1000)
  {
    unit = ' ';
  }
  else if (*prVal >= 1000 && *prVal < 1000000)
  {
    *prVal /= 1000;
    unit = 'K';
  }
  else if (*prVal >= 1000000 && *prVal < 1000000000)
  {
    *prVal /= 1000000;
    unit = 'M';
  }
  else
  {
    *prVal /= 1000000000;
    unit = 'G';
  }

  // Cycle the decimal number in prVal until its whole number is 0.
  // Note that counter 'k' is decremented from 2 to 0 (inclusive),
  // which gives us the 3-digit precision we're looking for.
  for (int k=2, s=10; k >= 0; k--, s*=10)
  {
    if ((int)(*prVal) / s == 0)
    {
      dtostrf(*prVal, 4, k, fStr); // convert the float result to a string
      break;
    }
  }

  return unit;
}