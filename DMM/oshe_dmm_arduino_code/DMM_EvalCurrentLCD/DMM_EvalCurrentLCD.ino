//This is using Eval Board
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal.h>
Adafruit_INA219 ina219;
// include the library code:


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
 float current_mA = 0;
void setup() {
  
 
    Serial.begin(9600);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
    Serial.println("");
  Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.clear();
  lcd.print("hello, world!");
}

void loop() {
  
  



 current_mA = ina219.getCurrent_mA();


  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");

 lcd.print("Current: ");
 lcd.print(current_mA);
 lcd.print("mA");
 lcd.setCursor(0, 1);
 lcd.print("OSHE DMM");
  delay(2000);
 lcd.clear();
 delay(1000);
  
}

