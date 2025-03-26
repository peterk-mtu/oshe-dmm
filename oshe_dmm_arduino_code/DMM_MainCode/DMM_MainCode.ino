// INA219 Library
#include <INA219.h>
#include <Wire.h>
//LCD Screen Library
#include <LiquidCrystal.h>
//Voltage Library
#include <Adafruit_ADS1X15.h>
//Voltage Initialization
Adafruit_ADS1115 ads;  // Create an instance for ADS1115
//Initialize LCD Screen
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); //(rs, enable, d4, d5, d6, d7)

//Initialize INA219/Contructor
INA219 INA(0x40); //0x40 because both A1 and A0 pins are grounded adress is set to 1000000 in binary which is 0x40 in hexicimal
//Current Measurement in amps



//////////////////////////////////////////////////////////////////
//Define Inputs Outputs


int mode_selector = A3;  //Mode Selector Value



// Define Variables
int mode=0;
int modePrev=0;
int modeRead=0;
void setup()
{

while (!Serial) {
      // pause until available
      delay(1);
  }
/**
Voltage Starter
**/
Serial.begin(9600) //communication to serial monitor
Wire.begin() //i2c communication/ inirialize i2c bus sets mircrontroller as master
Serial.println("");
Serial.println("Hello!");
  if (!ads.begin(0x48)) {  // Use correct I2C address (check with scanner)
        Serial.println("ERROR: ADS1115 not found at 0x48! Check wiring.");
        while (1);  // Halt execution if not found
    }

        // Set gain to avoid clipping (allows input up to ±6.144V)
    ads.setGain(GAIN_ONE);

   // Print current gain setting
    Serial.print("Gain Setting: ");
    Serial.println(ads.getGain());

    delay(1000);


  //LCD screen helpful 
  lcd.begin(16,2); //Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display. begin() needs to be called before any other LCD library commands.
  lcd.print("hello, world!"); //prints data on LCD display
  lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left corner.
  lcd.display(); //Turns on the LCD display, after it's been turned off with noDisplay(). This will restore the text (and cursor) that was on the display.
  lcd.noDisplay(); //Turns off the LCD display, without losing the text currently shown on it.

  //INA219 Helpful
  if(!INA.begin())
  {
    Serial.println("Could not connect");
  }

INA.setMaxCurrentShunt(5,.05); //Set max current and shunt resistance can be changed
Serial.println("Is it calibrated "+INA.isCalibrated());
Serial.println("LSB current: "+INA.getCurentLSB());
Serial.println("Shunt resistance "+INA.getShunt());
Serial.println("Max Current "+INA.getMaxCurrent());

modePrev=mode;

}

void loop() {
//Questions/Todo. 
//How neccessary is delaying
//delay(ms)=pause program No other reading of sensors, mathematical calculations, or pin manipulation can go on during the delay function, so in effect, it brings most other activity to a halt. 
//how does microcontroller receive measurements to send to LCD: In the INA219 libary, calling contructor defaults to the Wire Instnance and directly uses it. The library handes all i2c communication with INA21
//it starts transimmison ends transmmission writes to register and reads all on its own
//the ina219 begin only uses wire to check if the ina219 is connected checks if the INA219 responds at the specified address by starting a transmission and verifying the response (endTransmission() == 0 means success).
//begin() relies on this to confirm the sensor is present but does not call
//so we must say Wire.being();
//microntroller should store current just by setting the getcurrent to a value and LCD.print() should display it


//analog read
//How it works: In the loop() function, the code reads the analog value from A3 using analogRead(mode_selector). This returns a value between 0 and 1023, corresponding to a voltage between 0 and 5V. The code then uses these values to determine the selected mode:


//Figure out what Mode we are in
//go to if statement and perfrom meausrement. Repeat


int mode = analogRead(mode_selector);

if(mode==1){
 float voltage=0;
ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false);
    while (!ads.conversionComplete()) {
        delay(10);
    }

    // Read ADC result
    int16_t rawValue = ads.getLastConversionResults();
    float voltage = ads.computeVolts(rawValue);

    // Print results
    Serial.print("Raw ADC Value: ");
    Serial.println(rawValue);

    Serial.print("Differential Voltage (V) [AIN2 - AIN3]: ");
    Serial.println(voltage, 6);

    Serial.println("Conversion completed successfully.");
lcd.clear();
lcd.print("Voltage: ")
lcd.print(voltage);
lcd.print(" V")
lcd.setCursor(0, 1);
lcd.print("OSHE DMM");
    // Wait 1 second before next reading
    delay(1000);

}


//Current
if(mode==2){
  float current=0;
current=INA.getCurrent();
Serial.println("Current: "+current+" A");
lcd.clear();
lcd.print("Current: ")
lcd.print(current);
lcd.print(" A")
lcd.setCursor(0, 1);
lcd.print("OSHE DMM");

}

void setInputs(){
  //figure out what pins are inputs and set them


//Mode Selecgor
  pinMode(mode_selector,INPUT); //Pin 26 - A3 - PC3 - USED FOR MODE SELECTION - ADC INPUT CHANNEL 3

}



}


