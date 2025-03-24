// INA219 Library
#include <INA219.h>
#include <Wire.h>
//LCD Screen Library
#include <LiquidCrystal.h>
//Voltage Library

//Voltage Initialization

//Initialize LCD Screen
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); //(rs, enable, d4, d5, d6, d7)

//Initialize INA219/Contructor
INA219 INA(0x40); //0x40 because both A1 and A0 pins are grounded adress is set to 1000000 in binary which is 0x40 in hexicimal
//Current Measurement in amps

//////////////////////////////////////////////////////////////////
//Define Inputs Outputs
int mode_selector = A3;  //NEED TO SET ANALOG VALUES

//Voltage Mode
int VOLT_PIN_0 = 7;      //For Voltage mode side of ADC0
int VOLT_PIN_1 = 12;     //For Voltage mode side of ADC1



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

Serial.begin(9600) //communication to serial monitor
Wire.begin() //i2c communication/ inirialize i2c bus sets mircrontroller as master

Serial.println("");
  Serial.println("Hello!");



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

INA.setMaxCurrentShunt(5,.1); //Set max current and shunt resistance can be changed
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


// int modeRead=analogRead();
//Serial.print(modeRead);



//Mode Read
/**


if(mode_read > 190 && mode_read < 245){         //Voltage mode
    mode = 1;    
  }

  else if(mode_read > 437 && mode_read < 720){    //Resistance mode    
    if(mode != 7 && mode != 8){
      mode = 2;  
    }     
  }

  else if(mode_read > 317 && mode_read < 437){    //Inductance mode
    mode = 3;    
  }

  else if(mode_read > 245 && mode_read < 317){    //Capacitance mode
    if(mode != 6){
      mode = 4;  
    }      
  }  

  else if(mode_read > 720){                       //Current mode
    mode = 5;    
  }

  else{
    Serial.println("No Mode");
  }

  if(mode != mode_prev){
    set_all_inputs();
    mode_prev = mode;
  }
**/



//Make sure to add delays

//Voltage
if(mode==1){
 int voltage=0;

lcd.clear();
lcd.print("Voltage: ")
lcd.print(voltage);
lcd.print(" V")
lcd.setCursor(0, 1);
lcd.print("OSHE DMM");

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

  //REFERENCE CODE FROM OTHER GUY SETS ALL A0-A7 & D2-D13 AS INPUTS -- VERIFY

  pinMode(12,OUTPUT); //Pin 12 - D8 - PB0 - TIMER/COUNTER1 INPUT CAPTURE INPUT, DIVIDED SYSTEM CLOCK OUTPUT
  pinMode(13,OUTPUT); //Pin 13 - D9 - PB1 - TIMER/COUNTER1 OUTPUT COMPARE
  //pinMode(14,INPUT); //Pin 14 - D10 - PB2 - D10 PUSH BUTTON (ALREADY INPUT)
  pinMode(15,OUTPUT); //Pin 15 - MOSI/D11 - PB3 - SPI MASTER OUT SLAVE IN, TIMER/COUNTER2 OUTPUT COMPARE
  pinMode(VOLT_PIN_1,OUTPUT); //Pin 16 - MISO/D12 - PB4 - SPI MASTER IN SLAVE OUT
  pinMode(17,OUTPUT); //Pin 17 - SCK/D13 - PB5 - SPI MASTER CLOCK INPUT
  pinMode(7,INPUT); //Pin 7 - XTAL1/PB6 - LOOPS TO PIN 8, CLOCK OSCILLATOR PIN 1
  pinMode(8,INPUT); //Pin 8 - XTAL2/PB7 - LOOPS TO PIN 7, CLOCK OSCILLATOR PIN 2

  pinMode(23,INPUT); //Pin 23 - A0 - PC0 - ADC INPUT CHANNEL 0
  pinMode(24,INPUT); //Pin 24 - A1 - PC1 - ADC INPUT CHANNEL 1
  pinMode(25,INPUT); //Pin 25 - A2 - PC2 - ADC INPUT CHANNEL 2
  pinMode(mode_selector,INPUT); //Pin 26 - A3 - PC3 - USED FOR MODE SELECTION - ADC INPUT CHANNEL 3
  pinMode(currentPin,INPUT);  //Pin 27 - SDA - PC4 - I/O in schem - ADC INPUT CHANNEL 4
  pinMode(currentPin,INPUT);  //Pin 28 - SCL - PC5 - I/O in schem - ADC INPUT CHANNEL 5
  pinMode(29,INPUT); //Pin 29 - DTR & 5V - RESET/PC6 - RESET PIN

  pinMode(30,INPUT); //Pin 30 - RXD - PD0 - USART INPUT
  pinMode(31,INPUT); //Pin 31 - TXD - PD1 - USART OUTPUT
  pinMode(32,INPUT); //Pin 32 - D2 - PD2 - EXTERNAL INTERRUPT 0 INPUT
  pinMode(1,OUTPUT); //Pin 1 - D3 - PD3 - EXTERNAL INTERRUPT 1 INPUT, TIMER/COUNTER2 OUTPUT COMPARE
  pinMode(2,OUTPUT); //Pin 2 - D4 - PD4 - USART EXTERNAL CLOCK INPUT/OUTPUT
  pinMode(9,OUTPUT); //Pin 9 - D5 - PD5 - TIMER/COUNTER1 EXTERNAL COUNTER INPUT, TIMER/COUNTER0 OUTPUT COMPARE
  pinMode(10,OUTPUT); //Pin 10 - D6 - PD6 - ANALOG COMPARATOR POSITIVE INPUT, TIMER/COUNTER0 OUTPUT COMPARE
  pinMode(VOLT_PIN_0,OUTPUT); //Pin 11 - D7 - PD7 - ANALOG COMPARATOR NEGATIVE INPUT, PIN CHANGE INTERRUPT 23

}



}


