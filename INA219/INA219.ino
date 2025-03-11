#include "INA219.h"

INA219 INA(0x40);


void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("INA219_LIB_VERSION: ");
  Serial.println(INA219_LIB_VERSION);

  Wire.begin();
  if (!INA.begin() )
  {
    Serial.println("Could not connect. Fix and Reboot");
  }
  INA.setMaxCurrentShunt(3.2, 0.064);
  delay(1000);
 

  Serial.println(INA.getBusVoltageRange());

}


void loop()
{
  Serial.println("\n\tBUS\t\tSHUNT\t\tCURRENT\t\tPOWER\t\tOVF\t\tCNVR");
  for (int i = 0; i < 20; i++)
  {
    Serial.print("\t");
    Serial.print(INA.getBusVoltage(), 2);
    Serial.print("\t\t");
    Serial.print(INA.getShuntVoltage_mV(), 2);
    Serial.print("\t\t");
    Serial.print(INA.getCurrent_mA(), 2);
    Serial.print("\t\t");
    Serial.print(INA.getPower_mW(), 2);
    Serial.print("\t\t");
    Serial.print(INA.getMathOverflowFlag());
    Serial.print("\t\t");
    Serial.print(INA.getConversionFlag());
    Serial.println();
    delay(1000);
  }
  delay(1000);
}