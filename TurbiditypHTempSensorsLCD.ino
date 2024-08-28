/* 

This code is for checking values of sensor values of:

Gravity: Analog Turbidity Sensor SKU:SEN0189
Gravity: Analog pH Sensor SKU:SEN0161
DS18B20 Digital Temperature Sensor SKU:DFR0198

Then using it to print out the values both in console and
on an LCD screen (I2C 20x4 Arduino LCD Display Module SKU:DFR0154)
and determining if the liquid (water in our case) is safe
to drink considering water regulations and drinking water standards.

The code has taken inspiration from other codes focusing on single components 
and different tasks, but used and changed to fit our own problem and requirements.

The codes taken inspiration from are:

Code for temperature sensor taken from sample code:
https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_

Code for pH sensor taken from sample code:
https://wiki.dfrobot.com/Gravity__Analog_pH_Sensor_Meter_Kit_V2_SKU_SEN0161-V2?fbclid=IwZXh0bgNhZW0CMTEAAR2W1q9ndMaK2HnFz4lwb6G7JPep7M41BJvkRPlAB24sfFcH1qsqpaVYXPc_aem_RlS5wJxhZZIKnlCyrnP7uA

Code for turbidity sensor taken from example code example 1:
https://wiki.dfrobot.com/Turbidity_sensor_SKU__SEN0189

Code for printing text on LCD screen taken from course TEL100-1 24H on canvas in the file directory:
Live Coding Sessions -> LCS 2 -> sketch.ino

*/

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <OneWire.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>

#define PH_PIN A1 // Signal pin on analog 1
#define TURB_PIN A0 // Signal pin on analog 0
#define DS18S20_PIN 9 // DS18S20 Signal pin on digital 9

// LCD screen
LiquidCrystal_I2C lcd(0x27,20,4); // set the LCD address to 0x27 for a 20 chars and 4 line display

// Temperature chip i/o
OneWire ds(DS18S20_PIN);  // on digital pin 9

// Default values if no signal is given to avoid errors and bugs
float voltage,phValue,temperature = 25;
DFRobot_PH ph;


void setup() {
  Serial.begin(115200); //Baud rate: 115200
  // Initialize the pH sensor
  ph.begin();
  // Initialize and clear old data on LCD screen
  lcd.init();
  lcd.clear();
}


void loop() {
  // Loops through to continually get the values of all sensors
  // and then prints it in both console and LCD screen

  // Get NTU value of liquid
  float ntu = getTurbidity();
  // Get temperature of liquid
  float temperature = getTemp();
  // Get pH of liquid
  float phValue = getpH(temperature);

  // Print out values in console
  Serial.print("ntu: ");
  Serial.print(ntu, 1);
  Serial.print(",  ");
  Serial.print("temperature: ");
  Serial.print(temperature,2);
  Serial.print("^C,  pH: ");
  Serial.println(phValue,3);

  // Prints out collected data on LCD screen
  PrintOnScreen(temperature, phValue, ntu);

  // Add delay for each iteration
  delay(1000);
}


float getTurbidity() {
  // Code for reading voltage of turbidity sensor and converting it to ntu

  int sensorValue = analogRead(TURB_PIN);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float ntu = -1120.4*sq(voltage)+5742.3*voltage-4352.9;

  return ntu;
}


float getpH(float temperature){
  // Code for returning value from pH sensor with temperature compensation

  voltage = analogRead(PH_PIN)/1024.0*5000;  // read values from sensor and convert it to voltage
  phValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation
  //}
  ph.calibration(voltage,temperature);       // calibration process by Serail CMD

  return phValue;
}


float getTemp(){
  // Returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      // No more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}


void PrintOnScreen(float temp, float pH, float ntu){
  /* Prints out temperature, pH, NTU and
     if the water is safe to drink or not on an
     LCD screen, based on the pH                */

  lcd.backlight();
  bool Cleared = true;

  lcd.setCursor(4,0);
  lcd.print("Temp:         ");
  lcd.setCursor(9,0);
  lcd.print(temp);
  lcd.print("C");
  lcd.setCursor(2,1);
  lcd.print("pH:");
  lcd.print(pH);
  lcd.print(" NTU:");
  lcd.print(ntu);
  lcd.setCursor(6,2);
  lcd.print("Water is");
  if(6.5 < pH && pH < 9.5){
    lcd.setCursor(0,3);
    lcd.print("      drinkable   ");
  }
  else{
    lcd.setCursor(0,3);
    lcd.print("  not drinkable :(");
  }
}
