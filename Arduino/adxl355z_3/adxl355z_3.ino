// Add the SPI library so we can communicate with the ADXL355 sensor
#include <SPI.h>
#include "ADXL355.h"


//Assign the Chip Select signal to pin 10.
int CS = 10;

//This buffer will hold values read from the ADXL355 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
byte buff;


void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin(10);
  //The speed is set by the setClockDivider() function, which divides the master clock (84MHz on Due) down to a frequency between 42MHz (/2) and X (/128).
  SPI.setClockDivider(128);  
  //Configure the SPI connection for the ADXL355. The timing scheme follows the clock polarity (CPOL) = 0 and clock phase (CPHA) = 0 and is therefore SPI mode 0.
  SPI.setDataMode(SPI_MODE0);
  //Sets the order of the bits shifted out of and into the SPI bus, either LSBFIRST (least-significant bit first) or MSBFIRST (most-significant bit first).
  SPI.setBitOrder(MSBFIRST);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Measurement mode
  digitalWrite(CS, LOW);
  SPI.transfer(POWER_CTL);
  SPI.transfer(0x00);
  digitalWrite(CS, HIGH);  

 //Put the ADXL355 into Measurement Mode by writing 0 to the POWER_CTL register.
//digitalWrite(POWER_CTL, 0x00);  //Measurement mode  
} 

void loop(){
  digitalWrite(CS, LOW);
  SPI.transfer(DATAX3);
  values[0] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
//  
  digitalWrite(CS, LOW);
  SPI.transfer(DATAX2);
  values[1] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);  
//
  digitalWrite(CS, LOW);
  SPI.transfer(DATAY3);
  values[2] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
//  
  digitalWrite(CS, LOW);
  SPI.transfer(DATAY2);
  values[3] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
//
  digitalWrite(CS, LOW);
  SPI.transfer(DATAZ3);
  values[4] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
//
  digitalWrite(CS, LOW);
  SPI.transfer(DATAZ2);
  values[5] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);

 //The x value is stored in values[0] and values[1].
 x = ((int)values[0]<<8)|(int)values[1];
 //The Y value is stored in values[2] and values[3].
 y = ((int)values[2]<<8)|(int)values[3];
 //The Z value is stored in values[4] and values[5].
 z = ((int)values[4]<<8)|(int)values[5];
 
  
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);   
  //delay(100); 
}


