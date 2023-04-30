
// read the encoder over SPI and send data to OpemCM board over serial comm.
#include <SPI.h>

const uint8_t CS_PIN = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  Serial.begin(1000000);
  
   while (Serial.available()) {
   Serial.read();
   }

    pinMode(LED_BUILTIN, OUTPUT);

     pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); 
  SPI.begin();
  delay(20);


}

void loop() {
  // put your main code here, to run repeatedly:
  //float encval= 123.42;
  //float curval = 0.13;
 if (Serial.available()) {

   char inChar = (char)Serial.read();
   if (inChar == 'e') 
   // encoder value
   {
      uint8_t data[10] = {0};
  int index = 0;

  
  //min time between each bit transfer is 7 microseconds so set to 125kHz which is the lowest
  //SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE1));   
SPI.beginTransaction(SPISettings(256000, MSBFIRST, SPI_MODE1));   

  //Start transaction
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(40);

     //Send start byte
  data[index++] = SPI.transfer(0xAA);
  delayMicroseconds(40);

  //Send and receive data bytes
  for(uint8_t i = 1; i < 10; i++){
    data[index++] = SPI.transfer(0xFF); 
    delayMicroseconds(40);
  }
  
  //End transaction
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  //Extract and calculate angle
  uint16_t data_bytes = (data[2] << 8) | data[3];  // Combine two bytes
  uint16_t first14 = data_bytes >> 2; // Extract first 14 bits
  float value = (float)(first14 / 16384.0);  // Divide by 2^14 and multiply by 360 to get degrees
  float angle = (value - 0.1)/(0.9 - 0.1) * 360;


    //if(angle<400){
     Serial.println(angle);
     //}

   }

  //if (inChar == 'c') 
   // current sensor value
   //{
    // Serial.println(curval);

  // }

     if (inChar == 'i') 
   // current sensor value
   {
    digitalWrite(LED_BUILTIN, HIGH); 

   }

     if (inChar == 'o') 
   // current sensor value
   {
    digitalWrite(LED_BUILTIN, LOW); 

   }



 }
 
}
