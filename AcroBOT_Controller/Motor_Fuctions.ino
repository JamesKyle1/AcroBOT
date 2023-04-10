
#include <DynamixelWorkbench.h>



float getJ0EncoderPos() {
  // send command to UNO
  Serial1.print('e');
  static byte ndx = 0;
  char rc;
  // receive data from UNO
  while (Serial1.available() > 0) {
    rc = Serial1.read();
    if (rc != '\n') {
      receivedChars[ndx] = rc;
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
    }
  }
  encoderPos = atof(receivedChars) ;
  return encoderPos;
}


//return the reading from INA 169 current sensor
// now it returns the raw ADC values
// TODO -  convert the ADC value to current on Amperes
int getMotorLoads()
{
  return(analogRead(analogInPin));
}

// set M1 position
// values ranging from 0 to 1023
// Mid position is 512
// TODO -  convert position into angles
void setM1Position(float pos) {
  dxl_wb.goalPosition(dxl_id0, (int32_t)pos);
}


// set M2 position
// values ranging from 0 to 1023
// Mid position is 512
// TODO -  convert position into angles
void setM2Position(float pos) {
  dxl_wb.goalPosition(dxl_id1, (int32_t)pos);
}

// return the joint position of M1
float getM1Position()
{
  // TODO
}

// return the joint position of M2
float getM2Position()
{
  // TODO
}

// enable torque of M1
void enableM1() {
  dxl_wb.torqueOn(dxl_id0 );
  //dxl_wb.torque(dxl_id0, 1);
}
void enableM2() {
  dxl_wb.torqueOn(dxl_id1 );
  //dxl_wb.torque(dxl_id1,  1);
}
// disable torque 
void disableM1() {
  dxl_wb.torqueOff(dxl_id0 );
 // dxl_wb.torque(dxl_id0, 0);
}

void disableM2() {
  dxl_wb.torqueOff(dxl_id1 );
  //dxl_wb.torque(dxl_id1, 0);
}

int testSubSystems()
{
Serial.println(" Testing start");

Serial.print(" Current Sensor ADC  = ");
Serial.println(getMotorLoads());

Serial.print(" Encoder Position1 = ");
Serial.println(getJ0EncoderPos());

setM1Position(100);
setM2Position(100);
delay(1000);
setM1Position(1000);
setM2Position(1000);
delay(1000);
disableM1();
disableM2();

delay(3000);

enableM1();
enableM2();
delay(100);
setM1Position(100);
setM2Position(100);

delay(1000);
disableM1();
disableM2();



}
