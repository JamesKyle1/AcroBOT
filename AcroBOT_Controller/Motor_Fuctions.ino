
#include <DynamixelWorkbench.h>



//read encoder position from the arduino UNO
double getJ0EncoderPos() {
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
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
    }
  }
  encoderPos = atof(receivedChars) - 6.0;
  return encoderPos;
}


//return the reading from INA 169 current sensor
// now it returns the raw ADC values
// TODO -  convert the ADC value to current on Amperes
float getMotorLoads() {
  return analogRead(analogInPin)*(5.0/1024.0);
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
double getM1Position() {

  int32_t get_data = 0;
  dxl_wb.itemRead(dxl_id0, "Present_Position", &get_data);  //, &log);
  return get_data;
}

// return the joint position of M2
double getM2Position() {
  int32_t get_data = 0;
  dxl_wb.itemRead(dxl_id1, "Present_Position", &get_data);  //, &log);
  return get_data;
}

// enable torque of M1
void enableM1() {
  dxl_wb.torqueOn(dxl_id0);
  //dxl_wb.torque(dxl_id0, 1);
}
void enableM2() {
  dxl_wb.torqueOn(dxl_id1);
  //dxl_wb.torque(dxl_id1,  1);
}
// disable torque
void disableM1() {
  dxl_wb.torqueOff(dxl_id0);
  // dxl_wb.torque(dxl_id0, 0);
}

void disableM2() {
  dxl_wb.torqueOff(dxl_id1);
  //dxl_wb.torque(dxl_id1, 0);
}

int testSubSystems() {
  Serial.println(" Testing start");

  Serial.print(" Current Sensor ADC  = ");
  Serial.println(getMotorLoads());

  Serial.print(" Encoder Position1 = ");
  Serial.println(getJ0EncoderPos());


//  setM1Position(512);
//  setM2Position(512);
  delay(1000);
  
  Serial.println("Testing complete!");
  delay(100);
  //setM1Position(600);
  //setM2Position(600);
  //delay(1000);
  //disableM1();
  //disableM2();

  //delay(1000);

  //enableM1();
  //enableM2();
  //delay(100);
  //setM1Position(400);
  //setM2Position(400);

  //delay(1000);
  //disableM1();
  //disableM2();
}
