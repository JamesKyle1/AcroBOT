#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <DynamixelWorkbench.h>



#if defined(__OPENCM904__)
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

// The dynamixel baudrate
#define BAUDRATE  1000000

#define FREE_FALL       0
#define ALPHA           45.0
#define BETA            245.0
#define GAMMA           33.3


// ================================== /
// ======== Dynamixel Setup ========= /
// ================================== /

DynamixelWorkbench dxl_wb;

uint8_t dxl_id0 = 6;
uint8_t dxl_id1 = 4;


//for the encoder data from UNO
const byte numChars = 32;
char receivedChars[numChars];
float encoderPos = 0.0;

// the current sensor is connected to A0
const int analogInPin = A0;



// ================================= /
// ======= Controller Setup ======== /
// ================================= /

int n = 30;

double alpha_1[3] = {0.0, 0.0, 0.0};
double alpha_2[3] = {0.0, 0.0, 0.0};
double alpha_3[3] = {0.0, 0.0, 0.0};
double alpha_4[3] = {0.0, 0.0, 0.0};

//double** traj = nullptr;
double** traj_2 = nullptr;
double** traj_3 = nullptr;
double** traj_4 = nullptr;
double* q_goal = new double[3];

double* hollow_region = new double[2];
double* arch_region = new double[2];

int contactMode = 1;
double tol = 1e-6;


//Controller timer
unsigned long currTime;
long goal_loop_time = 10000;
unsigned long loop_timer;



bool takeData = false;
bool printHeaders = false;




void setup() {
  // put your setup code here, to run once:
  // Default serial is the OPENCM - USB one
  Serial.begin(115200);
  delay(100);
  // Serial  1 is available on PINS 11 and 12, and is connected to arduino UNO for reading the encoder
  Serial1.begin(115200);
  delay(100);

  //send commands to blink the LED on UNO, to verify everything is working
  Serial1.print('i');
  delay(1000);
  Serial1.print('o');
  delay(1000);

  Serial1.print('i');
  delay(1000);
  Serial1.print('o');
  delay(1000);
  //flush all buffers by reading any error values of any
  getJ0EncoderPos();
  getJ0EncoderPos();
  delay(100);


  // initialize dynamixels

  bool result = false;
  const char *log;


  uint16_t model_number = 0;
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  //ping both dynamixels
  result = dxl_wb.ping(dxl_id0, &model_number, &log);
  result = dxl_wb.ping(dxl_id1, &model_number, &log);
  // set joint mode to position control mode
  result = dxl_wb.jointMode(dxl_id0, 0, 0, &log);
  result = dxl_wb.jointMode(dxl_id1, 0, 0, &log);

  delay(100);
  testSubSystems();

  arch_region[0] = BETA;
  arch_region[1] = BETA - GAMMA;
  hollow_region[0] = 90.0 + (270.0 - BETA + GAMMA);
  hollow_region[1] = 90.0 + (270.0 - BETA);


  //Pre-calculate trajectories
  alpha_1[0] = arch_region[0];
  alpha_1[1] = 0.0;
  alpha_1[2] = 0.0;

  alpha_2[0] = arch_region[1];
  alpha_2[1] = -ALPHA;
  alpha_2[2] = ALPHA;

  alpha_3[0] = hollow_region[0];
  alpha_3[1] = 0.0;
  alpha_3[2] = 0.0;

  alpha_4[0] = hollow_region[1];
  alpha_4[1] = ALPHA;
  alpha_4[2] = -ALPHA;

  calculate_traj(traj_2, alpha_1, alpha_2, n);
  calculate_traj(traj_3, alpha_2, alpha_3, n);
  calculate_traj(traj_4, alpha_3, alpha_4, n);


  setM1Position(512);
  setM2Position(512);


}

void loop() {
  currTime = micros();
  loop_timer = micros();

  // put your main code here, to run repeatedly:
  double q0 = getJ0EncoderPos();
  double q1 = getM1Position() * 300.0 / 1024.0 - 150.0;
  double q2 = getM2Position() * 300.0 / 1024.0 - 150.0;
  double q_curr[3] = {q0, q1, q2}; //Should be set to the current states of the joints

  int guardTriggered = checkGuard(q_curr, hollow_region, arch_region, contactMode);

  if (guardTriggered > 0) {
    contactMode = guardTriggered;
  }

  //Set goal angle based on current position
  if (contactMode == 1 || contactMode == 5) {
    q_goal[0] = 0.0;
    q_goal[1] = 0.0;
    q_goal[2] = 0.0;
  }


  if (contactMode == 2) {
    find_closest_point(q_goal, q_curr, traj_2, n);
  }

  if (contactMode == 3) {
    find_closest_point(q_goal, q_curr, traj_3, n);
  }

  if (contactMode == 4) {
    find_closest_point(q_goal, q_curr, traj_4, n);
  }

  if (Serial.available() > 0) {
    int data = Serial.parseInt();
    if (data == 1) {
      takeData = true;
      printHeaders = true;
    } else if (data == 2) {
      takeData = false;
    }
  }

  if (printHeaders) {
    Serial.println("AcroBOT Hardware Experiments...");
    Serial.println("Alpha [deg]\t Beta [deg]\t Gamma [deg]");
    Serial.print(ALPHA);
    Serial.print(", ");
    Serial.print(BETA);
    Serial.print(", ");
    Serial.print(GAMMA);
    Serial.println();
    Serial.println("=================================================================================");
    Serial.println("Time [s]\t J0 Actual [deg]\t J0 Goal [deg]\t J1 Actual [deg]\t J1 Goal [deg]\tJ2 Actual [deg]\t J2 Goal [deg]\t Current [A]\t ");
    printHeaders = false;
  }

  if (takeData) {
    Serial.print(millis()/1000.0,3);
    Serial.print(", ");
    Serial.print(q_curr[0]);
    Serial.print(", ");
    Serial.print(q_goal[0]);
    Serial.print(", ");
    Serial.print(q_curr[1]);
    Serial.print(", ");
    Serial.print(q_goal[1]);
    Serial.print(", ");
    Serial.print(q_curr[2]);
    Serial.print(", ");
    Serial.print(q_goal[2]);
    Serial.print(", ");
    Serial.print(getMotorLoads());
    Serial.println();
  }


  // Set joint angles
  int th2 = map(q_goal[1] + 150.0, 0.0, 300.0, 0, 1023);
  int th3 = map(q_goal[2] + 150.0, 0.0, 300.0, 0, 1023);

//  setM1Position(th2);
//  setM2Position(th3);
//  setM1Position(512);
//  setM2Position(512);

  currTime = micros() - currTime;

  if (goal_loop_time > currTime) {
    delayMicroseconds(goal_loop_time - currTime);
  }

  loop_timer = micros() - loop_timer;

  //  Serial.print("Loop Time: ");
  //  Serial.print(loop_timer);
//  Serial.println();
  //  delay(100);


}

//double* get_states() {
//  //read servo positions
//  //Inputs: not sure
//  //Outputs: q{Vector} --> Servo positions 3x1
//  double* q = static_cast<double *>(malloc(3 * sizeof(double)));
//
//  q[0] = getJ0EncoderPos();
//
//  q[1] = getM1Position();
//
//  q[2] = getM2Position();
//
//  return q;
//}


// Function to find the closest point in a 2-D array to a 1-D array
void find_closest_point(double*& q_goal, double* q, double**& traj, int n) {
  double min_dist = INFINITY;
  double closestPoint[3] = {0.0, 0.0, 0.0};

  for (int i = 0; i < n; i++) {
    double d1 = q[0] - traj[0][i];
    //    double d2 = q[1] - traj[1][i];
    //    double d3 = q[2] - traj[2][i];
    double dist = sqrt(d1 * d1);
    //    Serial.print("Dist: ");
    //    Serial.println(dist);
    if (dist < min_dist) {
      min_dist = dist;
      closestPoint[0] = traj[0][i];
      closestPoint[1] = traj[1][i];
      closestPoint[2] = traj[2][i];
    }
  }

  //    Serial.print("Closest Point = {");
  //    Serial.print(closestPoint[0]);
  //    Serial.print(", ");
  //    Serial.print(closestPoint[1]);
  //    Serial.print(", ");
  //    Serial.print(closestPoint[2]);
  //    Serial.println("}\t");

  for (int i = 0; i < 3; i++) {
    q_goal[i] = closestPoint[i];
  }

  //  Serial.print("Closest Point = {");
  //  Serial.print(q_goal[0]);
  //  Serial.print(", ");
  //  Serial.print(q_goal[1]);
  //  Serial.print(", ");
  //  Serial.print(q_goal[2]);
  //  Serial.println("}\t");

}


void calculate_traj(double**& traj, double * q_curr, double * q_goal, int n) {
  int reverse;
  double t1 = 0;
  double t2 = 1;

  if (traj == nullptr) {
    traj = new double*[3];
    for (int i = 0; i < 3; i++) {
      traj[i] = new double[n];
    }
  }


  double trajTime[n];
  double th1_traj[n];
  for (int i = 0; i < n; i++) {
    trajTime[i] = t1 + ((double) i) * (t2 - t1) / (((double) n) - 1.0);
    th1_traj[i] = q_curr[0] + ((double) i) * (q_goal[0] - q_curr[0]) / (((double) n) - 1.0);
    traj[0][i] = th1_traj[i];
  }

  for (int i = 1; i < 3; i++) {
    for (int j = 0; j < n; j++) {
      double tau = (trajTime[j] - t1) / (t2 - t1);
      traj[i][j] = q_curr[i] + (q_goal[i] - q_curr[i]) * (6.0 * tau * tau * tau * tau * tau - 15.0 * tau * tau * tau * tau + 10.0 * tau * tau * tau);
    }
  }

}


int checkGuard(double * q_n, double*& hollow_region, double*& arch_region, int contactMode) {
  static double val_prev[4] = {0.0, 0.0, 0.0, 0.0};
  static int set_prev = 0;

  double val_curr[4];
  int trippedGuard = 0;


  val_curr[0] = q_n[0] - arch_region[0];
  val_curr[1] = q_n[0] - arch_region[1];
  val_curr[2] = q_n[0] - hollow_region[0];
  val_curr[3] = q_n[0] - hollow_region[1];

  //    Serial.print("Guard Values: {");
  //    Serial.print(val_curr[0]);
  //    Serial.print(", ");
  //    Serial.print(val_curr[1]);
  //    Serial.print(", ");
  //    Serial.print(val_curr[2]);
  //    Serial.print(", ");
  //    Serial.print(val_curr[3]);
  //    Serial.print("}");
  //    Serial.print("\t");
  //  Serial.println();

//  if (val_curr[0]*val_prev[0] <= 0 && val_curr[0] < val_prev[0]) { //Entering arch region with negative velocity
//    //    Serial.println("Guard 1 Tripped");
//    trippedGuard = 2;
//    //    Serial.println("Guard Tripped Set");
//  } else if (val_curr[0]*val_prev[0] <= 0 && val_curr[0] > val_prev[0]) { //Exiting arch region with negative velocity
//    if (contactMode == 2 || contactMode == 5) {
//      trippedGuard = 1;
//    }
//    //        Serial.println("Guard 2 Tripped");
//  } else if (val_curr[1]*val_prev[1] <= 0 && val_curr[1] < val_prev[1]) { //Exiting arch region with negative velocity
//    trippedGuard = 3;
//    //        Serial.println("Guard 2 Tripped");
//  } else if (val_curr[1]*val_prev[1] <= 0 && val_curr[1] > val_prev[1]) { //Exiting arch region with negative velocity
//    trippedGuard = 2;
//    //        Serial.println("Guard 2 Tripped");
//  } else if (val_curr[2]*val_prev[2] <= 0 && val_curr[2] < val_prev[2]) { //Entering hollow region with negative velocity
//    trippedGuard = 4;
//    //        Serial.println("Guard 3 Tripped");
//  } else if (val_curr[2]*val_prev[2] <= 0 && val_curr[2] > val_prev[2]) { //Entering hollow region with negative velocity
//    trippedGuard = 3;
//    //        Serial.println("Guard 3 Tripped");
//  } else if (val_curr[3]*val_prev[3] <= 0 && val_curr[3] < val_prev[3]) { //Exiting hollow region with negative velocity
//    trippedGuard = 5;
//    //            Serial.println("Guard 4 Tripped");
//  } else if (val_curr[3]*val_prev[3] <= 0 && val_curr[3] > val_prev[3]) { //Entering hollow region with negative velocity
//    trippedGuard = 4;
//    //        Serial.println("Guard 3 Tripped");
//  }


  if (q_n[0] > arch_region[0]) {
    trippedGuard = 1;
  }
  if (q_n[0] <= arch_region[0] && q_n[0] >= arch_region[1]) {
    trippedGuard = 2;
  }
  if (q_n[0] <= arch_region[1] && q_n[0] >= hollow_region[0]) {
    trippedGuard = 3;
  }
  if (q_n[0] <= hollow_region[0] && q_n[0] >= hollow_region[1]) {
    trippedGuard = 4;
  }
  if (q_n[0] < hollow_region[1]) {
    trippedGuard = 5;
  }

  val_prev[0] = val_curr[0];
  val_prev[1] = val_curr[1];
  val_prev[2] = val_curr[2];
  val_prev[3] = val_curr[3];
  set_prev = 0;

  return trippedGuard;
}
