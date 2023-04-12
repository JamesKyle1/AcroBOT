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
#define BETA            250.0
#define GAMMA           45.0


// ================================== /
// ======== Dynamixel Setup ========= /
// ================================== /

DynamixelWorkbench dxl_wb;

uint8_t dxl_id0 = 8;
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

int n = 20;

double alpha[3] = {0.0, 0.0, 0.0};

double** traj = nullptr;
double* q_goal = new double[3];

double* hollow_region = new double[2];
double* arch_region = new double[2];

int contactMode = 0;
double tol = 1e-6;





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

}

void loop() {
  // put your main code here, to run repeatedly:

  double t1 = 1;
  double t2 = 5;

  double q0 = getJ0EncoderPos();
  double q1 = getM1Position() * 300.0 / 1024.0 - 150.0;
  double q2 = getM2Position() * 300.0 / 1024.0 - 150.0;
  double q_curr[3] = {q0, q1, q2}; //Should be set to the current states of the joints

  alpha[0] = ALPHA;
  alpha[1] = -ALPHA;
  alpha[2] = ALPHA;

  int guardTriggered = checkGuard(q_curr, hollow_region, arch_region);

  if (guardTriggered > 0) {
    contactMode = guardTriggered;

    if (contactMode == 2) {
      alpha[0] = ALPHA;
      alpha[1] = -ALPHA;
      alpha[2] = ALPHA;
    } else if (contactMode == 4) {
      alpha[0] = -ALPHA;
      alpha[1] = ALPHA;
      alpha[2] = -ALPHA;
    } else {
      alpha[0] = 0.0;
      alpha[1] = 0.0;
      alpha[2] = 0.0;
    }

    calculate_traj(traj, q_curr, alpha, t1, t2, n);
  }

  //Set goal angle based on current position
  if (contactMode == 1 || contactMode == 3) {
    q_goal[0] = 0.0;
    q_goal[1] = 0.0;
    q_goal[2] = 0.0;
  }
  
  if (contactMode == 2 || contactMode == 4) {
    if ((q_goal[0] - alpha[0]) * (q_goal[0] - alpha[0]) < tol * tol) {
      if ((q_goal[1] - alpha[1]) * (q_goal[1] - alpha[1]) < tol * tol) {
        if ((q_goal[2] - alpha[2]) * (q_goal[2] - alpha[2]) < tol * tol) {
          q_goal[0] = alpha[0];
          q_goal[1] = alpha[1];
          q_goal[2] = alpha[2];
        }
      }
    } else {
      find_closest_point(q_goal, q_curr, traj, n);
    }
  }

  //  Serial.print("Trajectory: {");
  //  for (int i = 0; i < 3; i++) {
  //    Serial.print("{");
  //    for (int j = 0; j < n; j++) {
  //      Serial.print(traj[1][i]);
  //      Serial.print(", ");
  //    }
  //    Serial.print("}, \n");
  //  }
  //  Serial.print("}");

  Serial.print("Contact Mode: ");
  Serial.print(contactMode);
  Serial.print("\t\t");
  //
  //  Serial.print("Alpha: {");
  //  Serial.print(alpha[0]);
  //  Serial.print(", ");
  //  Serial.print(alpha[1]);
  //  Serial.print(", ");
  //  Serial.print(alpha[2]);
  //  Serial.print("}");
  //  Serial.print("\t");

  Serial.print("J0: ");
  Serial.print(q_curr[0]);
  Serial.print("\t\t");
  //  Serial.print("J0 Goal: ");
  //  Serial.print(q_goal[0]);
  //  Serial.print("\t");

    Serial.print("J1: ");
    Serial.print(q_curr[1]);
    Serial.print("\t");
  Serial.print("J1 Goal: ");
  Serial.print(q_goal[1]);
  Serial.print("\t\t");

    Serial.print("J2: ");
    Serial.print(q_curr[2]);
    Serial.print("\t");
  Serial.print("J2 Goal: ");
  Serial.print(q_goal[2]);

  double angle = 150.0;
  int angle_in = map(angle, 0.0, 300.0, 0, 1023);

  setM1Position(angle_in);
  setM2Position(angle_in);

  Serial.println();

  delay(50);



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

  for (int i = 0; i < n; i++) {
    double d1 = q[0] - traj[0][i];
    double d2 = q[1] - traj[1][i];
    double d3 = q[2] - traj[2][i];
    double dist = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
    if (dist < min_dist) {
      min_dist = dist;
      q_goal[0] = traj[0][i];
      q_goal[1] = traj[1][i];
      q_goal[2] = traj[2][i];
    }
  }
}


void calculate_traj(double**& traj, double * q_curr, double * q_goal, double ts, double tf, int n) {
  int reverse;
  double t1, t2;

  if (traj == nullptr) {
    traj = new double*[3];
    for (int i = 0; i < 3; i++) {
      traj[i] = new double[n];
    }
  }

  if (tf < ts) {
    t1 = tf;
    t2 = ts;
    reverse = 1;
  } else {
    t1 = ts;
    t2 = tf;
    reverse = 0;
  }

  double trajTime[n];
  for (int i = 0; i < n; i++) {
    trajTime[i] = t1 + ((double) i) * (t2 - t1) / (((double) n) - 1.0);
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < n; j++) {
      double tau = (trajTime[j] - t1) / (t2 - t1);

      traj[i][j] = (double)(q_curr[i] + (q_goal[i] - q_curr[i]) * (6.0 * pow(tau, 5.0) - 15.0 * pow(tau, 4.0) + 10.0 * pow(tau, 3.0)));
    }
  }

}


int checkGuard(double * q_n, double*& hollow_region, double*& arch_region) {
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

  if (val_curr[0]*val_prev[0] <= 0 && val_curr[0] < val_prev[0]) { //Entering arch region with negative velocity
    //    Serial.println("Guard 1 Tripped");
    trippedGuard = 2;
    //    Serial.println("Guard Tripped Set");
  } else if (val_curr[0]*val_prev[0] <= 0 && val_curr[0] > val_prev[0]) { //Exiting arch region with negative velocity
    trippedGuard = 1;
    //        Serial.println("Guard 2 Tripped");
  } else if (val_curr[1]*val_prev[1] <= 0 && val_curr[1] < val_prev[1]) { //Exiting arch region with negative velocity
    trippedGuard = 3;
    //        Serial.println("Guard 2 Tripped");
  } else if (val_curr[1]*val_prev[1] <= 0 && val_curr[1] > val_prev[1]) { //Exiting arch region with negative velocity
    trippedGuard = 2;
    //        Serial.println("Guard 2 Tripped");
  } else if (val_curr[2]*val_prev[2] <= 0 && val_curr[2] < val_prev[2]) { //Entering hollow region with negative velocity
    trippedGuard = 4;
    //        Serial.println("Guard 3 Tripped");
  } else if (val_curr[2]*val_prev[2] <= 0 && val_curr[2] > val_prev[2]) { //Entering hollow region with negative velocity
    trippedGuard = 3;
    //        Serial.println("Guard 3 Tripped");
  } else if (val_curr[3]*val_prev[3] <= 0 && val_curr[3] < val_prev[3]) { //Exiting hollow region with negative velocity
    trippedGuard = 1;
    //        Serial.println("Guard 4 Tripped");
  } else if (val_curr[3]*val_prev[3] <= 0 && val_curr[3] > val_prev[3]) { //Entering hollow region with negative velocity
    trippedGuard = 4;
    //        Serial.println("Guard 3 Tripped");
  }

  val_prev[0] = val_curr[0];
  val_prev[1] = val_curr[1];
  val_prev[2] = val_curr[2];
  val_prev[3] = val_curr[3];
  set_prev = 0;

  return trippedGuard;
}
