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
#define ALPHA           PI/4.0
#define BETA            PI/6.0
#define GAMMA           PI/4.0


// ================================== /
// ======== Dynamixel Setup ========= /
// ================================== /

DynamixelWorkbench dxl_wb;

uint8_t dxl_id0 = 1;
uint8_t dxl_id1 = 6;
 

//for the encoder data from UNO
const byte numChars = 32;
char receivedChars[numChars];
float encoderPos = 0.0;

// the current sensor is connected to A0
const int analogInPin = A0;



// ================================= /
// ======= Controller Setup ======== /
// ================================= /

double hollow_region[2] = {PI + BETA + GAMMA, PI + BETA};
double arch_region[2] = {2 * PI - BETA, 2 * PI - BETA - GAMMA};

int n = 20;

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

}

void loop() {
  // put your main code here, to run repeatedly:

  double t1 = 5;
  double t2 = 1;

  double* q_curr = get_states(); //Should be set to the current states of the joints
  double alpha[3] = {0.0, 0.0, 0.0};

  double** traj = static_cast<double **>(calloc(n, 3 * sizeof(double)));
  double* q_goal = static_cast<double *>(malloc(3 * sizeof(double)));

  int guardTriggered = checkGuard(q_curr, hollow_region, arch_region);

  if (guardTriggered > 0) {
    contactMode = guardTriggered;

    if (contactMode == 1) {
      alpha[0] = ALPHA;
      alpha[1] = ALPHA;
      alpha[2] = ALPHA;
    } else if (contactMode == 3) {
      alpha[0] = -ALPHA;
      alpha[1] = -ALPHA;
      alpha[2] = -ALPHA;
    } else {
      alpha[0] = 0.0;
      alpha[1] = 0.0;
      alpha[2] = 0.0;
    }

    traj = calculate_traj(q_curr, alpha, t1, t2, n);
  }


  //Set goal angle based on current position
  if ((q_goal[0] - alpha[0]) * (q_goal[0] - alpha[0]) < tol * tol) {
    if ((q_goal[1] - alpha[1]) * (q_goal[1] - alpha[1]) < tol * tol) {
      if ((q_goal[2] - alpha[2]) * (q_goal[2] - alpha[2]) < tol * tol) {
        q_goal[0] = alpha[0];
        q_goal[1] = alpha[1];
        q_goal[2] = alpha[2];
      }
    }
  } else {
    q_goal = find_closest_point(q_curr, traj, n);
  }
  

  Serial.print("J0: ");
  Serial.print(q_curr[0]);
  Serial.print("\t");
  Serial.print("J1: ");
  Serial.print(q_curr[1]);
  Serial.print("\t");
  Serial.print("J2: ");
  Serial.print(q_curr[2]);
  Serial.println();



}

double* get_states() {
  //read servo positions
  //Inputs: not sure
  //Outputs: q{Vector} --> Servo positions 3x1
  double* q = static_cast<double *>(malloc(3 * sizeof(double)));

  q[0] = getJ0EncoderPos();

  q[1] = getM1Position();

  q[2] = getM2Position();

  return q;
}


// Function to find the closest point in a 2-D array to a 1-D array
double* find_closest_point(double* q, double** traj, int n) {
  double min_dist = INFINITY;
  double* q_goal = NULL;
  for (int i = 0; i < n; i++) {
    double d1 = q[0] - traj[i][0];
    double d2 = q[1] - traj[i][1];
    double d3 = q[2] - traj[i][2];
    double dist = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
    if (dist < min_dist) {
      min_dist = dist;
      q_goal = traj[i];
    }
  }
  return q_goal;
}

double* linspace(double t1, double t2, int n) {
  double* vec = static_cast<double *>(malloc(n * sizeof(double)));
  double dt = (t2 - t1) / (n - 1);
  for (int i = 0; i < n; i++) {
    vec[i] = t1 + i * dt;
  }
  return vec;
}

double** calculate_traj(double* q_curr, double* q_goal, double ts, double tf, int n) {
  int reverse;
  double t1, t2;
  double** trajPos = static_cast<double **>(calloc(3,n * sizeof(double*)));
  
  if (tf < ts) {
    t1 = tf;
    t2 = ts;
    reverse = 1;
  } else {
    t1 = ts;
    t2 = tf;
    reverse = 0;
  }
  double* trajTime = linspace(t1, t2, n);
  for (int i = 0; i < n; i++) {
    double tau = (trajTime[i] - t1) / (t2 - t1);
    for (int j = 0; j < 3; j++) {
      trajPos[i][j] = q_curr[j] + (q_goal[j] - q_curr[j]) * (6 * pow(tau, 5) - 15 * pow(tau, 4) + 10 * pow(tau, 3));
    }
  }
  free(trajTime);
  return trajPos;
}


int checkGuard(double* q_n, double* hollow_region, double* arch_region) {
  static double val_prev[4] = {0.0, 0.0, 0.0, 0.0};

  double* val_curr = static_cast<double *>(malloc(4 * sizeof(double)));
  int trippedGuard = 0;


  val_curr[0] = (2 * PI + q_n[0]) - arch_region[0];
  val_curr[1] = (2 * PI + q_n[0]) - arch_region[1];
  val_curr[2] = (2 * PI + q_n[0]) - hollow_region[0];
  val_curr[3] = (2 * PI + q_n[0]) - hollow_region[1];

  if (val_curr[0]*val_prev[0] < 0 && val_curr[0] < val_prev[0]) { //Entering arch region with negative velocity
    trippedGuard = 1;
  } else if (val_curr[1]*val_prev[1] < 0 && val_curr[1] < val_prev[1]) { //Exiting arch region with negative velocity
    trippedGuard = 2;
  } else if (val_curr[2]*val_prev[2] < 0 && val_curr[2] < val_prev[2]) { //Entering hollow region with negative velocity
    trippedGuard = 3;
  } else if (val_curr[3]*val_prev[3] < 0 && val_curr[3] < val_prev[3]) { //Exiting hollow region with negative velocity
    trippedGuard = 4;
  }

    for (int i = 0; i < 4; i++) {
        val_prev[i] = val_curr[i];
    }

  return trippedGuard;
}
