//#include <stdio.h>
//#include <math.h>
//#include <stdlib.h>
//
//#define INFINITY 1.0/0.0 // define INFINITY constant
//
//// Function to find the closest point in a 2-D array to a 1-D array
//double* find_closest_point(double* q, double** traj, int n) {
//    double min_dist = INFINITY;
//    double* q_goal = NULL;
//    for (int i = 0; i < n; i++) {
//        double d1 = q[0] - traj[i][0];
//        double d2 = q[1] - traj[i][1];
//        double d3 = q[2] - traj[i][2];
//        double dist = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
//        if (dist < min_dist) {
//            min_dist = dist;
//            q_goal = traj[i];
//        }
//    }
//    return q_goal;
//}
//
//double* linspace(double t1, double t2, int n) {
//    double* vec = malloc(n * sizeof(double));
//    double dt = (t2-t1)/(n-1);
//    for (int i = 0; i < n; i++) {
//        vec[i] = t1 + i*dt;
//    }
//    return vec;
//}
//
//double** calculate_traj(double* q_curr, double* q_goal, double ts, double tf, int n){
//    int reverse;
//    double t1, t2;
//    double** trajPos = malloc(n * sizeof(double*));
//    for (int i = 0; i < n; i++) {
//        trajPos[i] = malloc(3 * sizeof(double));
//    }
//    if (tf < ts) {
//        t1 = tf;
//        t2 = ts;
//        reverse = 1;
//    } else {
//        t1 = ts;
//        t2 = tf;
//        reverse = 0;
//    }
//    double* trajTime = linspace(t1, t2, n);
//    for (int i = 0; i < n; i++) {
//        double tau = (trajTime[i] - t1)/(t2-t1);
//        for (int j = 0; j < 3; j++) {
//            trajPos[i][j] = q_curr[j] + (q_goal[j] - q_curr[j])*(6*pow(tau, 5) - 15*pow(tau, 4) + 10*pow(tau, 3));
//        }
//    }
//    free(trajTime);
//    return trajPos;
//}
//
//// Example usage of the function
//int main() {
//    double t1 = 5;
//    double t2 = 1;
//    int n = 20;
//    
//    double q_curr[3] = {1.0, 1.0, 1.0};
//    double q_n[3] = {4.0, 4.0, 3.0};
//    double alpha[3] = {5.0, 5.0, 5.0};
//    
//    double** traj = calculate_traj(q_curr, alpha, t1, t2, n);
//    double* q_goal = find_closest_point(q_n, traj, n);
//    
//    return 0;
//}
