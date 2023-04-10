//#include <stdio.h>
//#include <math.h>
//
//// Function to find the closest point in a 2-D array to a 1-D array
//float* closest_point(float* q, float** traj, int n, int m) {
//    float min_dist = INFINITY;
//    float* closest = NULL;
//    for (int i = 0; i < n; i++) {
//        float d1 = q[0] - traj[i][0];
//        float d2 = q[1] - traj[i][1];
//        float d3 = q[1] - traj[i][1];
//        float dist = sqrt(d1 * d1 + d2 * d2 + d3 * d3);
//        if (dist < min_dist) {
//            min_dist = dist;
//            closest = traj[i];
//        }
//    }
//    return closest;
//}
//
//// Example usage of the function
//int main() {
//    float q[] = {12.0, 12.0, 3.0};
//    float* currTraj[][3] = {(float[]){0.0, 0.0, 0.0}, (float[]){1.0, 1.0, 1.0},
//                    (float[]){2.0, 2.0, 2.0}, (float[]){2.0, 3.0, 3.0},
//                    (float[]){4.0, 6.0, 5.0}, (float[]){7.0, 7.0, 7.0},
//                    (float[]){10.0, 10.0, 20.0}, (float[]){1.0, 5.0, 5.0},
//                    (float[]){7.0, 9.0, 9.0}, (float[]){8.0, 8.0, 10.0}};
//                    
//    int n = 10;
//    int m = 3;
//    float* closest = closest_point(q, currTraj, n, m);
//    if (closest != NULL) {
//        printf("Closest point: (%f, %f, %f)\n", closest[0], closest[1], closest[2]);
//    } else {
//        printf("Error: closest point is NULL\n");
//    }
//    return 0;
//}
