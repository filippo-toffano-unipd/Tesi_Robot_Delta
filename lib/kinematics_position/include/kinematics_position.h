#ifndef KINEMATICS_POSITION_H
#define KINEMATICS_POSITION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"

//coordinate
#define X 0
#define Y 1
#define Z 2
#define I1 0
#define I2 1
#define I3 2

#define RAD_TO_DEG (180.0 / M_PI)

#define MAX_RADIUS (0.646 - H)
#define MAX_Z (-(0.865 - 0.275))
#define MIN_Z (-(1.115 - 0.275))

#define A 0.366
#define B 0.800
#define R (0.565 - A)
#define H 0.048
#define K (R - H)

//coefficienti sfera
#define ALPHA (-2 * *mat_elem(pg, X, 0))
#define BETA (-2 * *mat_elem(pg, Y, 0))
#define GAMMA (-2 * *mat_elem(pg, Z, 0))
#define DELTA (pow(*mat_elem(pg, X, 0), 2) + pow(*mat_elem(pg, Y, 0), 2) + pow(*mat_elem(pg, Z, 0), 2) - pow(B, 2))

//coefficienti per il calcolo di q1
#define K1 (pow(A, 2) + pow(K, 2) + ALPHA*K + DELTA)
#define A1 (K1 - A*(2*K + ALPHA))
#define B1 (-2*GAMMA*A)
#define C1 (K1 + A*(2*K + ALPHA))

//coefficienti per il calcolo di q2
#define K2 (pow(A, 2) + pow(K, 2) - ALPHA/2 *K + sqrt(3)/2 *BETA*K + DELTA)
#define A2 (K2 - A*(2*K - ALPHA/2 + sqrt(3)/2 *BETA))
#define B2 B1
#define C2 (K2 + A*(2*K - ALPHA/2 + sqrt(3)/2 *BETA))

//coefficienti per il calcolo q3
#define K3 (pow(A, 2) + pow(K, 2) - ALPHA/2 *K - sqrt(3)/2 *BETA*K + DELTA)
#define A3 (K3 - A*(2*K - ALPHA/2 - sqrt(3)/2 *BETA))
#define B3 B1
#define C3 (K3 + A*(2*K - ALPHA/2 - sqrt(3)/2 *BETA))

matrix *get_motor_pos(const matrix *pg);

void update_motor_pos(const matrix *pg, matrix *out);

void valid_pos(const matrix *pg);


#endif