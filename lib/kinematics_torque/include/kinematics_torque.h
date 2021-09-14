#ifndef KINEMATICS_TORQUE_H
#define KINEMATICS_TORQUE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kinematics_position.h"
#include "kinematics_velocity.h"
#include "kinematics_acceleration.h"
#include "matrix.h"

#define G 9.80665

#define PGII_X *mat_elem(pg_II, X, 0)
#define PGII_Y *mat_elem(pg_II, Y, 0)
#define PGII_Z *mat_elem(pg_II, Z, 0)

#define M_UP_ARM 0.836
#define M_BAR 0.1
#define M_MOV_PLAT 0.65
#define M_UP_JOINT 0.1
#define M_DWN_JOINT 0.2
#define M_SPRING 0.03
#define M_RING 0.005

#define M1 (M_BAR + 2*M_UP_JOINT + M_SPRING + 2*M_RING)
#define M2 (M_BAR + 2*M_DWN_JOINT + M_SPRING + 2*M_RING)
#define PAYLOAD (m + M_MOV_PLAT)
#define IQ ((1.0/3.0*M_UP_ARM + M1)*pow(A, 2))

#define F {(PAYLOAD + M2)*PGII_X,\
           (PAYLOAD + M2)*PGII_Y,\
           (PAYLOAD + M2)*(PGII_Z + G)}



matrix *get_motor_tor(const matrix *pg, const matrix *q, const matrix *pg_II, const matrix *q_II, const float m);

void update_motor_tor(const matrix *pg, const matrix *q, const matrix *pg_II, const matrix *q_II, const float m, matrix *out);

#endif