#ifndef KINEMATICS_VELOCITY_H
#define KINEMATICS_VELOCITY_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kinematics_position.h"
#include "matrix.h"

#define RAD_S_TO_RPM (60 /(2*M_PI))

#define PG_X *mat_elem(pg, X, 0)
#define PG_Y *mat_elem(pg, Y, 0)
#define PG_Z *mat_elem(pg, Z, 0)

#define Q_1 *mat_elem(q, I1, 0)
#define Q_2 *mat_elem(q, I2, 0)
#define Q_3 *mat_elem(q, I3, 0)

//matrice jacobiano
#define J {2*PG_X - 2*A*cos(Q_1) - 2*K,\
               2*PG_Y,\
                   2*(PG_Z + A*sin(Q_1)),\
           \
           2*PG_X + A*cos(Q_2) + K,\
               2*PG_Y - sqrt(3)*(K + A*cos(Q_2)),\
                   2*(PG_Z + A*sin(Q_2)),\
           \
           2*PG_X + A*cos(Q_3) + K,\
               2*PG_Y + sqrt(3)*(K + A*cos(Q_3)),\
                   2*(PG_Z + A*sin(Q_3))}

//matrice dei rapporti di velocità
#define RV {2*A*sin(Q_1)*PG_X +\
            2*A*cos(Q_1)*PG_Z -\
            2*A*sin(Q_1)*K,\
                0,\
                    0,\
            \
            0,\
                -A*sin(Q_2)*PG_X +\
                A*sqrt(3)*sin(Q_2)*PG_Y +\
                2*A*cos(Q_2)*PG_Z -\
                2*A*sin(Q_2)*K,\
                    0,\
            \
            0,\
                0,\
                    -A*sin(Q_3)*PG_X -\
                    A*sqrt(3)*sin(Q_3)*PG_Y +\
                    2*A*cos(Q_3)*PG_Z -\
                    2*A*sin(Q_3)*K}

matrix *get_motor_vel(const matrix *pg, const matrix *q, const matrix *pg_I);

void update_motor_vel(const matrix *pg, const matrix *q, const matrix *pg_I, matrix *out);

void valid_vel(const matrix *vel);

#endif