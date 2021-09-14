#ifndef KINEMATICS_ACCELERATION_H
#define KINEMATICS_ACCELERATION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kinematics_position.h"
#include "kinematics_velocity.h"
#include "matrix.h"

#define PGI_X *mat_elem(pg_I, X, 0)
#define PGI_Y *mat_elem(pg_I, Y, 0)
#define PGI_Z *mat_elem(pg_I, Z, 0)

#define QI_1 *mat_elem(q_I, I1, 0)
#define QI_2 *mat_elem(q_I, I2, 0)
#define QI_3 *mat_elem(q_I, I3, 0)

//derivata matrice jacobiano
#define dJ {2*PGI_X + 2*A*sin(Q_1)*QI_1,\
                2*PGI_Y,\
                    2*(PGI_Z + A*cos(Q_1)*QI_1),\
            \
            2*PGI_X - A*sin(Q_2)*QI_2,\
                2*PGI_Y + sqrt(3)*A*sin(Q_2)*QI_2,\
                    2*(PGI_Z + A*cos(Q_2)*QI_2),\
            \
            2*PGI_X - A*sin(Q_3)* QI_3,\
                2*PGI_Y - sqrt(3)*A*sin(Q_3)*QI_3,\
                    2*(PGI_Z + A*cos(Q_3)*QI_3)}

//derivata matrice dei rapporti di velocità
#define dRV {2*A*sin(Q_1)*PGI_X +\
             2*A*cos(Q_1)*PGI_Z +\
             2*A*cos(Q_1)*QI_1*PG_X -\
             2*A*sin(Q_1)*QI_1*PG_Z -\
             2*A*cos(Q_1)*K*QI_1,\
                 0,\
                     0,\
             \
             0,\
                 -A*sin(Q_2)*PGI_X +\
                 A*sqrt(3)*sin(Q_2)*PGI_Y +\
                 2*A*cos(Q_2)*PGI_Z -\
                 A*cos(Q_2)*QI_2*PG_X +\
                 A*sqrt(3)*cos(Q_2)*QI_2*PG_Y -\
                 2*A*sin(Q_2)*QI_2*PG_Z -\
                 2*A*cos(Q_2)*K*QI_2,\
                     0,\
             \
             0,\
                 0,\
                     -A*sin(Q_3)*PGI_X -\
                     A*sqrt(3)*sin(Q_3)*PGI_Y +\
                     2*A*cos(Q_3)*PGI_Z -\
                     A*cos(Q_3)*QI_3*PG_X -\
                     A*sqrt(3)*cos(Q_3)*QI_3*PG_Y -\
                     2*A*sin(Q_3)*QI_3*PG_Z -\
                     2*A*cos(Q_3)*K*QI_3}

matrix *get_motor_acc(const matrix *pg, const matrix *q, const matrix *pg_I, const matrix *q_I, const matrix *pg_II);

void update_motor_acc(const matrix *pg, const matrix *q, const matrix *pg_I, const matrix *q_I, const matrix *pg_II, matrix *out);

void valid_acc(const matrix *vel);

#endif