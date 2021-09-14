#include <stdio.h>
#include "matrix.h"
#include "kinematics_position.h"
#include "kinematics_velocity.h"
#include "kinematics_acceleration.h"
#include "kinematics_torque.h"
#include "path_generation.h"


#define POINTS 2


int main(void)
{

/*
    double points_elem[3*POINTS] = {    +0.0,   +0.0,   +0.1,   -0.2,   -0.2,
                                        +0.0,   +0.0,   +0.2,   +0.2,   +0.0,
                                        -0.6,   -0.7,   -0.7,   -0.7,   -0.8};
*/
    
    double points_elem[3*POINTS] = {    +0.0,   +0.1,
                                        +0.0,   +0.3,
                                        -0.6,   -0.7};

/*
    double points_elem[3*POINTS] = {    +0.4,   -0.1,
                                        +0.0,   +0.3,
                                        -0.6,   -0.8};
*/
/*
    double points_elem[3*POINTS] = {    +0.0,   +0.0,    +0.2,   -0.1,   +0.3,   -0.3,   -0.1,
                                        +0.0,   -0.2,    +0.3,   +0.1,   +0.2,   +0.3,   +0.0,
                                        -0.6,   -0.7,    -0.6,   -0.7,   -0.7,   -0.7,   -0.8};
*/

    matrix *points = fill_matrix(3, POINTS, points_elem);
    matrix *pos = NULL;
    matrix *vel = NULL;
    matrix *acc = NULL;

    path(points, 1.0, 0.01, 500, &pos, &vel, &acc);

    write_data(pos, vel, acc, 1.0);

    free_matrix(points);
    free_matrix(pos);
    free_matrix(vel);
    free_matrix(acc);

    printf("Done!\n");

    return 0;
}