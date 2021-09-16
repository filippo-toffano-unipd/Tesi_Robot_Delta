#ifndef PATH_H
#define PATH_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "kinematics_position.h"
#include "kinematics_velocity.h"
#include "kinematics_acceleration.h"
#include "kinematics_torque.h"
#include "matrix.h"

double distance(const matrix *points);

double dist_p_to_p(const matrix *points, uint from, uint to);

void update_directions(const matrix *points, matrix *prev_dir, matrix *curr_dir, matrix *next_dir, const int i);

matrix *time_division(const matrix *points, const double time);

double path_smoothness(const matrix *pos, const matrix *vel);

void p_to_p(const matrix *pos_points, const matrix *vel_points, const matrix *acc_points, const double time, matrix *pos_func, matrix *vel_func, matrix *acc_func);

void quantization(const matrix *function, const double time, const uint resolution, matrix *out);

void path(const matrix *points, const double time, const double tune_precision, const uint resolution, matrix **pos, matrix **vel, matrix **acc);

void write_data(const matrix *pos, const matrix *vel, const matrix *acc, const double m);

void circular_path(const double radius, const double z, const double period, const uint resolution, matrix **pos, matrix **vel, matrix **acc);

#endif