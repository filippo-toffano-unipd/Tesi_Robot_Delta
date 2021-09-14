#include "path_generation.h"


double distance(const matrix *points){
    double out = 0.0;

    for (int i = 0; i < points->mxn[1] - 1; i++)
        out = out + sqrt(pow(*mat_elem(points, X, i + 1) -*mat_elem(points, X, i), 2) +
                         pow(*mat_elem(points, Y, i + 1) -*mat_elem(points, Y, i), 2) +
                         pow(*mat_elem(points, Z, i + 1) -*mat_elem(points, Z, i), 2));

    return out;
}


double dist_p_to_p(const matrix *points, uint from, uint to){
    if ((int)from > points->mxn[1] || (int)to > points->mxn[1]){
        printf("Error <invalid point index input>\n");
        exit(EXIT_FAILURE);
    }

    double out = 0.0;

    for (int i = (int)from; i < (int)to; i++)
        out = out + sqrt(pow(*mat_elem(points, X, i + 1) -*mat_elem(points, X, i), 2) +
                         pow(*mat_elem(points, Y, i + 1) -*mat_elem(points, Y, i), 2) +
                         pow(*mat_elem(points, Z, i + 1) -*mat_elem(points, Z, i), 2));

    return out;
}


void update_directions(const matrix *points, matrix *prev_dir, matrix *curr_dir, matrix *next_dir, const int i){
    if (points->mxn[1] == 2){
        for (int j = 0; j < 3; j++){
            *mat_elem(curr_dir, j, 0) = (*mat_elem(points, j, i + 1) - *mat_elem(points, j, i)) / dist_p_to_p(points, i, i + 1);
        }
    }
    
    else{
        for (int j = 0; j < 3; j++){
            if (i == 0){
                *mat_elem(curr_dir, j, 0) = (*mat_elem(points, j, i + 1) - *mat_elem(points, j, i)) / dist_p_to_p(points, i, i + 1);
                *mat_elem(next_dir, j, 0) = (*mat_elem(points, j, i + 2) - *mat_elem(points, j, i + 1)) / dist_p_to_p(points, i + 1, i + 2);
            }

            else if (i == (points->mxn[1] - 1) - 1){
                *mat_elem(prev_dir, j, 0) = *mat_elem(curr_dir, j, 0);
                *mat_elem(curr_dir, j, 0) = *mat_elem(next_dir, j, 0);
                *mat_elem(next_dir, j, 0) = 0.0;
            }

            else{
                *mat_elem(prev_dir, j, 0) = *mat_elem(curr_dir, j, 0);
                *mat_elem(curr_dir, j, 0) = *mat_elem(next_dir, j, 0);
                *mat_elem(next_dir, j, 0) = (*mat_elem(points, j, i + 2) - *mat_elem(points, j, i + 1)) / dist_p_to_p(points, i + 1, i + 2);
            }
        }
    }
}


matrix *time_division(const matrix *points, const double time){
    matrix *out = new_matrix(1, points->mxn[1] - 1);

    if (points->mxn[1] == 2)
        *mat_elem(out, 0, 0) = time;
    else{
        double total_dist = distance(points);

        for (int i = 0; i < points->mxn[1] - 1; i++)
            *mat_elem(out, 0, i) = sqrt(pow(*mat_elem(points, X, i + 1) -*mat_elem(points, X, i), 2) +
                                        pow(*mat_elem(points, Y, i + 1) -*mat_elem(points, Y, i), 2) +
                                        pow(*mat_elem(points, Z, i + 1) -*mat_elem(points, Z, i), 2))/total_dist*time;
    }

    return out;
}


double path_smoothness(const matrix *pos, const matrix *vel){
    double out = 0.0;

    for (int i = 0; i < vel->mxn[1] - 1; i++)
        for (int j = 0; j < 3; j++)
            out = out + pow(*mat_elem(pos, j, i + 1) - *mat_elem(pos, j, i), 2) + pow(*mat_elem(vel, j, i + 1) - *mat_elem(vel, j, i), 2);

    return out;
}


void p_to_p(const matrix *pos_points, const matrix *vel_points, const matrix *acc_points, const double time, matrix *pos_func, matrix *vel_func, matrix *acc_func){
    double a0[3];
    double a1[3];
    double a2[3];
    double a3[3];
    double a4[3];
    double a5[3];

    matrix *system;

    for (int i = 0; i < 3; i++){
        a0[i] = *mat_elem(pos_points, i, 0);
        a1[i] = *mat_elem(vel_points, i, 0);
        a2[i] = *mat_elem(acc_points, i, 0)/2;

        double sys_elem[3*4] = {pow(time, 3), pow(time, 4), pow(time, 5), *mat_elem(pos_points, i, 1) - a2[i]*pow(time, 2) - a1[i]*time - a0[i],
                                pow(time, 2)*3, pow(time, 3)*4, pow(time, 4)*5, *mat_elem(vel_points, i, 1) - a2[i]*2*time - a1[i],
                                time*6, pow(time, 2)*12, pow(time, 3)*20, *mat_elem(acc_points, i, 1) - a2[i]*2};

        system = fill_matrix(3, 4, sys_elem);

        gauss_jordan_alg(system);

        for (int j = 0; j < 3; j++)
            if (fabs(*mat_elem(system, j, j) - 1.0) > IS_ZERO){
                printf("Error: <impossible eq system>\n");
                exit(EXIT_FAILURE);
            }

        a3[i] = *mat_elem(system, 0, 3);
        a4[i] = *mat_elem(system, 1, 3);
        a5[i] = *mat_elem(system, 2, 3);

        free_matrix(system);

        float pos_func_elem[6] = {a0[i], a1[i], a2[i], a3[i], a4[i], a5[i]};
        float vel_func_elem[6] = {a1[i], 2*a2[i], 3*a3[i], 4*a4[i], 5*a5[i], 0.0};
        float acc_func_elem[6] = {2*a2[i], 6*a3[i], 12*a4[i], 20*a5[i], 0.0, 0.0};

        for (int j = 0; j < 6; j++){
            *mat_elem(pos_func, i, j) = pos_func_elem[j];
            *mat_elem(vel_func, i, j) = vel_func_elem[j];
            *mat_elem(acc_func, i, j) = acc_func_elem[j];
        }
    }
}


void quantization(const matrix *function, const double time, const uint resolution, matrix *out){
double value = 0.0;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < (int)resolution; j++){
            for (int exp = 0; exp < 6; exp++)
                value = value + *mat_elem(function, i, exp)*pow(time*j/(resolution - 1), exp);
            *mat_elem(out, i, j) = value;
            value = 0.0;
        }
}


void path(const matrix *points, const double time, const double tune_precision, const uint resolution, matrix **pos, matrix **vel, matrix **acc){
    if (points->mxn[0] != 3){
        printf("Error <not 3 dimensions>\n");
        exit(EXIT_FAILURE);
    }

    if (points->mxn[1] < 2){
        printf("Error <less than 2 points>\n");
        exit(EXIT_FAILURE);
    }
    
    matrix *tim_div = time_division(points, time);

    matrix *tmp_pos_points = new_matrix(3, 2);
    matrix *tmp_vel_points = new_matrix(3, 2);
    matrix *tmp_acc_points = new_matrix(3, 2);

    matrix *tmp_pos = new_matrix(3, resolution);
    matrix *tmp_vel = new_matrix(3, resolution);
    matrix *tmp_acc = new_matrix(3, resolution);

    matrix *pos_function = new_matrix(3, 6);
    matrix *vel_function = new_matrix(3, 6);
    matrix *acc_function = new_matrix(3, 6);

    matrix *prev_dir = new_matrix(3, 1);
    matrix *curr_dir = new_matrix(3, 1);
    matrix *next_dir = new_matrix(3, 1);

    matrix *sum1 = NULL;
    matrix *sum2 = NULL;
    matrix *sum3 = NULL;

    double estim_vel = distance(points)/time;
    double estim_acc = 0.0;

    double tune_vel = 1.0;
    double prev_tune_vel = 0.0;
    double prev_prev_tune_vel = 0.0;
    double tune_acc = 1.0;
    double prev_tune_acc = 0.0;
    double prev_prev_tune_acc = 0.0;

    bool vel_last_operation = false;
    bool vel_is_better = false;
    bool acc_last_operation = false;
    bool acc_is_better = false;

    double vel_smoothness = 0.0;
    double vel_prev_smoothness = 0.0;
    double acc_smoothness = 0.0;
    double acc_prev_smoothness = 0.0;

    uint vel_cycles = 0;
    uint acc_cycles = 0;

    for (int i = 0; i < points->mxn[1] - 1; i++){
        update_directions(points, prev_dir, curr_dir, next_dir, i);

        estim_acc = estim_vel/(*mat_elem(tim_div, 0, i)/4);

        while (true){
            while (true){
                for (int j = 0; j < 3; j++){
                    *mat_elem(tmp_pos_points, j, 0) = *mat_elem(points, j, i);
                    *mat_elem(tmp_pos_points, j, 1) = *mat_elem(points, j, i + 1);

                    if (points->mxn[1] == 2){
                        *mat_elem(tmp_vel_points, j, 0) = 0.0;
                        *mat_elem(tmp_vel_points, j, 1) = 0.0;
                        
                        *mat_elem(tmp_acc_points, j, 0) = *mat_elem(curr_dir, j, 0)*estim_acc*tune_acc;
                        *mat_elem(tmp_acc_points, j, 1) = -*mat_elem(curr_dir, j, 0)*estim_acc*tune_acc;
                    }

                    else{
                        if (i == 0){
                            sum1 = matrix_sum(next_dir, curr_dir, '+');
                            sum2 = matrix_sum(next_dir, curr_dir, '-');
                            *mat_elem(tmp_vel_points, j, 0) = 0.0;
                            *mat_elem(tmp_vel_points, j, 1) = *mat_elem(sum1, j, 0)/vector_module(sum1)*estim_vel*tune_vel;

                            *mat_elem(tmp_acc_points, j, 0) = *mat_elem(curr_dir, j, 0)*estim_acc*tune_acc;
                            *mat_elem(tmp_acc_points, j, 1) = *mat_elem(sum2, j, 0)/vector_module(sum2)*estim_acc*tune_acc;
                            free_matrix(sum1);
                            free_matrix(sum2);
                        }

                        else if (i == (points->mxn[1] - 1) - 1){
                            *mat_elem(tmp_vel_points, j, 0) = *mat_elem(*vel, j, (*vel)->mxn[1] - 1);
                            *mat_elem(tmp_vel_points, j, 1) = 0.0;

                            sum1 = matrix_sum(curr_dir, prev_dir, '-');
                            *mat_elem(tmp_acc_points, j, 0) = *mat_elem(sum1, j, 0)/vector_module(sum1)*estim_acc*tune_acc;
                            *mat_elem(tmp_acc_points, j, 1) = -*mat_elem(curr_dir, j, 0)*estim_acc*tune_acc;
                            free_matrix(sum1);
                        }

                        else{
                            sum1 = matrix_sum(next_dir, curr_dir, '+');
                            sum2 = matrix_sum(curr_dir, prev_dir, '-');
                            sum3 = matrix_sum(next_dir, curr_dir, '-');
                            *mat_elem(tmp_vel_points, j, 0) = *mat_elem(*vel, j, (*vel)->mxn[1] - 1);
                            *mat_elem(tmp_vel_points, j, 1) = *mat_elem(sum1, j, 0)/vector_module(sum1)*estim_vel*tune_vel;

                            *mat_elem(tmp_acc_points, j, 0) = *mat_elem(sum2, j, 0)/vector_module(sum2)*estim_acc*tune_acc;
                            *mat_elem(tmp_acc_points, j, 1) = *mat_elem(sum3, j, 0)/vector_module(sum3)*estim_acc*tune_acc;
                            free_matrix(sum1);
                            free_matrix(sum2);
                            free_matrix(sum3);
                        }
                    }
                }

                p_to_p(tmp_pos_points, tmp_vel_points, tmp_acc_points, *mat_elem(tim_div, 0, i), pos_function, vel_function, acc_function);

                quantization(vel_function, *mat_elem(tim_div, 0, i), resolution, tmp_vel);
                quantization(acc_function, *mat_elem(tim_div, 0, i), resolution, tmp_acc);
                quantization(pos_function, *mat_elem(tim_div, 0, i), resolution, tmp_pos);


                acc_prev_smoothness = acc_smoothness;
                acc_smoothness = path_smoothness(tmp_pos, tmp_vel);

                prev_prev_tune_acc = prev_tune_acc;
                prev_tune_acc = tune_acc;

                if (acc_smoothness < acc_prev_smoothness)
                    acc_is_better = true;
                else
                    acc_is_better = false;

                if (acc_is_better){
                    if (acc_last_operation)
                        tune_acc = tune_acc + tune_precision;
                    else
                        tune_acc = tune_acc - tune_precision;
                }
                else{
                    if (acc_last_operation){
                        tune_acc = tune_acc - tune_precision;
                        acc_last_operation = false;
                    }
                    else{
                        tune_acc = tune_acc + tune_precision;
                        acc_last_operation = true;
                    }
                }

                if (prev_prev_tune_acc == tune_acc && acc_cycles > 3){
                    acc_is_better = false;
                    acc_last_operation = false;
                    acc_cycles = 0;
                    break;
                }

                acc_cycles++;
            }

            vel_prev_smoothness = vel_smoothness;
            vel_smoothness = path_smoothness(tmp_pos, tmp_vel);

            prev_prev_tune_vel = prev_tune_vel;
            prev_tune_vel = tune_vel;

            if (vel_smoothness < vel_prev_smoothness)
                vel_is_better = true;
            else
                vel_is_better = false;

            if (vel_is_better){
                if (vel_last_operation)
                    tune_vel = tune_vel + tune_precision;
                else
                    tune_vel = tune_vel - tune_precision;
            }
            else{
                if (vel_last_operation){
                    tune_vel = tune_vel - tune_precision;
                    vel_last_operation = false;
                }
                else{
                    tune_vel = tune_vel + tune_precision;
                    vel_last_operation = true;
                }
            }

            if (prev_prev_tune_vel == tune_vel && vel_cycles > 3){
                vel_is_better = false;
                vel_last_operation = false;
                tune_vel = 1.0;
                tune_acc = 1.0;
                prev_tune_vel = 0.0;
                prev_tune_acc = 0.0;
                prev_prev_tune_vel = 0.0;
                prev_prev_tune_acc = 0.0;
                vel_smoothness = 0.0;
                acc_smoothness = 0.0;
                vel_prev_smoothness = 0.0;
                acc_prev_smoothness = 0.0;
                vel_cycles = 0;
                break;
            }

            vel_cycles++;
        }

        if (i == 0){
            *pos = copy_matrix(tmp_pos);
            *vel = copy_matrix(tmp_vel);
            *acc = copy_matrix(tmp_acc);
        }
        else{
            *pos = glue_matrix(*pos, tmp_pos);
            *vel = glue_matrix(*vel, tmp_vel);
            *acc = glue_matrix(*acc, tmp_acc);
        }
    }

    free_matrix(tim_div);

    free_matrix(tmp_pos_points);
    free_matrix(tmp_vel_points);
    free_matrix(tmp_acc_points);

    free_matrix(tmp_pos);
    free_matrix(tmp_vel);
    free_matrix(tmp_acc);

    free_matrix(prev_dir);
    free_matrix(curr_dir);
    free_matrix(next_dir);

    free_matrix(pos_function);
    free_matrix(vel_function);
    free_matrix(acc_function);
}


void write_data(const matrix *pos, const matrix *vel, const matrix *acc, const double m){

    FILE *fp_x = fopen("../data/data_x.dat", "w");
    FILE *fp_y = fopen("../data/data_y.dat", "w");
    FILE *fp_z = fopen("../data/data_z.dat", "w");

    if (fp_x == NULL || fp_y == NULL || fp_z == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_xI = fopen("../data/data_xI.dat", "w");
    FILE *fp_yI = fopen("../data/data_yI.dat", "w");
    FILE *fp_zI = fopen("../data/data_zI.dat", "w");

    if (fp_xI == NULL || fp_yI == NULL || fp_zI == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_xII = fopen("../data/data_xII.dat", "w");
    FILE *fp_yII = fopen("../data/data_yII.dat", "w");
    FILE *fp_zII = fopen("../data/data_zII.dat", "w");

    if (fp_xII == NULL || fp_yII == NULL || fp_zII == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_xyz = fopen("../data/data_xyz.dat", "w");

    if (fp_xyz == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_q1 = fopen("../data/data_q1.dat", "w");
    FILE *fp_q2 = fopen("../data/data_q2.dat", "w");
    FILE *fp_q3 = fopen("../data/data_q3.dat", "w");

    if (fp_q1 == NULL || fp_q2 == NULL || fp_q3 == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_q1I = fopen("../data/data_q1I.dat", "w");
    FILE *fp_q2I = fopen("../data/data_q2I.dat", "w");
    FILE *fp_q3I = fopen("../data/data_q3I.dat", "w");

    if (fp_q1I == NULL || fp_q2I == NULL || fp_q3I == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_q1II = fopen("../data/data_q1II.dat", "w");
    FILE *fp_q2II = fopen("../data/data_q2II.dat", "w");
    FILE *fp_q3II = fopen("../data/data_q3II.dat", "w");

    if (fp_q1II == NULL || fp_q2II == NULL || fp_q3II == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    FILE *fp_torque1 = fopen("../data/data_torque1.dat", "w");
    FILE *fp_torque2 = fopen("../data/data_torque2.dat", "w");
    FILE *fp_torque3 = fopen("../data/data_torque3.dat", "w");

    if (fp_torque1 == NULL || fp_torque2 == NULL || fp_torque3 == NULL){
        printf("Error: <Can't open file>\n");
        exit(EXIT_FAILURE);
    }

    matrix *pg = new_matrix(3, 1);
    matrix *q = new_matrix(3, 1);

    matrix *pg_I = new_matrix(3, 1);
    matrix *q_I = new_matrix(3, 1);

    matrix *pg_II = new_matrix(3, 1);
    matrix *q_II = new_matrix(3, 1);

    matrix *torque = new_matrix(3, 1);

    double time = 0.0;

    for (int i = 0; i < pos->mxn[1]; i++){
        *mat_elem(pg, X, 0) = *mat_elem(pos, X, i);
        *mat_elem(pg, Y, 0) = *mat_elem(pos, Y, i);
        *mat_elem(pg, Z, 0) = *mat_elem(pos, Z, i);

        *mat_elem(pg_I, X, 0) = *mat_elem(vel, X, i);
        *mat_elem(pg_I, Y, 0) = *mat_elem(vel, Y, i);
        *mat_elem(pg_I, Z, 0) = *mat_elem(vel, Z, i);

        *mat_elem(pg_II, X, 0) = *mat_elem(acc, X, i);
        *mat_elem(pg_II, Y, 0) = *mat_elem(acc, Y, i);
        *mat_elem(pg_II, Z, 0) = *mat_elem(acc, Z, i);

        time = (double)i/(pos->mxn[1] - 1);

        fprintf(fp_x, "%lf\t%lf\n", time, *mat_elem(pg, X, 0));
        fprintf(fp_y, "%lf\t%lf\n", time, *mat_elem(pg, Y, 0));
        fprintf(fp_z, "%lf\t%lf\n", time, *mat_elem(pg, Z, 0));

        fprintf(fp_xI, "%lf\t%lf\n", time, *mat_elem(pg_I, X, 0));
        fprintf(fp_yI, "%lf\t%lf\n", time, *mat_elem(pg_I, Y, 0));
        fprintf(fp_zI, "%lf\t%lf\n", time, *mat_elem(pg_I, Z, 0));

        fprintf(fp_xyz, "%lf\t%lf\t%lf\t%lf\n", *mat_elem(pg, X, 0), *mat_elem(pg, Y, 0), *mat_elem(pg, Z, 0), sqrt(pow(*mat_elem(pg_II, X, 0), 2) + pow(*mat_elem(pg_II, Y, 0), 2) + pow(*mat_elem(pg_II, Z, 0), 2)));

        fprintf(fp_xII, "%lf\t%lf\n", time, *mat_elem(pg_II, X, 0));
        fprintf(fp_yII, "%lf\t%lf\n", time, *mat_elem(pg_II, Y, 0));
        fprintf(fp_zII, "%lf\t%lf\n", time, *mat_elem(pg_II, Z, 0));

        update_motor_pos(pg, q);
        update_motor_vel(pg, q, pg_I, q_I);
        update_motor_acc(pg, q, pg_I, q_I, pg_II, q_II);
        update_motor_tor(pg, q, pg_II, q_II, m, torque);

        fprintf(fp_q1, "%lf\t%lf\n", time, *mat_elem(q, I1, 0));
        fprintf(fp_q2, "%lf\t%lf\n", time, *mat_elem(q, I2, 0));
        fprintf(fp_q3, "%lf\t%lf\n", time, *mat_elem(q, I3, 0));

        fprintf(fp_q1I, "%lf\t%lf\n", time, *mat_elem(q_I, I1, 0));
        fprintf(fp_q2I, "%lf\t%lf\n", time, *mat_elem(q_I, I2, 0));
        fprintf(fp_q3I, "%lf\t%lf\n", time, *mat_elem(q_I, I3, 0));

        fprintf(fp_q1II, "%lf\t%lf\n", time, *mat_elem(q_II, I1, 0));
        fprintf(fp_q2II, "%lf\t%lf\n", time, *mat_elem(q_II, I2, 0));
        fprintf(fp_q3II, "%lf\t%lf\n", time, *mat_elem(q_II, I3, 0));

        fprintf(fp_torque1, "%lf\t%lf\n", time, *mat_elem(torque, I1, 0));
        fprintf(fp_torque2, "%lf\t%lf\n", time, *mat_elem(torque, I2, 0));
        fprintf(fp_torque3, "%lf\t%lf\n", time, *mat_elem(torque, I3, 0));
    }

    free_matrix(pg);
    free_matrix(q);

    free_matrix(pg_I);
    free_matrix(q_I);

    free_matrix(pg_II);
    free_matrix(q_II);

    free_matrix(torque);

    fclose(fp_x);
    fclose(fp_y);
    fclose(fp_z);

    fclose(fp_xI);
    fclose(fp_yI);
    fclose(fp_zI);

    fclose(fp_xII);
    fclose(fp_yII);
    fclose(fp_zII);

    fclose(fp_xyz);

    fclose(fp_q1);
    fclose(fp_q2);
    fclose(fp_q3);

    fclose(fp_q1I);
    fclose(fp_q2I);
    fclose(fp_q3I);

    fclose(fp_q1II);
    fclose(fp_q2II);
    fclose(fp_q3II);

    fclose(fp_torque1);
    fclose(fp_torque2);
    fclose(fp_torque3);
}


matrix *linear_path(const matrix *start, const matrix *finish, const uint resolution){
    matrix *out = new_matrix(3, resolution + 1);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j <= (int)resolution; j++)
            *mat_elem(out, i, j) = (*mat_elem(finish, i, 0) - *mat_elem(start, i, 0))*j/resolution + *mat_elem(start, i, 0);

    return out;
}


matrix *circle_path(const float radius, const float z, const uint resolution){
    matrix *out = new_matrix(3, resolution + 1);

    for (int j = 0; j <= (int)resolution; j++)
            *mat_elem(out, X, j) = radius*cos(2*M_PI*j/resolution);

    for (int j = 0; j <= (int)resolution; j++)
            *mat_elem(out, Y, j) = radius*sin(2*M_PI*j/resolution);

    for (int j = 0; j <= (int)resolution; j++)
            *mat_elem(out, Z, j) = z;

    return out;   
    }