#include "kinematics_velocity.h"

matrix *get_motor_vel(const matrix *pg, const matrix *q,  const matrix *pg_I){
    valid_vel(pg_I);

    double J_elements[3*3] = J;
    double RV_elements[3*3] = RV;

    matrix *j = fill_matrix(3, 3, J_elements);
    matrix *rv = fill_matrix(3, 3, RV_elements);
    
    matrix *inv_rv = matrix_inv(rv);
    matrix *neg_inv_rv = matrix_by_n(inv_rv, -1.0);
    matrix *conv_matrix = matrix_molt(neg_inv_rv, j);
    matrix *out = matrix_molt(conv_matrix, pg_I);

    free_matrix(j);
    free_matrix(rv);
    free_matrix(inv_rv);
    free_matrix(neg_inv_rv);
    free_matrix(conv_matrix);

    return out;
}


void update_motor_vel(const matrix *pg, const matrix *q,  const matrix *pg_I, matrix *out){
    valid_vel(pg_I);

    double J_elements[3*3] = J;
    double RV_elements[3*3] = RV;

    matrix *j = fill_matrix(3, 3, J_elements);
    matrix *rv = fill_matrix(3, 3, RV_elements);
    
    matrix *inv_rv = matrix_inv(rv);
    matrix *neg_inv_rv = matrix_by_n(inv_rv, -1.0);
    matrix *conv_matrix = matrix_molt(neg_inv_rv, j);
    matrix *tmp_out = matrix_molt(conv_matrix, pg_I);

    copy_matrix_in(tmp_out, out);

    free_matrix(j);
    free_matrix(rv);
    free_matrix(inv_rv);
    free_matrix(neg_inv_rv);
    free_matrix(conv_matrix);
    free_matrix(tmp_out);
}


void valid_vel(const matrix *vel){
    if (vel->mxn[0] != 3 || vel->mxn[1] != 1){
        printf("Error! <matrix_has_no_coordinates>\n");
        exit(EXIT_FAILURE);
    }

    /*
    if (((pow(*mat_elem(pos, X, 0), 2) + pow(*mat_elem(pos, Y, 0), 2)) > pow(MAX_RADIUS, 2)) || (*mat_elem(pos, Z, 0) > MAX_Z) || (*mat_elem(pos, Z, 0) < MIN_Z)){
        printf("Error! <invalid_coordinates>\n");
        exit(EXIT_FAILURE);
    }*/
}

