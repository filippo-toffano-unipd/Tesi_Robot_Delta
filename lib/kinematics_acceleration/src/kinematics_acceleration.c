#include "kinematics_acceleration.h"


matrix *get_motor_acc(const matrix *pg, const matrix *q, const matrix *pg_I, const matrix *q_I, const matrix *pg_II){
    valid_acc(pg_II);

    double J_elements[3*3] = J;
    double RV_elements[3*3] = RV;
    double dJ_elements[3*3] = dJ;
    double dRV_elements[3*3] = dRV;

    matrix *j = fill_matrix(3, 3, J_elements);
    matrix *rv = fill_matrix(3, 3, RV_elements);
    matrix *dj = fill_matrix(3, 3, dJ_elements);
    matrix *drv = fill_matrix(3, 3, dRV_elements);

    matrix *inv_rv = matrix_inv(rv);
    matrix *neg_inv_rv = matrix_by_n(inv_rv, -1.0);
    matrix *j_pgII = matrix_molt(j, pg_II);
    matrix *dj_pgI = matrix_molt(dj, pg_I);
    matrix *drv_qI = matrix_molt(drv, q_I);
    matrix *sum1 = matrix_sum(j_pgII, dj_pgI, '+');
    matrix *sum2 = matrix_sum(sum1, drv_qI, '+');
    matrix *out = matrix_molt(neg_inv_rv, sum2);

    free_matrix(j);
    free_matrix(rv);
    free_matrix(dj);
    free_matrix(drv);
    free_matrix(inv_rv);
    free_matrix(neg_inv_rv);
    free_matrix(j_pgII);
    free_matrix(dj_pgI);
    free_matrix(drv_qI);
    free_matrix(sum1);
    free_matrix(sum2);

    return out;
}


void update_motor_acc(const matrix *pg, const matrix *q, const matrix *pg_I, const matrix *q_I, const matrix *pg_II, matrix *out){
    valid_acc(pg_II);

    double J_elements[3*3] = J;
    double RV_elements[3*3] = RV;
    double dJ_elements[3*3] = dJ;
    double dRV_elements[3*3] = dRV;

    matrix *j = fill_matrix(3, 3, J_elements);
    matrix *rv = fill_matrix(3, 3, RV_elements);
    matrix *dj = fill_matrix(3, 3, dJ_elements);
    matrix *drv = fill_matrix(3, 3, dRV_elements);

    matrix *inv_rv = matrix_inv(rv);
    matrix *neg_inv_rv = matrix_by_n(inv_rv, -1.0);
    matrix *j_pgII = matrix_molt(j, pg_II);
    matrix *dj_pgI = matrix_molt(dj, pg_I);
    matrix *drv_qI = matrix_molt(drv, q_I);
    matrix *sum1 = matrix_sum(j_pgII, dj_pgI, '+');
    matrix *sum2 = matrix_sum(sum1, drv_qI, '+');
    matrix *tmp_out = matrix_molt(neg_inv_rv, sum2);

    copy_matrix_in(tmp_out, out);

    free_matrix(j);
    free_matrix(rv);
    free_matrix(dj);
    free_matrix(drv);
    free_matrix(inv_rv);
    free_matrix(neg_inv_rv);
    free_matrix(j_pgII);
    free_matrix(dj_pgI);
    free_matrix(drv_qI);
    free_matrix(sum1);
    free_matrix(sum2);
    free_matrix(tmp_out);
}


void valid_acc(const matrix *acc){
    if (acc->mxn[0] != 3 || acc->mxn[1] != 1){
        printf("Error! <matrix_has_no_coordinates>\n");
        exit(EXIT_FAILURE);
    }

    /*
    if (((pow(*mat_elem(pos, X, 0), 2) + pow(*mat_elem(pos, Y, 0), 2)) > pow(MAX_RADIUS, 2)) || (*mat_elem(pos, Z, 0) > MAX_Z) || (*mat_elem(pos, Z, 0) < MIN_Z)){
        printf("Error! <invalid_coordinates>\n");
        exit(EXIT_FAILURE);
    }*/
}