#include "kinematics_torque.h"


matrix *get_motor_tor(const matrix *pg, const matrix *q, const matrix *pg_II, const matrix *q_II, const float m){
    double J_elements[3*3] = J;
    double RV_elements[3*3] = RV;
    double F_elements[3*1] = F;
    double cos_q_elements[3*1] = {cos(Q_1), cos(Q_2), cos(Q_3)};

    matrix *j = fill_matrix(3, 3, J_elements);
    matrix *rv = fill_matrix(3, 3, RV_elements);
    matrix *f = fill_matrix(3, 1, F_elements);
    matrix *cos_q = fill_matrix(3, 1, cos_q_elements);

    matrix *inv_rv = matrix_inv(rv);
    matrix *neg_inv_rv = matrix_by_n(inv_rv, -1.0);
    matrix *k = matrix_molt(neg_inv_rv, j);
    matrix *iner1 = matrix_by_n(q_II, IQ);
    matrix *iner2 = matrix_molt(k, f);
    matrix *arm_w = matrix_by_n(cos_q, (M_UP_ARM/2 + M1)*A);
    matrix *sum1 = matrix_sum(iner1, iner2, '+');
    matrix *out = matrix_sum(sum1, arm_w, '-');

    free_matrix(j);
    free_matrix(rv);
    free_matrix(f);
    free_matrix(cos_q);
    free_matrix(inv_rv);
    free_matrix(neg_inv_rv);
    free_matrix(k);
    free_matrix(iner1);
    free_matrix(iner2);
    free_matrix(arm_w);
    free_matrix(sum1);

    return out;
}


void update_motor_tor(const matrix *pg, const matrix *q, const matrix *pg_II, const matrix *q_II, const float m, matrix *out){
    double J_elements[3*3] = J;
    double RV_elements[3*3] = RV;
    double F_elements[3*1] = F;
    double cos_q_elements[3*1] = {cos(Q_1), cos(Q_2), cos(Q_3)};

    matrix *j = fill_matrix(3, 3, J_elements);
    matrix *rv = fill_matrix(3, 3, RV_elements);
    matrix *f = fill_matrix(3, 1, F_elements);
    matrix *cos_q = fill_matrix(3, 1, cos_q_elements);

    matrix *inv_rv = matrix_inv(rv);
    matrix *neg_inv_rv = matrix_by_n(inv_rv, -1.0);
    matrix *k = matrix_molt(neg_inv_rv, j);
    matrix *iner1 = matrix_by_n(q_II, IQ);
    matrix *iner2 = matrix_molt(k, f);
    matrix *arm_w = matrix_by_n(cos_q, (M_UP_ARM/2 + M1)*A);
    matrix *sum1 = matrix_sum(iner1, iner2, '+');
    matrix *tmp_out = matrix_sum(sum1, arm_w, '-');

    copy_matrix_in(tmp_out, out);

    free_matrix(j);
    free_matrix(rv);
    free_matrix(f);
    free_matrix(cos_q);
    free_matrix(inv_rv);
    free_matrix(neg_inv_rv);
    free_matrix(k);
    free_matrix(iner1);
    free_matrix(iner2);
    free_matrix(arm_w);
    free_matrix(sum1);
    free_matrix(tmp_out);
}