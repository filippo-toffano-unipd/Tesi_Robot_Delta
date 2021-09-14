#include "kinematics_position.h"

matrix *get_motor_pos(const matrix *pg){
    valid_pos(pg);

    double tmp_1[3*1];
    double tmp_2[3*1];
    double q_elements[3*1];

    tmp_1[I1] = 2.0*atan((-B1 - sqrt(pow(B1, 2) - 4.0*A1*C1)) / (2*A1));
    tmp_1[I2] = 2.0*atan((-B2 - sqrt(pow(B2, 2) - 4.0*A2*C2)) / (2*A2));
    tmp_1[I3] = 2.0*atan((-B3 - sqrt(pow(B3, 2) - 4.0*A3*C3)) / (2*A3));

    tmp_2[I1] = 2.0*atan((-B1 + sqrt(pow(B1, 2) - 4.0*A1*C1)) / (2*A1));
    tmp_2[I2] = 2.0*atan((-B2 + sqrt(pow(B2, 2) - 4.0*A2*C2)) / (2*A2));
    tmp_2[I3] = 2.0*atan((-B3 + sqrt(pow(B3, 2) - 4.0*A3*C3)) / (2*A3));

    for (int i = 0; i < 3; i++)
        if(fabs(tmp_1[i]) < fabs(tmp_2[i]))
            q_elements[i] = tmp_1[i];
        else
            q_elements[i] = tmp_2[i];

    matrix *out = fill_matrix(3, 1, q_elements);

    return out;
}


void update_motor_pos(const matrix *pg, matrix *out){
    valid_pos(pg);

    if (out->mxn[0] != 3 || out->mxn[1] != 1){
        printf("Error! <matrix_has_no_coordinates>\n");
        exit(EXIT_FAILURE);
    }

    double tmp_1[3*1];
    double tmp_2[3*1];
    double q_elements[3*1];

    tmp_1[I1] = 2.0*atan((-B1 - sqrt(pow(B1, 2) - 4.0*A1*C1)) / (2*A1));
    tmp_1[I2] = 2.0*atan((-B2 - sqrt(pow(B2, 2) - 4.0*A2*C2)) / (2*A2));
    tmp_1[I3] = 2.0*atan((-B3 - sqrt(pow(B3, 2) - 4.0*A3*C3)) / (2*A3));

    tmp_2[I1] = 2.0*atan((-B1 + sqrt(pow(B1, 2) - 4.0*A1*C1)) / (2*A1));
    tmp_2[I2] = 2.0*atan((-B2 + sqrt(pow(B2, 2) - 4.0*A2*C2)) / (2*A2));
    tmp_2[I3] = 2.0*atan((-B3 + sqrt(pow(B3, 2) - 4.0*A3*C3)) / (2*A3));

    for (int i = 0; i < 3; i++)
        if(fabs(tmp_1[i]) < fabs(tmp_2[i]))
            q_elements[i] = tmp_1[i];
        else
            q_elements[i] = tmp_2[i];

    *mat_elem(out, I1, 0) = q_elements[I1];
    *mat_elem(out, I2, 0) = q_elements[I2];
    *mat_elem(out, I3, 0) = q_elements[I3];
}


void valid_pos(const matrix *pos){
    if (pos->mxn[0] != 3 || pos->mxn[1] != 1){
        printf("Error! <matrix_has_no_coordinates>\n");
        exit(EXIT_FAILURE);
    }

    if (((pow(*mat_elem(pos, X, 0), 2) + pow(*mat_elem(pos, Y, 0), 2)) > pow(MAX_RADIUS, 2)) || (*mat_elem(pos, Z, 0) > MAX_Z) || (*mat_elem(pos, Z, 0) < MIN_Z)){
        printf("Error! <invalid_coordinates>\n");
        exit(EXIT_FAILURE);
    }
}