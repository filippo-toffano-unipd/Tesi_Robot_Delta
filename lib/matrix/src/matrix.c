#include "matrix.h"

matrix *new_matrix(const int m, const int n){
    matrix *out = (matrix *)malloc(sizeof(matrix));
    out->mat = (double *)calloc(m * n, sizeof(double));

    if (out == NULL || out->mat == NULL){
        printf("Error! <heap_failure>\n");
        exit(EXIT_FAILURE);
    }

    out->mxn[0] = m;
    out->mxn[1] = n;

    return out;
}


matrix *fill_matrix(const int m, const int n, const double *elements){
    matrix *out = new_matrix(m, n);

    for (int i = 0; i < m*n; i++)
        out->mat[i] = elements[i];

    return out;
}


void free_matrix(matrix *a){
    free(a->mat);
    free(a);
}


void print_matrix(const matrix *a, const char *label){
    printf("%s\n", label);

    for (int row = 0; row < a->mxn[0]; row++){
        for (int col = 0; col < a->mxn[1]; col++){
            printf("%15lf  ", *mat_elem(a, row, col));
        }
        printf("\n");
    }
    
    printf("\n");
}


double *mat_elem(const matrix * a, const int row, const int col){
    if (row >= a->mxn[0] || col >= a->mxn[1]){
        printf("Error! <impossible_matrix_access>\n");
        exit(EXIT_FAILURE);
    }

    double *out = a->mat + row*a->mxn[1] + col;

    if (fabs(*out) < IS_ZERO)
        *out = 0.0;
    
    return out;
}


double vector_module(const matrix *a){
    if (a->mxn[1] != 1){
        printf("Error <not a vector>\n");
        exit(EXIT_FAILURE);
    }

    double out = 0.0;

    for (int i = 0; i < a->mxn[0]; i++){
        out = out + pow(*mat_elem(a, i, 0), 2);
    }

    return sqrt(out);
}


void matrix_swap_row(matrix *a, const int row1,const int row2){
    if (row1 > a->mxn[0] || row2 > a->mxn[0]){
        printf("Error! <row out of matrix>\n");
        exit(EXIT_FAILURE);
    }

    double tmp = 0.0;

    for (int col = 0; col < a->mxn[1]; col++){
        tmp = *mat_elem(a, row1, col);
        *mat_elem(a, row1, col) = *mat_elem(a, row2, col);
        *mat_elem(a, row2, col) = tmp;
    }
}


void matrix_row_by_n(matrix *a, const int row, const double n){
    if (row > a->mxn[0]){
        printf("Error! <row out of matrix>\n");
        exit(EXIT_FAILURE);
    }
    for (int col = 0; col < a->mxn[1]; col++)
        *mat_elem(a, row, col) = *mat_elem(a, row, col) * n;
}


void matrix_row_sum(matrix *a, const int row1, const int row2){
    if (row1 > a->mxn[0] || row2 > a->mxn[0]){
        printf("Error! <row out of matrix>\n");
        exit(EXIT_FAILURE);
    }

    for (int col = 0; col < a->mxn[1]; col++)
        *mat_elem(a, row1, col) = *mat_elem(a, row1, col) + *mat_elem(a, row2, col);
}


matrix *glue_matrix(matrix *a, const matrix *b){
    if (a->mxn[0] != b->mxn[0]){
        printf("Error! <can't glue matrix>\n");
        exit(EXIT_FAILURE);
    }

    matrix *out = new_matrix(a->mxn[0], a->mxn[1] + b->mxn[1]);

    for (int row = 0; row < a->mxn[0]; row++)
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(out, row, col) = *mat_elem(a, row, col);

    for (int row = 0; row < b->mxn[0]; row++)
        for (int col = 0; col < b->mxn[1]; col++)
            *mat_elem(out, row, col + a->mxn[1]) = *mat_elem(b, row, col);

    free_matrix(a);

    return out;
}


matrix *copy_matrix(const matrix *a){
    matrix *out = new_matrix(a->mxn[0], a->mxn[1]);

    for (int row = 0; row < a->mxn[0]; row++)
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(out, row, col) = *mat_elem(a, row, col);
    
    return out;
}


void copy_matrix_in(const matrix *a, matrix *b){
    if (a->mxn[0] != b->mxn[0] || a->mxn[1] != b->mxn[1]){
        printf("Error <invalid matrix input for copy>\n");
        exit(EXIT_FAILURE);
    }

    for (int row = 0; row < a->mxn[0]; row++)
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(b, row, col) = *mat_elem(a, row, col);
}


matrix *matrix_molt(const matrix *a, const matrix *b){
    if (a->mxn[1] != b->mxn[0]){
        printf("Error! <invalid_matrix_input_for_multiplication>\n");
        exit(EXIT_FAILURE);
    }

    matrix *out = new_matrix(a->mxn[0], b->mxn[1]);

    for (int row = 0; row < a->mxn[0]; row++)
        for (int col = 0; col < b->mxn[1]; col++)
            for (int i = 0; i < a->mxn[1]; i++)
                *mat_elem(out, row, col) = *mat_elem(out, row, col) + *mat_elem(a, row, i) * *mat_elem(b, i, col);

    return out;
}


matrix *matrix_sum(const matrix *a, const matrix *b, const char operation){
    if (a->mxn[0] != b->mxn[0] || a->mxn[1] != b->mxn[1]){
        printf("Error! <invalid_matrix_input_for_sum>\n");
        exit(EXIT_FAILURE);
    }

    matrix *out = new_matrix(a->mxn[0], a->mxn[1]);

    if (operation == '+')
        for (int row = 0; row < a->mxn[0]; row++)
            for (int col = 0; col < a->mxn[1]; col++)
                *mat_elem(out, row, col) = *mat_elem(a, row, col) + *mat_elem(b, row, col);
    else if (operation == '-')
        for (int row = 0; row < a->mxn[0]; row++)
            for (int col = 0; col < a->mxn[1]; col ++)
                *mat_elem(out, row, col) = *mat_elem(a, row, col) - *mat_elem(b, row, col);
    else {
        printf("Error! <invalid_operation>\n");
        exit(EXIT_FAILURE);
    }

    return out;
}


matrix *matrix_by_n(const matrix *a, const double n){
    matrix *out = new_matrix(a->mxn[0], a->mxn[1]);

    for (int row = 0; row < a->mxn[0]; row++)
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(out, row, col) = n * *mat_elem(a, row, col);

    return out;
}


matrix *matrix_transpose(const matrix *a){
    matrix *out = new_matrix(a->mxn[1], a->mxn[0]);

    for (int row = 0; row < a->mxn[0]; row++)
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(out, col, row) = *mat_elem(a, row, col);

    return out;
}


matrix *get_submatrix(const matrix *a, int row_to_delete, int col_to_delete){
    if (row_to_delete > a->mxn[0] || col_to_delete > a->mxn[1]){
        printf("Error! <invalid_row_input_for_submatrix>\n");
        exit(EXIT_FAILURE);
    }
    
    matrix *out;
    
    if (row_to_delete < 0 && col_to_delete < 0)
        out = new_matrix(a->mxn[0], a->mxn[1]);
    else if (row_to_delete < 0 && col_to_delete >= 0)
        out = new_matrix(a->mxn[0], a->mxn[1] - 1);
    else if (row_to_delete >= 0 && col_to_delete < 0)
        out = new_matrix(a->mxn[0] - 1, a->mxn[1]);
    else
        out = new_matrix(a->mxn[0] - 1, a->mxn[1] - 1);
    
    int new_row = 0;
    int new_col = 0;

    for (int row = 0; row < a->mxn[0]; row++){
        if (row != row_to_delete){
            for (int col = 0; col < a->mxn[1]; col++){
                if (col != col_to_delete){
                    *mat_elem(out, new_row, new_col) = *mat_elem(a, row, col);

                    new_col++;
                }
            }

            new_col = 0;
            new_row++;
        }
    }

    return out;
}


double matrix_det(const matrix *a){
    if (a->mxn[0] != a->mxn[1]){
        printf("Error! <invalid_matrix_not_square>\n");
        exit(EXIT_FAILURE);
    }

    if (a->mxn[0] == 1){
        return *mat_elem(a, 0, 0);
    }
    else{
        double tmp_det = 0.0;

        for (int row = 0; row < a->mxn[0]; row++){
            tmp_det = tmp_det + *mat_elem(a, row, 0) * get_cofactor(a, row, 0);
        }

        return tmp_det;
    }
}


double get_cofactor(const matrix *a, int row, int col){
    int sign = 0;
    if ((row + col) % 2 == 0)
        sign = 1;
    else
        sign = -1;

    double out = 0.0;

    matrix *sub_a = get_submatrix(a, row, col);
    out = (double)sign * matrix_det(sub_a);

    free_matrix(sub_a);

    return out;
}


void gauss_alg(matrix *a){
    int on_row = 0;
    double coeff = 0.0;
    matrix *sub_a;

    while (*mat_elem(a, on_row, 0) == 0.0){
        on_row++;
        if (on_row == a->mxn[0]){
            sub_a = get_submatrix(a, -1, 0);
            gauss_alg(sub_a);

            for (int row = 0; row < sub_a->mxn[0]; row++)
                for (int col = 0; col < sub_a->mxn[1]; col++)
                    *mat_elem(a, row, col + 1) = *mat_elem(sub_a, row, col);

            free_matrix(sub_a);
            return;
        }
    }

    if (on_row != 0)
        matrix_swap_row(a, 0, on_row);

    for (int row = 1; row < a->mxn[0]; row++){
        if (*mat_elem(a, row, 0) != 0.0){
            coeff = -*mat_elem(a, row, 0) / *mat_elem(a, 0, 0);
            matrix_row_by_n(a, 0, coeff);
            matrix_row_sum(a, row, 0);
        }
    }

    if (a->mxn[0] != 1 && a->mxn[1] != 1){
        sub_a = get_submatrix(a, 0, 0);
        gauss_alg(sub_a);

        for (int row = 0; row < sub_a->mxn[0]; row++)
            for (int col = 0; col < sub_a->mxn[1]; col++)
                *mat_elem(a, row + 1, col + 1) = *mat_elem(sub_a, row, col);

        free_matrix(sub_a);
    }
}


void gauss_jordan_alg(matrix *a){
    matrix *sub_a = copy_matrix(a);
    matrix *tmp;

    double coeff = 0.0;

    int deleted_cols = 0;

    gauss_alg(sub_a);

    while (*mat_elem(sub_a, 0, 0) == 0.0){
        deleted_cols++;
        tmp = get_submatrix(sub_a, -1, 0);
        free_matrix(sub_a);
        sub_a = tmp;
    }

    while (*mat_elem(sub_a, sub_a->mxn[0] - 1, sub_a->mxn[0] - 1) == 0.0){
        tmp = get_submatrix(sub_a, sub_a->mxn[0] - 1, -1);
        free_matrix(sub_a);
        sub_a = tmp;
    }

    for (int row = sub_a->mxn[0] - 1; row > 0; row--){
        if (*mat_elem(sub_a, row - 1, sub_a->mxn[0] - 1) != 0.0){
            coeff = -*mat_elem(sub_a, row - 1, sub_a->mxn[0] - 1) / * mat_elem(sub_a, sub_a->mxn[0] - 1, sub_a->mxn[0] - 1);
            matrix_row_by_n(sub_a, sub_a->mxn[0] - 1, coeff);
            matrix_row_sum(sub_a, row - 1, sub_a->mxn[0] - 1);
        }
    }

    if (sub_a->mxn[0] != 1){
        tmp = get_submatrix(sub_a, sub_a->mxn[0] - 1, sub_a->mxn[0] - 1);
        gauss_jordan_alg(tmp);

        for (int row = 0; row < tmp->mxn[0]; row++)
            for (int col = 0; col < tmp->mxn[1]; col++)
                if (col < tmp->mxn[0])
                    *mat_elem(sub_a, row, col) = *mat_elem(tmp, row, col);
                else
                    *mat_elem(sub_a, row, col + 1) = *mat_elem(tmp, row, col);

        free_matrix(tmp);
    }

    for (int row = 0; row < sub_a->mxn[0]; row++)
        matrix_row_by_n(sub_a, row, 1 / *mat_elem(sub_a, row, row));

    for (int row = 0; row < sub_a->mxn[0]; row++)
        for (int col = 0; col < sub_a->mxn[1]; col++)
            *mat_elem(a, row, col + deleted_cols) = *mat_elem(sub_a, row, col);

    free_matrix(sub_a);
}


matrix *matrix_inv(const matrix *a){
    if (a->mxn[0] != a->mxn[1]){
        printf("Error! <invalid_matrix_not_square>\n");
        exit(EXIT_FAILURE);
    }

    matrix *big_a = new_matrix(a->mxn[0], a->mxn[1] * 2);
    matrix *out = new_matrix(a->mxn[0], a->mxn[1]);

    for (int row = 0; row < a->mxn[0]; row++){
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(big_a, row, col) = *mat_elem(a, row, col);

        *mat_elem(big_a, row, row + a->mxn[1]) = 1.0;
    }

    gauss_jordan_alg(big_a);


    for (int row = 0; row < a->mxn[0]; row++){
        if (fabs(*mat_elem(big_a, row, row) - 1) > IS_ZERO){
            printf("Error! <matrix_not_invertible>\n");
            exit(EXIT_FAILURE);
        }
        for (int col = 0; col < a->mxn[1]; col++)
            *mat_elem(out, row, col) = *mat_elem(big_a, row, col + a->mxn[1]);
    }

    free_matrix(big_a);

    return out;
}