#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define IS_ZERO 1.0e-10

typedef struct matrix{
    int mxn[2];
    double *mat;
} matrix;

matrix *new_matrix(const int m, const int n);

matrix *fill_matrix(const int m, const int n, const double *elements);

void free_matrix(matrix *a);

void print_matrix(const matrix *a, const char *label);

double *mat_elem(const matrix *a, const int row, const int col);

double vector_module(const matrix *a);

void matrix_swap_row(matrix *a, const int row1, const int row2);

void matrix_row_by_n(matrix *a, const int row, const double n);

void matrix_row_sum(matrix *a, const int row1, const int row2);

matrix *glue_matrix(matrix *a, const matrix *b);

matrix *copy_matrix(const matrix *a);

void copy_matrix_in(const matrix *a, matrix *b);

matrix *matrix_molt(const matrix *a, const matrix *b);

matrix *matrix_sum(const matrix *a, const matrix *b, const char operation);

matrix *matrix_by_n(const matrix *a, const double n);

matrix *matrix_transpose(const matrix *a);

matrix *get_submatrix(const matrix *a, const int row_to_delete, const int col_to_delete);

double matrix_det(const matrix *a);

double get_cofactor(const matrix *a, int row, int col);

void gauss_alg(matrix *a);

void gauss_jordan_alg(matrix *a);

matrix *matrix_inv(const matrix *a);

#endif