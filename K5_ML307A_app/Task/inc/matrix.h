/*
 * matrix.h
 *
 *	Created on: Jun 21, 2024
 *		Author: lvvv
 */


#ifndef MATRIX_H_
#define MATRIX_H_

#include <config.h>
#include "app_sys.h"

typedef struct 
{
  int rows;
  int cols;

  double** data;
} matrix_s;


matrix_s* matrix_create(int rows, int cols);
void matrix_destroy(matrix_s* p_m);
void matrix_set(matrix_s* p_m, ...);
void matrix_set_identity(matrix_s* p_m);
void matrix_copy(matrix_s source, matrix_s* p_destination);
void matrix_print(matrix_s m);
void matrix_add(matrix_s a, matrix_s b, matrix_s* p_c);
void matrix_subtraction(matrix_s a, matrix_s b, matrix_s* p_c);
void matrix_subtract_from_identity(matrix_s* p_a);
void matrix_multiply(matrix_s a, matrix_s b, matrix_s* p_c);
void matrix_multiply_by_transpose(matrix_s a, matrix_s b, matrix_s* p_c);
void matrix_transpose(matrix_s input, matrix_s* p_output);
BOOL matrix_equal(matrix_s a, matrix_s b, double tolerance);
void matrix_scale(matrix_s* p_m, double scalar);
void matrix_swap_rows(matrix_s* p_m, int r1, int r2);
void matrix_scale_row(matrix_s* p_m, int r, double scalar);
void matrix_shear_row(matrix_s* p_m, int r1, int r2, double scalar);
BOOL matrix_destructive_invert(matrix_s input, matrix_s* p_output);

#endif /* MATRIX_H_ */

