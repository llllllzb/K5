/*
 * matrix.c
 *
 *	Created on: Jun 21, 2024
 *		Author: lvvv 
 *
 * �����㷨
 * ��Ҫ���ڿ������˲��㷨��
 */


#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "matrix.h"


#define ASSERT_RETURN(c) \
if(!c)\
{\
	return;\
}\

static double doubleabs(double data) 
{
	if (data > 0) 
	{
		return data;
	}
	else
	{
		return -1.0 * data;
	}
}

/**************************************************
@bref		��������
@param
	rows ����
	cols ����
@return
	p_m����ָ��
@note
**************************************************/
static int32_t max_cnt = 0;
matrix_s* matrix_create(int rows, int cols) 
{
	matrix_s* p_m = (matrix_s*)malloc(sizeof(matrix_s));
	int i = 0;
	p_m->rows = rows;
	p_m->cols = cols;
	p_m->data = (double**) malloc(sizeof(double*) * p_m->rows);
	for (i = 0; i < p_m->rows; ++i) 
	{	
		int j = 0;
    	p_m->data[i] = (double*) malloc(sizeof(double) * p_m->cols);
    	for (j = 0; j < p_m->cols; ++j) 
		{
      		p_m->data[i][j] = 0.0;
    	}
  	}
  	//LogPrintf(DEBUG_ALL, "matrix_destroy max_cnt:%d", (max_cnt += (sizeof(matrix_s) + sizeof(double*) * p_m->rows + sizeof(double) * p_m->cols * p_m->rows)));
  	return p_m;
}

/**************************************************
@bref		ɾ������
@param
	p_m ����
@return
@note
**************************************************/

void matrix_destroy(matrix_s* p_m) 
{
	int i = 0;
	ASSERT_RETURN(p_m->data);
	for (i = 0; i < p_m->rows; ++i) 
	{
    	free(p_m->data[i]);
  	}
  	free(p_m->data);
	free(p_m);
	//LogPrintf(DEBUG_ALL, "matrix_destroy:%d", (max_cnt -= (sizeof(*p_m->data[i]) * p_m->rows + sizeof(*p_m->data) + sizeof(*p_m))));
}

/**************************************************
@bref		���þ���ֵ
@param
@return
@note
**************************************************/

void matrix_set(matrix_s* p_m, ...) 
{
	int i = 0;
	va_list ap;
	va_start(ap, p_m);
  
	for (i = 0; i < p_m->rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < p_m->cols; ++j) 
		{
     		p_m->data[i][j] = va_arg(ap, double);
    	}
	}
	va_end(ap);
}

/**************************************************
@bref		���õ�λ����
@param
@return
@note
**************************************************/
void matrix_set_identity(matrix_s* p_m) 
{
	int i = 0;
	ASSERT_RETURN (p_m->rows == p_m->cols);
	
	for (i = 0; i < p_m->rows; ++i) 
	{
		int j = 0;
	    for (j = 0; j < p_m->cols; ++j) 
		{
	      	if (i == j) 
			{
				p_m->data[i][j] = 1.0;
	      	} 
			else 
			{
				p_m->data[i][j] = 0.0;
	      	}
	    }
  	}
}

/**************************************************
@bref		������source���Ƶ�destination
@param
	src ԭ����
	p_destination ��Ŀ�ĵ�
@return
@note
**************************************************/
void matrix_copy(matrix_s src, matrix_s* p_destination)
{
	int i = 0;
	ASSERT_RETURN (src.rows == p_destination->rows);

	ASSERT_RETURN (src.cols ==p_destination->cols);
	
  	for (i = 0; i < src.rows; ++i) 
	{
		int j = 0;
    	for (j = 0; j < src.cols; ++j)
		{
      		p_destination->data[i][j] = src.data[i][j];
    	}
  	}
}

/**************************************************
@bref		��ӡ��������
@param
@return
@note
**************************************************/
void matrix_print(matrix_s m) 
{
	int i = 0;
	char buf[1024] = { 0 };
  	for (i = 0; i < m.rows; ++i) 
	{
		int j = 0;
    	for (j = 0; j < m.cols; ++j) 
		{
      		if (j > 0) 
			{
				snprintf(buf + strlen(buf),sizeof(buf) - strlen(buf),"%s"," ");
      		}
			snprintf(buf + strlen(buf),sizeof(buf) - strlen(buf),"%6.2f", m.data[i][j]);
    	}
    	LogMessage(DEBUG_ALL, buf);
  	}
}

/**************************************************
@bref		����ӷ�
@param
	a �������
	b �������
	c �����
@return
@note
�Ѿ���a�;���b�ӵ�����c
**************************************************/

void matrix_add(matrix_s a, matrix_s b, matrix_s* p_c)
{
	int i = 0;

	ASSERT_RETURN(a.rows == b.rows);
	ASSERT_RETURN(a.rows == p_c->rows);
	ASSERT_RETURN(a.cols == b.cols);
	ASSERT_RETURN(a.cols == p_c->cols);
	for (i = 0; i < a.rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < a.cols; ++j) 
		{
	  		p_c->data[i][j] = a.data[i][j] + b.data[i][j];
		}
	}
}
/**
 * Function:   �������
 * Description:
 * Input:	   a������������b��������
 * Output:	   c�������
 * Return:	   ��
 * Others:	   ��
 */

/**************************************************
@bref		�������
@param
	a ������
	b ����
	c ��
@return
@note
**************************************************/
void matrix_subtraction(matrix_s a, matrix_s b, matrix_s* p_c) 
{
	int i = 0;
	ASSERT_RETURN(a.rows == b.rows);
	ASSERT_RETURN(a.rows == p_c->rows);
	ASSERT_RETURN(a.cols == b.cols);
	ASSERT_RETURN(a.cols == p_c->cols);
	for (i = 0; i < a.rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < a.cols; ++j) 
		{
	      	p_c->data[i][j] = a.data[i][j] - b.data[i][j];
	    }
	}
}

/**************************************************
@bref		��λ�����м�ȥ����a
@param
@return
@note
�ӵ�λ�����м�ȥa���õ��Ľ������a
**************************************************/
void matrix_subtract_from_identity(matrix_s* p_a) 
{
	int i = 0;

	ASSERT_RETURN(p_a->rows == p_a->cols);
	for (i = 0; i < p_a->rows; ++i)
	{
		int j = 0;
		for (j = 0; j < p_a->cols; ++j) 
		{
		  	if (i == j) 
		  	{
				p_a->data[i][j] = 1.0 - p_a->data[i][j];
		  	} 
			else 
			{
				p_a->data[i][j] = 0.0 - p_a->data[i][j];
		  	}
		}
	}
}

/**************************************************
@bref		����˷�
@param
	a ����
	b ����
	c �˷����
@return
@note
����˷�����:a����Ҫ��b���������,c����������a������,
c����������b������
**************************************************/

void matrix_multiply(matrix_s a, matrix_s b, matrix_s* p_c)
{
	int i = 0;
	ASSERT_RETURN(a.cols == b.rows);
	ASSERT_RETURN(a.rows == p_c->rows);
	ASSERT_RETURN(b.cols == p_c->cols);
	for (i = 0; i < p_c->rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < p_c->cols; ++j)
		{
			int k = 0;
			p_c->data[i][j] = 0.0;
			for (k = 0; k < a.cols; ++k) 
			{
				p_c->data[i][j] += a.data[i][k] * b.data[k][j];
			}
		}
	}
}

/**************************************************
@bref		a����b��ת�þ���
@param
@return
@note
a����Ҫ��b�����������,c����������a������,c����������b������
�������c��
**************************************************/
void matrix_multiply_by_transpose(matrix_s a, matrix_s b, matrix_s* p_c) 
{
	int i = 0;

	ASSERT_RETURN(a.cols == b.cols);
	ASSERT_RETURN(a.rows == p_c->rows);
	ASSERT_RETURN(b.rows == p_c->cols);
	for (i = 0; i < p_c->rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < p_c->cols; ++j) 
		{
			int k = 0;
			p_c->data[i][j] = 0.0;
			for (k = 0; k < a.cols; ++k) 
			{
				p_c->data[i][j] += a.data[i][k] * b.data[j][k];
			}
		}
	}
}

/**************************************************
@bref		����ת��
@param
	input �������
	output �������
@return
@note
**************************************************/
void matrix_transpose(matrix_s input, matrix_s* p_output) 
{
	int i = 0;

	ASSERT_RETURN(input.rows == p_output->cols);
	ASSERT_RETURN(input.cols == p_output->rows);
	for (i = 0; i < input.rows; ++i) 
	{
		int j = 0;
		for ( j = 0; j < input.cols; ++j) 
		{
			p_output->data[j][i] = input.data[i][j];
		}
	}
}

/**************************************************
@bref		�ж����������Ƿ�������
@param
	a ����
	b ����
	tolerance ��������
@return
	TRUE ������� FALSE �����
@note
**************************************************/
BOOL matrix_equal(matrix_s a, matrix_s b, double tolerance)
{
	int i = 0;
  
	if (a.rows != b.rows)
	{
		return FALSE;
	}
	if (a.cols != b.cols)
	{
		return FALSE;
	}
	
	for (i = 0; i < a.rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < a.cols; ++j) 
		{
			if (doubleabs(a.data[i][j] - b.data[i][j]) > tolerance) 
			{
				return FALSE;
			}
		}
	}
	return TRUE;
}

/**************************************************
@bref		�������һ������
@param
	m �������
	scalar ����
	m �������
@return
@note
**************************************************/
void matrix_scale(matrix_s* p_m, double scalar) 
{
	int i = 0;
	for (i = 0; i < p_m->rows; ++i) 
	{
		int j = 0;
		for (j = 0; j < p_m->cols; ++j) 
		{
		 	p_m->data[i][j] *= scalar;
		}
	}
}

/**************************************************
@bref		�����б任--��������m�ĵ�r1��r2��
@param
	m �������
	r1 �������к�
	r2 �������к�
	m �������
@return
@note
**************************************************/
void matrix_swap_rows(matrix_s* p_m, int r1, int r2) 
{
	double* tmp = p_m->data[r1];

	ASSERT_RETURN(r1 != r2);
	p_m->data[r1] = p_m->data[r2];
	p_m->data[r2] = tmp;
}

/**************************************************
@bref		�����б任--����ĵ�r�г���һ������
@param
	m �������
	r �к�
	scalar ����
	m �������
@return
@note
**************************************************/

void matrix_scale_row(matrix_s* p_m, int r, double scalar) 
{
	int i = 0;

	for (i = 0; i < p_m->cols; ++i) 
	{
		p_m->data[r][i] *= scalar;
	}
}

/**************************************************
@bref		�����б任--������ĵ�r2�г���һ�������ټӵ���r1��
@param
	m �������
	r �к�
	scalar ����
	m �������
@return
@note
**************************************************/
void matrix_shear_row(matrix_s* p_m, int r1, int r2, double scalar) 
{
	int i = 0;

	ASSERT_RETURN(r1 != r2);
	for (i = 0; i < p_m->cols; ++i)
	{
		p_m->data[r1][i] += scalar * p_m->data[r2][i];
	}
}

/**************************************************
@bref		����������
@param
	input �������
	p_output �������
@return
	TRUE:���� FALSE:������
@note
����ľ���һ������
**************************************************/
BOOL matrix_destructive_invert(matrix_s input, matrix_s* p_output)
{
	int i = 0;

	LogPrintf(DEBUG_ALL,"matrix_destructive_invert:input rows=%d,cols=%d", input.rows, input.cols);
	LogPrintf(DEBUG_ALL,"matrix_destructive_invert:output rows=%d,cols=%d", p_output->rows, p_output->cols);

	if(input.rows != input.cols)
	{
		return FALSE;
	}
	
	if(input.rows != p_output->rows)
	{
		return FALSE;
	}
	
	if(input.rows != p_output->cols)
	{
		return FALSE;
	}
	

	matrix_set_identity(p_output);

	for (i = 0; i < input.rows; ++i) 
	{
		double scalar = 0;
		int j = 0;
		
		if (input.data[i][i] == 0.0)
		{
			int r = 0;
			for (r = i + 1; r < input.rows; ++r)
			{
				if (input.data[r][i] != 0.0) 
				{
				    break;
				}
			}
			if (r == input.rows)
			{
				return FALSE;
			}
			matrix_swap_rows(&input, i, r);
			matrix_swap_rows(p_output, i, r);
		}

		scalar = 1.0 / input.data[i][i];
		matrix_scale_row(&input, i, scalar);
		matrix_scale_row(p_output, i, scalar);

		for (j = 0; j < input.rows; ++j) 
		{
			double shear_needed = -input.data[j][i];
			if (i == j) 
			{
				continue;
			}
			matrix_shear_row(&input, j, i, shear_needed);
			matrix_shear_row(p_output, j, i, shear_needed);
		}
	}
	return TRUE;
}

