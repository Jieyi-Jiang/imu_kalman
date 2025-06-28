#include "mini_matrix.h"

void __mm_init(MiniMatrix *mm, uint8_t rows, uint8_t cols, float *data_ptr, uint8_t data_size)
{
    mm->data = NULL;
    mm->rows = 0;
    mm->cols = 0;

    if (data_size != rows * cols) {
        printf("Data size does not match matrix dimensions\n");
        return;
    }
    mm->rows = rows;
    mm->cols = cols;
    mm->data = data_ptr;
}

void mm_print(MiniMatrix *mm)
{
    for (uint8_t i = 0; i < mm->rows; i++) {
        for (uint8_t j = 0; j < mm->cols; j++) {
            printf("%f ", mm->data[i * mm->cols + j]);
        }
        printf("\n");
    }
    printf("\n");
}


void mm_add(MiniMatrix *a, MiniMatrix *b, MiniMatrix *result)
{
    if (a->rows != b->rows || a->cols != b->cols) {
        printf("Matrix dimensions do not match\n");
        return;
    }
    
    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < a->cols; j++) {
            result->data[i * result->cols + j] = a->data[i * a->cols + j] + b->data[i * b->cols + j];
        }
    }
}
void mm_sub(MiniMatrix *a, MiniMatrix *b, MiniMatrix *result)
{
    if (a->rows != b->rows || a->cols != b->cols) {
        printf("Matrix dimensions do not match\n");
        return;
    }

    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < a->cols; j++) {
            result->data[i * result->cols + j] = a->data[i * a->cols + j] - b->data[i * b->cols + j];
        }
    }
}
void mm_mul(MiniMatrix *a, MiniMatrix *b, MiniMatrix *result)
{
    if (a->cols != b->rows) {
        printf("Matrix dimensions do not match\n");
        return;
    }

    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < b->cols; j++) {
            result->data[i * result->cols + j] = 0;
            for (uint8_t k = 0; k < a->cols; k++) {
                result->data[i * result->cols + j] += a->data[i * a->cols + k] * b->data[k * b->cols + j];
            }
        }
    }
}

void mm_transpose(MiniMatrix *a, MiniMatrix *result)
{
    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < a->cols; j++) {
            result->data[j * result->cols + i] = a->data[i * a->cols + j];
        }
    }
}


void mm_scalar_mul(MiniMatrix *a, float scalar, MiniMatrix *result)
{
    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < a->cols; j++) {
            result->data[i * result->cols + j] = a->data[i * a->cols + j] * scalar;
        }
    }
}
