#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


typedef struct {
    float *data;
    uint8_t rows;
    uint8_t cols;
} MiniMatrix;

void __mm_init(MiniMatrix *mm, uint8_t rows, uint8_t cols, float *data_ptr, uint8_t data_size);
#define mm_init(mm, rows, cols, data) (__mm_init((mm), (rows), (cols), (data), sizeof(data)/sizeof(float)))

inline void mm_set_elem(MiniMatrix *mm, uint8_t row, uint8_t col, float value)
{
    mm->data[row * mm->cols + col - 1] = value;
}

inline float mm_get_elem(MiniMatrix *mm, uint8_t row, uint8_t col)
{
    return mm->data[row * mm->cols + col - 1];
}

void mm_print(MiniMatrix *mm);

void mm_add(MiniMatrix *a, MiniMatrix *b, MiniMatrix *result);
void mm_sub(MiniMatrix *a, MiniMatrix *b, MiniMatrix *result);
void mm_mul(MiniMatrix *a, MiniMatrix *b, MiniMatrix *result);
void mm_transpose(MiniMatrix *a, MiniMatrix *result);
void mm_scalar_mul(MiniMatrix *a, float scalar, MiniMatrix *result);
