#include "filter_utils.h"

float SlidingAverageFilter(float *buffer, int size, float new_val, uint8_t *index)
{
    buffer[*index] = new_val;
    *index         = (*index + 1) % size;

    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }

    return sum / size;
}

float LowPassFilter(float last_val, float new_val, float alpha)
{
    return alpha * new_val + (1.0f - alpha) * last_val;
}