#pragma once

#include <stdint.h>

/**
 * @brief 滑动平均滤波器
 *
 * @param buffer   滑动窗口数组
 * @param size     滤波窗口大小（buffer 长度）
 * @param new_val  新的传感器原始值
 * @param index    当前写入下标指针
 * @return float   平均值结果
 */
float SlidingAverageFilter(float *buffer, int size, float new_val, uint8_t *index);

/**
 * @brief 一阶低通滤波器
 *
 * @param last_val 上次的滤波输出值
 * @param new_val  当前新的采样值
 * @param alpha    滤波系数，越大越跟随，越小越平滑（0~1）
 * @return float   新的滤波输出
 */
float LowPassFilter(float last_val, float new_val, float alpha);