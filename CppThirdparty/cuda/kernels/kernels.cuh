#ifndef TEST_CUH__
#define TEST_CUH__

#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>

void CppCUDAUsage();

// 在CPU上运行的函数
void CPUFunction();

// 在GPU上运行的函数
__global__ void GPUFunction();

#endif

