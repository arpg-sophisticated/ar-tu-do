// compile with nvcc hello_world_gpu.cu -o hello_world_gpu
// execute using ./hello_world_gpu

#include <stdio.h>

__global__ void hello_cuda_multi(float f) {
    printf("Hello block %d thread %d, f=%f\n", blockIdx.x, threadIdx.x, f);
}

int main() {
    // 5 Blocks with 3 Thread
    // try and execute multiple times and have a look at the order
    hello_cuda_multi<<<5, 3>>>(1.2345f);
    cudaDeviceSynchronize();
    return 0;
}