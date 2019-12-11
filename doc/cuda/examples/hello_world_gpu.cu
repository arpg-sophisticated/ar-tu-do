// compile with nvcc hello_world_gpu.cu -o hello_world_gpu
// execute using ./hello_world_gpu

#include <stdio.h>

// device code, called kernel
__global__ void mykernel() {
  printf("Hello World from GPU!\n");
}

int main(void) {
  // kernel launch, async
  // 1 Block with 1 Thread
  mykernel<<<1,1>>>();

  // wait for device to finish execution
  cudaDeviceSynchronize();

  return 0;
}