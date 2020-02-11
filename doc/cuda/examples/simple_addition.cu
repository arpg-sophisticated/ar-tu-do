// compile with nvcc simple_addition.cu -o simple_addition
// execute using ./simple_addition

#include <stdio.h>

__global__ void add(int *a, int *b, int *c) {
  *c = *a + *b;
  printf("d_a: %d, d_b: %d, d_c: %d\n", *a, *b, *c);
}

int main(void) {
  int a, b, c;            // host copies of a, b, c
  int *d_a, *d_b, *d_c;   // device copies of a, b, c
  int size = sizeof(int);

  // Allocate space for device copies of a, b, c
  cudaMalloc((void **)&d_a, size);
  cudaMalloc((void **)&d_b, size);
  cudaMalloc((void **)&d_c, size);

  // Setup input values
  a = 2;
  b = 7;

  printf("Initial state\n");
  printf("a: %d, b: %d, c: %d\n", a, b, c);
  
  // Copy inputs to device
  cudaMemcpy(d_a, &a, size, cudaMemcpyHostToDevice);
  cudaMemcpy(d_b, &b, size, cudaMemcpyHostToDevice);

  printf("Copy to device\n");
  printf("a: %d, b: %d, c: %d\n", a, b, c);
  
  // Launch add() kernel on GPU
  add<<<1,1>>>(d_a, d_b, d_c);

  printf("After kernel launch\n");
  printf("a: %d, b: %d, c: %d\n", a, b, c);
  
  // Copy result back to host
  cudaMemcpy(&c, d_c, size, cudaMemcpyDeviceToHost);

  printf("After copy to host\n");
  printf("a: %d, b: %d, c: %d\n", a, b, c);
  
  // Cleanup
  cudaFree(d_a); cudaFree(d_b); cudaFree(d_c);

  printf("After cleanup\n");
  printf("a: %d, b: %d, c: %d\n", a, b, c);
  
  return 0;
}