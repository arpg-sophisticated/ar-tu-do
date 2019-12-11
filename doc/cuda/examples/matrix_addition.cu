// compile with nvcc matrix_addition.cu -o matrix_addition
// execute using ./matrix_addition

#include <stdio.h>

__global__ void add(int *a, int *b, int *c) {
    c[blockIdx.x*threadIdx.x+threadIdx.x] = a[blockIdx.x*threadIdx.x+threadIdx.x] + b[blockIdx.x*threadIdx.x+threadIdx.x];
    printf("Block %d Thread %d -- a: %d, b: %d, c: %d\n", blockIdx.x, threadIdx.x, c[blockIdx.x*threadIdx.x+threadIdx.x], a[blockIdx.x*threadIdx.x+threadIdx.x], b[blockIdx.x*threadIdx.x+threadIdx.x]);
}

void random_ints(int* a, int N, int M) {
    int i, j;
    for (i = 0; i < N; ++i) {
        for (j = 0; j < M; ++j) {
            a[i*j+j] = rand();
        }
    }
}

// 4096 values in total
#define N 64
#define M 64
int main(void) {
    int *a, *b, *c;         // host copies of a, b, c
    int *d_a, *d_b, *d_c;   // device copies of a, b, c
    int size = N * M * sizeof(int);

    // Alloc space for device copies of a, b, c
    cudaMalloc((void **)&d_a, size);
    cudaMalloc((void **)&d_b, size);
    cudaMalloc((void **)&d_c, size);

    // Alloc space for host copies of a, b, c and setup input values
    a = (int *)malloc(size); random_ints(a, N, M);
    b = (int *)malloc(size); random_ints(b, N, M);
    c = (int *)malloc(size);
    
    // Copy inputs to device
    cudaMemcpy(d_a, a, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, size, cudaMemcpyHostToDevice);
    
    // Launch add() kernel on GPU with N blocks
    add<<<N,M>>>(d_a, d_b, d_c);
    
    // Copy result back to host
    cudaMemcpy(c, d_c, size, cudaMemcpyDeviceToHost);
    
    // Cleanup
    free(a); free(b); free(c);
    cudaFree(d_a); cudaFree(d_b); cudaFree(d_c);
    
    return 0;
}