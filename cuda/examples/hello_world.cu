#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

__gloabl__ void mykernel(void) { }

int main(void) {
  mykernel<<1,1>>();
  printf("Hello World!\n");
  return 0;
}