#include "warmup.h"
#include "renderer.h"
#include <cstdlib>
#include <cuda_runtime.h>
#include <future>
#include <stdexcept>
namespace {
std::future<void> warmup;
}
void gpuStartWarmup() {
  if (!getenv("GPU_SERIAL_INIT"))
    warmup = std::async(std::launch::async, [] {
      cudaError_t result = cudaFree(nullptr);
      if (result != cudaSuccess)
        throw std::runtime_error(cudaGetErrorString(result));
    });
}
void gpuWaitWarmup() {
  if (warmup.valid())
    warmup.get();
}
