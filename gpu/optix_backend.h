#pragma once
#include "types.cuh"
#include <vector>
void rtBuild(const std::vector<Primitive> &primitives);
void rtRender(Scene scene, CameraGPU camera, unsigned char *pixels, float cutoff, cudaEvent_t start,
              cudaEvent_t stop);
void rtShutdown();
