#pragma once
#include "types.cuh"
struct LaunchParams {
  Scene scene;
  CameraGPU camera;
  unsigned char *pixels;
  float cutoff;
};
struct HitData {
  const int *ids;
};
