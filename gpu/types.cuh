#pragma once
#include <cmath>
#include <cuda_runtime.h>
#define HD __host__ __device__
struct V {
  float x, y, z;
  V() = default;
  HD V(float a, float b, float c) : x(a), y(b), z(c) {}
  HD V operator+(V b) const { return {x + b.x, y + b.y, z + b.z}; }
  HD V operator-(V b) const { return {x - b.x, y - b.y, z - b.z}; }
  HD V operator*(float b) const { return {x * b, y * b, z * b}; }
  HD float dot(V b) const { return x * b.x + y * b.y + z * b.z; }
  HD V cross(V b) const { return {y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x}; }
  HD V unit() const { return *this * (1.0f / sqrtf(dot(*this))); }
};
struct Material {
  V color;
  float opacity, reflection, ambient;
  int offset, w, h;
};
enum Kind { SPHERE, PLANE, BOX, DISK, TRIANGLE };
struct Primitive {
  V center, normal, right, up, inverseRight, inverseUp;
  float d, tx, ty, third, radius, yaw, pitch, mx, my, ox, oy;
  int kind, texture, normalMap;
};
struct Lamp {
  V center;
  float red;
};
struct Node {
  V lo, hi;
  int left, right, start, count;
};
struct Scene {
  const Primitive *primitives;
  const Material *materials;
  const uchar4 *texels;
  const Lamp *lights;
  const Node *nodes;
  const int *indices;
  const int *planes;
  int count, lightCount, planeCount, nodeCount, sky, depth, brute;
  unsigned long long traversable;
};
struct CameraGPU {
  V origin, forward, right, up;
  int width, height;
};
