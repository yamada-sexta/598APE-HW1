#define GPU_OPTIX_DEVICE
#include "optix_params.h"
#include "trace.cuh"
extern "C" {
__constant__ LaunchParams params;
}
static __forceinline__ __device__ int primitiveId() {
  const HitData *data = reinterpret_cast<const HitData *>(optixGetSbtDataPointer());
  return data->ids[optixGetPrimitiveIndex()];
}
extern "C" __global__ void __raygen__render() {
  uint3 index = optixGetLaunchIndex();
  CameraGPU c = params.camera;
  V v = c.forward + c.right * (float(index.x) / c.width - .5f) +
        c.up * (.5f - float(index.y) / c.height);
  V rgb = traceIterative(params.scene, c.origin, v, params.cutoff);
  size_t i = 3 * (size_t(index.y) * c.width + index.x);
  params.pixels[i] = (unsigned char)fminf(255.f, fmaxf(0.f, rgb.x));
  params.pixels[i + 1] = (unsigned char)fminf(255.f, fmaxf(0.f, rgb.y));
  params.pixels[i + 2] = (unsigned char)fminf(255.f, fmaxf(0.f, rgb.z));
}
extern "C" __global__ void __miss__empty() {}
extern "C" __global__ void __closesthit__surface() {
  optixSetPayload_0(primitiveId());
  optixSetPayload_1(__float_as_uint(optixGetRayTmax()));
}
extern "C" __global__ void __intersection__analytic() {
  float3 a = optixGetObjectRayOrigin(), b = optixGetObjectRayDirection();
  float t = intersect(params.scene.primitives[primitiveId()], V(a.x, a.y, a.z), V(b.x, b.y, b.z));
  if (t < FAR_T)
    optixReportIntersection(t, 0);
}
extern "C" __global__ void __anyhit__shadow() {
  const Scene &s = params.scene;
  const Primitive &p = s.primitives[primitiveId()];
  if (optixGetRayTmax() >= 1.f) {
    optixIgnoreIntersection();
    return;
  }
  if (p.kind != SPHERE && s.materials[p.texture].opacity > 1.f - 1e-6f) {
    optixSetPayload_0(0);
    optixTerminateRay();
    return;
  }
  float3 a = optixGetWorldRayOrigin(), b = optixGetWorldRayDirection();
  V o(a.x, a.y, a.z), v(b.x, b.y, b.z);
  Sample c = surface(s, p, o + v * optixGetRayTmax(), true);
  if (c.opacity > 1.f - 1e-6f) {
    optixSetPayload_0(0);
    optixTerminateRay();
    return;
  }
  optixSetPayload_0(__float_as_uint(__uint_as_float(optixGetPayload_0()) * c.color.x / 255.f));
  optixIgnoreIntersection();
}
