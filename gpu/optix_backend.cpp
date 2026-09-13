#include "optix_backend.h"
#include "optix_params.h"
#include <algorithm>
#include <cstdio>
#include <cuda.h>
#include <fstream>
#include <iterator>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>
#include <stdexcept>
#include <string>
namespace {
void ck(OptixResult r, const char *call) {
  if (r != OPTIX_SUCCESS)
    throw std::runtime_error(std::string(call) + ": " + optixGetErrorName(r));
}
void ck(cudaError_t r, const char *call) {
  if (r != cudaSuccess)
    throw std::runtime_error(std::string(call) + ": " + cudaGetErrorString(r));
}
#define CK(x) ck(x, #x)
template <class T> struct Memory {
  T *p = nullptr;
  size_t cap = 0;
  ~Memory() {
    if (p)
      cudaFree(p);
  }
  void alloc(size_t n) {
    if (n > cap) {
      if (p)
        CK(cudaFree(p));
      p = nullptr;
      cap = 0;
      CK(cudaMalloc(&p, n * sizeof(T)));
      cap = n;
    }
  }
  void put(const std::vector<T> &v) {
    alloc(v.size());
    if (v.size())
      CK(cudaMemcpy(p, v.data(), v.size() * sizeof(T), cudaMemcpyHostToDevice));
  }
  CUdeviceptr address() { return reinterpret_cast<CUdeviceptr>(p); }
};
template <class T> struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) Record {
  char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  T data;
};
using HitRecord = Record<HitData>;
using EmptyRecord = Record<int>;
struct Backend {
  OptixDeviceContext context = nullptr;
  OptixModule module = nullptr;
  OptixPipeline pipeline = nullptr;
  OptixProgramGroup raygen = nullptr, miss = nullptr, triangle = nullptr, custom = nullptr;
  OptixTraversableHandle handle = 0;
  OptixShaderBindingTable sbt{};
  Memory<V> vertices;
  Memory<OptixAabb> boxes;
  Memory<int> triIds, customIds;
  Memory<unsigned char> temp, accel, triangleAccel, customAccel;
  Memory<OptixInstance> instances;
  Memory<LaunchParams> launch;
  Memory<EmptyRecord> rayRecords, missRecords;
  Memory<HitRecord> hitRecords;
  Backend() {
    CK(optixInit());
    CK(optixDeviceContextCreate(nullptr, nullptr, &context));
    std::ifstream f(OPTIX_PTX_PATH, std::ios::binary);
    if (!f)
      throw std::runtime_error("Cannot read OptiX PTX at " OPTIX_PTX_PATH
                               "; rebuild with make -f Makefile.nvidia optix");
    std::string ptx((std::istreambuf_iterator<char>(f)), {});
    OptixModuleCompileOptions mo{};
    mo.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_3;
    mo.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
    OptixPipelineCompileOptions po{};
    po.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_LEVEL_INSTANCING;
    po.numPayloadValues = 2;
    po.numAttributeValues = 2;
    po.pipelineLaunchParamsVariableName = "params";
    po.usesPrimitiveTypeFlags =
        OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE | OPTIX_PRIMITIVE_TYPE_FLAGS_CUSTOM;
    char log[8192];
    size_t size = sizeof(log);
    OptixResult result =
        optixModuleCreate(context, &mo, &po, ptx.data(), ptx.size(), log, &size, &module);
    if (result != OPTIX_SUCCESS)
      fprintf(stderr, "OptiX module log: %s\n", log);
    CK(result);
    OptixProgramGroupOptions go{};
    OptixProgramGroupDesc d{};
    d.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    d.raygen.module = module;
    d.raygen.entryFunctionName = "__raygen__render";
    CK(optixProgramGroupCreate(context, &d, 1, &go, nullptr, nullptr, &raygen));
    d = {};
    d.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    d.miss.module = module;
    d.miss.entryFunctionName = "__miss__empty";
    CK(optixProgramGroupCreate(context, &d, 1, &go, nullptr, nullptr, &miss));
    d = {};
    d.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    d.hitgroup.moduleCH = module;
    d.hitgroup.entryFunctionNameCH = "__closesthit__surface";
    d.hitgroup.moduleAH = module;
    d.hitgroup.entryFunctionNameAH = "__anyhit__shadow";
    CK(optixProgramGroupCreate(context, &d, 1, &go, nullptr, nullptr, &triangle));
    d.hitgroup.moduleIS = module;
    d.hitgroup.entryFunctionNameIS = "__intersection__analytic";
    CK(optixProgramGroupCreate(context, &d, 1, &go, nullptr, nullptr, &custom));
    OptixProgramGroup groups[] = {raygen, miss, triangle, custom};
    OptixPipelineLinkOptions lo{};
    lo.maxTraceDepth = 1;
    size = sizeof(log);
    result = optixPipelineCreate(context, &po, &lo, groups, 4, log, &size, &pipeline);
    if (result != OPTIX_SUCCESS)
      fprintf(stderr, "OptiX pipeline log: %s\n", log);
    CK(result);
    OptixStackSizes stack{};
    for (auto g : groups)
      CK(optixUtilAccumulateStackSizes(g, &stack, pipeline));
    unsigned a, b, c;
    CK(optixUtilComputeStackSizes(&stack, 1, 0, 0, &a, &b, &c));
    CK(optixPipelineSetStackSize(pipeline, a, b, c, 2));
    EmptyRecord rg{}, ms{};
    CK(optixSbtRecordPackHeader(raygen, &rg));
    CK(optixSbtRecordPackHeader(miss, &ms));
    rayRecords.put({rg});
    missRecords.put({ms, ms});
    sbt.raygenRecord = rayRecords.address();
    sbt.missRecordBase = missRecords.address();
    sbt.missRecordCount = 2;
    sbt.missRecordStrideInBytes = sizeof(EmptyRecord);
    fprintf(stderr, "OptiX 9.1: RTX hardware traversal and triangle intersections enabled\n");
  }
  ~Backend() {
    if (pipeline)
      optixPipelineDestroy(pipeline);
    for (auto g : {raygen, miss, triangle, custom})
      if (g)
        optixProgramGroupDestroy(g);
    if (module)
      optixModuleDestroy(module);
    if (context)
      optixDeviceContextDestroy(context);
  }
  void build(const std::vector<Primitive> &primitives) {
    std::vector<V> v;
    std::vector<int> triangles, analytic;
    std::vector<OptixAabb> bounds;
    for (size_t i = 0; i < primitives.size(); ++i) {
      const auto &p = primitives[i];
      if (p.kind == PLANE)
        continue;
      V right = p.right - p.normal * (p.normal.dot(p.right) / p.normal.dot(p.normal));
      V up = p.up - p.normal * (p.normal.dot(p.up) / p.normal.dot(p.normal));
      if (p.kind == TRIANGLE) {
        triangles.push_back(i);
        v.push_back(p.center);
        v.push_back(p.center + right * p.tx);
        v.push_back(p.center + right * p.third + up * p.ty);
      } else {
        analytic.push_back(i);
        V extent;
        if (p.kind == SPHERE)
          extent = V(p.radius, p.radius, p.radius);
        else {
          float scale = p.kind == BOX ? .5f : 1.f;
          extent = V(fabsf(right.x * p.tx) + fabsf(up.x * p.ty),
                     fabsf(right.y * p.tx) + fabsf(up.y * p.ty),
                     fabsf(right.z * p.tx) + fabsf(up.z * p.ty)) *
                   scale;
        }
        extent = extent + V(1e-4f, 1e-4f, 1e-4f);
        V low = p.center - extent, high = p.center + extent;
        bounds.push_back({low.x, low.y, low.z, high.x, high.y, high.z});
      }
    }
    vertices.put(v);
    triIds.put(triangles);
    boxes.put(bounds);
    customIds.put(analytic);
    std::vector<OptixBuildInput> inputs;
    std::vector<HitRecord> records;
    unsigned flags = OPTIX_GEOMETRY_FLAG_REQUIRE_SINGLE_ANYHIT_CALL;
    CUdeviceptr vb = vertices.address(), ab = boxes.address();
    if (!v.empty()) {
      OptixBuildInput in{};
      in.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
      auto &t = in.triangleArray;
      t.vertexBuffers = &vb;
      t.numVertices = v.size();
      t.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
      t.vertexStrideInBytes = sizeof(V);
      t.flags = &flags;
      t.numSbtRecords = 1;
      inputs.push_back(in);
      HitRecord r{};
      CK(optixSbtRecordPackHeader(triangle, &r));
      r.data.ids = triIds.p;
      records.push_back(r);
      records.push_back(r);
    }
    if (!bounds.empty()) {
      OptixBuildInput in{};
      in.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
      auto &t = in.customPrimitiveArray;
      t.aabbBuffers = &ab;
      t.numPrimitives = bounds.size();
      t.strideInBytes = sizeof(OptixAabb);
      t.flags = &flags;
      t.numSbtRecords = 1;
      inputs.push_back(in);
      HitRecord r{};
      CK(optixSbtRecordPackHeader(custom, &r));
      r.data.ids = customIds.p;
      records.push_back(r);
      records.push_back(r);
    }
    handle = 0;
    if (inputs.empty())
      return;
    OptixAccelBuildOptions options{};
    options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
    options.operation = OPTIX_BUILD_OPERATION_BUILD;
    auto buildOne = [&](OptixBuildInput &input, Memory<unsigned char> &storage) {
      OptixAccelBufferSizes sizes{};
      CK(optixAccelComputeMemoryUsage(context, &options, &input, 1, &sizes));
      temp.alloc(sizes.tempSizeInBytes);
      storage.alloc(sizes.outputSizeInBytes);
      OptixTraversableHandle result = 0;
      CK(optixAccelBuild(context, 0, &options, &input, 1, temp.address(), sizes.tempSizeInBytes,
                         storage.address(), sizes.outputSizeInBytes, &result, nullptr, 0));
      return result;
    };
    std::vector<OptixInstance> inst;
    for (size_t i = 0; i < inputs.size(); ++i) {
      auto &storage =
          inputs[i].type == OPTIX_BUILD_INPUT_TYPE_TRIANGLES ? triangleAccel : customAccel;
      OptixInstance item{};
      item.transform[0] = item.transform[5] = item.transform[10] = 1.f;
      item.instanceId = i;
      item.sbtOffset = 2 * i;
      item.visibilityMask = 255;
      item.traversableHandle = buildOne(inputs[i], storage);
      inst.push_back(item);
    }
    instances.put(inst);
    OptixBuildInput instanceInput{};
    instanceInput.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
    instanceInput.instanceArray.instances = instances.address();
    instanceInput.instanceArray.numInstances = inst.size();
    handle = buildOne(instanceInput, accel);
    hitRecords.put(records);
    sbt.hitgroupRecordBase = hitRecords.address();
    sbt.hitgroupRecordCount = records.size();
    sbt.hitgroupRecordStrideInBytes = sizeof(HitRecord);
  }
};
Backend *backend = nullptr;
} 
void rtBuild(const std::vector<Primitive> &primitives) {
  if (!backend)
    backend = new Backend;
  backend->build(primitives);
}
void rtRender(Scene scene, CameraGPU camera, unsigned char *pixels, float cutoff, cudaEvent_t start,
              cudaEvent_t stop) {
  scene.traversable = backend->handle;
  scene.brute = 0;
  LaunchParams p{scene, camera, pixels, cutoff};
  backend->launch.put({p});
  CK(cudaEventRecord(start));
  CK(optixLaunch(backend->pipeline, 0, backend->launch.address(), sizeof(p), &backend->sbt,
                 camera.width, camera.height, 1));
  CK(cudaEventRecord(stop));
}
void rtShutdown() {
  delete backend;
  backend = nullptr;
}
