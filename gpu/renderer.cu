#include "fsr.cuh"
#include "renderer.h"
#ifdef ENABLE_OPTIX
#include "optix_backend.h"
#endif
#include "../src/Textures/imagetexture.h"
#include "../src/box.h"
#include "../src/disk.h"
#include "../src/sphere.h"
#include "../src/triangle.h"
#include "trace.cuh"
#include "warmup.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#undef inf
namespace {
float renderScale = 1.f, fsrSharpness = .2f;
const char *fsrMode = "off";
void check(cudaError_t e, const char *where) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string(where) + ": " + cudaGetErrorString(e));
}
#define CUDA(call) check(call, #call)
V cv(const Vector &p) { return V(p.x, p.y, p.z); }
template <class T> struct Buffer {
  T *ptr = nullptr;
  size_t capacity = 0;
  ~Buffer() {
    if (ptr)
      cudaFree(ptr);
  }
  void reserve(size_t n) {
    if (n > capacity) {
      if (ptr)
        CUDA(cudaFree(ptr));
      ptr = nullptr;
      capacity = 0;
      CUDA(cudaMalloc(&ptr, n * sizeof(T)));
      capacity = n;
    }
  }
  void upload(const std::vector<T> &v) {
    reserve(v.size());
    if (!v.empty())
      CUDA(cudaMemcpy(ptr, v.data(), v.size() * sizeof(T), cudaMemcpyHostToDevice));
  }
};
struct Bounds {
  V lo = V(FAR_T, FAR_T, FAR_T), hi = V(-FAR_T, -FAR_T, -FAR_T);
};
V minimum(V a, V b) { return V(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)); }
V maximum(V a, V b) { return V(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)); }
void extend(Bounds &b, V p) {
  b.lo = minimum(b.lo, p);
  b.hi = maximum(b.hi, p);
}
float axis(V a, int k) { return k == 0 ? a.x : k == 1 ? a.y : a.z; }
struct Renderer {
  Autonoma *owner = nullptr;
  bool useOptix = false;
  bool dirty = true;
  Scene cachedScene{};
  std::string backend = "auto";
  Buffer<Primitive> primitives;
  Buffer<Material> materials;
  Buffer<uchar4> texels;
  Buffer<Lamp> lights;
  Buffer<Node> nodes;
  Buffer<int> indices, planes;
  Buffer<unsigned char> pixels, upscaled;
  Buffer<float4> easuPixels;
  cudaEvent_t upscaleStop;
  std::vector<Primitive> hostPrimitives, previous;
  std::vector<Material> hostMaterials;
  std::vector<uchar4> hostTexels;
  std::vector<Node> hostNodes;
  std::vector<int> hostIndices, hostPlanes;
  std::vector<Bounds> boxes;
  std::unordered_map<Texture *, int> textureIds;
  unsigned char *registeredHost = nullptr;
  size_t registeredBytes = 0;
  cudaEvent_t start, stop;
  int skyId = 0, frame = 0;
  Renderer() {
    if (getenv("GPU_BACKEND"))
      backend = getenv("GPU_BACKEND");
    if (backend != "auto" && backend != "cuda" && backend != "optix")
      throw std::runtime_error("GPU_BACKEND must be auto, cuda, or optix");
#ifndef ENABLE_OPTIX
    if (backend == "optix")
      throw std::runtime_error("This binary has no OptiX support; build raytracer-optix");
#endif
    auto initStart = std::chrono::steady_clock::now();
    CUDA(cudaFree(nullptr));
    auto contextReady = std::chrono::steady_clock::now();
    if ((getenv("GPU_ITERATIVE") && atoi(getenv("GPU_ITERATIVE")) == 0) ||
        getenv("GPU_LEGACY_STACK"))
      CUDA(cudaDeviceSetLimit(cudaLimitStackSize, 16384));
    auto stackReady = std::chrono::steady_clock::now();
    if (getenv("GPU_PROFILE_SETUP"))
      fprintf(stderr, "setup context_ms=%.3f stack_ms=%.3f\n",
              std::chrono::duration<double, std::milli>(contextReady - initStart).count(),
              std::chrono::duration<double, std::milli>(stackReady - contextReady).count());
    CUDA(cudaEventCreate(&start));
    CUDA(cudaEventCreate(&stop));
    CUDA(cudaEventCreate(&upscaleStop));
    cudaDeviceProp p;
    CUDA(cudaGetDeviceProperties(&p, 0));
    fprintf(stderr, "CUDA device: %s (%d SMs)\n", p.name, p.multiProcessorCount);
  }
  ~Renderer() {
    if (registeredHost)
      cudaHostUnregister(registeredHost);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    cudaEventDestroy(upscaleStop);
  }
  int material(Texture *t) {
    if (!t)
      return -1;
    auto found = textureIds.find(t);
    if (found != textureIds.end())
      return found->second;
    Material m{};
    m.opacity = t->opacity;
    m.reflection = t->reflection;
    m.ambient = t->ambient;
    m.offset = -1;
    if (auto *image = dynamic_cast<ImageTexture *>(t)) {
      m.offset = int(hostTexels.size());
      m.w = image->w;
      m.h = image->h;
      size_t count = size_t(m.w) * m.h;
      hostTexels.resize(hostTexels.size() + count);
      static_assert(sizeof(uchar4) == 4, "Packed RGBA8 required");
      memcpy(hostTexels.data() + m.offset, image->imageData, count * 4);
    } else if (auto *c = dynamic_cast<ColorTexture *>(t))
      m.color = V(c->r, c->g, c->b);
    else
      throw std::runtime_error("Unsupported GPU texture type");
    int id = hostMaterials.size();
    hostMaterials.push_back(m);
    textureIds[t] = id;
    return id;
  }
  Primitive primitive(Shape *s) {
    Primitive p{};
    p.center = cv(s->center);
    p.texture = material(s->texture);
    p.normalMap = material(s->normalMap);
    p.tx = s->textureX;
    p.ty = s->textureY;
    p.yaw = s->yaw;
    p.pitch = s->pitch;
    p.mx = s->mapX;
    p.my = s->mapY;
    p.ox = s->mapOffX;
    p.oy = s->mapOffY;
    if (auto *sphere = dynamic_cast<Sphere *>(s)) {
      p.kind = SPHERE;
      p.radius = sphere->radius;
    } else if (auto *plane = dynamic_cast<Plane *>(s)) {
      p.kind = PLANE;
      p.normal = cv(plane->vect);
      p.right = cv(plane->right);
      p.up = cv(plane->up);
      p.d = plane->d;
      Vector a = plane->up.cross(plane->vect), b = plane->vect.cross(plane->right);
      double det = plane->right.dot(a);
      p.inverseRight = cv(a / det);
      p.inverseUp = cv(b / det);
      if (auto *tri = dynamic_cast<Triangle *>(s)) {
        p.kind = TRIANGLE;
        p.third = tri->thirdX;
      } else if (dynamic_cast<Box *>(s))
        p.kind = BOX;
      else if (dynamic_cast<Disk *>(s))
        p.kind = DISK;
    } else
      throw std::runtime_error("Unsupported GPU primitive type");
    return p;
  }
  Bounds bound(const Primitive &p) {
    Bounds b;
    if (p.kind == SPHERE) {
      V r(p.radius, p.radius, p.radius);
      extend(b, p.center - r);
      extend(b, p.center + r);
    } else if (p.kind == TRIANGLE) {
      V right = p.right - p.normal * (p.normal.dot(p.right) / p.normal.dot(p.normal));
      V up = p.up - p.normal * (p.normal.dot(p.up) / p.normal.dot(p.normal));
      extend(b, p.center);
      extend(b, p.center + right * p.tx);
      extend(b, p.center + right * p.third + up * p.ty);
    } else {
      float scale = p.kind == BOX ? .5f : 1.f;
      for (int x : {-1, 1})
        for (int y : {-1, 1})
          extend(b, p.center + p.right * (x * p.tx * scale) + p.up * (y * p.ty * scale));
    }
    V pad =
        maximum(V(1e-5f, 1e-5f, 1e-5f), maximum(V(fabsf(b.lo.x), fabsf(b.lo.y), fabsf(b.lo.z)),
                                                V(fabsf(b.hi.x), fabsf(b.hi.y), fabsf(b.hi.z))) *
                                            2e-6f);
    b.lo = b.lo - pad;
    b.hi = b.hi + pad;
    return b;
  }
  int build(int begin, int end) {
    int id = hostNodes.size();
    hostNodes.push_back({});
    Bounds b, centers;
    for (int j = begin; j < end; ++j) {
      auto v = boxes[hostIndices[j]];
      extend(b, v.lo);
      extend(b, v.hi);
      extend(centers, (v.lo + v.hi) * .5f);
    }
    Node n{};
    n.lo = b.lo;
    n.hi = b.hi;
    n.start = begin;
    n.count = end - begin;
    if (end - begin > 4) {
      V extent = centers.hi - centers.lo;
      int k = extent.y > extent.x ? 1 : 0;
      if (extent.z > axis(extent, k))
        k = 2;
      int mid = (begin + end) / 2;
      std::nth_element(hostIndices.begin() + begin, hostIndices.begin() + mid,
                       hostIndices.begin() + end, [&](int a, int c) {
                         return axis(boxes[a].lo + boxes[a].hi, k) <
                                axis(boxes[c].lo + boxes[c].hi, k);
                       });
      n.count = 0;
      n.left = build(begin, mid);
      n.right = build(mid, end);
    }
    hostNodes[id] = n;
    return id;
  }
  Scene prepare(Autonoma *s) {
    if (owner && owner != s)
      throw std::runtime_error("Call gpuShutdown before rendering a different scene");
    owner = s;
    if (!dirty)
      return cachedScene;
    if (hostMaterials.empty()) {
      std::unordered_set<Texture *> textures;
      textures.insert(s->skybox);
      for (auto *n = s->listStart; n; n = n->next) {
        textures.insert(n->data->texture);
        textures.insert(n->data->normalMap);
      }
      size_t texelsNeeded = 0;
      for (auto *t : textures)
        if (auto *image = dynamic_cast<ImageTexture *>(t))
          texelsNeeded += size_t(image->w) * image->h;
      hostTexels.reserve(texelsNeeded);
    }
    size_t oldMaterials = hostMaterials.size();
    skyId = material(s->skybox);
    hostPrimitives.clear();
    for (auto *n = s->listStart; n; n = n->next)
      hostPrimitives.push_back(primitive(n->data));
#ifdef ENABLE_OPTIX
    useOptix = backend == "optix" || (backend == "auto" && hostPrimitives.size() >= 64);
#endif
    if (s->depth > 10)
      throw std::runtime_error("GPU renderer supports at most 10 secondary-ray levels");
    if (oldMaterials != hostMaterials.size()) {
      materials.upload(hostMaterials);
      texels.upload(hostTexels);
    }
    bool changed = previous.size() != hostPrimitives.size() ||
                   (!previous.empty() && memcmp(previous.data(), hostPrimitives.data(),
                                                previous.size() * sizeof(Primitive)));
    bool geometryChanged = previous.size() != hostPrimitives.size();
    if (!geometryChanged)
      for (size_t i = 0; i < previous.size(); ++i) {
        Primitive a = previous[i], b = hostPrimitives[i];
        a.texture = b.texture = 0;
        a.normalMap = b.normalMap = 0;
        a.yaw = b.yaw = a.pitch = b.pitch = a.mx = b.mx = a.my = b.my = a.ox = b.ox = a.oy = b.oy =
            0;
        if (a.kind == SPHERE)
          a.tx = b.tx = a.ty = b.ty = 0;
        if (memcmp(&a, &b, sizeof(a))) {
          geometryChanged = true;
          break;
        }
      }
    if (changed || frame == 0)
      primitives.upload(hostPrimitives);
    if (geometryChanged || frame == 0) {
      hostIndices.clear();
      hostPlanes.clear();
      boxes.resize(hostPrimitives.size());
      hostNodes.clear();
      for (size_t i = 0; i < hostPrimitives.size(); ++i) {
        if (hostPrimitives[i].kind == PLANE)
          hostPlanes.push_back(i);
        else if (!useOptix) {
          boxes[i] = bound(hostPrimitives[i]);
          hostIndices.push_back(i);
        }
      }
      if (!hostIndices.empty())
        build(0, hostIndices.size());
      nodes.upload(hostNodes);
      indices.upload(hostIndices);
      planes.upload(hostPlanes);
#ifdef ENABLE_OPTIX
      if (useOptix)
        rtBuild(hostPrimitives);
#endif
    }
    previous = hostPrimitives;
    std::vector<Lamp> lamps;
    for (auto *n = s->lightStart; n; n = n->next)
      lamps.push_back({cv(n->data->center), float(n->data->color[0]) / 255.f});
    lights.upload(lamps);
    int brute = getenv("GPU_BRUTE") ? atoi(getenv("GPU_BRUTE")) : (hostPrimitives.size() <= 4);
    dirty = false;
    return cachedScene = {primitives.ptr,
                          materials.ptr,
                          texels.ptr,
                          lights.ptr,
                          nodes.ptr,
                          indices.ptr,
                          planes.ptr,
                          int(hostPrimitives.size()),
                          int(lamps.size()),
                          int(hostPlanes.size()),
                          int(hostNodes.size()),
                          skyId,
                          int(s->depth),
                          brute};
  }
};
Renderer *renderer = nullptr;
template <bool iterative>
__global__ void renderKernel(Scene scene, CameraGPU cam, unsigned char *rgb, float cutoff) {
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= cam.width || y >= cam.height)
    return;
  V v = cam.forward + cam.right * (float(x) / cam.width - .5f) +
        cam.up * (.5f - float(y) / cam.height);
  V c = iterative ? traceIterative(scene, cam.origin, v, cutoff) : trace(scene, cam.origin, v, 0);
  size_t i = 3 * (size_t(y) * cam.width + x);
  rgb[i] = (unsigned char)c.x;
  rgb[i + 1] = (unsigned char)c.y;
  rgb[i + 2] = (unsigned char)c.z;
}
} 
void gpuConfigureFsr(const char *mode, float sharpnessStops) {
  if (renderer)
    throw std::runtime_error("Configure FSR before rendering");
  std::string name(mode);
  if (name == "off")
    renderScale = 1.f;
  else if (name == "ultra-quality")
    renderScale = 1.f / 1.3f;
  else if (name == "quality")
    renderScale = 1.f / 1.5f;
  else if (name == "balanced")
    renderScale = 1.f / 1.7f;
  else if (name == "performance")
    renderScale = .5f;
  else
    throw std::runtime_error(
        "FSR mode must be off, ultra-quality, quality, balanced, or performance");
  if (!std::isfinite(sharpnessStops) || sharpnessStops < 0 || sharpnessStops > 2)
    throw std::runtime_error("FSR sharpness must be 0 to 2 stops (0 strongest)");
  fsrMode = mode;
  fsrSharpness = sharpnessStops;
}
void gpuRender(Autonoma *s, unsigned char *rgb, int width, int height) {
  auto frameStart = std::chrono::steady_clock::now();
  gpuWaitWarmup();
  if (!renderer)
    renderer = new Renderer;
  auto &r = *renderer;
  auto initDone = std::chrono::steady_clock::now();
  size_t bytes = size_t(width) * height * 3;
  if (!getenv("GPU_PAGEABLE") && (r.registeredHost != rgb || r.registeredBytes != bytes)) {
    if (r.registeredHost)
      CUDA(cudaHostUnregister(r.registeredHost));
    r.registeredHost = nullptr;
    r.registeredBytes = 0;
    CUDA(cudaHostRegister(rgb, bytes, cudaHostRegisterDefault));
    r.registeredHost = rgb;
    r.registeredBytes = bytes;
  }
  auto pinDone = std::chrono::steady_clock::now();
  Scene scene = r.prepare(s);
  auto prepareDone = std::chrono::steady_clock::now();
  int rw = std::max(1, int(std::ceil(width * renderScale))),
      rh = std::max(1, int(std::ceil(height * renderScale)));
  bool upscale = rw != width || rh != height;
  r.pixels.reserve(size_t(rw) * rh * 3);
  if (upscale) {
    r.upscaled.reserve(bytes);
    r.easuPixels.reserve(size_t(width) * height);
  }
  auto buffersDone = std::chrono::steady_clock::now();
  CameraGPU cam{
      cv(s->camera.focus), cv(s->camera.forward), cv(s->camera.right), cv(s->camera.up), rw, rh};
  if (upscale)
    cam.forward =
        cam.forward + cam.right * (.5f / rw - .5f / width) + cam.up * (.5f / height - .5f / rh);
  int bx = getenv("GPU_BLOCK_X") ? atoi(getenv("GPU_BLOCK_X")) : (scene.count >= 64 ? 8 : 32);
  int by = getenv("GPU_BLOCK_Y") ? atoi(getenv("GPU_BLOCK_Y")) : (scene.count >= 64 ? 8 : 4);
  if (bx <= 0 || by <= 0 || bx * by > 1024)
    throw std::runtime_error("Invalid GPU block dimensions");
  int iterative = getenv("GPU_ITERATIVE") ? atoi(getenv("GPU_ITERATIVE")) : 1;
  float cutoff = getenv("GPU_CUTOFF") ? atof(getenv("GPU_CUTOFF")) : 0.001f;
  if (!std::isfinite(cutoff) || cutoff < 0 || cutoff > .01f)
    throw std::runtime_error("GPU_CUTOFF must be between 0 and 0.01");
  dim3 block(bx, by);
  dim3 grid((rw + block.x - 1) / block.x, (rh + block.y - 1) / block.y);
#ifdef ENABLE_OPTIX
  if (r.useOptix)
    rtRender(scene, cam, r.pixels.ptr, cutoff, r.start, r.stop);
  else
#endif
  {
    CUDA(cudaEventRecord(r.start));
    if (iterative)
      renderKernel<true><<<grid, block>>>(scene, cam, r.pixels.ptr, cutoff);
    else
      renderKernel<false><<<grid, block>>>(scene, cam, r.pixels.ptr, cutoff);
    CUDA(cudaGetLastError());
    CUDA(cudaEventRecord(r.stop));
  }
  auto submitDone = std::chrono::steady_clock::now();
  unsigned char *result = r.pixels.ptr;
  if (upscale) {
    dim3 tile(16, 16), tiles((width + 15) / 16, (height + 15) / 16);
    fsr::easuKernel<<<tiles, tile>>>(r.pixels.ptr, r.easuPixels.ptr, rw, rh, width, height);
    CUDA(cudaGetLastError());
    fsr::rcasKernel<<<tiles, tile>>>(r.easuPixels.ptr, r.upscaled.ptr, width, height,
                                     std::exp2(-fsrSharpness));
    CUDA(cudaGetLastError());
    result = r.upscaled.ptr;
  }
  if (upscale)
    CUDA(cudaEventRecord(r.upscaleStop));
  CUDA(cudaEventSynchronize(upscale ? r.upscaleStop : r.stop));
  float ms, upscaleMs = 0;
  CUDA(cudaEventElapsedTime(&ms, r.start, r.stop));
  if (upscale)
    CUDA(cudaEventElapsedTime(&upscaleMs, r.stop, r.upscaleStop));
  CUDA(cudaMemcpy(rgb, result, size_t(width) * height * 3, cudaMemcpyDeviceToHost));
  if (r.frame == 0 && getenv("GPU_PROFILE_SETUP")) {
    auto elapsed = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    fprintf(stderr,
            "setup constructor_ms=%.3f pin_ms=%.3f prepare_ms=%.3f buffers_ms=%.3f submit_ms=%.3f "
            "finish_ms=%.3f\n",
            elapsed(frameStart, initDone), elapsed(initDone, pinDone),
            elapsed(pinDone, prepareDone), elapsed(prepareDone, buffersDone),
            elapsed(buffersDone, submitDone),
            elapsed(submitDone, std::chrono::steady_clock::now()));
  }
  double frameMs =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frameStart)
          .count();
  fprintf(stderr,
          "GPU frame=%d kernel_ms=%.6f frame_ms=%.6f upscale_ms=%.6f gpu_total_ms=%.6f "
          "internal=%dx%d fsr=%s primitives=%d bvh_nodes=%d mode=%s\n",
          r.frame++, ms, frameMs, upscale ? upscaleMs : 0.f, ms + (upscale ? upscaleMs : 0.f), rw,
          rh, fsrMode, scene.count, scene.nodeCount,
          r.useOptix    ? "optix"
          : scene.brute ? "brute"
                        : "bvh");
}
void gpuSceneChanged() {
  if (renderer)
    renderer->dirty = true;
}
void gpuShutdown() {
#ifdef ENABLE_OPTIX
  rtShutdown();
#endif
  delete renderer;
  renderer = nullptr;
}
