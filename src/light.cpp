
#include "light.h"
#include "shape.h"
#include "triangle.h"
#include "camera.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <unordered_set>
#if defined(__GNUC__) && defined(__x86_64__)
#include <emmintrin.h>
#define RAY_BVH4_SSE2 1
#endif
#ifdef _OPENMP
#include <omp.h>
#endif
      
Light::Light(const Vector & cente, unsigned char* colo) : center(cente){
   color = colo;
}

unsigned char* Light::getColor(unsigned char a, unsigned char b, unsigned char c){
   unsigned char* r = (unsigned char*)malloc(sizeof(unsigned char)*3);
   r[0] = a;
   r[1] = b;
   r[2] = c;
   return r;
}

Autonoma::Autonoma(const Camera& c): camera(c), useBVH4(false), bvh4MaxDepth(0){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
#if defined(USE_CUDA) || defined(RAY_REFERENCE_SHADING)
   depth = 10;
#else
   depth = 3;
#endif
   skybox = BLACK;
}

Autonoma::Autonoma(const Camera& c, Texture* tex): camera(c), useBVH4(false), bvh4MaxDepth(0){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
#if defined(USE_CUDA) || defined(RAY_REFERENCE_SHADING)
   depth = 10;
#else
   depth = 3;
#endif
   skybox = tex;
}

void Autonoma::addShape(Shape* r){
   accelerationDirty = true;
   ShapeNode* hi = (ShapeNode*)malloc(sizeof(ShapeNode));
   hi->data = r;
   hi->next = hi->prev = NULL;
   if(listStart==NULL){
      listStart = listEnd = hi;
   }
   else{
      listEnd->next = hi;
      hi->prev = listEnd;
      listEnd = hi;
   }
}

void Autonoma::removeShape(ShapeNode* s){
   accelerationDirty = true;
   if(s==listStart){
      if(s==listEnd){
         listStart = listEnd = NULL;
      }
      else{
         listStart = s->next;
         listStart->prev = NULL;
      }
   }
   else if(s==listEnd){
      listEnd = s->prev;
      listEnd->next = NULL;
   }
   else{
      ShapeNode *b4 = s->prev, *aft = s->next;
      b4->next = aft;
      aft->prev = b4;
   }
   free(s);
}

void Autonoma::addLight(Light* r){
   LightNode* hi = (LightNode*)malloc(sizeof(LightNode));
   hi->data = r;
   hi->next = hi->prev = NULL;
   if(lightStart==NULL){
      lightStart = lightEnd = hi;
   }
   else{
      lightEnd->next = hi;
      hi->prev = lightEnd;
      lightEnd = hi;
   }
}

void Autonoma::removeLight(LightNode* s){
   if(s==lightStart){
      if(s==lightEnd){
         lightStart = lightEnd = NULL;
      }
      else{
         lightStart = s->next;
         lightStart->prev = NULL;
      }
   }
   else if(s==lightEnd){
      lightEnd = s->prev;
      lightEnd->next = NULL;
   }
   else{
      LightNode *b4 = s->prev, *aft = s->next;
      b4->next = aft;
      aft->prev = b4;
   }
   free(s);
}

typedef int (*TrianglePacketFunction)(const TrianglePacket&, const Ray&, double, double&);
static TrianglePacketFunction trianglePacketFunction = NULL;
static size_t trianglePacketWidth = 4;
static size_t triangleLeafSize = 4;
static const char* triangleSIMDName = "scalar";

#if defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__))
typedef float RayVec4f __attribute__((vector_size(16)));
typedef float RayVec8f __attribute__((vector_size(32)));
typedef float RayVec16f __attribute__((vector_size(64)));

#define DEFINE_TRIANGLE_PACKET(name, targetName, Vec, lanes) \
__attribute__((target(targetName))) \
static int name(const TrianglePacket& packet, const Ray& ray, double maximum, double& closest) { \
   Vec vx, vy, vz, e1x, e1y, e1z, e2x, e2y, e2z; \
   __builtin_memcpy(&vx, packet.vertexX, sizeof(Vec)); \
   __builtin_memcpy(&vy, packet.vertexY, sizeof(Vec)); \
   __builtin_memcpy(&vz, packet.vertexZ, sizeof(Vec)); \
   __builtin_memcpy(&e1x, packet.edge1X, sizeof(Vec)); \
   __builtin_memcpy(&e1y, packet.edge1Y, sizeof(Vec)); \
   __builtin_memcpy(&e1z, packet.edge1Z, sizeof(Vec)); \
   __builtin_memcpy(&e2x, packet.edge2X, sizeof(Vec)); \
   __builtin_memcpy(&e2y, packet.edge2Y, sizeof(Vec)); \
   __builtin_memcpy(&e2z, packet.edge2Z, sizeof(Vec)); \
   Vec dx = {}, dy = {}, dz = {}, originX = {}, originY = {}, originZ = {}, one = {}; \
   for (size_t lane = 0; lane < lanes; ++lane) { \
      dx[lane] = (float)ray.vector.x; dy[lane] = (float)ray.vector.y; dz[lane] = (float)ray.vector.z; \
      originX[lane] = (float)ray.point.x; originY[lane] = (float)ray.point.y; originZ[lane] = (float)ray.point.z; one[lane] = 1.0f; \
   } \
   const Vec ox = originX - vx, oy = originY - vy, oz = originZ - vz; \
   const Vec px = dy*e2z - dz*e2y, py = dz*e2x - dx*e2z, pz = dx*e2y - dy*e2x; \
   const Vec determinant = e1x*px + e1y*py + e1z*pz; \
   const Vec inverse = one / determinant; \
   const Vec u = (ox*px + oy*py + oz*pz) * inverse; \
   const Vec qx = oy*e1z - oz*e1y, qy = oz*e1x - ox*e1z, qz = ox*e1y - oy*e1x; \
   const Vec v = (dx*qx + dy*qy + dz*qz) * inverse; \
   const Vec time = (e2x*qx + e2y*qy + e2z*qz) * inverse; \
   int best = -1; \
   closest = maximum; \
   for (size_t lane = 0; lane < packet.count; ++lane) { \
      const double determinantValue = determinant[lane]; \
      if (std::abs(determinantValue) < 1e-12 || u[lane] < 0.0 || u[lane] > 1.0 || \
          v[lane] < 0.0 || u[lane] + v[lane] > 1.0 || time[lane] <= 0.0 || time[lane] >= closest) continue; \
      closest = time[lane]; \
      best = (int)lane; \
   } \
   return best; \
}

DEFINE_TRIANGLE_PACKET(intersectTriangleSSE2, "sse2", RayVec4f, 4)
DEFINE_TRIANGLE_PACKET(intersectTriangleAVX, "avx", RayVec8f, 8)
DEFINE_TRIANGLE_PACKET(intersectTriangleAVX2, "avx2,fma", RayVec8f, 8)
DEFINE_TRIANGLE_PACKET(intersectTriangleAVX512, "avx512f", RayVec16f, 16)
#undef DEFINE_TRIANGLE_PACKET

static void selectTriangleSIMD(size_t primitiveCount) {
   trianglePacketFunction = NULL;
   trianglePacketWidth = 4;
   triangleLeafSize = 4;
   triangleSIMDName = "scalar";
   const char* requested = std::getenv("RAY_SIMD");
   __builtin_cpu_init();
   const bool automatic = requested == NULL || std::strcmp(requested, "auto") == 0;
   int workers = 1;
#ifdef _OPENMP
   workers = omp_get_max_threads();
#endif
   const bool preferScalar = automatic && primitiveCount >= 10000 && workers >= 4;
   if (!preferScalar && (automatic || std::strcmp(requested, "avx512") == 0) && __builtin_cpu_supports("avx512f")) {
      trianglePacketFunction = intersectTriangleAVX512;
      trianglePacketWidth = 16;
      triangleLeafSize = 16;
      triangleSIMDName = "avx512";
   } else if (!preferScalar && (automatic || std::strcmp(requested, "avx2") == 0) && __builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma")) {
      trianglePacketFunction = intersectTriangleAVX2;
      trianglePacketWidth = 8;
      triangleLeafSize = 8;
      triangleSIMDName = "avx2";
   } else if (!preferScalar && (automatic || std::strcmp(requested, "avx") == 0) && __builtin_cpu_supports("avx")) {
      trianglePacketFunction = intersectTriangleAVX;
      trianglePacketWidth = 8;
      triangleLeafSize = 8;
      triangleSIMDName = "avx";
   } else if (!preferScalar && (automatic || std::strcmp(requested, "sse2") == 0) && __builtin_cpu_supports("sse2")) {
      trianglePacketFunction = intersectTriangleSSE2;
      trianglePacketWidth = 4;
      triangleLeafSize = 4;
      triangleSIMDName = "sse2";
   }
   if (automatic && primitiveCount >= 10000 && triangleLeafSize > 8)
      triangleLeafSize = 8;
   const char* forcedLeafSize = std::getenv("RAY_PACKET_SIZE");
   if (trianglePacketFunction != NULL && forcedLeafSize != NULL) {
      const long value = std::strtol(forcedLeafSize, NULL, 10);
      if (value >= 1 && (size_t)value <= trianglePacketWidth) triangleLeafSize = (size_t)value;
   }
   if (std::getenv("RAY_SIMD_REPORT") != NULL)
      std::fprintf(stderr, "triangle SIMD: %s (lanes=%zu leaf=%zu primitives=%zu workers=%d)\n",
                   triangleSIMDName, trianglePacketWidth, triangleLeafSize, primitiveCount, workers);
}
#else
static void selectTriangleSIMD(size_t primitiveCount) {
   (void)primitiveCount;
   trianglePacketFunction = NULL;
   trianglePacketWidth = 4;
   triangleLeafSize = 4;
   triangleSIMDName = "scalar";
   if (std::getenv("RAY_SIMD_REPORT") != NULL)
      std::fprintf(stderr, "triangle SIMD: scalar (non-x86 build)\n");
}
#endif

struct PreparedRay {
   double origin[3], inverseDirection[3];
   bool parallel[3];

   PreparedRay(const Ray& ray) {
      origin[0] = ray.point.x;
      origin[1] = ray.point.y;
      origin[2] = ray.point.z;
      const double direction[3] = {ray.vector.x, ray.vector.y, ray.vector.z};
      for (int axis = 0; axis < 3; ++axis) {
         parallel[axis] = std::abs(direction[axis]) < 1e-15;
         inverseDirection[axis] = parallel[axis] ? 0.0 : 1.0 / direction[axis];
      }
   }
};

static bool intersectsBounds(const BVHNode& node, const PreparedRay& ray,
                             double maximum, double* nearDistance = NULL) {
   double near = 0.0;
   double far = maximum;
   for (int axis = 0; axis < 3; ++axis) {
      if (ray.parallel[axis]) {
         if (ray.origin[axis] < node.boundsMin[axis] || ray.origin[axis] > node.boundsMax[axis]) return false;
         continue;
      }
      double first = (node.boundsMin[axis] - ray.origin[axis]) * ray.inverseDirection[axis];
      double second = (node.boundsMax[axis] - ray.origin[axis]) * ray.inverseDirection[axis];
      if (first > second) std::swap(first, second);
      if (first > near) near = first;
      if (second < far) far = second;
      if (near > far) return false;
   }
   if (nearDistance != NULL) *nearDistance = near;
   return far >= 0.0;
}

static void intersectsBVH4(const BVH4Node& node, const PreparedRay& ray,
                           double maximum, bool hit[4], double nearDistance[4]) {
#if defined(RAY_BVH4_SSE2)
   if (!ray.parallel[0] && !ray.parallel[1] && !ray.parallel[2]) {
      __m128d near0 = _mm_setzero_pd(), near1 = near0;
      __m128d far0 = _mm_set1_pd(maximum), far1 = far0;
      const __m128d origin[3] = {
         _mm_set1_pd(ray.origin[0]), _mm_set1_pd(ray.origin[1]),
         _mm_set1_pd(ray.origin[2])};
      for (int axis = 0; axis < 3; ++axis) {
         const __m128d lo0 = _mm_loadu_pd(&node.boundsMin[axis][0]);
         const __m128d lo1 = _mm_loadu_pd(&node.boundsMin[axis][2]);
         const __m128d hi0 = _mm_loadu_pd(&node.boundsMax[axis][0]);
         const __m128d hi1 = _mm_loadu_pd(&node.boundsMax[axis][2]);
         const __m128d inv = _mm_set1_pd(ray.inverseDirection[axis]);
         const __m128d a0 = _mm_mul_pd(_mm_sub_pd(lo0, origin[axis]), inv);
         const __m128d b0 = _mm_mul_pd(_mm_sub_pd(hi0, origin[axis]), inv);
         const __m128d a1 = _mm_mul_pd(_mm_sub_pd(lo1, origin[axis]), inv);
         const __m128d b1 = _mm_mul_pd(_mm_sub_pd(hi1, origin[axis]), inv);
         const __m128d first0 = _mm_min_pd(a0, b0), second0 = _mm_max_pd(a0, b0);
         const __m128d first1 = _mm_min_pd(a1, b1), second1 = _mm_max_pd(a1, b1);
         near0 = _mm_max_pd(near0, first0); near1 = _mm_max_pd(near1, first1);
         far0 = _mm_min_pd(far0, second0); far1 = _mm_min_pd(far1, second1);
      }
      const __m128d zero = _mm_setzero_pd();
      const int m0 = _mm_movemask_pd(_mm_and_pd(_mm_cmple_pd(near0, far0),
                                                 _mm_cmpge_pd(far0, zero)));
      const int m1 = _mm_movemask_pd(_mm_and_pd(_mm_cmple_pd(near1, far1),
                                                 _mm_cmpge_pd(far1, zero)));
      _mm_storeu_pd(nearDistance, near0); _mm_storeu_pd(nearDistance + 2, near1);
      hit[0] = (m0 & 1) != 0; hit[1] = (m0 & 2) != 0;
      hit[2] = (m1 & 1) != 0; hit[3] = (m1 & 2) != 0;
      return;
   }
#endif
   for (int child = 0; child < 4; ++child) {
      double near = 0.0, far = maximum;
      hit[child] = true;
      for (int axis = 0; axis < 3; ++axis) {
         if (ray.parallel[axis]) {
            if (ray.origin[axis] < node.boundsMin[axis][child] ||
                ray.origin[axis] > node.boundsMax[axis][child]) { hit[child] = false; break; }
            continue;
         }
         double first = (node.boundsMin[axis][child] - ray.origin[axis]) * ray.inverseDirection[axis];
         double second = (node.boundsMax[axis][child] - ray.origin[axis]) * ray.inverseDirection[axis];
         if (first > second) std::swap(first, second);
         if (first > near) near = first;
         if (second < far) far = second;
         if (near > far) break;
      }
      nearDistance[child] = near;
      hit[child] = hit[child] && far >= 0.0 && near <= far;
   }
}

void Autonoma::buildAcceleration() {
   accelerationDirty = false;
   boundedShapes.clear();
   unboundedShapes.clear();
   bvhNodes.clear();
   bvh4Nodes.clear();
   trianglePackets.clear();
   useBVH4 = false;
   bvh4MaxDepth = 0;

   std::unordered_set<Texture*> textures;
   textures.insert(skybox);
   for (ShapeNode* node = listStart; node != NULL; node = node->next) {
      textures.insert(node->data->texture);
      textures.insert(node->data->normalMap);
   }
   for (Texture* texture : textures)
      if (texture != NULL) texture->prepareRendering();

   for (ShapeNode* node = listStart; node != NULL; node = node->next) {
      BVHPrimitive primitive;
      primitive.shape = node->data;
      if (node->data->getBounds(primitive.boundsMin, primitive.boundsMax)) {
         for (int axis = 0; axis < 3; ++axis) {
            primitive.centroid[axis] =
               0.5 * (primitive.boundsMin[axis] + primitive.boundsMax[axis]);
         }
         boundedShapes.push_back(primitive);
      } else {
         unboundedShapes.push_back(node->data);
      }
   }

   selectTriangleSIMD(boundedShapes.size());

   if (!boundedShapes.empty()) {
      bvhNodes.reserve(boundedShapes.size() * 2);
      if (trianglePacketFunction != NULL)
         trianglePackets.reserve((boundedShapes.size() + triangleLeafSize - 1) /
                                 triangleLeafSize);
      buildBVHNode(0, boundedShapes.size());
      const char* wide = std::getenv("RAY_BVH4");
      if ((wide == NULL && boundedShapes.size() >= 256) ||
          (wide != NULL && std::strcmp(wide, "1") == 0))
         buildBVH4();
   }
}

int Autonoma::buildBVHNode(size_t start, size_t end) {
   BVHNode node;
   for (int axis = 0; axis < 3; ++axis) {
      node.boundsMin[axis] = std::numeric_limits<double>::infinity();
      node.boundsMax[axis] = -std::numeric_limits<double>::infinity();
      for (size_t index = start; index < end; ++index) {
         node.boundsMin[axis] = std::min(node.boundsMin[axis], boundedShapes[index].boundsMin[axis]);
         node.boundsMax[axis] = std::max(node.boundsMax[axis], boundedShapes[index].boundsMax[axis]);
      }
   }
   node.left = -1;
   node.right = -1;
   node.start = start;
   node.count = end - start;

   const int nodeIndex = (int)bvhNodes.size();
   bvhNodes.push_back(node);
   size_t leafLimit = 4;
   bool allTriangles = trianglePacketFunction != NULL;
   if (trianglePacketFunction != NULL) {
      for (size_t index = start; index < end; ++index) allTriangles &= boundedShapes[index].shape->triangle;
      if (allTriangles) leafLimit = triangleLeafSize;
   }
   if (node.count <= leafLimit) {
      if (allTriangles) {
         TrianglePacket packet = {};
         packet.count = node.count;
         packet.opaque = true;
         for (size_t lane = 0; lane < node.count; ++lane) {
            Shape* shape = boundedShapes[start + lane].shape;
            const Triangle* triangle = static_cast<const Triangle*>(shape);
            packet.vertexX[lane] = triangle->vertex.x;
            packet.vertexY[lane] = triangle->vertex.y;
            packet.vertexZ[lane] = triangle->vertex.z;
            packet.edge1X[lane] = triangle->edge1.x;
            packet.edge1Y[lane] = triangle->edge1.y;
            packet.edge1Z[lane] = triangle->edge1.z;
            packet.edge2X[lane] = triangle->edge2.x;
            packet.edge2Y[lane] = triangle->edge2.y;
            packet.edge2Z[lane] = triangle->edge2.z;
            packet.shapes[lane] = shape;
            packet.opaque &= shape->texture->opacity > 1-1E-6;
         }
         const int packetIndex = (int)trianglePackets.size();
         trianglePackets.push_back(packet);
         bvhNodes[nodeIndex].left = -packetIndex - 2;
      }
      return nodeIndex;
   }

   double centroidMin[3] = {inf, inf, inf};
   double centroidMax[3] = {-inf, -inf, -inf};
   for (int axis = 0; axis < 3; ++axis) {
      for (size_t index = start; index < end; ++index) {
         centroidMin[axis] = std::min(centroidMin[axis], boundedShapes[index].centroid[axis]);
         centroidMax[axis] = std::max(centroidMax[axis], boundedShapes[index].centroid[axis]);
      }
   }

   const int binCount = 12;
   int bestAxis = -1, bestBin = -1;
   double bestCost = inf;
   for (int axis = 0; axis < 3; ++axis) {
      const double extent = centroidMax[axis] - centroidMin[axis];
      if (extent <= 1e-15) continue;
      struct Bin {
         size_t count;
         double minimum[3], maximum[3];
      } bins[binCount];
      for (int bin = 0; bin < binCount; ++bin) {
         bins[bin].count = 0;
         for (int component = 0; component < 3; ++component) {
            bins[bin].minimum[component] = inf;
            bins[bin].maximum[component] = -inf;
         }
      }
      const double scale = binCount / extent;
      for (size_t index = start; index < end; ++index) {
         int bin = (int)((boundedShapes[index].centroid[axis] - centroidMin[axis]) * scale);
         if (bin >= binCount) bin = binCount - 1;
         ++bins[bin].count;
         for (int component = 0; component < 3; ++component) {
            bins[bin].minimum[component] = std::min(bins[bin].minimum[component], boundedShapes[index].boundsMin[component]);
            bins[bin].maximum[component] = std::max(bins[bin].maximum[component], boundedShapes[index].boundsMax[component]);
         }
      }

      size_t leftCount[binCount - 1], rightCount[binCount - 1];
      double leftArea[binCount - 1], rightArea[binCount - 1];
      double minimum[3] = {inf, inf, inf};
      double maximum[3] = {-inf, -inf, -inf};
      size_t count = 0;
      for (int bin = 0; bin < binCount - 1; ++bin) {
         count += bins[bin].count;
         for (int component = 0; component < 3; ++component) {
            minimum[component] = std::min(minimum[component], bins[bin].minimum[component]);
            maximum[component] = std::max(maximum[component], bins[bin].maximum[component]);
         }
         const double x = maximum[0] - minimum[0];
         const double y = maximum[1] - minimum[1];
         const double z = maximum[2] - minimum[2];
         leftCount[bin] = count;
         leftArea[bin] = 2.0 * (x*y + x*z + y*z);
      }
      minimum[0] = minimum[1] = minimum[2] = inf;
      maximum[0] = maximum[1] = maximum[2] = -inf;
      count = 0;
      for (int bin = binCount - 1; bin > 0; --bin) {
         count += bins[bin].count;
         for (int component = 0; component < 3; ++component) {
            minimum[component] = std::min(minimum[component], bins[bin].minimum[component]);
            maximum[component] = std::max(maximum[component], bins[bin].maximum[component]);
         }
         const double x = maximum[0] - minimum[0];
         const double y = maximum[1] - minimum[1];
         const double z = maximum[2] - minimum[2];
         rightCount[bin - 1] = count;
         rightArea[bin - 1] = 2.0 * (x*y + x*z + y*z);
      }
      for (int bin = 0; bin < binCount - 1; ++bin) {
         if (leftCount[bin] == 0 || rightCount[bin] == 0) continue;
         const double cost = leftCount[bin] * leftArea[bin] + rightCount[bin] * rightArea[bin];
         if (cost < bestCost) {
            bestCost = cost;
            bestAxis = axis;
            bestBin = bin;
         }
      }
   }

   size_t middle;
   if (bestAxis >= 0) {
      const double split = centroidMin[bestAxis] +
         (centroidMax[bestAxis] - centroidMin[bestAxis]) * (bestBin + 1) / binCount;
      std::vector<BVHPrimitive>::iterator middleIterator =
         std::partition(boundedShapes.begin() + start, boundedShapes.begin() + end,
                        [bestAxis, split](const BVHPrimitive& primitive) {
                           return primitive.centroid[bestAxis] < split;
                        });
      middle = middleIterator - boundedShapes.begin();
   } else {
      middle = start;
   }
   if (middle == start || middle == end) {
      int splitAxis = 0;
      if (centroidMax[1] - centroidMin[1] > centroidMax[splitAxis] - centroidMin[splitAxis]) splitAxis = 1;
      if (centroidMax[2] - centroidMin[2] > centroidMax[splitAxis] - centroidMin[splitAxis]) splitAxis = 2;
      middle = start + (end - start) / 2;
      std::nth_element(boundedShapes.begin() + start, boundedShapes.begin() + middle,
                       boundedShapes.begin() + end,
                       [splitAxis](const BVHPrimitive& first, const BVHPrimitive& second) {
                          return first.centroid[splitAxis] < second.centroid[splitAxis];
                       });
   }
   const int left = buildBVHNode(start, middle);
   const int right = buildBVHNode(middle, end);
   bvhNodes[nodeIndex].left = left;
   bvhNodes[nodeIndex].right = right;
   bvhNodes[nodeIndex].count = 0;
   return nodeIndex;
}

int Autonoma::buildBVH4Node(int binaryNode, size_t depth) {
   if (depth > bvh4MaxDepth) bvh4MaxDepth = depth;
   std::vector<int> candidates(1, binaryNode);
   while (candidates.size() < 4) {
      size_t expand = candidates.size();
      double largestArea = -1.0;
      for (size_t i = 0; i < candidates.size(); ++i) {
         const BVHNode& candidate = bvhNodes[candidates[i]];
         if (candidate.count == 0) {
            const double x = candidate.boundsMax[0] - candidate.boundsMin[0];
            const double y = candidate.boundsMax[1] - candidate.boundsMin[1];
            const double z = candidate.boundsMax[2] - candidate.boundsMin[2];
            const double area = 2.0 * (x*y + x*z + y*z);
            if (area > largestArea) { largestArea = area; expand = i; }
         }
      }
      if (expand == candidates.size()) break;
      const BVHNode& node = bvhNodes[candidates[expand]];
      candidates[expand] = node.left;
      candidates.push_back(node.right);
   }
   BVH4Node wide = {};
   wide.count = (unsigned char)candidates.size();
   const int wideIndex = (int)bvh4Nodes.size();
   bvh4Nodes.push_back(wide);
   for (size_t slot = 0; slot < candidates.size(); ++slot) {
      const int binaryChild = candidates[slot];
      const BVHNode& child = bvhNodes[binaryChild];
      for (int axis = 0; axis < 3; ++axis) {
         bvh4Nodes[wideIndex].boundsMin[axis][slot] = child.boundsMin[axis];
         bvh4Nodes[wideIndex].boundsMax[axis][slot] = child.boundsMax[axis];
      }
      if (child.count == 0)
         bvh4Nodes[wideIndex].child[slot] = buildBVH4Node(binaryChild, depth + 1);
      else
         bvh4Nodes[wideIndex].child[slot] = -(binaryChild + 1);
   }
   return wideIndex;
}

void Autonoma::buildBVH4() {
   useBVH4 = false;
   bvh4Nodes.clear();
   if (bvhNodes.empty() || bvhNodes[0].count != 0) return;
   bvh4Nodes.reserve((bvhNodes.size() + 2) / 3);
   buildBVH4Node(0, 0);
   useBVH4 = !bvh4Nodes.empty() && 1 + 3 * bvh4MaxDepth <= 512;
   if (std::getenv("RAY_BVH4_REPORT") != NULL)
      std::fprintf(stderr, "BVH4: %zu nodes, binary nodes %zu, max depth %zu, %s, bytes %zu vs %zu\n",
                   bvh4Nodes.size(), bvhNodes.size(),
                   bvh4MaxDepth, useBVH4 ? "enabled" : "stack-limit fallback",
                   bvh4Nodes.size() * sizeof(BVH4Node),
                   bvhNodes.size() * sizeof(BVHNode));
}

static inline void intersectClosestLeaf(const Autonoma& scene, const BVHNode& leaf,
                                       const Ray& ray, double& closest,
                                       Shape*& closestShape) {
   if (leaf.left <= -2) {
      const TrianglePacket& packet = scene.trianglePackets[(size_t)(-leaf.left - 2)];
      double packetClosest;
      const int lane = trianglePacketFunction(packet, ray, closest, packetClosest);
      if (lane >= 0) { closest = packetClosest; closestShape = packet.shapes[lane]; }
   } else {
      const size_t end = leaf.start + leaf.count;
      for (size_t index = leaf.start; index < end; ++index) {
         const double time = scene.boundedShapes[index].shape->getIntersection(ray);
         if (time < closest) { closest = time; closestShape = scene.boundedShapes[index].shape; }
      }
   }
}

static inline bool intersectShadowLeaf(const Autonoma& scene, const BVHNode& leaf,
                                      const Ray& ray, double* fill) {
   if (leaf.left <= -2) {
      const TrianglePacket& packet = scene.trianglePackets[(size_t)(-leaf.left - 2)];
      double packetClosest;
      if (packet.opaque)
         return trianglePacketFunction(packet, ray, 1.0, packetClosest) >= 0;
      for (size_t lane = 0; lane < packet.count; ++lane)
         if (packet.shapes[lane]->getLightIntersection(ray, fill)) return true;
   } else {
      const size_t end = leaf.start + leaf.count;
      for (size_t index = leaf.start; index < end; ++index)
         if (scene.boundedShapes[index].shape->getLightIntersection(ray, fill)) return true;
   }
   return false;
}

Shape* Autonoma::closestIntersection(const Ray& ray, double& closest) const {
   Shape* closestShape = NULL;
   for (size_t index = 0; index < unboundedShapes.size(); ++index) {
      const double time = unboundedShapes[index]->getIntersection(ray);
      if (time < closest) {
         closest = time;
         closestShape = unboundedShapes[index];
      }
   }

   if (bvhNodes.empty()) return closestShape;
   const PreparedRay preparedRay(ray);
   if (useBVH4) {
      double rootNear;
      if (!intersectsBounds(bvhNodes[0], preparedRay, closest, &rootNear)) return closestShape;
      int stack[512]; double stackNear[512]; int size = 1; stack[0] = 0; stackNear[0] = rootNear;
      while (size) {
         --size;
         const int ref = stack[size];
         if (stackNear[size] > closest) continue;
         if (ref >= 0) {
            const BVH4Node& wide = bvh4Nodes[ref];
            bool hit[4]; double nearDistance[4];
            intersectsBVH4(wide, preparedRay, closest, hit, nearDistance);
            int order[4], count = 0;
            for (int i = 0; i < wide.count; ++i) if (hit[i]) {
               int j = count++;
               while (j > 0 && nearDistance[order[j - 1]] > nearDistance[i]) { order[j] = order[j - 1]; --j; }
               order[j] = i;
            }
            for (int i = count - 1; i >= 0; --i) {
               stack[size] = wide.child[order[i]];
               stackNear[size++] = nearDistance[order[i]];
            }
            continue;
         }
         intersectClosestLeaf(*this, bvhNodes[-ref - 1], ray, closest, closestShape);
      }
      return closestShape;
   }
   int nodeStack[128];
   double nearStack[128];
   int stackSize = 0;
   double rootNear;
   if (!intersectsBounds(bvhNodes[0], preparedRay, closest, &rootNear)) return closestShape;
   nodeStack[stackSize] = 0;
   nearStack[stackSize++] = rootNear;
   while (stackSize != 0) {
      --stackSize;
      if (nearStack[stackSize] > closest) continue;
      const BVHNode& node = bvhNodes[nodeStack[stackSize]];
      if (node.count != 0) {
         intersectClosestLeaf(*this, node, ray, closest, closestShape);
         continue;
      }

      double leftNear, rightNear;
      const bool hitLeft = intersectsBounds(bvhNodes[node.left], preparedRay, closest, &leftNear);
      const bool hitRight = intersectsBounds(bvhNodes[node.right], preparedRay, closest, &rightNear);
      if (hitLeft && hitRight) {
         if (leftNear < rightNear) {
            nodeStack[stackSize] = node.right;
            nearStack[stackSize++] = rightNear;
            nodeStack[stackSize] = node.left;
            nearStack[stackSize++] = leftNear;
         } else {
            nodeStack[stackSize] = node.left;
            nearStack[stackSize++] = leftNear;
            nodeStack[stackSize] = node.right;
            nearStack[stackSize++] = rightNear;
         }
      } else if (hitLeft) {
         nodeStack[stackSize] = node.left;
         nearStack[stackSize++] = leftNear;
      } else if (hitRight) {
         nodeStack[stackSize] = node.right;
         nearStack[stackSize++] = rightNear;
      }
   }
   return closestShape;
}

bool Autonoma::lightIntersection(const Ray& ray, double* fill) const {
   for (size_t index = 0; index < unboundedShapes.size(); ++index) {
      if (unboundedShapes[index]->getLightIntersection(ray, fill)) return true;
   }

   if (bvhNodes.empty()) return false;
   const PreparedRay preparedRay(ray);
   if (useBVH4) {
      if (!intersectsBounds(bvhNodes[0], preparedRay, 1.0)) return false;
      int stack[512]; int size = 1; stack[0] = 0;
      while (size) {
         const int ref = stack[--size];
         if (ref >= 0) {
            const BVH4Node& wide = bvh4Nodes[ref];
            bool hit[4]; double nearDistance[4];
            intersectsBVH4(wide, preparedRay, 1.0, hit, nearDistance);
            for (int i = 0; i < wide.count; ++i) if (hit[i]) stack[size++] = wide.child[i];
            continue;
         }
         if (intersectShadowLeaf(*this, bvhNodes[-ref - 1], ray, fill)) return true;
      }
      return false;
   }
   int nodeStack[128];
   int stackSize = 0;
   if (!intersectsBounds(bvhNodes[0], preparedRay, 1.0)) return false;
   nodeStack[stackSize++] = 0;
   while (stackSize != 0) {
      --stackSize;
      const BVHNode& node = bvhNodes[nodeStack[stackSize]];
      if (node.count != 0) {
         if (intersectShadowLeaf(*this, node, ray, fill)) return true;
      } else {
         double leftNear, rightNear;
         const bool hitLeft = intersectsBounds(bvhNodes[node.left], preparedRay, 1.0, &leftNear);
         const bool hitRight = intersectsBounds(bvhNodes[node.right], preparedRay, 1.0, &rightNear);
         if (hitLeft && hitRight) {
            if (leftNear < rightNear) {
               nodeStack[stackSize] = node.right;
               ++stackSize;
               nodeStack[stackSize] = node.left;
               ++stackSize;
            } else {
               nodeStack[stackSize] = node.left;
               ++stackSize;
               nodeStack[stackSize] = node.right;
               ++stackSize;
            }
         } else if (hitLeft) {
            nodeStack[stackSize++] = node.left;
         } else if (hitRight) {
            nodeStack[stackSize++] = node.right;
         }
      }
   }
   return false;
}

void getLight(double* tColor, Autonoma* aut, Vector point, Vector norm, unsigned char flip,
              unsigned int depth){
   tColor[0] = tColor[1] = tColor[2] = 0.;
   LightNode *t = aut->lightStart;
   while(t!=NULL){
      double lightColor[3];     
      lightColor[0] = t->data->color[0]/255.;
      lightColor[1] = t->data->color[1]/255.;
      lightColor[2] = t->data->color[2]/255.;
      Vector ra = t->data->center-point;
#ifdef RAY_REFERENCE_SHADING
      const bool traceShadow = true;
#else
      const bool traceShadow = depth == 0;
#endif
      const bool hit = traceShadow && aut->lightIntersection(Ray(point+ra*.01, ra), lightColor);
      double perc = norm.dot(ra) / ra.mag();
      if(!hit){
      if(flip && perc<0) perc=-perc;
        if(perc>0){
      
         tColor[0]+= perc*(lightColor[0]);
         tColor[1]+= perc*(lightColor[0]);
         tColor[2]+= perc*(lightColor[0]);
         if(tColor[0]>1.) tColor[0] = 1.;
         if(tColor[1]>1.) tColor[1] = 1.;
         if(tColor[2]>1.) tColor[2] = 1.;
        }
      }
      t =t->next;
   }
}
