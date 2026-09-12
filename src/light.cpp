
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

Autonoma::Autonoma(const Camera& c): camera(c){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
   depth = 3;
   skybox = BLACK;
}

Autonoma::Autonoma(const Camera& c, Texture* tex): camera(c){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
   depth = 3;
   skybox = tex;
}

void Autonoma::addShape(Shape* r){
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
   if(s==listStart){
      if(s==listEnd){
         listStart = listStart = NULL;
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
         lightStart = lightStart = NULL;
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

void Autonoma::buildAcceleration() {
   boundedShapes.clear();
   unboundedShapes.clear();
   bvhNodes.clear();
   trianglePackets.clear();

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
         if (node.left <= -2) {
            const TrianglePacket& packet = trianglePackets[(size_t)(-node.left - 2)];
            double packetClosest;
            const int lane = trianglePacketFunction(packet, ray, closest, packetClosest);
            if (lane >= 0) {
               closest = packetClosest;
               closestShape = packet.shapes[lane];
            }
            continue;
         }
         const size_t end = node.start + node.count;
         for (size_t index = node.start; index < end; ++index) {
            const double time = boundedShapes[index].shape->getIntersection(ray);
            if (time < closest) {
               closest = time;
               closestShape = boundedShapes[index].shape;
            }
         }
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
   int nodeStack[128];
   int stackSize = 0;
   if (!intersectsBounds(bvhNodes[0], preparedRay, 1.0)) return false;
   nodeStack[stackSize++] = 0;
   while (stackSize != 0) {
      --stackSize;
      const BVHNode& node = bvhNodes[nodeStack[stackSize]];
      if (node.count != 0) {
         if (node.left <= -2) {
            const TrianglePacket& packet = trianglePackets[(size_t)(-node.left - 2)];
            double packetClosest;
            if (packet.opaque) {
               if (trianglePacketFunction(packet, ray, 1.0, packetClosest) >= 0) return true;
               continue;
            }
            for (size_t lane = 0; lane < packet.count; ++lane) {
               if (packet.shapes[lane]->getLightIntersection(ray, fill)) return true;
            }
            continue;
         }
         const size_t end = node.start + node.count;
         for (size_t index = node.start; index < end; ++index) {
            if (boundedShapes[index].shape->getLightIntersection(ray, fill)) return true;
         }
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
      const bool hit = depth == 0 && aut->lightIntersection(Ray(point+ra*.01, ra), lightColor);
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
