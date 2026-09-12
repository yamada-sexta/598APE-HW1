#ifndef __LIGHT_H__
#define __LIGHT_H__
#include "vector.h"
#include "camera.h"
#include "Textures/texture.h"
#include "Textures/colortexture.h"
#include <cstddef>
#include <vector>

class Light{
  public:
   unsigned char* color;
   unsigned char* getColor(unsigned char a, unsigned char b, unsigned char c);
   Vector center;
   Light(const Vector & cente, unsigned char* colo);
};

struct LightNode{
   Light* data;
   LightNode* prev, *next;
};

class Shape;
struct ShapeNode{
   Shape* data;
   ShapeNode* prev, *next;
};

struct BVHPrimitive {
   Shape* shape;
   double boundsMin[3], boundsMax[3], centroid[3];
};

struct alignas(64) TrianglePacket {
   float vertexX[16], vertexY[16], vertexZ[16];
   float edge1X[16], edge1Y[16], edge1Z[16];
   float edge2X[16], edge2Y[16], edge2Z[16];
   Shape* shapes[16];
   size_t count;
   bool opaque;
};

struct BVHNode {
   double boundsMin[3], boundsMax[3];
   int left, right;
   size_t start, count;
};

class Autonoma{
public:
   Camera camera;
   Texture* skybox;
   unsigned int depth;
   ShapeNode *listStart, *listEnd;
   LightNode *lightStart, *lightEnd;
   std::vector<BVHPrimitive> boundedShapes;
   std::vector<Shape*> unboundedShapes;
   std::vector<BVHNode> bvhNodes;
   std::vector<TrianglePacket> trianglePackets;
   Autonoma(const Camera &c);
   Autonoma(const Camera &c, Texture* tex);
   void addShape(Shape* s);
   void removeShape(ShapeNode* s);
   void addLight(Light* s);
   void removeLight(LightNode* s);
   void buildAcceleration();
   Shape* closestIntersection(const Ray& ray, double& closest) const;
   bool lightIntersection(const Ray& ray, double* fill) const;
private:
   int buildBVHNode(size_t start, size_t end);
};

void getLight(double* toFill, Autonoma* aut, Vector point, Vector norm, unsigned char r,
              unsigned int depth);

#endif
