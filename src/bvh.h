#ifndef __BVH_H__
#define __BVH_H__
#include "vector.h"
#include <vector>

class Shape;

class BVH{
public:
   void build(const std::vector<Shape*>& shapes);
   double closest(const Ray& ray, Shape*& out) const;
   bool shadow(const Ray& ray, double* fill) const;
   bool empty() const { return prims.empty(); }
private:
   struct Node{
      double lo[3], hi[3];
      int start, count;
      int right;
   };
   std::vector<Node> nodes;
   std::vector<Shape*> prims;
   int buildRange(int start, int count, int depth);
};

#endif
