#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__
#include "plane.h"

class Triangle : public Plane{
public:
   double thirdX;
   double boundsMin[3], boundsMax[3];
   Vector vertex, edge1, edge2;
   Triangle(Vector c, Vector b, Vector a, Texture* t);
   double getIntersection(Ray ray);
   bool getLightIntersection(Ray ray, double* fill);
   bool getBounds(double outMin[3], double outMax[3]) const;
private:
   bool intersect(const Ray& ray, double& time, double& u, double& v) const;
};

#endif
