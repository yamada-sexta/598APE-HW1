#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__
#include "plane.h"

class Triangle : public Plane{
public:
   double thirdX;
   Vector bmin, bmax;
   Triangle(Vector c, Vector b, Vector a, Texture* t);
   double getIntersection(Ray ray);
   bool getLightIntersection(Ray ray, double* fill);
   bool getBounds(Vector& lo, Vector& hi);
};

#endif
