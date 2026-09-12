#ifndef __SPHERE_H__
#define __SPHERE_H__
#include "shape.h"

class Sphere : public Shape{
public:
  double radius;
  Sphere(const Vector &c, Texture* t, double ya, double pi, double ro, double radius);
  double getIntersection(const Ray& ray);
  void move();
  bool getLightIntersection(const Ray& ray, double* fill);
  void getColor(unsigned char* toFill, double* am, double* op, double* ref, Autonoma* r, Ray ray, unsigned int depth);
  Vector getNormal(Vector point, unsigned int depth);
  bool getBounds(double boundsMin[3], double boundsMax[3]) const;
  unsigned char reversible();
  void setAngles(double a, double b, double c);
  void setYaw(double a);
  void setPitch(double b);
  void setRoll(double c);
};
#endif
