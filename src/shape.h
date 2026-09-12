#ifndef __SHAPE_H__
#define __SHAPE_H__
#include "light.h"

class Shape{
  public:
   Shape(const Vector &c, Texture* t, double ya, double pi, double ro);
   double yaw, pitch, roll, xsin, xcos, ysin, ycos, zsin, zcos;
   Vector center;
   Texture* texture;
   double textureX, textureY, mapX, mapY, mapOffX, mapOffY;
   Texture* normalMap;
   virtual double getIntersection(const Ray& ray) = 0;
   virtual bool getLightIntersection(const Ray& ray, double* fill) = 0;
   virtual void move() = 0;
   virtual unsigned char reversible() = 0;
   virtual void getColor(unsigned char* toFill, double* am, double* op, double* ref, Autonoma* r, const Ray& ray, unsigned int depth) = 0;
   virtual Vector getNormal(Vector point, unsigned int depth) = 0;
   virtual void setAngles(double yaw, double pitch, double roll) = 0;
   virtual void setYaw(double d) = 0;
   virtual void setPitch(double d) = 0;
   virtual void setRoll(double d) = 0;
   virtual bool getBounds(double boundsMin[3], double boundsMax[3]) const;
};

void calcColor(unsigned char* toFill, Autonoma*, const Ray& ray, unsigned int depth);

#endif
