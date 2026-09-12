#include "box.h"

Box::Box(const Vector &c, Texture* t, double ya, double pi, double ro, double tx, double ty):Plane(c, t, ya, pi, ro, tx, ty){}
Box::Box(const Vector &c, Texture* t, double ya, double pi, double ro, double tx):Plane(c, t, ya, pi, ro, tx,tx){}

double Box::getIntersection(const Ray& ray){
   double time = Plane::getIntersection(ray);
   if(time==inf) 
      return time;
   Vector dist = solveScalers(right, up, vect, ray.point+ray.vector*time-center);
   return ( ((dist.x>=0)?dist.x:-dist.x)>textureX/2 || ((dist.y>=0)?dist.y:-dist.y)>textureY/2 )?inf:time;
}

bool Box::getLightIntersection(const Ray& ray, double* fill){
   const double t = ray.vector.dot(vect);
   const double norm = vect.dot(ray.point)+d;
   const double r = -norm/t;
   if(r<=0. || r>=1.) return false;
   Vector dist = solveScalers(right, up, vect, ray.point+ray.vector*r-center);
   if( ((dist.x>=0)?dist.x:-dist.x)>textureX/2 || ((dist.y>=0)?dist.y:-dist.y)>textureY/2 ) return false;

   if(texture->opacity>1-1E-6) return true;   
   unsigned char temp[4];
   double amb, op, ref;
   texture->getColor(temp, &amb, &op, &ref,fix(dist.x/textureX-.5), fix(dist.y/textureY-.5));
   if(op>1-1E-6) return true;
   fill[0]*=temp[0]/255.;
   fill[1]*=temp[1]/255.;
   fill[2]*=temp[2]/255.;
   return false;
}

bool Box::getBounds(double boundsMin[3], double boundsMax[3]) const {
   const double extent[3] = {
      std::abs(right.x) * textureX * 0.5 + std::abs(up.x) * textureY * 0.5,
      std::abs(right.y) * textureX * 0.5 + std::abs(up.y) * textureY * 0.5,
      std::abs(right.z) * textureX * 0.5 + std::abs(up.z) * textureY * 0.5
   };
   const double position[3] = {center.x, center.y, center.z};
   for (int axis = 0; axis < 3; ++axis) {
      const double padding = 1e-9 * (1.0 + extent[axis]);
      boundsMin[axis] = position[axis] - extent[axis] - padding;
      boundsMax[axis] = position[axis] + extent[axis] + padding;
   }
   return true;
}
