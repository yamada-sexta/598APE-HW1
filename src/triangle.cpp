#include "triangle.h"
#include <algorithm>

Triangle::Triangle(Vector c, Vector b, Vector a, Texture* t):Plane(Vector(0,0,0), t, 0., 0., 0., 0., 0.), vertex(c), edge1(b-c), edge2(a-c){
   triangle = true;
   const double epsilon = 1e-9;
   boundsMin[0] = std::min(c.x, std::min(b.x, a.x)) - epsilon;
   boundsMin[1] = std::min(c.y, std::min(b.y, a.y)) - epsilon;
   boundsMin[2] = std::min(c.z, std::min(b.z, a.z)) - epsilon;
   boundsMax[0] = std::max(c.x, std::max(b.x, a.x)) + epsilon;
   boundsMax[1] = std::max(c.y, std::max(b.y, a.y)) + epsilon;
   boundsMax[2] = std::max(c.z, std::max(b.z, a.z)) + epsilon;
   center = c;
   Vector righta = (b-c);
   textureX = righta.mag();
   right = righta/textureX;
   vect = right.cross(b-a).normalize();

   xsin = -right.z;
   if(xsin<-1.)xsin = -1;
   else if (xsin>1.)xsin=1.; 
   yaw = asin(xsin);
   xcos = sqrt(1.-xsin*xsin);

   zcos = right.x/xcos;
   zsin = -right.y/xcos;
   if(zsin<-1.)zsin = -1;
   else if (zsin>1.)zsin=1.;
   if(zcos<-1.)zcos = -1;
   else if (zcos>1.)zcos=1.;
   roll = asin(zsin);

   ycos = vect.z/xcos;
   if(ycos<-1.)ycos = -1;
   else if (ycos>1.)ycos=1.;
   pitch = acos(ycos);
   ysin = sqrt(1-ycos*ycos);

   up.x = -xsin*ysin*zcos+ycos*zsin;
   up.y = ycos*zcos+xsin*ysin*zsin;
   up.z = -xcos*ysin;
   Vector temp = vect.cross(right);
   Vector np = solveScalers(right, up, vect, a-c);
   textureY = np.y;
   thirdX = np.x;
   
   d = -vect.dot(center);
}

bool Triangle::getBounds(double outMin[3], double outMax[3]) const {
   for (int axis = 0; axis < 3; ++axis) {
      outMin[axis] = boundsMin[axis];
      outMax[axis] = boundsMax[axis];
   }
   return true;
}

double Triangle::getIntersection(const Ray& ray){
   double time, u, v;
   return intersect(ray, time, u, v) ? time : inf;
}

bool Triangle::getLightIntersection(const Ray& ray, double* fill){
   double time, u, v;
   if (!intersect(ray, time, u, v) || time >= 1.0) return false;
   if(texture->opacity>1-1E-6) return true;   
   unsigned char temp[4];
   double amb, op, ref;
   const double textureU = u + v * thirdX / textureX;
   texture->getColor(temp, &amb, &op, &ref, fix(textureU-.5), fix(v-.5));
   if(op>1-1E-6) return true;
   fill[0]*=temp[0]/255.;
   fill[1]*=temp[1]/255.;
   fill[2]*=temp[2]/255.;
   return false;
}

bool Triangle::intersect(const Ray& ray, double& time, double& u, double& v) const {
   const Vector p = ray.vector.cross(edge2);
   const double determinant = edge1.dot(p);
   if (std::abs(determinant) < 1e-12) return false;
   const double inverseDeterminant = 1.0 / determinant;
   const Vector offset = ray.point - vertex;
   u = offset.dot(p) * inverseDeterminant;
   if (u < 0.0 || u > 1.0) return false;
   const Vector q = offset.cross(edge1);
   v = ray.vector.dot(q) * inverseDeterminant;
   if (v < 0.0 || u + v > 1.0) return false;
   time = edge2.dot(q) * inverseDeterminant;
   return time > 0.0;
}
