#include<string.h>
#include<stdio.h>
#include<limits>
#include<math.h>
#include<stdlib.h>
//#include <printf.h>
#include <stddef.h>
#include "vector.h"

Vector::Vector(double a, double b, double c) : x(a), y(b), z(c) {
}
void Vector::operator -= (const Vector& rhs) {
   x-=rhs.x; y-=rhs.y; z-=rhs.z;
}
void Vector::operator += (const Vector& rhs) {
   x+=rhs.x; y+=rhs.y; z+=rhs.z;
}
void Vector::operator *= (const double rhs) {
   x*=rhs; y*=rhs; z*=rhs;
}
void Vector::operator *= (const float rhs) {
   x*=rhs; y*=rhs; z*=rhs;
}
void Vector::operator *= (const int rhs) {
   x*=rhs; y*=rhs; z*=rhs;
}
void Vector::operator /= (const double rhs) {
   x/=rhs; y/=rhs; z/=rhs;
}
void Vector::operator /= (const float rhs) {
   x/=rhs; y/=rhs; z/=rhs;
}
void Vector::operator /= (const int rhs) {
   x/=rhs; y/=rhs; z/=rhs;
}


Vector Vector::operator - (const Vector& rhs) const {
   return Vector(x-rhs.x, y-rhs.y, z-rhs.z);
}
Vector Vector::operator + (const Vector& rhs) const {
   return Vector(x+rhs.x, y+rhs.y, z+rhs.z);
}
/*
Vector Vector::operator * (const Vector a) {
   return Vector(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);
}*/
Vector Vector::operator * (const double rhs) const {
   return Vector(x*rhs, y*rhs, z*rhs);
}
Vector Vector::operator * (const float rhs) const {
   return Vector(x*rhs, y*rhs, z*rhs);
}
Vector Vector::operator * (const int rhs) const {
   return Vector(x*rhs, y*rhs, z*rhs);
}
Vector Vector::operator / (const double rhs) const {
   return Vector(x/rhs, y/rhs, z/rhs);
}
Vector Vector::operator / (const float rhs) const {
   return Vector(x/rhs, y/rhs, z/rhs);
}
Vector Vector::operator / (const int rhs) const {
   return Vector(x/rhs, y/rhs, z/rhs);
}
Vector Vector::cross(const Vector& a) const {
   return Vector(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);
}
double Vector::mag2() const {
   return x*x+y*y+z*z; 
}
double Vector::mag() const {
   return sqrt(x*x+y*y+z*z); 
}
double Vector::dot(const Vector& a) const {
   return x*a.x+y*a.y+z*a.z;
}
Vector Vector::normalize() const {
   double m = mag();
   return Vector(x/m, y/m, z/m); 
}

  
Vector solveScalers(const Vector& v1, const Vector& v2, const Vector& v3, const Vector& value){
#ifdef USE_CUDA
   double denom = v1.z*v2.y*v3.x-v1.y*v2.z*v3.x-v1.z*v2.x*v3.y+v1.x*v2.z*v3.y+v1.y*v2.x*v3.z-v1.x*v2.y*v3.z;
   double a = value.z*v2.y*v3.x-value.y*v2.z*v3.x-value.z*v2.x*v3.y+value.x*v2.z*v3.y+value.y*v2.x*v3.z-value.x*v2.y*v3.z;
   double b = -value.z*v1.y*v3.x+value.y*v1.z*v3.x+value.z*v1.x*v3.y-value.x*v1.z*v3.y-value.y*v1.x*v3.z+value.x*v1.y*v3.z;
   double c = value.z*v1.y*v2.x-value.y*v1.z*v2.x-value.z*v1.x*v2.y+value.x*v1.z*v2.y+value.y*v1.x*v2.z-value.x*v1.y*v2.z;
   return Vector(a/denom, b/denom, c/denom);
#else
   return Vector(value.dot(v1), value.dot(v2), value.dot(v3));
#endif
}

Ray::Ray(const Vector& po, const Vector& ve): point(po), vector(ve){}
