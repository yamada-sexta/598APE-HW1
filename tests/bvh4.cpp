#include "src/sphere.h"
#include "src/triangle.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

int main() {
   setenv("RAY_BVH4", "1", 1);
   std::mt19937 random(598);
   std::uniform_real_distribution<double> coordinate(-12.0, 12.0);
   auto point = [&]() { return Vector(coordinate(random), coordinate(random), coordinate(random)); };
   ColorTexture opaque(210, 150, 80, 1.0, 0.0, 0.3);
   ColorTexture transparent(128, 180, 230, 0.5, 0.0, 0.3);
   Autonoma scene(Camera(Vector(0, 0, 0)));
   for (int i = 0; i < 128; ++i) {
      const Vector p = point();
      scene.addShape(new Sphere(p, i % 3 ? &opaque : &transparent, 0, 0, 0, 0.25 + (i % 7) * 0.15));
      scene.addShape(new Triangle(p, p + Vector(0.5, 0.1, 0.0), p + Vector(0.1, 0.0, 0.7), &opaque));
   }
   scene.buildAcceleration();
   if (!scene.useBVH4) return 2;
   for (int i = 0; i < 100000; ++i) {
      Vector origin = i % 2 ? point() : Vector(0, 0, 0);
      Vector direction = point();
      if (i % 4 == 0) {
         direction = Vector(0, 0, 0);
         const double sign = (i % 8) ? 1.0 : -1.0;
         if (i % 3 == 0) direction.x = sign;
         else if (i % 3 == 1) direction.y = sign;
         else direction.z = sign;
      }
      const Ray ray(origin, direction);
      double a = inf, b = inf;
      double colorA[3] = {1, 1, 1}, colorB[3] = {1, 1, 1};
      scene.useBVH4 = false;
      Shape* hitA = scene.closestIntersection(ray, a);
      const bool shadowA = scene.lightIntersection(ray, colorA);
      scene.useBVH4 = true;
      Shape* hitB = scene.closestIntersection(ray, b);
      const bool shadowB = scene.lightIntersection(ray, colorB);
      if (hitA != hitB || (hitA && std::abs(a - b) > 1e-10 * std::max(1.0, std::abs(a))) || shadowA != shadowB) {
         std::fprintf(stderr, "intersection mismatch at ray %d\n", i);
         return 1;
      }
      if (!shadowA) for (int channel = 0; channel < 3; ++channel) {
         if (std::abs(colorA[channel] - colorB[channel]) > 1e-12) return 3;
      }
   }
   std::puts("100000 BVH2/BVH4 nearest-hit and shadow comparisons passed");
}
