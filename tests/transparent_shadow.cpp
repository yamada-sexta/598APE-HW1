#include "src/sphere.h"
#include <cmath>
#include <cstdio>

int main() {
   ColorTexture material(128, 64, 192, .5, 0., .3);
   Sphere sphere(Vector(0, 0, 3), &material, 0, 0, 0, 1.);
   const Ray ray(Vector(0, 0, 0), Vector(0, 0, 10));
   double light[3] = {1., 1., 1.};
   if (sphere.getLightIntersection(ray, light)) return 1;
   const double expected[3] = {128. / 255., 64. / 255., 192. / 255.};
   for (int c = 0; c < 3; ++c)
      if (std::abs(light[c] - expected[c]) > 1e-12) return 2;
   material.opacity = 1.;
   if (!sphere.getLightIntersection(ray, light)) return 3;
   std::puts("transparent uniform sphere tint and opaque shortcut passed");
}
