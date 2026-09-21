#define main rayProgramMain
#include "../main.cpp"
#undef main
#include <vector>
#include <unistd.h>

static void require(bool condition, const char* message) {
   if (!condition) {
      std::fprintf(stderr, "%s\n", message);
      std::exit(1);
   }
}

struct CountingTexture : ColorTexture {
   int preparations = 0;
   CountingTexture() : ColorTexture(180, 120, 90, 1., 0., 1.) {}
   void prepareRendering() override { ++preparations; }
};

int main() {
   W = 101; H = 97;
   std::vector<unsigned char> actual((size_t)W * H * 3), expected(actual.size());
   DATA = actual.data();
   CountingTexture material;
   Autonoma scene(Camera(Vector(0, 0, 0)));
   Sphere sphere(Vector(0, 0, 4), &material, 0, 0, 0, .5);
   scene.buildAcceleration();
   scene.addShape(&sphere);
   require(scene.accelerationDirty, "addShape must invalidate acceleration");
   refresh(&scene);
   require(!scene.accelerationDirty && scene.boundedShapes.size() == 1,
           "refresh must rebuild acceleration after addShape");
   const auto withSphere = actual;
   scene.removeShape(scene.listStart);
   require(scene.listStart == NULL && scene.listEnd == NULL,
           "removing the last shape must clear both list endpoints");
   refresh(&scene);
   require(scene.boundedShapes.empty() && actual != withSphere,
           "removing a shape must remove it from the next render");
   scene.addShape(&sphere);
   refresh(&scene);
   require(actual == withSphere, "re-adding a shape must restore the image");
   char mappingAnimation[] = "/tmp/ray-mapping-animation-XXXXXX";
   const int mappingFd = mkstemp(mappingAnimation);
   require(mappingFd >= 0, "could not create mapping animation fixture");
   FILE* mappingFile = fdopen(mappingFd, "w");
   require(mappingFile != NULL, "could not open mapping animation fixture");
   std::fputs("linear object 0 mapOffX 0 1\nlinear object 0 yaw 0 .4\n", mappingFile);
   std::fclose(mappingFile);
   const int beforeMapping = material.preparations;
   setFrame(mappingAnimation, &scene, 1, 1);
   unlink(mappingAnimation);
   require(material.preparations == beforeMapping,
           "mapping-only animation must reuse acceleration and texture caches");
   require(actual == withSphere, "uniform sphere mapping must preserve the image");
   scene.removeShape(scene.listStart);

   Box panel(Vector(0, 0, 4), &material, 0, 0, 0, .5, 1.);
   scene.addShape(&panel);
   refresh(&scene);
   const auto beforeAnimation = actual;
   char animation[] = "/tmp/ray-acceleration-animation-XXXXXX";
   const int fd = mkstemp(animation);
   require(fd >= 0, "could not create animation fixture");
   FILE* file = fdopen(fd, "w");
   require(file != NULL, "could not open animation fixture");
   std::fputs("linear object 0 textureX .5 3\nlinear object 0 yaw 0 .4\n", file);
   std::fclose(file);
   setFrame(animation, &scene, 1, 1);
   unlink(animation);
   require(!scene.accelerationDirty && actual != beforeAnimation,
           "object animation must refresh acceleration and change the image");
   DATA = expected.data();
   scene.buildAcceleration();
   refresh(&scene);
   require(actual == expected, "animated render must match an explicit rebuild");
   DATA = NULL;
   std::puts("acceleration add/remove/re-add and object animation checks passed");
}
