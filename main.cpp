//#include<printf.h>
#include "src/vector.h"
#include "src/shape.h"
#include "src/sphere.h"
#include "src/plane.h"
#include "src/light.h"
#include "src/box.h"
#include "src/disk.h"
#include "src/triangle.h"
#include "src/Textures/imagetexture.h"
#include "src/Textures/colortexture.h"
#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include <iostream>
#include <string>
#include <signal.h>
#ifdef USE_CUDA
#include "gpu/renderer.h"
#include <exception>
#endif
#ifdef _OPENMP
#include <omp.h>
#endif
using namespace std;

#include <time.h>
#include <cmath>

double tdiff(const struct timespec *start, const struct timespec *end) {
  return (end->tv_sec-start->tv_sec) + 1e-9*(end->tv_nsec-start->tv_nsec);
}


unsigned char* getColor(unsigned char a, unsigned char b, unsigned char c){
   unsigned char* r = (unsigned char*)malloc(sizeof(unsigned char)*3);
   r[0] = a;
   r[1] = b;
   r[2] = c;
   return r;
}
     
int W = 1000, H = 1000;

unsigned char* DATA = NULL;
unsigned char get(int i, int j, int k){
   return DATA[3*((size_t)i+(size_t)j*(size_t)W)+(size_t)k];
}
unsigned char* getPos(int i, int j){
   return &DATA[3*((size_t)i+(size_t)j*(size_t)W)];
}
void set(int i, int j, unsigned char r, unsigned char g, unsigned char b){
   const size_t pixel = (size_t)i + (size_t)j * (size_t)W;
   DATA[3*pixel] = r;
   DATA[3*pixel+1] = g;
   DATA[3*pixel+2] = b;
}

inline void renderPixel(Autonoma* c, size_t n) {
   Vector ra = c->camera.forward+((double)(n%W)/W-.5)*((c->camera.right))+(.5-(double)(n/W)/H)*((c->camera.up));
   calcColor(&DATA[3*n], c, Ray(c->camera.focus, ra), 0);
}

struct ScreenBounds {
   int left, right, top, bottom;
   bool valid, empty;
};

static ScreenBounds projectedBounds(const Autonoma* c) {
   ScreenBounds result = {0, W, 0, H, false, false};
   if (c->boundedShapes.size() < 256 || !c->unboundedShapes.empty() ||
       !c->skybox->isUniform() || c->bvhNodes.empty()) return result;

   const Vector& f = c->camera.forward;
   const Vector& r = c->camera.right;
   const Vector& u = c->camera.up;
   const double eps = 1e-9;
   const Vector ru = r.cross(u), uf = u.cross(f), fr = f.cross(r);
   const double determinant = f.dot(ru);
   if (!std::isfinite(determinant) || std::abs(determinant) <= eps) return result;

   double minX = inf, maxX = -inf, minY = inf, maxY = -inf;
   double minDepth = inf, maxDepth = -inf;
   const BVHNode& root = c->bvhNodes[0];
   for (int mask = 0; mask < 8; ++mask) {
      Vector q(root.boundsMin[0] + ((mask & 1) ? 1.0 : 0.0) *
                  (root.boundsMax[0] - root.boundsMin[0]),
               root.boundsMin[1] + ((mask & 2) ? 1.0 : 0.0) *
                  (root.boundsMax[1] - root.boundsMin[1]),
               root.boundsMin[2] + ((mask & 4) ? 1.0 : 0.0) *
                  (root.boundsMax[2] - root.boundsMin[2]));
      q = q - c->camera.focus;
      const double depth = q.dot(ru) / determinant;
      if (!std::isfinite(depth)) return result;
      minDepth = std::min(minDepth, depth);
      maxDepth = std::max(maxDepth, depth);
      if (depth > 1e-9) {
         const double x = (q.dot(uf) / determinant) / depth;
         const double y = (q.dot(fr) / determinant) / depth;
         if (!std::isfinite(x) || !std::isfinite(y)) return result;
         minX = std::min(minX, W * (x + .5));
         maxX = std::max(maxX, W * (x + .5));
         minY = std::min(minY, H * (.5 - y));
         maxY = std::max(maxY, H * (.5 - y));
      }
   }
   if (maxDepth < -eps) { result.valid = true; result.empty = true; return result; }
   if (minDepth <= eps || !std::isfinite(minX) || !std::isfinite(maxX) ||
       !std::isfinite(minY) || !std::isfinite(maxY)) return result;
   minX = std::max(-2.0, std::min((double)W + 2.0, minX));
   maxX = std::max(-2.0, std::min((double)W + 2.0, maxX));
   minY = std::max(-2.0, std::min((double)H + 2.0, minY));
   maxY = std::max(-2.0, std::min((double)H + 2.0, maxY));
   const double paddedLeft = std::floor(minX) - 2.0;
   const double paddedRight = std::ceil(maxX) + 2.0;
   const double paddedTop = std::floor(minY) - 2.0;
   const double paddedBottom = std::ceil(maxY) + 2.0;
   result.left = (int)std::max(0.0, std::min((double)W, paddedLeft));
   result.right = (int)std::max(0.0, std::min((double)W, paddedRight));
   result.top = (int)std::max(0.0, std::min((double)H, paddedTop));
   result.bottom = (int)std::max(0.0, std::min((double)H, paddedBottom));
   result.valid = true;
   result.empty = result.left >= result.right || result.top >= result.bottom;
   return result;
}

#if defined(__GNUC__)
__attribute__((noinline))
#endif
static void fillSkySpan(int y, int begin, int end, const unsigned char sky[3]) {
   const unsigned char red = sky[0], green = sky[1], blue = sky[2];
   unsigned char* row = DATA + 3 * (size_t)y * (size_t)W;
   for (int x = begin; x < end; ++x) {
      unsigned char* pixel = row + 3 * (size_t)x;
      pixel[0] = red; pixel[1] = green; pixel[2] = blue;
   }
}

static void renderBoundedRow(Autonoma* c, const ScreenBounds& screen, int y,
                             const unsigned char sky[3]) {
   fillSkySpan(y, 0, screen.left, sky);
   fillSkySpan(y, screen.right, W, sky);
   for (int x = screen.left; x < screen.right; ++x)
      renderPixel(c, (size_t)x + (size_t)y * (size_t)W);
}

void refresh(Autonoma* c){
#ifdef USE_CUDA
   gpuRender(c, DATA, W, H);
#else
   if (c->accelerationDirty) c->buildAcceleration();
   const size_t pixelCount = (size_t)H * (size_t)W;
   int workerCount = 1;
#ifdef _OPENMP
   workerCount = omp_get_max_threads();
#endif
   const char* screenSetting = std::getenv("RAY_SCREEN_BOUNDS");
   const bool screenGate = screenSetting == NULL || strcmp(screenSetting, "0") != 0;
   const ScreenBounds screen = screenGate ? projectedBounds(c) :
      ScreenBounds{0, W, 0, H, false, false};
   if (screenGate && screen.valid) {
      unsigned char sky[3]; double am, op, ref;
      c->skybox->getColor(sky, &am, &op, &ref, 0.0, 0.0);
      if (std::getenv("RAY_SCREEN_REPORT") != NULL)
         std::fprintf(stderr, "screen bounds: %s [%d,%d) x [%d,%d)\n",
                      screen.empty ? "empty" : "enabled", screen.left,
                      screen.right, screen.top, screen.bottom);
      if (screen.empty) {
         for (int y = 0; y < H; ++y) fillSkySpan(y, 0, W, sky);
         return;
      }
      for (int y = 0; y < screen.top; ++y) fillSkySpan(y, 0, W, sky);
      for (int y = screen.bottom; y < H; ++y) fillSkySpan(y, 0, W, sky);
#ifdef _OPENMP
      if (workerCount > 1 && pixelCount >= 65536) {
#pragma omp parallel for schedule(static)
         for (int y = screen.top; y < screen.bottom; ++y)
            renderBoundedRow(c, screen, y, sky);
      } else
#endif
      for (int y = screen.top; y < screen.bottom; ++y)
         renderBoundedRow(c, screen, y, sky);
      return;
   }
   if (workerCount == 1 || pixelCount < 65536) {
      for (size_t n = 0; n < pixelCount; ++n) renderPixel(c, n);
   } else if (c->boundedShapes.size() >= 256) {
#pragma omp parallel for schedule(static)
      for (long long n = 0; n < (long long)pixelCount; ++n) renderPixel(c, (size_t)n);
   } else {
#pragma omp parallel for schedule(dynamic, 16)
      for (long long n = 0; n < (long long)pixelCount; ++n) renderPixel(c, (size_t)n);
   }
#endif
}

std::string shellQuote(const char* value) {
   std::string result="\'";
   for (const char* p=value; *p; ++p) result += *p=='\'' ? "\'\\\'\'" : std::string(1,*p);
   return result+"\'";
}

void outputPPM(FILE* f){
   fprintf(f, "P6 %d %d 255 ", W, H);
   fwrite(DATA, 1, (size_t)W * (size_t)H * 3, f);
}

void outputPPM(char* file){
   FILE* f = fopen(file, "wb");
   if (!f) { perror(file); exit(1); }
   outputPPM(f);
   if (ferror(f) || fclose(f) != 0) { perror(file); exit(1); }
}
void output(char* file){
   char command[2000];
   FILE* f;
   snprintf(command, sizeof(command), "magick ppm:- %.1900s", file);
   printf("%s\n",command);
   f = popen(command, "w");
   outputPPM(f);
   pclose(f);
}

int streq(const char* a, const char* b) {
   return strcmp(a, b) == 0;
}

// Similar to fscanf, except ignore empty and comment lines
// Kept as a macro to preserve compiler warnings for mismatch input type
#define lscanf(f, ...) \
({\
   char line[1000];\
   char* linePtr = line;\
   size_t len = sizeof(line);\
   int retval;\
   while ((retval = getline(&linePtr, &len, f)) != EOF) {\
      if (line[0] == '#') continue;\
      if (line[0] == '\n') continue;\
      if (line[0] == '\0') continue;\
      sscanf(line, __VA_ARGS__);\
      break;\
   }\
   retval;\
})

Texture* parseTexture(FILE* f, bool allowNull) {
   char texture_type[80];

   if (lscanf(f, "%s", texture_type) == EOF) {
      printf("Found EOF while parsing texture type\n");
      exit(1);
   }
   if (streq(texture_type, "null")) {
      if (allowNull)
         return NULL;
      printf("Null texture not permitted\n");
      exit(1);
   }
   if (streq(texture_type, "color")) {
      int r, g, b;
      double opacity=1., reflection=0., ambient=.3;
      if (lscanf(f, "%d %d %d %lf %lf %lf\n", &r, &g, &b, &opacity, &reflection, &ambient) == EOF) {
         printf("Could not read <r> <g> <b> <opacity> <reflection> <ambient>\n");
         exit(1);
      }
      return new ColorTexture((unsigned char)r, (unsigned char)g, (unsigned char)b, opacity, reflection, ambient);
   }
   if (streq(texture_type, "image")) {
      char image_file[100];
      if (lscanf(f, "%s\n", image_file) == EOF) {
         printf("Could not read <image path>\n");
         exit(1);
      }
      return new ImageTexture(image_file);
   }
   if (streq(texture_type, "maskedimage")) {
      char image_file[100];
      if (lscanf(f, "%s\n", image_file) == EOF) {
         printf("Could not read <image path>\n");
         exit(1);
      }
      ImageTexture *text = new ImageTexture(image_file);
      text->maskImageAlpha();
      return text;
   }
   if (streq(texture_type, "inlineimage")) {
      int w, h;
      double opacity, reflection, ambient;
      if (lscanf(f, "%d %d %lf %lf %lf\n", &w, &h, &opacity, &reflection, &ambient) == EOF) {
         printf("Could not read <w> <h> <b> <opacity> <reflection> <ambient>\n");
         exit(1);
      }

      ImageTexture* text = new ImageTexture(w, h);
      for (int x=0; x<w; x++) {
         for (int y=0; y<h; y++) {
            int r, g, b;
            if (lscanf(f, "%d %d %d\n", &r, &g, &b) == EOF) {
               printf("Could not read <r> <g> <b>\n");
               exit(1);
            }
           text->setColor(x, y, r, g, b);
         }
      }
      text->opacity = opacity;
      text->reflection = reflection;
      text->ambient = ambient;
      return text;
   }

   printf("Unknown texture type \"%s\"\n", texture_type);
   exit(1);
}


Vector* getVectors(FILE* f, int len){
   Vector* vec = (Vector*)malloc(len*sizeof(Vector));
   float x, y, z;
   for(int i = 0; i<len; i++){
      if (fscanf(f, "%f %f %f\n", &x, &y, &z) == EOF) {
         printf("Failed to read vectors\n");
         exit(1);
      }
      vec[i].x = x;
      vec[i].y = y;
      vec[i].z = z;
   }
   return vec;
}
unsigned int* getTriangles(FILE* f, int len){
   unsigned int* vec = (unsigned int*)malloc(3*len*sizeof(unsigned int));
   int a, b, d;
   for(int i = 0; i<3*len; i+=3){
      if (fscanf(f, "%d %d %d\n", &vec[i], &vec[i+1], &vec[i+2]) == EOF) {
         printf("Failed to read triangles\n");
         exit(1);
      }
   }
   return vec;
}

Autonoma* createInputs(const char* inputFile) {
   
   double camera_x = 0;
   double camera_y = 2;
   double camera_z = 0;
   double yaw = 0;
   double pitch = 0;
   double roll = 0;
   Texture *background = NULL;

   FILE *f = NULL;
   if (inputFile) {
      f = fopen(inputFile, "r");
      if (!f) {
         printf("Could not open input file %s\n", inputFile);
         exit(1);
      }
      if (lscanf(f, "%lf %lf %lf %lf %lf %lf\n", &camera_x, &camera_y, &camera_z, &yaw, &pitch, &roll) == EOF) {
         printf("Could not read <camera_x> <camera_y> <camera_z> <yaw> <pitch> <roll>\n");
         exit(1);
      }
      background = parseTexture(f, false);
   }
   if (!background) {
      const char* texture_path = "images/skybox.jpg";
      background = new ImageTexture(texture_path);
   }
   Autonoma* MAIN_DATA = new Autonoma(Camera(Vector(camera_x, camera_y, camera_z), yaw, pitch, roll),background);

   if (f) {
      char object_type[80];
      while (lscanf(f, "%s", object_type) != EOF) {
         if (streq(object_type, "light")) {
            double light_x, light_y, light_z;
            int color_r, color_g, color_b;
            if (lscanf(f, "%lf %lf %lf %d %d %d\n", &light_x, &light_y, &light_z, &color_r, &color_g, &color_b) == EOF) {
               printf("Could not read <light_x> <light_y> <light_z> <color_r> <color_g> <color_b>\n");
               exit(1);
            }
            Light *light = new Light(Vector(light_x, light_y, light_z), getColor(color_r, color_g, color_b));
            MAIN_DATA->addLight(light);
         } else if (streq(object_type, "plane")) {
            double plane_x, plane_y, plane_z;
            double yaw, pitch, roll;
            double tx, ty;
            if (lscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &plane_x, &plane_y, &plane_z, &yaw, &pitch, &roll, &tx, &ty) == EOF) {
               printf("Could not read <plane_x> <plane_y> <plane_z> <yaw> <pitch> <roll> <tx> <ty>\n");
               exit(1);
            }
            Texture *texture = parseTexture(f, false);
            Plane *shape = new Plane(Vector(plane_x, plane_y, plane_z), texture, yaw, pitch, roll, tx, ty);
            MAIN_DATA->addShape(shape);
            shape->normalMap = parseTexture(f, true);
         } else if (streq(object_type, "disk")) {
            double disk_x, disk_y, disk_z;
            double yaw, pitch, roll;
            double tx, ty;
            if (lscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &disk_x, &disk_y, &disk_z, &yaw, &pitch, &roll, &tx, &ty) == EOF) {
               printf("Could not read <disk_x> <disk_y> <disk_z> <yaw> <pitch> <roll> <tx> <ty>\n");
               exit(1);
            }
            Texture *texture = parseTexture(f, false);
            Disk* shape = new Disk(Vector(disk_x, disk_y, disk_z), texture, yaw, pitch, roll, tx, ty);
            MAIN_DATA->addShape(shape);
            shape->normalMap = parseTexture(f, true);
         } else if (streq(object_type, "box")) {
            double box_x, box_y, box_z;
            double yaw, pitch, roll;
            double tx, ty;
            if (lscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &box_x, &box_y, &box_z, &yaw, &pitch, &roll, &tx, &ty) == EOF) {
               printf("Could not read <box_x> <box_y> <box_z> <yaw> <pitch> <roll> <tx> <ty>\n");
               exit(1);
            }
            Texture *texture = parseTexture(f, false);
            Box* shape = new Box(Vector(box_x, box_y, box_z), texture, yaw, pitch, roll, tx, ty);
            MAIN_DATA->addShape(shape);
            shape->normalMap = parseTexture(f, true);
         } else if (streq(object_type, "triangle")) {
            double x1, y1, z1;
            double x2, y2, z2;
            double x3, y3, z3;
            if (lscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &x1, &y1, &z1, &x2, &y2, &z2, &x3, &y3, &z3) == EOF) {
               printf("Could not read <x1> <y1> <z1> <x2> <y2> <z2> <x3> <y3> <z3>\n");
               exit(1);
            }
            Texture *texture = parseTexture(f, false);
            Triangle* shape = new Triangle(Vector(x1, y1, z1), Vector(x2, y2, z2), Vector(x3, y3, z3), texture);
            MAIN_DATA->addShape(shape);
            shape->normalMap = parseTexture(f, true);
         } else if (streq(object_type, "sphere")) {
            double sphere_x, sphere_y, sphere_z;
            double yaw, pitch, roll;
            double radius;
            if (lscanf(f, "%lf %lf %lf %lf %lf %lf %lf\n", &sphere_x, &sphere_y, &sphere_z, &yaw, &pitch, &roll, &radius) == EOF) {
               printf("Could not read <sphere_x> <sphere_y> <sphere_z> <yaw> <pitch> <roll> <radius>\n");
               exit(1);
            }
            Texture *texture = parseTexture(f, false);
            Sphere* shape = new Sphere(Vector(sphere_x, sphere_y, sphere_z), texture, yaw, pitch, roll, radius);
            MAIN_DATA->addShape(shape);
            shape->normalMap = parseTexture(f, true);
         } else if (streq(object_type, "mesh")) {
             char point_filepath[100];
             char poly_filepath[100];
             int num_points;
             int num_polygons;
             double off_x;
             double off_y;
             double off_z;
            if (lscanf(f, "%s %d %s %d %lf %lf %lf\n", point_filepath, &num_points, poly_filepath, &num_polygons, &off_x, &off_y, &off_z) == EOF) {
               printf("Could not read <point filepath> <num_points> <polygons filepath> <num_polygons> <off_x> <off_y> <off_z>\n");
               exit(1);
            }
            Texture *texture = parseTexture(f, false);
            Texture *normalMap = parseTexture(f, true);

            FILE* vectors = fopen(point_filepath,"r"), *triangles = fopen(poly_filepath,"r");
            if (!vectors) {
               printf("Could not open point file %s\n", point_filepath);
               exit(1);
            }
            if (!triangles) {
               printf("Could not open triangles file %s\n", poly_filepath);
               exit(1);
            }
            Vector* points = getVectors(vectors, num_points);
            fclose(vectors);
            unsigned int* polys = getTriangles(triangles, num_polygons);
            fclose(triangles);
            Vector offset(off_x, off_y, off_z); 
            for(int i = 0; i<num_polygons; i++){
               Triangle* shape = new Triangle(points[polys[3*i]] + offset, points[polys[3*i+1]] + offset, points[polys[3*i+2]] + offset, texture);
               MAIN_DATA->addShape(shape);
               shape->normalMap = normalMap;
            }
         } else {
           printf("Unknown object type %s\n", object_type);
           exit(1);
         }
      }
   }

   if (f) fclose(f);
   return MAIN_DATA;
}

double linearfn(double x, double from, double to) {
   return (1 - x) * from + x * to;
}
double expfn(double x, double from, double to) {
   return (to - from) * exp(10 * x) / exp(10) + from;
}
double sinfn(double x, double from, double to) {
   return (to - from) * sin(x * 6.28) + from;
}
double cosfn(double x, double from, double to) {
   return (to - from) * cos(x * 6.28) + from;
}

void setFrame(const char* animateFile, Autonoma* MAIN_DATA, int frame, int frameLen) {
   if (animateFile) {
      char object_type[80];
      char transition_type[80];
      int obj_num;
      char field_type[80];
      double from;
      double to;
      FILE* f = fopen(animateFile, "r");
      if (!f) { perror(animateFile); exit(1); }
      while (lscanf(f, "%s %s %d %s %lf %lf", transition_type, object_type, &obj_num, field_type, &from, &to) != EOF) {
         double (*func)(double, double, double);
         if (streq(transition_type, "linear")) {
            func = linearfn;
         } else if (streq(transition_type, "exp")) {
            func = expfn;
         } else if (streq(transition_type, "sin")) {
            func = sinfn;
         } else if (streq(transition_type, "cos")) {
            func = cosfn;
         } else {
            printf("Unknown transition type %s, expected one of linear, exp, cos, or sin\n", transition_type);
            exit(1);
         }
         double result = func((double)frame / frameLen, from, to);

         if (streq(object_type, "camera")) {
            if (streq(field_type, "yaw")) {
               MAIN_DATA->camera.setYaw(result);
            } else if (streq(field_type, "pitch")) {
               MAIN_DATA->camera.setPitch(result);
            } else if (streq(field_type, "roll")) {
               MAIN_DATA->camera.setRoll(result);
            } else if (streq(field_type, "x")) {
               MAIN_DATA->camera.focus.x = result;
            } else if (streq(field_type, "y")) {
               MAIN_DATA->camera.focus.y = result;
            } else if (streq(field_type, "z")) {
               MAIN_DATA->camera.focus.z = result;
            } else {
               printf("Unknown camera field_type %s, expected one of yaw, pitch, roll, x, y, z\n", field_type);
               exit(1);
            }
         } else if (streq(object_type, "object")) {
#ifdef USE_CUDA
            gpuSceneChanged();
#endif
            ShapeNode* node = MAIN_DATA->listStart;
            for (int i=0; i<obj_num; i++) {
               if (node == MAIN_DATA->listEnd) {
                  printf("Could not find object number %d\n", obj_num);
                  exit(1);
               }
               if (i == obj_num)
                  break;
               node = node->next;
            }
            Shape* shape = node->data;
#ifndef USE_CUDA
            double oldMin[3], oldMax[3];
            const bool wasBounded = shape->getBounds(oldMin, oldMax);
#endif

            if (streq(field_type, "yaw")) {
               shape->setYaw(result);
            } else if (streq(field_type, "pitch")) {
               shape->setPitch(result);
            } else if (streq(field_type, "roll")) {
               shape->setRoll(result);
            } else if (streq(field_type, "textureX")) {
               shape->textureX = result;
            } else if (streq(field_type, "textureY")) {
               shape->textureY = result;
            } else if (streq(field_type, "mapX")) {
               shape->mapX = result;
            } else if (streq(field_type, "mapY")) {
               shape->mapY = result;
            } else if (streq(field_type, "mapOffX")) {
               shape->mapOffX = result;
            } else if (streq(field_type, "mapOffY")) {
               shape->mapOffY = result;
            } else {
               printf("Unknown shape field_type %s, expected one of yaw, pitch, roll, textureX, textureY, mapX, mapY, mapOffX, mapOffY\n", field_type);
               exit(1);
            }
#ifndef USE_CUDA
            double newMin[3], newMax[3];
            const bool isBounded = shape->getBounds(newMin, newMax);
            bool boundsChanged = wasBounded != isBounded;
            if (wasBounded && isBounded)
               for (int axis = 0; axis < 3; ++axis)
                  boundsChanged |= oldMin[axis] != newMin[axis] || oldMax[axis] != newMax[axis];
            MAIN_DATA->accelerationDirty |= boundsChanged;
#endif
         } else {
            printf("Unknown object_type %s, expected one of camera, object\n", field_type);
            exit(1);
         }
      }
      fclose(f);
   }

   refresh(MAIN_DATA);
}

int main(int argc, const char** argv){

   int frameLen = 1;
   const char* inFile = NULL;
   const char* animateFile = NULL;
   const char* outFile = NULL;
   bool toMovie = true;
   bool png = true;
   bool noOutput = false;
#ifdef USE_CUDA
   bool nvenc = true;
#else
   bool nvenc = false;
#endif
   for (int i=1; i<argc; i++) {
      if (streq(argv[i], "-H")) {
         if (i + 1 >= argc) {
            printf("Error -H option must be followed by an integer height\n"); return 1;
         }
         H = atoi(argv[i+1]);
         i++;
         continue;
      }
      if (streq(argv[i], "-W")) {
         if (i + 1 >= argc) {
            printf("Error -W option must be followed by an integer width\n"); return 1;
         }
         W = atoi(argv[i+1]);
         i++;
         continue;
      }
      if (streq(argv[i], "-F")) {
         if (i + 1 >= argc) {
            printf("Error -F option must be followed by an integer number of frames\n"); return 1;
         }
         frameLen = atoi(argv[i+1]);
         i++;
         continue;
      }
      if (streq(argv[i], "-o")) {
         if (i + 1 >= argc) {
            printf("Error -o option must be followed by an output file path\n"); return 1;
         }
         outFile = argv[i+1];
         i++;
         continue;
      }
      if (streq(argv[i], "-i")) {
         if (i + 1 >= argc) {
            printf("Error -i option must be followed by an input file path\n"); return 1;
         }
         inFile = argv[i+1];
         i++;
         continue;
      }
      if (streq(argv[i], "-a")) {
         if (i + 1 >= argc) {
            printf("Error -a option must be followed by an animation input file path\n"); return 1;
         }
         animateFile = argv[i+1];
         i++;
         continue;
      }
      if (streq(argv[i], "--cpu-encode")) { nvenc = false; continue; }
      if (streq(argv[i], "--nvenc")) { nvenc = true; continue; }
      if (streq(argv[i], "--no-output")) {
         noOutput = true; toMovie = false; continue;
      }
      if (streq(argv[i], "--movie")) {
         toMovie = true;
         continue;
      }
      if (streq(argv[i], "--no-movie")) {
         toMovie = false;
         continue;
      }
      if (streq(argv[i], "--ppm")) {
         png = false;
         continue;
      }
      if (streq(argv[i], "--png")) {
         png = true;
         continue;
      }
      if (streq(argv[i], "--help")) {
         printf("Usage %s [-H <height>] [-W <width>] [-F <framecount>] [--movie] [--no-movie] [--png] [--ppm] [--no-output] [--nvenc|--cpu-encode] [-a <animationfile>] [--help] [-o <outfile>] [-i <infile>]\n", argv[0]);
         return 0;
      }
      printf("Unknown option %s, look at %s --help\n", argv[i], argv[0]);
      return 1;
   }

   if (outFile == NULL) {
      if (frameLen == 1) {
         if (png) {
            outFile = "output/output.png";
         } else {
            outFile = "output/output.ppm";            
         }
      } else {
         outFile = "output/output.mp4";
      }
   }

   if (W <= 0 || H <= 0 || frameLen <= 0 || (size_t)W*H > 100000000) {
      fprintf(stderr, "Invalid dimensions or frame count\n"); return 1;
   }
   free(DATA);
   DATA = (unsigned char*)malloc((size_t)W*H*3);
   if (!DATA) { fprintf(stderr, "Image allocation failed\n"); return 1; }
#ifdef USE_CUDA
   try {
#endif
   Autonoma* MAIN_DATA = createInputs(inFile);
#ifndef USE_CUDA
   MAIN_DATA->buildAcceleration();
#endif

   
   int frame;
   char command[2000];
   FILE* video = nullptr;
   if (frameLen > 1 && toMovie && !noOutput && nvenc) {
      if (W%2 || H%2) { fprintf(stderr,"NVENC yuv420p requires even dimensions\n"); return 1; }
      std::string encoder = "ffmpeg -hide_banner -loglevel error -y -f image2pipe -vcodec ppm -framerate 24 -i - -c:v h264_nvenc -preset p4 -cq 18 -pix_fmt yuv420p " + shellQuote(outFile);
      signal(SIGPIPE, SIG_IGN);
      video = popen(encoder.c_str(), "w");
      if (!video) { perror("ffmpeg"); return 1; }
   }
   
   struct timespec start, end;
   clock_gettime(CLOCK_MONOTONIC_RAW, &start);
   for(frame = 0; frame<frameLen; frame++) {
      setFrame(animateFile, MAIN_DATA, frame, frameLen);      
      if (frameLen == 1) {
         snprintf(command, sizeof(command), "%s", outFile);    
      } else if (png) {
         snprintf(command, sizeof(command), "%s.tmp.%07d.png", outFile, frame);
      } else {
         snprintf(command, sizeof(command), "%s.tmp.%07d.ppm", outFile, frame);
      }
      if (video) {
         outputPPM(video);
         if (ferror(video)) { pclose(video); fprintf(stderr,"NVENC encoder failed\n"); return 1; }
      } else if (noOutput) {
      } else if (png) {
         output(command); 
      } else {
         outputPPM(command); 
      }     
      printf("Done Frame %7d|\n", frame);
   }

   if (video && pclose(video) != 0) { fprintf(stderr,"NVENC encoder failed\n"); return 1; }
   clock_gettime(CLOCK_MONOTONIC_RAW, &end);
   printf("Total time to create images=%0.9f seconds\n", tdiff(&start, &end));


#ifdef USE_CUDA
   gpuShutdown();
#endif
   if (frameLen > 1 && toMovie && !noOutput && !nvenc) {
      if (png) {
         snprintf(command, sizeof(command), "ffmpeg -y -r 24 -i %.400s.tmp.%%07d.png -vcodec ffv1 %.400s.tmp.avi && ffmpeg -y -i %.400s.tmp.avi -c:v libx264 -preset veryslow -qp 0 -r 24 %.400s", outFile, outFile, outFile, outFile);
      } else {
         snprintf(command, sizeof(command), "ffmpeg -y -r 24 -i %.400s.tmp.%%07d.ppm -vcodec ffv1 %.400s.tmp.avi && ffmpeg -y -i %.400s.tmp.avi -c:v libx264 -preset veryslow -qp 0 -r 24 %.400s", outFile, outFile, outFile, outFile);
      }
      return system(command);
   }   
   return 0;
#ifdef USE_CUDA
   } catch (const std::exception& error) {
      fprintf(stderr, "GPU rendering failed: %s\n", error.what());
      gpuShutdown(); return 1;
   }
#endif
}
