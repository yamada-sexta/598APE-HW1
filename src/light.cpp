
#include "light.h"
#include "shape.h"
#include "camera.h"
#include <algorithm>
#include <cmath>
#include <limits>
      
Light::Light(const Vector & cente, unsigned char* colo) : center(cente){
   color = colo;
}

unsigned char* Light::getColor(unsigned char a, unsigned char b, unsigned char c){
   unsigned char* r = (unsigned char*)malloc(sizeof(unsigned char)*3);
   r[0] = a;
   r[1] = b;
   r[2] = c;
   return r;
}

Autonoma::Autonoma(const Camera& c): camera(c){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
   depth = 10;
   skybox = BLACK;
}

Autonoma::Autonoma(const Camera& c, Texture* tex): camera(c){
   listStart = NULL;
   listEnd = NULL;
   lightStart = NULL;
   lightEnd = NULL;
   depth = 10;
   skybox = tex;
}

void Autonoma::addShape(Shape* r){
   ShapeNode* hi = (ShapeNode*)malloc(sizeof(ShapeNode));
   hi->data = r;
   hi->next = hi->prev = NULL;
   if(listStart==NULL){
      listStart = listEnd = hi;
   }
   else{
      listEnd->next = hi;
      hi->prev = listEnd;
      listEnd = hi;
   }
}

void Autonoma::removeShape(ShapeNode* s){
   if(s==listStart){
      if(s==listEnd){
         listStart = listStart = NULL;
      }
      else{
         listStart = s->next;
         listStart->prev = NULL;
      }
   }
   else if(s==listEnd){
      listEnd = s->prev;
      listEnd->next = NULL;
   }
   else{
      ShapeNode *b4 = s->prev, *aft = s->next;
      b4->next = aft;
      aft->prev = b4;
   }
   free(s);
}

void Autonoma::addLight(Light* r){
   LightNode* hi = (LightNode*)malloc(sizeof(LightNode));
   hi->data = r;
   hi->next = hi->prev = NULL;
   if(lightStart==NULL){
      lightStart = lightEnd = hi;
   }
   else{
      lightEnd->next = hi;
      hi->prev = lightEnd;
      lightEnd = hi;
   }
}

void Autonoma::removeLight(LightNode* s){
   if(s==lightStart){
      if(s==lightEnd){
         lightStart = lightStart = NULL;
      }
      else{
         lightStart = s->next;
         lightStart->prev = NULL;
      }
   }
   else if(s==lightEnd){
      lightEnd = s->prev;
      lightEnd->next = NULL;
   }
   else{
      LightNode *b4 = s->prev, *aft = s->next;
      b4->next = aft;
      aft->prev = b4;
   }
   free(s);
}

static double component(const Vector& vector, int axis) {
   if (axis == 0) return vector.x;
   if (axis == 1) return vector.y;
   return vector.z;
}

static bool intersectsBounds(const BVHNode& node, const Ray& ray,
                             double maximum, double* nearDistance = NULL) {
   double near = 0.0;
   double far = maximum;
   for (int axis = 0; axis < 3; ++axis) {
      const double origin = component(ray.point, axis);
      const double direction = component(ray.vector, axis);
      if (std::abs(direction) < 1e-15) {
         if (origin < node.boundsMin[axis] || origin > node.boundsMax[axis]) return false;
         continue;
      }
      double first = (node.boundsMin[axis] - origin) / direction;
      double second = (node.boundsMax[axis] - origin) / direction;
      if (first > second) std::swap(first, second);
      if (first > near) near = first;
      if (second < far) far = second;
      if (near > far) return false;
   }
   if (nearDistance != NULL) *nearDistance = near;
   return far >= 0.0;
}

void Autonoma::buildAcceleration() {
   boundedShapes.clear();
   unboundedShapes.clear();
   bvhNodes.clear();

   for (ShapeNode* node = listStart; node != NULL; node = node->next) {
      BVHPrimitive primitive;
      primitive.shape = node->data;
      if (node->data->getBounds(primitive.boundsMin, primitive.boundsMax)) {
         for (int axis = 0; axis < 3; ++axis) {
            primitive.centroid[axis] =
               0.5 * (primitive.boundsMin[axis] + primitive.boundsMax[axis]);
         }
         boundedShapes.push_back(primitive);
      } else {
         unboundedShapes.push_back(node->data);
      }
   }

   if (!boundedShapes.empty()) {
      bvhNodes.reserve(boundedShapes.size() * 2);
      buildBVHNode(0, boundedShapes.size());
   }
}

int Autonoma::buildBVHNode(size_t start, size_t end) {
   BVHNode node;
   for (int axis = 0; axis < 3; ++axis) {
      node.boundsMin[axis] = std::numeric_limits<double>::infinity();
      node.boundsMax[axis] = -std::numeric_limits<double>::infinity();
      for (size_t index = start; index < end; ++index) {
         node.boundsMin[axis] = std::min(node.boundsMin[axis], boundedShapes[index].boundsMin[axis]);
         node.boundsMax[axis] = std::max(node.boundsMax[axis], boundedShapes[index].boundsMax[axis]);
      }
   }
   node.left = -1;
   node.right = -1;
   node.start = start;
   node.count = end - start;

   const int nodeIndex = (int)bvhNodes.size();
   bvhNodes.push_back(node);
   if (node.count <= 4) return nodeIndex;

   int splitAxis = 0;
   double widest = -1.0;
   for (int axis = 0; axis < 3; ++axis) {
      double centroidMin = std::numeric_limits<double>::infinity();
      double centroidMax = -std::numeric_limits<double>::infinity();
      for (size_t index = start; index < end; ++index) {
         centroidMin = std::min(centroidMin, boundedShapes[index].centroid[axis]);
         centroidMax = std::max(centroidMax, boundedShapes[index].centroid[axis]);
      }
      if (centroidMax - centroidMin > widest) {
         widest = centroidMax - centroidMin;
         splitAxis = axis;
      }
   }

   const size_t middle = start + (end - start) / 2;
   std::nth_element(boundedShapes.begin() + start, boundedShapes.begin() + middle,
                    boundedShapes.begin() + end,
                    [splitAxis](const BVHPrimitive& first, const BVHPrimitive& second) {
                       return first.centroid[splitAxis] < second.centroid[splitAxis];
                    });
   const int left = buildBVHNode(start, middle);
   const int right = buildBVHNode(middle, end);
   bvhNodes[nodeIndex].left = left;
   bvhNodes[nodeIndex].right = right;
   bvhNodes[nodeIndex].count = 0;
   return nodeIndex;
}

Shape* Autonoma::closestIntersection(const Ray& ray, double& closest) const {
   Shape* closestShape = NULL;
   for (size_t index = 0; index < unboundedShapes.size(); ++index) {
      const double time = unboundedShapes[index]->getIntersection(ray);
      if (time < closest) {
         closest = time;
         closestShape = unboundedShapes[index];
      }
   }

   if (bvhNodes.empty()) return closestShape;
   int stack[128];
   int stackSize = 0;
   stack[stackSize++] = 0;
   while (stackSize != 0) {
      const BVHNode& node = bvhNodes[stack[--stackSize]];
      if (!intersectsBounds(node, ray, closest)) continue;
      if (node.count != 0) {
         const size_t end = node.start + node.count;
         for (size_t index = node.start; index < end; ++index) {
            const double time = boundedShapes[index].shape->getIntersection(ray);
            if (time < closest) {
               closest = time;
               closestShape = boundedShapes[index].shape;
            }
         }
         continue;
      }

      double leftNear, rightNear;
      const bool hitLeft = intersectsBounds(bvhNodes[node.left], ray, closest, &leftNear);
      const bool hitRight = intersectsBounds(bvhNodes[node.right], ray, closest, &rightNear);
      if (hitLeft && hitRight) {
         if (leftNear < rightNear) {
            stack[stackSize++] = node.right;
            stack[stackSize++] = node.left;
         } else {
            stack[stackSize++] = node.left;
            stack[stackSize++] = node.right;
         }
      } else if (hitLeft) {
         stack[stackSize++] = node.left;
      } else if (hitRight) {
         stack[stackSize++] = node.right;
      }
   }
   return closestShape;
}

bool Autonoma::lightIntersection(const Ray& ray, double* fill) const {
   for (size_t index = 0; index < unboundedShapes.size(); ++index) {
      if (unboundedShapes[index]->getLightIntersection(ray, fill)) return true;
   }

   if (bvhNodes.empty()) return false;
   int stack[128];
   int stackSize = 0;
   stack[stackSize++] = 0;
   while (stackSize != 0) {
      const BVHNode& node = bvhNodes[stack[--stackSize]];
      if (!intersectsBounds(node, ray, 1.0)) continue;
      if (node.count != 0) {
         const size_t end = node.start + node.count;
         for (size_t index = node.start; index < end; ++index) {
            if (boundedShapes[index].shape->getLightIntersection(ray, fill)) return true;
         }
      } else {
         stack[stackSize++] = node.left;
         stack[stackSize++] = node.right;
      }
   }
   return false;
}

void getLight(double* tColor, Autonoma* aut, Vector point, Vector norm, unsigned char flip){
   tColor[0] = tColor[1] = tColor[2] = 0.;
   LightNode *t = aut->lightStart;
   while(t!=NULL){
      double lightColor[3];     
      lightColor[0] = t->data->color[0]/255.;
      lightColor[1] = t->data->color[1]/255.;
      lightColor[2] = t->data->color[2]/255.;
      Vector ra = t->data->center-point;
      const bool hit = aut->lightIntersection(Ray(point+ra*.01, ra), lightColor);
      double perc = (norm.dot(ra)/(ra.mag()*norm.mag()));
      if(!hit){
      if(flip && perc<0) perc=-perc;
        if(perc>0){
      
         tColor[0]+= perc*(lightColor[0]);
         tColor[1]+= perc*(lightColor[0]);
         tColor[2]+= perc*(lightColor[0]);
         if(tColor[0]>1.) tColor[0] = 1.;
         if(tColor[1]>1.) tColor[1] = 1.;
         if(tColor[2]>1.) tColor[2] = 1.;
        }
      }
      t =t->next;
   }
}
