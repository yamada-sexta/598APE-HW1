#include "bvh.h"
#include "shape.h"
#include <algorithm>

namespace {
struct Item{
   Shape* shape;
   double lo[3], hi[3], cen[3];
};
std::vector<Item> items;   

inline void expand(double* lo, double* hi, const double* l, const double* h){
   for(int k=0;k<3;k++){ if(l[k]<lo[k]) lo[k]=l[k]; if(h[k]>hi[k]) hi[k]=h[k]; }
}
}

void BVH::build(const std::vector<Shape*>& shapes){
   nodes.clear();
   prims.clear();
   items.clear();
   for(Shape* s : shapes){
      Vector a(0,0,0), b(0,0,0);
      if(!s->getBounds(a, b)) continue;
      Item it;
      it.shape = s;
      it.lo[0]=a.x; it.lo[1]=a.y; it.lo[2]=a.z;
      it.hi[0]=b.x; it.hi[1]=b.y; it.hi[2]=b.z;
      it.cen[0]=0.5*(a.x+b.x); it.cen[1]=0.5*(a.y+b.y); it.cen[2]=0.5*(a.z+b.z);
      items.push_back(it);
   }
   if(items.empty()) return;
   nodes.reserve(2*items.size());
   buildRange(0, (int)items.size(), 0);
   prims.reserve(items.size());
   for(Item& it : items) prims.push_back(it.shape);
   items.clear();
}

int BVH::buildRange(int start, int count, int depth){
   int idx = (int)nodes.size();
   nodes.push_back(Node());

   double lo[3]={1e300,1e300,1e300}, hi[3]={-1e300,-1e300,-1e300};
   double clo[3]={1e300,1e300,1e300}, chi[3]={-1e300,-1e300,-1e300};
   for(int i=start;i<start+count;i++){
      expand(lo, hi, items[i].lo, items[i].hi);
      for(int k=0;k<3;k++){
         if(items[i].cen[k]<clo[k]) clo[k]=items[i].cen[k];
         if(items[i].cen[k]>chi[k]) chi[k]=items[i].cen[k];
      }
   }
   for(int k=0;k<3;k++){ nodes[idx].lo[k]=lo[k]; nodes[idx].hi[k]=hi[k]; }

   if(count<=4 || depth>62){
      nodes[idx].start=start; nodes[idx].count=count; nodes[idx].right=-1;
      return idx;
   }

   int axis=0;
   double ext=chi[0]-clo[0];
   if(chi[1]-clo[1]>ext){ ext=chi[1]-clo[1]; axis=1; }
   if(chi[2]-clo[2]>ext){ ext=chi[2]-clo[2]; axis=2; }

   int mid=start+count/2;
   std::nth_element(items.begin()+start, items.begin()+mid, items.begin()+start+count,
      [axis](const Item& a, const Item& b){ return a.cen[axis]<b.cen[axis]; });

   nodes[idx].start=-1; nodes[idx].count=0;
   buildRange(start, mid-start, depth+1);          
   nodes[idx].right = buildRange(mid, start+count-mid, depth+1);
   return idx;
}

double BVH::closest(const Ray& ray, Shape*& out) const{
   out=NULL;
   if(nodes.empty()) return inf;
   const double o[3]={ray.point.x, ray.point.y, ray.point.z};
   const double inv[3]={1.0/ray.vector.x, 1.0/ray.vector.y, 1.0/ray.vector.z};
   double best=inf;
   int stack[128]; int sp=0; stack[sp++]=0;
   while(sp){
      int ni=stack[--sp];
      const Node& n=nodes[ni];
      double t0=0, t1=best;
      for(int k=0;k<3;k++){
         double a=(n.lo[k]-o[k])*inv[k], b=(n.hi[k]-o[k])*inv[k];
         if(a>b){ double t=a; a=b; b=t; }
         if(a>t0) t0=a;
         if(b<t1) t1=b;
      }
      if(t0>t1) continue;
      if(n.right==-1){
         for(int i=n.start;i<n.start+n.count;i++){
            double t=prims[i]->getIntersection(ray);
            if(t<best){ best=t; out=prims[i]; }
         }
      } else {
         stack[sp++]=ni+1;
         stack[sp++]=n.right;
      }
   }
   return best;
}

bool BVH::shadow(const Ray& ray, double* fill) const{
   if(nodes.empty()) return false;
   const double o[3]={ray.point.x, ray.point.y, ray.point.z};
   const double inv[3]={1.0/ray.vector.x, 1.0/ray.vector.y, 1.0/ray.vector.z};
   int stack[128]; int sp=0; stack[sp++]=0;
   while(sp){
      int ni=stack[--sp];
      const Node& n=nodes[ni];
      double t0=0, t1=1.0;
      for(int k=0;k<3;k++){
         double a=(n.lo[k]-o[k])*inv[k], b=(n.hi[k]-o[k])*inv[k];
         if(a>b){ double t=a; a=b; b=t; }
         if(a>t0) t0=a;
         if(b<t1) t1=b;
      }
      if(t0>t1) continue;
      if(n.right==-1){
         for(int i=n.start;i<n.start+n.count;i++)
            if(prims[i]->getLightIntersection(ray, fill)) return true;
      } else {
         stack[sp++]=ni+1;
         stack[sp++]=n.right;
      }
   }
   return false;
}
