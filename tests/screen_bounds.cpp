#define main rayProgramMain
#include "../main.cpp"
#undef main
#include <vector>

int main() {
   W = 301; H = 257; 
   std::vector<unsigned char> full((size_t)W * H * 3), bounded(full.size());
   ColorTexture material(180, 120, 90, 1., 0., .3);
   Autonoma scene(Camera(Vector(0, 0, 0)));
   for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
         scene.addShape(new Sphere(Vector((x-8)*.08, (y-8)*.08, 6+(x%3)*.05),
                                   &material, 0, 0, 0, .04));
   scene.buildAcceleration();
   for (int workers : {1, 4}) {
#ifdef _OPENMP
      omp_set_num_threads(workers);
#endif
      for (int pose = 0; pose < 7; ++pose) {
         scene.camera.focus = Vector(0, 0, 0);
         scene.camera.setAngles(0, 0, 0);
         if (pose == 1) scene.camera.setYaw(3.141592653589793); 
         if (pose == 2) scene.camera.focus = Vector(0, 0, 6); 
         if (pose == 3) scene.camera.focus = Vector(100, 0, 0); 
         if (pose == 4) scene.camera.setAngles(.06, .03, .5); 
         if (pose == 5) { 
            scene.camera.right = Vector(1.4, .1, .05);
            scene.camera.up = Vector(.1, .8, 0);
         }
         if (pose == 6) scene.camera.up = scene.camera.right; 
         DATA = full.data();
         setenv("RAY_SCREEN_BOUNDS", "0", 1);
         refresh(&scene);
         DATA = bounded.data();
         setenv("RAY_SCREEN_BOUNDS", "1", 1);
         refresh(&scene);
         if (full != bounded) {
            std::fprintf(stderr, "screen bounds mismatch: pose %d workers %d\n", pose, workers);
            return 1;
         }
      }
   }
   DATA = NULL;
#ifdef _OPENMP
   std::puts("screen/full rendering matches for 7 camera cases with 1 and 4 workers");
#else
   std::puts("screen/full rendering matches for 7 camera cases without OpenMP");
#endif
}
