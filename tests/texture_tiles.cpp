#include "src/Textures/imagetexture.h"
#include <cstdio>
#include <cstdlib>

static bool compare(ImageTexture& texture) {
   for (unsigned int y = 0; y < texture.h; ++y) {
      for (unsigned int x = 0; x < texture.w; ++x) {
         unsigned char a[3], b[3];
         double amA, opA, refA, amB, opB, refB;
         texture.getColor(a, &amA, &opA, &refA, x, y);
         texture.getColor(b, &amB, &opB, &refB,
                          (x + 0.5) / texture.w, (y + 0.5) / texture.h);
         const unsigned char* pixel = texture.imageData + 4 * (x + y * texture.w);
         for (int c = 0; c < 3; ++c)
            if (a[c] != pixel[c] || b[c] != pixel[c]) return false;
         if (amA != texture.ambient || refA != texture.reflection ||
             opA != pixel[3] * texture.opacity / 255. ||
             amA != amB || refA != refB || opA != opB) return false;
      }
   }
   return true;
}

int main() {
   setenv("RAY_TEXTURE_TILES", "1", 1);
   ImageTexture texture(65, 67);
   for (unsigned int y = 0; y < texture.h; ++y)
      for (unsigned int x = 0; x < texture.w; ++x)
         texture.setColor(x, y, x * 3, y * 5, x + y);
   texture.prepareRendering();
   if (!compare(texture)) return 1;
   texture.setColor(64, 66, 5, 6, 7);
   if (!compare(texture)) return 2;
   texture.prepareRendering();
   texture.maskImageAlpha();
   if (!compare(texture)) return 3;
   texture.prepareRendering();
   if (!compare(texture)) return 4;
   texture.imageData[0] = 123; 
   texture.prepareRendering();
   if (!compare(texture)) return 5;
   setenv("RAY_TEXTURE_TILES", "0", 1);
   texture.prepareRendering();
   if (!compare(texture)) return 6;
   std::puts("tiled/linear texture samples and invalidation passed");
}
