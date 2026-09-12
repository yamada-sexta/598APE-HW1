#ifndef __IMAGE_TEXTURE_H__
#define __IMAGE_TEXTURE_H__
#include "colortexture.h"
#include <vector>

class ImageTexture: public Texture{
/** from 0 to 1 **/
   struct alignas(64) Tile { unsigned char texels[64]; };
   std::vector<Tile> tiledData;
   unsigned int tiledWidth = 0;
   bool tiledReady = false;
   void invalidateRenderCache();
   const unsigned char* cachedPixel(unsigned int x, unsigned int y) const;
public:
   unsigned int w, h;
   unsigned char* imageData;
   void getColor(unsigned char* toFill, double* am, double* op, double* ref, double x, double y);
   void getColor(unsigned char* toFill, double* am, double *op, double* ref, unsigned int x, unsigned int y);
   ImageTexture(unsigned char* data, unsigned int ww, unsigned int hh);
   ImageTexture(unsigned int ww, unsigned int hh);
   ImageTexture(const char* file);
   void prepareRendering();
   unsigned char* setColor(unsigned int x, unsigned int y, unsigned char* data);
   unsigned char* setColor(unsigned int x, unsigned int y, unsigned char r, unsigned char g, unsigned char b);
   void readPPM(FILE* f, const char* file);
   void maskImage(unsigned char r, unsigned char g, unsigned char b);
   void maskImage(unsigned char r, unsigned char g, unsigned char b, unsigned char rm, unsigned char gm, unsigned char mb,unsigned char m);
   void maskImage(unsigned char r, unsigned char g, unsigned char b, unsigned char m);
      void maskImageU(unsigned char r, unsigned char g, unsigned char b, unsigned char m);
      void maskImageA(unsigned char r, unsigned char g, unsigned char b, unsigned char m);
   void maskImage(ColorTexture b);
   void maskImage(ColorTexture b, unsigned char m);
   void maskImage(ColorTexture* b);
   void maskImage(ColorTexture* b, unsigned char m);
   void maskImageAlpha();
};

#endif
