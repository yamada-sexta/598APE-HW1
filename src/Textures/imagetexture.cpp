#include "imagetexture.h"
#include <cstdlib>

void ImageTexture::invalidateRenderCache(){
   tiledReady = false;
   tiledData.clear();
}

const unsigned char* ImageTexture::cachedPixel(unsigned int x, unsigned int y) const{
   const Tile& tile = tiledData[(size_t)(y >> 2) * tiledWidth + (x >> 2)];
   return tile.texels + (((y & 3u) << 2) + (x & 3u)) * 4u;
}

void ImageTexture::prepareRendering(){
   tiledReady = false;
   tiledData.clear();
   const char* enabled = std::getenv("RAY_TEXTURE_TILES");
   if((enabled != NULL && enabled[0] == '0') || w == 0 || h == 0 ||
      (size_t)w * h < 4096 || imageData == NULL) return;
   tiledWidth = (w + 3u) >> 2;
   const unsigned int tiledHeight = (h + 3u) >> 2;
   tiledData.resize((size_t)tiledWidth * tiledHeight);
   for(unsigned int y = 0; y < h; ++y){
      for(unsigned int x = 0; x < w; ++x){
         unsigned char* dst = tiledData[(y >> 2) * tiledWidth + (x >> 2)].texels +
            (((y & 3u) << 2) + (x & 3u)) * 4u;
         const unsigned char* src = imageData + ((size_t)y * w + x) * 4u;
         dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2]; dst[3] = src[3];
      }
   }
   tiledReady = true;
}

void ImageTexture::getColor(unsigned char* toFill, double* am, double *op, double *ref, double x, double y){
   int xi = (int)(x*w), yi = (int)(y*h);
   int p1 = 4*(xi+w*yi);
   const unsigned char* pixel = (tiledReady && xi >= 0 && yi >= 0 &&
      (unsigned int)xi < w && (unsigned int)yi < h) ?
      cachedPixel((unsigned int)xi, (unsigned int)yi) : imageData + p1;
   toFill[0] = pixel[0];
   toFill[1] = pixel[1];
   toFill[2] = pixel[2];
   *op = pixel[3]*opacity/255.;
   *ref = reflection;
   *am = ambient;
}

void ImageTexture::maskImageAlpha(){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         {
            imageData[total+3]=imageData[total];
            imageData[total]=255;
            imageData[total+1]=255;
            imageData[total+2]=255;
         }
      }          
}


void ImageTexture::maskImage(unsigned char r, unsigned char g, unsigned char b, unsigned char rm, unsigned char gm, unsigned char bm, unsigned char m){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==r && imageData[total+1]==g && imageData[total+2]==b)
         {
            imageData[total]=rm;
            imageData[total+1]=gm;
            imageData[total+2]=bm;
            imageData[total+3]=m;
         }
      }          
}

void ImageTexture::maskImageA(unsigned char r, unsigned char g, unsigned char b, unsigned char m){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]>=r && imageData[total+1]>=g && imageData[total+2]>=b)
         {
            imageData[total+3]=m;
         }
      }          
}
void ImageTexture::maskImageU(unsigned char r, unsigned char g, unsigned char b, unsigned char m){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]<=r && imageData[total+1]<=g && imageData[total+2]<=b)
         {
            imageData[total+3]=m;
         }
      }          
}
void ImageTexture::maskImage(unsigned char r, unsigned char g, unsigned char b, unsigned char m){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==r && imageData[total+1]==g && imageData[total+2]==b){
            imageData[total+3]=m;
         }
      }          
}
void ImageTexture::maskImage(unsigned char r, unsigned char g, unsigned char b){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==r && imageData[total+1]==g && imageData[total+2]==b){
            imageData[total+3]=0;
         }
      }          
}

void ImageTexture::maskImage(ColorTexture b, unsigned char m){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==b.r && imageData[total+1]==b.g && imageData[total+2]==b.b){
            imageData[total+3]=m;
         }
      }          
}
void ImageTexture::maskImage(ColorTexture b){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==b.r && imageData[total+1]==b.g && imageData[total+2]==b.b){
            imageData[total+3]=0;
         }
      }          
}


void ImageTexture::maskImage(ColorTexture* b, unsigned char m){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==b->r && imageData[total+1]==b->g && imageData[total+2]==b->b){
            imageData[total+3]=m;
         }
      }          
}
void ImageTexture::maskImage(ColorTexture* b){
invalidateRenderCache();
int x,y;
   for(y = h-1; y>=0; y--)
      for(x = 0; x<w; x++){
         int total = 4*(x+y*w);
         if(imageData[total]==b->r && imageData[total+1]==b->g && imageData[total+2]==b->b){
            imageData[total+3]=0;
         }
      }          
}


void ImageTexture::getColor(unsigned char* toFill, double* am, double *op, double *ref,unsigned int x, unsigned int y){
   int start = 4*(x+w*y);
   const unsigned char* pixel = (tiledReady && x < w && y < h) ? cachedPixel(x, y) : imageData + start;
   toFill[0] = pixel[0];
   toFill[1] = pixel[1];
   toFill[2] = pixel[2];
   *op = pixel[3]*opacity/255.;
   *ref = reflection;
   *am = ambient;
}

unsigned char* ImageTexture::setColor(unsigned int x, unsigned int y, unsigned char* data){
   invalidateRenderCache();
   int start = 4*(x+w*y);
   imageData[start] = data[0];
   imageData[start+1] = data[1];
   imageData[start+2] = data[2];
   return &imageData[start];
}


unsigned char* ImageTexture::setColor(unsigned int x, unsigned int y, unsigned char r, unsigned char g, unsigned char b){
   invalidateRenderCache();
   int start = 4*(x+w*y);
   imageData[start] = r;
   imageData[start+1] = g;
   imageData[start+2] = b;
   return &imageData[start];
}

ImageTexture::ImageTexture(unsigned int ww, unsigned int hh):Texture(.3, 1., 0.){
   w =ww;
   h = hh;
   imageData = (unsigned char*)malloc(4*w*h*sizeof(unsigned char));
   int i;
   for(i = 0; i<w*h; i++){ imageData[i*4+3] = 255; }
} 
ImageTexture::ImageTexture(unsigned char* data, unsigned int ww, unsigned int hh):Texture(.3, 1., 0.){
   imageData = data;
   w =ww;
   h = hh;
}

void ImageTexture::readPPM(FILE* f, const char* file){
   invalidateRenderCache();
   if (f == NULL){
      printf("File loading error!!! %s\n", file);
      exit(0);
   }
   int fchar = getc(f);
   if(fchar!='P'){
      printf("Header error --1st char not 'P' %s %c %d\n", file, fchar, fchar);
      exit(0);
   }
   int id = getc(f);
   while(fpeek(f)=='#'){
      int rr;
      do{
         rr = getc(f);
      } while(rr!='\n');
   }
   int x = 0, y = 0;
   if(id=='6'){
      int r = fscanf(f, "%u %u", &w, &h);
      if ( r < 2 ) {
         printf("Could not find width / height -6- %d %d %d\n", r, w, h);
         exit(0);
      }
      int ne = fpeek(f);
      while(ne == ' ' || ne=='\n' || ne=='\t'){ getc(f); ne = fpeek(f); }
      int d;
      r = fscanf(f, "%u", &d);
      if ( (r < 1) || ( d != 255 ) ){
         printf("Illegal max size %u %u", d, r);
         exit(0);
      }
      ne = fpeek(f);
      while(ne == ' ' || ne=='\n' || ne=='\t'){ getc(f); ne = fpeek(f); }
      imageData = (unsigned char*)malloc(4*w*h*(sizeof(unsigned char)));
      for(y = h-1; y>=0; y--)
         for(x = 0; x<w; x++){
            int total = 4*(x+y*w);
            imageData[total]=getc(f);
            imageData[total+1]=getc(f);
            imageData[total+2]=getc(f);
            imageData[total+3] = 255;
         }
   }
   else if(id=='3'){
      int ne = fpeek(f);
      while(ne == ' ' || ne=='\n' || ne=='\t'){ getc(f); ne = fpeek(f); }
      while(fpeek(f)=='#'){
         int rr;
         do{
            rr = getc(f);
         } while(rr!='\n');
      }
      int r = fscanf(f, "%u %u", &w, &h);
      if ( r < 2 ) {
         printf("Could not find width / height -3- %d %d %d\n", r, w, h);
         exit(0);
      }
      int d;
      r = fscanf(f, "%u", &d);
      if ( (r < 1) || ( d != 255 ) ){
         printf("Illegal max size %d %d %d %d", d, r, w, d);
         exit(0);
      }
      fseek(f, 1, SEEK_CUR); /* skip one byte, should be whitespace */
      id = getc(f);
      if(fpeek(f)=='#'){
         int rr;
         do{
            rr = getc(f);
         } while(rr!='\n');
      }
      imageData = (unsigned char*)malloc(4*w*h*(sizeof(unsigned char)));
         
      for(y = h-1; y>=0; y--)
         for(x = 0; x<w; x++){
            int total = 4*(x+y*w);
            unsigned int tmp;
            if (fscanf(f, "%u", &tmp) == EOF) {
               printf("Could not read byte\n");
               exit(1);
            }
            imageData[total] = (unsigned char)tmp;
            if (fscanf(f, "%u", &tmp) == EOF) {
               printf("Could not read byte\n");
               exit(1);
            }
            imageData[total+1] = (unsigned char)tmp;
            if (fscanf(f, "%u", &tmp) == EOF) {
               printf("Could not read byte\n");
               exit(1);
            }
            imageData[total+2] = (unsigned char)tmp;
            imageData[total+3] = 255;
         }
   }
   else{
      
      printf("Unknown PPM FILE!?\n");
      exit(0);
   }


}
ImageTexture::ImageTexture(const char* file):Texture(.3, 1., 0.){
   const char* ext = findExtension(file);
   if(extensionEquals(ext, "ppm")){
   
      FILE* f = fopen(file, "r");
      readPPM(f, file);
      fclose(f);
   }
   else{
      char command[2000];
      snprintf(command, sizeof(command), "magick %s ppm:-", file);
      FILE* f = popen(command, "r");
      readPPM(f, file);
      pclose(f);
   }


}
