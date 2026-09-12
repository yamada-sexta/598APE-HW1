#include "texture.h"

double interpolate(double a,double b,double x)
{
   double f=(1.0-cos(x * M_PI))* 0.5;
   return a*(1.0-f)+b*f;
}

Texture::Texture(double am, double op, double ref):ambient(am),opacity(op), reflection(ref){}

double fix(double a){
   return a - floor(a);
}

double rayAtan2(double y, double x){
#ifdef RAY_EXACT_TRIG
   return atan2(y, x);
#else
   const double absoluteY = fabs(y) + 1e-12;
   double ratio;
   double angle;
   if (x < 0.0) {
      ratio = (x + absoluteY) / (absoluteY - x);
      angle = 3.0 * M_PI / 4.0;
   } else {
      ratio = (x - absoluteY) / (x + absoluteY);
      angle = M_PI / 4.0;
   }
   angle += (0.1963 * ratio * ratio - 0.9817) * ratio;
   return y < 0.0 ? -angle : angle;
#endif
}

double ground(double e){
   return (e>1.)?1.:e;
}
const char* findExtension(const char* ola){
   const char* end = ola;
   while(*end!='\0') end++;
   const char* start = end;
   while(*start!='.' && start>ola) start--;
   if(*start=='.') start++;
   return start;
}

char lowerCase(char c){
   if ((c >= 'A') && (c <= 'Z')) 
      return c-'a'+'A'; 
   else 
      return c;
}
 
int fpeek(FILE *stream)
{
   int c;
   c = fgetc(stream);
   ungetc(c, stream);
   return c;
}
  
bool extensionEquals(const char* a, const char* knownExt){
   while(*a!='\0'){
      if(a[0] != knownExt[0]) 
         return false;
      a++;
      knownExt++;
   }
   return knownExt[0]=='\0';
}
