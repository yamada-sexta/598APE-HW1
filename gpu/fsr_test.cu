#include "fsr.cuh"
#include <vector>
#include <cstdio>
#include <stdexcept>
#include <string>
void check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
int main(){
  try{
    for(auto color:{0,1,64,128,254,255})for(auto size:{1,3,17}){
      int outW=size*2+1,outH=size+4;
      std::vector<unsigned char> input(size*size*3,color),output(outW*outH*3);
      unsigned char *src=nullptr,*dst=nullptr;float4* mid=nullptr;
      check(cudaMalloc(&src,input.size()));check(cudaMalloc(&dst,output.size()));check(cudaMalloc(&mid,outW*outH*sizeof(float4)));
      check(cudaMemcpy(src,input.data(),input.size(),cudaMemcpyHostToDevice));
      dim3 tile(16,16),grid((outW+15)/16,(outH+15)/16);
      fsr::easuKernel<<<grid,tile>>>(src,mid,size,size,outW,outH);check(cudaGetLastError());
      fsr::rcasKernel<<<grid,tile>>>(mid,dst,outW,outH,std::exp2(-.2f));check(cudaGetLastError());
      check(cudaMemcpy(output.data(),dst,output.size(),cudaMemcpyDeviceToHost));
      for(auto c:output)if(abs(int(c)-color)>1)throw std::runtime_error("Constant color changed");
      check(cudaFree(src));check(cudaFree(dst));check(cudaFree(mid));
    }
    int w=17,h=9,ow=34,oh=18;
    std::vector<unsigned char> input(w*h*3),output(ow*oh*3);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x)for(int c=0;c<3;++c)input[(y*w+x)*3+c]=x<8?32:224;
    unsigned char *src=nullptr,*dst=nullptr;float4* mid=nullptr;
    check(cudaMalloc(&src,input.size()));check(cudaMalloc(&dst,output.size()));check(cudaMalloc(&mid,ow*oh*sizeof(float4)));
    check(cudaMemcpy(src,input.data(),input.size(),cudaMemcpyHostToDevice));
    fsr::easuKernel<<<dim3(3,2),dim3(16,16)>>>(src,mid,w,h,ow,oh);check(cudaGetLastError());
    std::vector<float4> intermediate(ow*oh);
    check(cudaMemcpy(intermediate.data(),mid,intermediate.size()*sizeof(float4),cudaMemcpyDeviceToHost));
    for(auto v:intermediate)if(!std::isfinite(v.x)||v.x<32.f/255.f-1e-5f||v.x>224.f/255.f+1e-5f)throw std::runtime_error("EASU ringing or nonfinite result");
    fsr::rcasKernel<<<dim3(3,2),dim3(16,16)>>>(mid,dst,ow,oh,std::exp2(-.2f));check(cudaGetLastError());
    check(cudaMemcpy(output.data(),dst,output.size(),cudaMemcpyDeviceToHost));
    for(int i=0;i<ow*oh;++i)if(output[3*i]!=output[3*i+1]||output[3*i]!=output[3*i+2])throw std::runtime_error("Grayscale acquired a color cast");
    check(cudaFree(src));check(cudaFree(dst));check(cudaFree(mid));
    puts("FSR constants, borders, tiny/odd sizes, neutral edges and EASU anti-ringing: PASS");
  }catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());return 1;}
}
