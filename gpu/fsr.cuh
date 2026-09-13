/*
Copyright (c) 2021 Advanced Micro Devices, Inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#pragma once
#include "types.cuh"
namespace fsr {
__device__ inline V min3(V a, V b) { return V(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)); }
__device__ inline V max3(V a, V b) { return V(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)); }
__device__ inline float lowRcp(float a) { return __uint_as_float(0x7ef07ebb - __float_as_uint(a)); }
__device__ inline float lowRsq(float a) {
  return __uint_as_float(0x5f347d74 - (__float_as_uint(a) >> 1));
}
__device__ inline float medRcp(float a) {
  float b = __uint_as_float(0x7ef19fff - __float_as_uint(a));
  return b * (-b * a + 2.f);
}
__device__ inline V load(const unsigned char *rgb, int w, int h, int x, int y) {
  size_t i = 3 * (size_t(min(h - 1, max(0, y))) * w + min(w - 1, max(0, x)));
  return V(rgb[i], rgb[i + 1], rgb[i + 2]) * (1.f / 255.f);
}
__device__ inline float luma(V c) { return .5f * c.z + (.5f * c.x + c.y); }
__device__ inline void edge(float2 &dir, float &len, float w, float a, float b, float c, float d,
                            float e) {
  float dc = d - c, cb = c - b, dx = d - b;
  float lx = __saturatef(fabsf(dx) * lowRcp(fmaxf(fabsf(dc), fabsf(cb))));
  dir.x += dx * w;
  len += lx * lx * w;
  float ec = e - c, ca = c - a, dy = e - a;
  float ly = __saturatef(fabsf(dy) * lowRcp(fmaxf(fabsf(ec), fabsf(ca))));
  dir.y += dy * w;
  len += ly * ly * w;
}
__device__ inline void tap(V &sum, float &weight, float x, float y, float2 dir, float2 len,
                           float lob, float clip, V color) {
  float vx = (x * dir.x + y * dir.y) * len.x, vy = (-x * dir.y + y * dir.x) * len.y;
  float d = fminf(vx * vx + vy * vy, clip);
  float b = .4f * d - 1.f, a = lob * d - 1.f;
  b *= b;
  a *= a;
  b = 1.5625f * b - .5625f;
  float w = b * a;
  sum = sum + color * w;
  weight += w;
}
__device__ inline V easu(const unsigned char *rgb, int w, int h, int outW, int outH, int x, int y) {
  float px = (x + .5f) * float(w) / outW - .5f, py = (y + .5f) * float(h) / outH - .5f;
  int ix = int(floorf(px)), iy = int(floorf(py));
  px -= ix;
  py -= iy;
  V b = load(rgb, w, h, ix, iy - 1), c = load(rgb, w, h, ix + 1, iy - 1);
  V e = load(rgb, w, h, ix - 1, iy), f = load(rgb, w, h, ix, iy), g = load(rgb, w, h, ix + 1, iy),
    hh = load(rgb, w, h, ix + 2, iy);
  V i = load(rgb, w, h, ix - 1, iy + 1), j = load(rgb, w, h, ix, iy + 1),
    k = load(rgb, w, h, ix + 1, iy + 1), l = load(rgb, w, h, ix + 2, iy + 1);
  V n = load(rgb, w, h, ix, iy + 2), o = load(rgb, w, h, ix + 1, iy + 2);
  float2 dir = make_float2(0, 0);
  float len = 0;
  edge(dir, len, (1 - px) * (1 - py), luma(b), luma(e), luma(f), luma(g), luma(j));
  edge(dir, len, px * (1 - py), luma(c), luma(f), luma(g), luma(hh), luma(k));
  edge(dir, len, (1 - px) * py, luma(f), luma(i), luma(j), luma(k), luma(n));
  edge(dir, len, px * py, luma(g), luma(j), luma(k), luma(l), luma(o));
  float d = dir.x * dir.x + dir.y * dir.y;
  bool zero = d < 1.f / 32768.f;
  float inv = zero ? 1.f : lowRsq(d);
  dir.x = zero ? 1.f : dir.x;
  dir.x *= inv;
  dir.y *= inv;
  len *= .5f;
  len *= len;
  float stretch = (dir.x * dir.x + dir.y * dir.y) * lowRcp(fmaxf(fabsf(dir.x), fabsf(dir.y)));
  float2 extent = make_float2(1 + (stretch - 1) * len, 1 - .5f * len);
  float lob = .5f + (.25f - .04f - .5f) * len, clip = lowRcp(lob);
  V low = min3(min3(f, g), min3(j, k)), high = max3(max3(f, g), max3(j, k)), sum{};
  float weight = 0;
  tap(sum, weight, 0 - px, -1 - py, dir, extent, lob, clip, b);
  tap(sum, weight, 1 - px, -1 - py, dir, extent, lob, clip, c);
  tap(sum, weight, -1 - px, 1 - py, dir, extent, lob, clip, i);
  tap(sum, weight, 0 - px, 1 - py, dir, extent, lob, clip, j);
  tap(sum, weight, 0 - px, 0 - py, dir, extent, lob, clip, f);
  tap(sum, weight, -1 - px, 0 - py, dir, extent, lob, clip, e);
  tap(sum, weight, 1 - px, 1 - py, dir, extent, lob, clip, k);
  tap(sum, weight, 2 - px, 1 - py, dir, extent, lob, clip, l);
  tap(sum, weight, 2 - px, 0 - py, dir, extent, lob, clip, hh);
  tap(sum, weight, 1 - px, 0 - py, dir, extent, lob, clip, g);
  tap(sum, weight, 1 - px, 2 - py, dir, extent, lob, clip, o);
  tap(sum, weight, 0 - px, 2 - py, dir, extent, lob, clip, n);
  return min3(high, max3(low, sum * (1.f / weight)));
}
__device__ inline V load(const float4 *p, int w, int h, int x, int y) {
  float4 c = p[size_t(min(h - 1, max(0, y))) * w + min(w - 1, max(0, x))];
  return V(c.x, c.y, c.z);
}
__device__ inline float channelLobe(float low, float high, float center) {
  float a = high > 0 ? fminf(low, center) / (4 * high) : 0;
  float b = low < 1 ? (1 - fmaxf(high, center)) / (4 * low - 4) : 0;
  return fmaxf(-a, b);
}
__device__ inline V rcas(const float4 *p, int w, int h, int x, int y, float sharpness) {
  V b = load(p, w, h, x, y - 1), d = load(p, w, h, x - 1, y), e = load(p, w, h, x, y),
    f = load(p, w, h, x + 1, y), hh = load(p, w, h, x, y + 1);
  V low = min3(min3(b, d), min3(f, hh)), high = max3(max3(b, d), max3(f, hh));
  float lobe = fmaxf(channelLobe(low.x, high.x, e.x),
                     fmaxf(channelLobe(low.y, high.y, e.y), channelLobe(low.z, high.z, e.z)));
  lobe = fmaxf(-.1875f, fminf(lobe, 0.f)) * sharpness;
  return (b * lobe + d * lobe + hh * lobe + f * lobe + e) * medRcp(4 * lobe + 1);
}
__global__ void easuKernel(const unsigned char *input, float4 *output, int w, int h, int outW,
                           int outH) {
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= outW || y >= outH)
    return;
  V c = easu(input, w, h, outW, outH, x, y);
  output[size_t(y) * outW + x] = make_float4(c.x, c.y, c.z, 1);
}
__global__ void rcasKernel(const float4 *input, unsigned char *output, int w, int h,
                           float sharpness) {
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h)
    return;
  V c = rcas(input, w, h, x, y, sharpness);
  size_t i = 3 * (size_t(y) * w + x);
  output[i] = __float2uint_rn(__saturatef(c.x) * 255.f);
  output[i + 1] = __float2uint_rn(__saturatef(c.y) * 255.f);
  output[i + 2] = __float2uint_rn(__saturatef(c.z) * 255.f);
}
} 
