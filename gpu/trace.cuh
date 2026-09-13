#pragma once
#include "types.cuh"
#ifdef GPU_OPTIX_DEVICE
#include <optix_device.h>
#endif
constexpr float FAR_T = 1.e30f;
__device__ inline float wrap(float x) { return x - floorf(x); }
struct Sample {
  V color;
  float opacity, reflection, ambient;
};
__device__ inline Sample sample(const Scene &s, int id, float u, float v) {
  Material m = s.materials[id];
  V rgb = m.color;
  float op = m.opacity;
  if (m.offset >= 0) {
    int x = min(m.w - 1, max(0, int(wrap(u) * m.w)));
    int y = min(m.h - 1, max(0, int(wrap(v) * m.h)));
    uchar4 c = s.texels[m.offset + y * m.w + x];
    rgb = V(c.x, c.y, c.z);
    op *= float(c.w) / 255.f;
  }
  return {rgb, op, m.reflection, m.ambient};
}
__device__ inline Sample surface(const Scene &s, const Primitive &p, V point, bool shadow = false) {
  V v = point - p.center;
  if (p.kind == SPHERE) {
    float vertical = (p.radius - v.y) / (2.f * p.radius), angle = atan2f(v.z, v.x);
    if (shadow)
      return sample(s, p.texture, (p.yaw + vertical) / 6.28318530718f / p.tx,
                    wrap(p.pitch / 6.28318530718f - angle) / p.ty);
    return sample(s, p.texture, (p.yaw + angle) / 6.28318530718f / p.tx,
                  (p.pitch / 6.28318530718f - vertical) / p.ty);
  }
  return sample(s, p.texture, p.inverseRight.dot(v) / p.tx - .5f, p.inverseUp.dot(v) / p.ty - .5f);
}
__device__ inline V normalAt(const Scene &s, const Primitive &p, V point) {
  V v = point - p.center, norm = p.kind == SPHERE ? v : p.normal;
  if (p.normalMap < 0)
    return norm.unit();
  V right = p.right, up = p.up;
  float u, w;
  if (p.kind == SPHERE) {
    norm = norm.unit();
    right = V(norm.x, norm.z, -norm.y);
    up = V(norm.z, norm.y, -norm.x);
    u = (2 * p.ox + atan2f(v.z, v.x)) / 6.28318530718f / p.mx;
    w = (2 * p.oy / 6.28318530718f - (p.radius - v.y) / (2 * p.radius)) / p.my;
  } else {
    u = p.inverseRight.dot(v) / p.mx - .5f + p.ox;
    w = p.inverseUp.dot(v) / p.my - .5f + p.oy;
  }
  V c = sample(s, p.normalMap, u, w).color;
  return (right * (c.x - 128) + up * (c.y - 128) + norm * c.z).unit();
}
__device__ inline float intersect(const Primitive &p, V o, V v) {
  V delta = o - p.center;
  if (p.kind == SPHERE) {
    float a = v.dot(v), b = v.dot(delta), c = delta.dot(delta) - p.radius * p.radius;
    float disc = b * b - a * c;
    if (disc < 0)
      return FAR_T;
    float r = sqrtf(disc), t = (-b - r) / a;
    if (t <= 0)
      t = (-b + r) / a;
    return t > 0 ? t : FAR_T;
  }
  float t = -(p.normal.dot(delta)) / p.normal.dot(v);
  if (!(t > 0) || t >= FAR_T)
    return FAR_T;
  if (p.kind == PLANE)
    return t;
  V q = delta + v * t;
  float x = p.inverseRight.dot(q), y = p.inverseUp.dot(q);
  if (p.kind == BOX && (fabsf(x) > p.tx * .5f || fabsf(y) > p.ty * .5f))
    return FAR_T;
  if (p.kind == DISK && x * x / (p.tx * p.tx) + y * y / (p.ty * p.ty) > 1)
    return FAR_T;
  if (p.kind == TRIANGLE) {
    bool a = (p.third - x) * p.ty + (p.third - p.tx) * (y - p.ty) < 0;
    if (a != (p.tx * y < 0) || a != (x * p.ty - p.third * y < 0))
      return FAR_T;
  }
  return t;
}
__device__ inline bool bounds(const Node &n, V o, V v, float limit) {
  float low = 0, high = limit;
  for (int axis = 0; axis < 3; ++axis) {
    float a = axis == 0 ? n.lo.x : axis == 1 ? n.lo.y : n.lo.z;
    float b = axis == 0 ? n.hi.x : axis == 1 ? n.hi.y : n.hi.z;
    float orig = axis == 0 ? o.x : axis == 1 ? o.y : o.z;
    float dir = axis == 0 ? v.x : axis == 1 ? v.y : v.z;
    if (dir == 0) {
      if (orig < a || orig > b)
        return false;
    } else {
      float t1 = (a - orig) / dir, t2 = (b - orig) / dir;
      low = fmaxf(low, fminf(t1, t2));
      high = fminf(high, fmaxf(t1, t2));
    }
  }
  return high >= low;
}
__device__ inline int closest(const Scene &s, V o, V v, float &time) {
  int hit = -1;
  time = FAR_T;
  if (s.brute) {
    for (int i = 0; i < s.count; ++i) {
      float t = intersect(s.primitives[i], o, v);
      if (t < time) {
        time = t;
        hit = i;
      }
    }
    return hit;
  }
  for (int j = 0; j < s.planeCount; ++j) {
    int i = s.planes[j];
    float t = intersect(s.primitives[i], o, v);
    if (t < time) {
      time = t;
      hit = i;
    }
  }
#ifdef GPU_OPTIX_DEVICE
  if (s.traversable) {
    unsigned int id = hit, bits = __float_as_uint(time);
    optixTrace(s.traversable, make_float3(o.x, o.y, o.z), make_float3(v.x, v.y, v.z), 1e-7f, time,
               0.f, 255, OPTIX_RAY_FLAG_DISABLE_ANYHIT, 0, 2, 0, id, bits);
    hit = (int)id;
    time = __uint_as_float(bits);
  }
  return hit;
#else
  int stack[64], top = 0;
  if (s.nodeCount)
    stack[top++] = 0;
  while (top) {
    Node n = s.nodes[stack[--top]];
    if (!bounds(n, o, v, time))
      continue;
    if (n.count) {
      for (int j = 0; j < n.count; ++j) {
        int i = s.indices[n.start + j];
        float t = intersect(s.primitives[i], o, v);
        if (t < time || (t == time && i < hit)) {
          time = t;
          hit = i;
        }
      }
    } else {
      stack[top++] = n.right;
      stack[top++] = n.left;
    }
  }
  return hit;
#endif
}
__device__ inline bool occludes(const Scene &s, int i, V o, V v, float &red) {
  const Primitive &p = s.primitives[i];
  float t = intersect(p, o, v);
  if (t >= 1)
    return false;
  if (p.kind != SPHERE && s.materials[p.texture].opacity > 1.f - 1e-6f)
    return true;
  Sample c = surface(s, p, p.kind == PLANE ? o : o + v * t, true);
  if (c.opacity > 1.f - 1e-6f)
    return true;
  red *= c.color.x / 255.f;
  return false;
}
__device__ inline float visibility(const Scene &s, V o, V v, float red) {
  if (s.brute) {
    for (int i = 0; i < s.count; ++i)
      if (occludes(s, i, o, v, red))
        return 0;
    return red;
  }
  for (int j = 0; j < s.planeCount; ++j)
    if (occludes(s, s.planes[j], o, v, red))
      return 0;
#ifdef GPU_OPTIX_DEVICE
  if (s.traversable) {
    unsigned int bits = __float_as_uint(red), unused = 0;
    optixTrace(s.traversable, make_float3(o.x, o.y, o.z), make_float3(v.x, v.y, v.z), 1e-7f, 1.f,
               0.f, 255, OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT, 1, 2, 1, bits, unused);
    red = __uint_as_float(bits);
  }
  return red;
#else
  int stack[64], top = 0;
  if (s.nodeCount)
    stack[top++] = 0;
  while (top) {
    Node n = s.nodes[stack[--top]];
    if (!bounds(n, o, v, 1.f))
      continue;
    if (n.count) {
      for (int j = 0; j < n.count; ++j)
        if (occludes(s, s.indices[n.start + j], o, v, red))
          return 0;
    } else {
      stack[top++] = n.right;
      stack[top++] = n.left;
    }
  }
  return red;
#endif
}
__device__ inline V sky(const Scene &s, V direction) {
  V d = direction.unit();
  return sample(s, s.sky, atan2f(d.z, d.x) / 6.28318530718f, fabsf(d.y)).color;
}
__device__ inline V shade(const Scene &s, const Primitive &p, V point, V normal, Sample m) {
  float light = 0;
  for (int i = 0; i < s.lightCount; ++i) {
    Lamp l = s.lights[i];
    V v = l.center - point;
    float cosine = normal.dot(v.unit());
    if (p.kind != SPHERE)
      cosine = fabsf(cosine);
    if (cosine > 0)
      light = fminf(1.f, light + cosine * visibility(s, point + v * .01f, v, l.red));
  }
  return m.color * (m.ambient + light * (1 - m.ambient));
}
__device__ inline V quantize(V c) {
  return V((unsigned char)c.x, (unsigned char)c.y, (unsigned char)c.z);
}
__device__ __noinline__ V trace(const Scene &s, V o, V v, int depth) {
  float t;
  int hit = closest(s, o, v, t);
  if (hit < 0)
    return sky(s, v);
  const Primitive &p = s.primitives[hit];
  V point = o + v * t, normal = normalAt(s, p, point);
  Sample m = surface(s, p, point);
  V color = quantize(shade(s, p, point, normal, m));
  if (depth < s.depth) {
    if (m.opacity < 1.f - 1e-6f)
      color =
          quantize(color * m.opacity + trace(s, point + v * 1e-4f, v, depth + 1) * (1 - m.opacity));
    if (m.reflection > 1e-6f) {
      V d = v - normal * (2 * normal.dot(v));
      color = quantize(color * (1 - m.reflection) +
                       trace(s, point + d * 1e-4f, d, depth + 1) * m.reflection);
    }
  }
  return color;
}

struct PendingRay {
  V o, v;
  float weight;
  int depth;
};
__device__ inline V traceIterative(const Scene &s, V o, V v, float cutoff) {
  PendingRay stack[12];
  int top = 0, depth = 0;
  float weight = 1;
  V color{};
  for (;;) {
    float t;
    int hit = closest(s, o, v, t);
    if (hit < 0)
      color = color + sky(s, v) * weight;
    else {
      const Primitive &p = s.primitives[hit];
      V point = o + v * t, normal = normalAt(s, p, point);
      Sample m = surface(s, p, point);
      float op = depth < s.depth ? m.opacity : 1.f, ref = depth < s.depth ? m.reflection : 0.f;
      color = color + shade(s, p, point, normal, m) * (weight * op * (1 - ref));
      float transmit = weight * (1 - op) * (1 - ref), reflect = weight * ref;
      V reflected = v - normal * (2 * normal.dot(v));
      if (transmit > cutoff && reflect > cutoff)
        stack[top++] = {point + reflected * 1e-4f, reflected, reflect, depth + 1};
      if (transmit > cutoff) {
        o = point + v * 1e-4f;
        weight = transmit;
        ++depth;
        continue;
      }
      if (reflect > cutoff) {
        o = point + reflected * 1e-4f;
        v = reflected;
        weight = reflect;
        ++depth;
        continue;
      }
    }
    if (!top)
      break;
    PendingRay next = stack[--top];
    o = next.o;
    v = next.v;
    weight = next.weight;
    depth = next.depth;
  }
  return color;
}
