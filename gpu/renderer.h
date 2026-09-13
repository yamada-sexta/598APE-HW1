#pragma once
class Autonoma;
void gpuStartWarmup();
void gpuConfigureFsr(const char *mode, float sharpnessStops);
void gpuRender(Autonoma *scene, unsigned char *rgb, int width, int height);
void gpuSceneChanged();
void gpuShutdown();
