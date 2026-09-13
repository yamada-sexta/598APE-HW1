#pragma once
class Autonoma;
void gpuRender(Autonoma *scene, unsigned char *rgb, int width, int height);
void gpuSceneChanged();
void gpuShutdown();
