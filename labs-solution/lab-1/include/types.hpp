#pragma once
struct CameraConfig {
    int width;
    int height;
    int fps_target;
    CameraConfig(int w, int h, int f);
};