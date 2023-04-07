//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"
#include <thread>
#include<mutex>

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

    void RenderMultithread(const Scene& scene);
    static void RenderRow(const Scene& scene, std::vector<Vector3f>& framebuffer, const Vector3f& eye_pos, int beginRowRndex, int endRowRndex, float imageAspectRatio, float scale);
private:
    static int spp;
    static int currentRowIndex;
    static std::mutex RowIndexMutex;

    int thread_count = 16;
    std::vector<std::thread> thread_pools;
};
