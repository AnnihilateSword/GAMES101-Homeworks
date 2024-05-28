//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Intersection intersection = intersect(ray);
    if (!intersection.happened)
        return Vector3f();
    if (intersection.m->hasEmission())
        return intersection.m->getEmission();

    // Direct Light
    Vector3f L_dir{};
    Intersection L_dir_Inter;
    float pdf_light{};

    // 在场景的所有 光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度

    sampleLight(L_dir_Inter, pdf_light);
    // view point
    Vector3f p = intersection.coords;
    // light source position
    Vector3f x = L_dir_Inter.coords;
    // incident direction
    Vector3f wo = ray.direction;
    // light direction
    Vector3f ws = (x - p).normalized();
    Ray p_2_light_ray(p, ws);
    Intersection p_2_light_inter = intersect(p_2_light_ray);
    if (p_2_light_inter.distance - (x - p).norm() > -0.005f)
    {
        //给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值（这里和上课说的相反）

        Vector3f f_r = intersection.m->eval(wo, ws, intersection.normal);
        float distance2 = dotProduct(x - p, x - p);
        // L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
        L_dir = L_dir_Inter.emit * f_r * dotProduct(ws, intersection.normal) * 
                dotProduct(-ws, L_dir_Inter.normal) / distance2 / pdf_light;
    }

    // Indirect Light
    Vector3f L_indir{};
    if (get_random_float() > RussianRoulette)
        return L_dir;

    //按照该 材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向

    Vector3f wi = (intersection.m->sample(wo, intersection.normal)).normalized();
    Ray L_indir_Ray(p, wi);
    Intersection L_indir_Inter = intersect(L_indir_Ray);
    if (L_indir_Inter.happened && !L_indir_Inter.m->hasEmission())
    {
        //给定一对入射、出射方向与法向量，计算 sample 方法得到该出射 方向的概率密度

        float pdf = intersection.m->pdf(wo, wi, intersection.normal);
        // avoid white noise
        if (pdf > 0.00001f)
            L_indir = castRay(L_indir_Ray, depth + 1) * 
                        L_indir_Inter.m->eval(wo, wi, intersection.normal) * 
                        dotProduct(wi, intersection.normal) / pdf / RussianRoulette;
    }
    // 返回值限制 [0,1]，降低 BRDF 材质噪点

    return Vector3f::Min(Vector3f::Max(L_dir + L_indir, Vector3f(0.0f)), Vector3f(1.0f));
}