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
    auto inter = intersect(ray);
    if (!inter.happened) return Vector3f();
    if (inter.m->hasEmission()) return inter.m->getEmission();

    Intersection lightInter;
    float lightPdf = 0.0f;
    sampleLight(lightInter, lightPdf);
    Vector3f p2light = lightInter.coords - inter.coords;
    Vector3f p2lightDir = p2light.normalized();
    float p2lightPow = p2light.norm() * p2light.norm();

    Vector3f l_dir; // 直接光照

    Ray p2lightRay(inter.coords, p2lightDir);
    Intersection t = intersect(p2lightRay);
    // 由于浮点精度问题可能判断为遮挡,导致光线较暗
    if (t.distance > p2light.norm() - 0.0001)
    {
        // Li * fr * (cosΘ - cosΘ') / p2l_disatance^2
        l_dir = lightInter.emit * inter.m->eval(ray.direction, p2lightDir, inter.normal) 
            * dotProduct(p2lightDir, inter.normal) 
            * dotProduct(-p2lightDir, lightInter.normal) 
            / p2lightPow / lightPdf;
    }

    // 轮盘赌判断失败,不继续trace
    if (get_random_float() > RussianRoulette) return l_dir;

    Vector3f l_indir; // 间接光照
    // 抽样产生反射ray,交点为q
    Vector3f p2qDir = inter.m->sample(ray.direction, inter.normal).normalized();
    Ray p2qRay(inter.coords, p2qDir);
    Intersection p2qInter = intersect(p2qRay);
    if (p2qInter.happened && !p2qInter.m->hasEmission())
    {
        float pdf = inter.m->pdf(ray.direction, p2qDir, inter.normal);
        l_indir = castRay(p2qRay, depth + 1) 
            * inter.m->eval(ray.direction, p2qDir, inter.normal) 
            * dotProduct(p2qDir, inter.normal)
            / pdf / RussianRoulette; // 0.8的概率轮盘赌成功
    }

    return l_dir + l_indir;
}