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
    // TODO Implement Path Tracing Algorithm here

    Intersection intersection = intersect(ray);
    if (!intersection.happened) return Vector3f(0.0f);

    if (intersection.m->hasEmission()) return intersection.m->getEmission();

    Vector3f L_dir(0.0f), L_indir(0.0f);

    Intersection lightIntersection;
    float lightPDF;
    sampleLight(lightIntersection, lightPDF);
    Vector3f lightDirection = lightIntersection.coords - intersection.coords;
    float lightDistance = lightDirection.norm();
    lightDirection = normalize(lightDirection);
    Ray lightRay(intersection.coords, lightDirection);
    lightIntersection = intersect(lightRay);
    if (lightIntersection.happened && lightIntersection.obj->hasEmit()) {
        Vector3f albedo = intersection.m->eval(ray.direction, lightDirection, intersection.normal);
        L_dir = lightIntersection.m->getEmission() * albedo * dotProduct(intersection.normal, lightDirection) / (lightDistance * lightDistance * lightPDF);
    } else {
        L_dir = Vector3f(0.0f);
    }

    if (get_random_float() < RussianRoulette) {
        Vector3f outRayDirection = intersection.m->sample(ray.direction, intersection.normal);
        Ray outRay(intersection.coords, outRayDirection);
        Intersection outRayIntersection = intersect(outRay);
        if (outRayIntersection.happened && !outRayIntersection.m->hasEmission()) {
            L_indir = castRay(outRay, depth + 1);
        } else {
            L_indir = Vector3f(0.0f);
        }
    } else {
        L_indir = Vector3f(0.0f);
    }

    return L_dir + L_indir;
}