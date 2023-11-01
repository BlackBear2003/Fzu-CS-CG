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
    // TODO Use BVH.cpp's Intersect function instead of the current traversal method
	// Intersection dmin, tmp;
	// for (auto i : objects) {
	// 	tmp = i->getIntersection(ray);
	// 	dmin = dmin.distance > tmp.distance ? tmp : dmin;
	// }
	// return dmin; 
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
    Vector3f L_dir = 0.0, L_indir = 0.0;

    Intersection pInter = intersect(ray);

    if (!pInter.happened) {
        return 0.0;
    }

    Vector3f N = pInter.normal.normalized();
    Vector3f p = pInter.coords;

    Intersection xInter;
    float pdf_light = 0.0f;
    sampleLight(xInter, pdf_light);

    Vector3f x = xInter.coords;
    Vector3f NN = xInter.normal.normalized();
    Vector3f emit = xInter.emit;
    Vector3f ws = (x - p).normalized();

    Ray p2x(p, ws);

    Intersection p2xInter = intersect(p2x);
    auto pdf = pInter.m->pdf(ray.direction, ws, N);
    if (p2xInter.happened && p2xInter.distance - (x - p).norm() > -EPSILON) {
        L_dir = emit * pInter.m->eval(ray.direction, ws, N) * dotProduct(ws, N)
                * dotProduct(-ws, NN) / ((x-p).norm() * (x-p).norm()) / pdf_light;
    }

    if(get_random_float() > RussianRoulette) {
        return L_dir;
    }

    Vector3f wi = pInter.m->sample(ray.direction, N).normalized();

    Ray r(p, wi);

    Intersection rInter = intersect(r);

    if (rInter.happened && !rInter.m->hasEmission() &&  pInter.m->pdf(ray.direction, wi, N) > EPSILON ) {
        L_indir = castRay(r, depth+1) * pInter.m->eval(ray.direction, wi, N)
                  * dotProduct(wi, N) / pInter.m->pdf(ray.direction, wi, N) / RussianRoulette;
    }

    return pInter.m->getEmission() + L_dir + L_indir;

}

