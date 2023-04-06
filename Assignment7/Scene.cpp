//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1);
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

int N = 99;
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = Scene::intersect(ray);
    if (!inter.happened)
        return Vector3f(0.0f, 0.0f, 0.0f);

	if (inter.m->hasEmission())
		return inter.m->getEmission();

    if (get_random_float() > RussianRoulette)
        return Vector3f(0.0f, 0.0f, 0.0f);

    Vector3f wi;
    wi = inter.m->sample(-ray.direction, inter.normal).normalized();
    Ray outRay(inter.coords, wi);
    Intersection outInter = Scene::intersect(outRay);

    if (!outInter.happened)
        return Vector3f(0.0f, 0.0f, 0.0f);

	Vector3f result;
    float pdf_inv = 1.0f / outInter.m->pdf(wi, -ray.direction, inter.normal);
    float prr_inv = 1.0f / RussianRoulette;

    Vector3f f_r = inter.m->eval(wi, -ray.direction, inter.normal);
    if (outInter.obj->hasEmit())
    {
        result += outInter.emit * f_r * dotProduct(wi, inter.normal) * pdf_inv * prr_inv;
    }
    else
    {
        Ray qRay(outInter.coords, -wi);
        result += castRay(qRay, depth + 1) * f_r * dotProduct(wi, inter.normal) * pdf_inv * prr_inv;
    }

    return result;
}

Vector3f Scene::castRay_Sample_Light(const Ray& ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	Intersection inter = Scene::intersect(ray);
	if (!inter.happened)
		return Vector3f(0.0f, 0.0f, 0.0f);

    if (inter.m->hasEmission())
	    return inter.m->getEmission();

    // Direction Light
    Intersection LigthInter;
    float pdf;
    sampleLight(LigthInter, pdf);

    Vector3f pToLight = LigthInter.coords - inter.coords;
    
    Vector3f pToLightDir = pToLight.normalized();
    float pToLightDistance = pToLight.norm();

    Ray ShadowRay(inter.coords, pToLightDir);
    Intersection shadowInterSection = Scene::intersect(ShadowRay);

	Vector3f directLight;
	if (shadowInterSection.distance - pToLightDistance > -0.0005f)
	{
        LigthInter.normal.normalized();
        float cos = dotProduct(pToLightDir, inter.normal);
        float cos_ = dotProduct(-pToLightDir, LigthInter.normal);
		Vector3f f_r = inter.m->eval(ray.direction, pToLightDir, inter.normal);

        directLight = LigthInter.emit * f_r * cos * cos_ / pToLightDistance / pToLightDistance / pdf;
    }

    Vector3f indirectLight;
    float ksi = get_random_float();
    if (ksi < RussianRoulette)
    {
		Vector3f wi;
		wi = inter.m->sample(ray.direction, inter.normal).normalized();
		Ray outRay(inter.coords, wi);
		Intersection outInter = Scene::intersect(outRay);
        if (outInter.happened && !outInter.m->hasEmission())
        {
			float pdf_inv = 1.0f / outInter.m->pdf(ray.direction, wi, inter.normal);
			float prr_inv = 1.0f / RussianRoulette;

            Vector3f f_r = inter.m->eval(ray.direction, wi, inter.normal);
            Ray qRay(outInter.coords, wi);
            indirectLight = castRay(qRay, depth + 1) * f_r * dotProduct(wi, inter.normal) * pdf_inv * prr_inv;
        }
    }

	return indirectLight + directLight;
}

Vector3f Scene::castRay_Sample_Light_2(const Ray& ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here

	Intersection inter = Scene::intersect(ray);
	if (!inter.happened)
		return Vector3f(0.0f, 0.0f, 0.0f);

	if (inter.m->hasEmission())
		return inter.m->getEmission();

    // Direction Light
	Vector3f directLight;

	Intersection LigthInter;
	float pdf;
	sampleLight(LigthInter, pdf);

	Vector3f pToLight = LigthInter.coords - inter.coords;
	Vector3f pToLightDir = pToLight.normalized();
	float pToLightDistance = pToLight.norm();

	Ray ShadowRay(inter.coords, pToLightDir);
	Intersection shadowInterSection = Scene::intersect(ShadowRay);

    if(shadowInterSection.distance - pToLightDistance > - 0.0005f)
    {
		LigthInter.normal.normalized();
		float cos = std::max(0.0f, dotProduct(pToLightDir, inter.normal));
		float cos_ = std::max(0.0f, dotProduct(-pToLightDir, LigthInter.normal));

		Vector3f f_r = inter.m->eval(pToLightDir, -ray.direction, inter.normal);
		directLight = LigthInter.emit * f_r * cos * cos_ / pToLightDistance / pToLightDistance / pdf;
	}

	Vector3f indirectLight;
	if (get_random_float() < RussianRoulette)
	{
		Vector3f wi = inter.m->sample(-ray.direction, inter.normal).normalized();
        float pdf = inter.m->pdf(wi, -ray.direction, inter.normal);

        if (pdf > 0.0005f)
        {
			Ray outRay(inter.coords, wi);
			Intersection outInter = Scene::intersect(outRay);
			if (outInter.happened && !outInter.m->hasEmission())
			{
				Vector3f f_r = inter.m->eval(wi, -ray.direction, inter.normal);
                float cos = std::max(0.f, dotProduct(wi, inter.normal));
				Ray qRay(outInter.coords, -wi);
				indirectLight = castRay(qRay, depth + 1) * f_r * cos / pdf / RussianRoulette;
			}
        }
	}

	return indirectLight + directLight;
}