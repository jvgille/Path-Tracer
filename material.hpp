#ifndef MATERIAL
#define MATERIAL
#include <glm/glm.hpp>

using glm::vec3, glm::dot;

double random_real(double, double);

class Material {
  public:
    const vec3 color;
    const vec3 emittance;

    Material(const vec3 & color, const vec3 & emittance = vec3(0,0,0))
    : color(color), emittance(emittance) { }

    virtual vec3 sample_pdf(const vec3 & normal, const vec3 & incident) const = 0;
    virtual vec3 brdf() const = 0;
};

class Diffuse : public Material {
  public:
    Diffuse(const vec3 & color, const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & _) const {
        float u = random_real(0, 1);
        float v = random_real(0, 1);
        float theta = 2*M_PI*u;
        float phi = acos(2*v-1);

        // convert spherical to cartesian
        vec3 direction{cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi)};

        // if dir is not in hemisphere around normal, flip it
        if (glm::dot(direction, normal) < 0) {
            direction *= -1;
        }
        return direction;
    }

    vec3 brdf() const {
        return 2.0f * this->color;
    }
};

class Mirror : public Material {
  public:
    Mirror(const vec3 & color = vec3(1,1,1), const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & incident) const {
        return incident - 2*dot(normal, incident)*normal;
    }

    vec3 brdf() const {
        return this->color;
    }
};

class Glass : public Material {
    const float refractive_index = 1.5f; // air is assumed index 1
  public:
    Glass(const vec3 & color = vec3(1,1,1), const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & _normal, const vec3 & incident) const {
        vec3 normal = _normal;
        float n1, n2;
        if (dot(normal, incident) < 0) { // going into the material
            n1 = 1;
            n2 = refractive_index;
        } else {
            normal *= -1;
            n1 = refractive_index;
            n2 = 1;
        }

        float n = n1/n2; // ratio of refractive indices

        float cosI = -dot(normal, incident);
        float sin2T = n*n*(1 - cosI*cosI);
        float cosT = sqrt(1 - sin2T);

        float reflectance;
        if (sin2T > 1) { // total internal reflection
            reflectance = 1;
        } else {
            float rOrth = (n1*cosI - n2*cosT)/(n1*cosI + n2*cosT);
            float rPar = (n2*cosI - n1*cosT)/(n2*cosI + n1*cosT);
            reflectance = (rOrth*rOrth + rPar*rPar)/2;
        }

        if (random_real(0,1) < reflectance) { // reflection
            return incident + 2*cosI*normal;
        } else { // refraction
            return n*incident + (n*cosI - cosT)*normal;
        }
    }

    vec3 brdf() const {
        return this->color;
    }
};

namespace Materials {
    Diffuse red    (vec3(0.95, 0.05, 0.05));
    Diffuse yellow (vec3(0.95, 0.95, 0.05));
    Diffuse green  (vec3(0.05, 0.95, 0.05));
    Diffuse cyan   (vec3(0.05, 0.95, 0.95));
    Diffuse blue   (vec3(0.05, 0.05, 0.95));
    Diffuse purple (vec3(0.95, 0.05, 0.95));
    Diffuse white  (vec3(0.95, 0.95, 0.95));

    Diffuse lamp(white.color, 1.5f*vec3(1.0f, 1.0f, 1.0f));

    Mirror mirror;
    Glass glass;
}

#endif // MATERIAL
