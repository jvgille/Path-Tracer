#ifndef MATERIAL
#define MATERIAL
#include <glm/glm.hpp>

using glm::vec3, glm::dot;

double random_real(double, double);

enum MaterialType {
    DIFFUSE, MIRROR, GLASS
};

vec3 sample_diffuse(const vec3 & normal) {
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

vec3 sample_mirror(const vec3 & normal, const vec3 & incident) {
    return incident - 2*dot(normal, incident)*normal;
}

const float air_refractive_index = 1.0f;
const float glass_refractive_index = 1.5f;

vec3 sample_glass(vec3 normal, const vec3 & incident) {
    float n1, n2;
    if (dot(normal, incident) < 0) { // going into the material
        n1 = air_refractive_index;
        n2 = glass_refractive_index;
    } else {
        normal *= -1;
        n1 = glass_refractive_index;
        n2 = air_refractive_index;
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

class Material {
  public:
    const MaterialType type;
    const vec3 color;
    const vec3 emittance;

    Material(MaterialType type, const vec3 & color = vec3(1,1,1), const vec3 & emittance = vec3(0,0,0))
    : type(type), color(color), emittance(emittance) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & incident) const {
        switch (type) {
            case DIFFUSE:
                return sample_diffuse(normal);
            case MIRROR:
                return sample_mirror(normal, incident);
            case GLASS:
                return sample_glass(normal, incident);
        }
    }

    vec3 brdf(const vec3 & normal, const vec3 & reflected) const {
        switch (type) {
            case DIFFUSE:
                return 2.0f * this->color * dot(reflected, normal);
            case MIRROR: case GLASS:
                return this->color;
        }
    }
};

namespace Materials {
    Material red     (MaterialType::DIFFUSE, vec3(1.00, 0.05, 0.05));
    Material orange  (MaterialType::DIFFUSE, vec3(1.00, 0.40, 0.00));
    Material yellow  (MaterialType::DIFFUSE, vec3(1.00, 1.00, 0.05));
    Material green   (MaterialType::DIFFUSE, vec3(0.05, 1.00, 0.05));
    Material cyan    (MaterialType::DIFFUSE, vec3(0.05, 1.00, 1.00));
    Material blue    (MaterialType::DIFFUSE, vec3(0.05, 0.05, 1.00));
    Material purple  (MaterialType::DIFFUSE, vec3(1.00, 0.05, 1.00));
    Material white   (MaterialType::DIFFUSE, vec3(1.00, 1.00, 1.00));
    Material black   (MaterialType::DIFFUSE, vec3(0.05, 0.05, 0.05));
    Material lamp    (MaterialType::DIFFUSE, white.color, 5.0f*vec3(1.0f, 1.0f, 1.0f));
    Material dim_lamp(MaterialType::DIFFUSE, white.color, 1.5f*vec3(1.0f, 1.0f, 1.0f));

    Material mirror(MaterialType::MIRROR);
    Material glass(MaterialType::GLASS);
}

/* class Material {
  public:
    const vec3 color;
    const vec3 emittance;

    Material(const vec3 & color, const vec3 & emittance = vec3(0,0,0))
    : color(color), emittance(emittance) { }

    virtual vec3 sample_pdf(const vec3 & normal, const vec3 & incident) const = 0;
    virtual vec3 brdf(const vec3 & normal, const vec3 & incident, const vec3 & reflected) const = 0;
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

    vec3 brdf(const vec3 & normal, const vec3 & incident, const vec3 & reflected) const {
        return 2.0f * this->color * dot(reflected, normal);
    }
};

class Mirror : public Material {
  public:
    Mirror(const vec3 & color = vec3(1,1,1), const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & incident) const {
        return incident - 2*dot(normal, incident)*normal;
    }

    vec3 brdf(const vec3 & normal, const vec3 & incident, const vec3 & reflected) const {
        return this->color;
    }
};

class Glass : public Material {
    const float refractive_index = 1.5f;
    const float air_refractive_index = 1.0f;
  public:
    Glass(const vec3 & color = vec3(1,1,1), const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & _normal, const vec3 & incident) const {
        vec3 normal = _normal;
        float n1, n2;
        if (dot(normal, incident) < 0) { // going into the material
            n1 = air_refractive_index;
            n2 = refractive_index;
        } else {
            normal *= -1;
            n1 = refractive_index;
            n2 = air_refractive_index;
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

    vec3 brdf(const vec3 & normal, const vec3 & incident, const vec3 & reflected) const {
        return this->color;
    }
};

namespace Materials {
    Diffuse red    (vec3(1.00, 0.05, 0.05));
    Diffuse orange (vec3(1.00, 0.40, 0.00));
    Diffuse yellow (vec3(1.00, 1.00, 0.05));
    Diffuse green  (vec3(0.05, 1.00, 0.05));
    Diffuse cyan   (vec3(0.05, 1.00, 1.00));
    Diffuse blue   (vec3(0.05, 0.05, 1.00));
    Diffuse purple (vec3(1.00, 0.05, 1.00));
    Diffuse white  (vec3(1.00, 1.00, 1.00));
    Diffuse black  (vec3(0.05, 0.05, 0.05));

    Diffuse lamp(white.color, 5.0f*vec3(1.0f, 1.0f, 1.0f));
    Diffuse dim_lamp(white.color, 1.5f*vec3(1.0f, 1.0f, 1.0f));

    Mirror mirror;
    Glass glass;
}*/

#endif // MATERIAL
