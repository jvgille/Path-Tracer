#ifndef MATERIAL
#define MATERIAL
#include <glm/glm.hpp>

using glm::vec3;

double random_real(double, double);

class Material {
  public:
    const vec3 color;
    const vec3 emittance;

    Material(const vec3 & color, const vec3 & emittance = vec3(0,0,0))
    : color(color), emittance(emittance) { }

    virtual vec3 sample_pdf(const vec3 & normal, const vec3 & incoming) const = 0;
    virtual vec3 brdf() const = 0;
};

class Diffuse : public Material {
  public:
    Diffuse(const vec3 & color, const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & incoming) const {
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
    // ratio diffuse/specular

    Mirror(const vec3 & color = vec3(1,1,1))
    : Material(color) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & incoming) const {
        return incoming - 2*glm::dot(normal, incoming)*normal;
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
}

#endif // MATERIAL
