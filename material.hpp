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
};

class Diffuse : public Material {
  public:
    Diffuse(const vec3 & color, const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) { }

    vec3 sample_pdf(const vec3 & normal, const vec3 & incoming) const {
        float u = random_real(0, 1);
        float v = random_real(0, 1);
        float theta = glm::two_pi<float>()*u;
        float phi = acos(2*v-1);

        // convert spherical to cartesian
        vec3 direction{cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi)};

        // if dir is not in hemisphere around normal, flip it
        if (glm::dot(direction, normal) < 0) {
            direction *= -1;
        }
        return direction;
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
};

namespace Materials {
    Diffuse red    (vec3(0.75f, 0.15f, 0.15f));
    Diffuse yellow (vec3(0.75f, 0.75f, 0.15f));
    Diffuse green  (vec3(0.15f, 0.75f, 0.15f));
    Diffuse cyan   (vec3(0.15f, 0.75f, 0.75f));
    Diffuse blue   (vec3(0.15f, 0.15f, 0.75f));
    Diffuse purple (vec3(0.75f, 0.15f, 0.75f));
    Diffuse white  (vec3(0.75f, 0.75f, 0.75f));
    Diffuse pure_white(vec3(1, 1, 1));

    Diffuse lamp(white.color, 3.0f*vec3(1.0f, 1.0f, 0.75f));

    Mirror mirror;
}

#endif // MATERIAL
