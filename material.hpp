#ifndef MATERIAL
#define MATERIAL
#include <glm/glm.hpp>

using glm::vec3;

class Material {
  public:
    const vec3 color;
    const vec3 emittance;
    //float specular_ratio; //?
    // bsdf
    bool is_mirror;

    Material(const vec3 & color, const vec3 & emittance = vec3(0,0,0), bool mirror = false)
    : color(color), emittance(emittance), is_mirror(mirror) {

    }
};

class Mirror : public Material {
  public:
    // ratio diffuse/specular

    Mirror(const vec3 & color = vec3(1,1,1), const vec3 & emittance = vec3(0,0,0))
    : Material(color, emittance) {

    }

    vec3 sample_pdf(const vec3 & normal, vec3 incoming) { // todo ref to surface
        return incoming - 2*glm::dot(normal, incoming)*normal;
    }
};

namespace Materials {
    Material red    (vec3(0.75f, 0.15f, 0.15f));
    Material yellow (vec3(0.75f, 0.75f, 0.15f));
    Material green  (vec3(0.15f, 0.75f, 0.15f));
    Material cyan   (vec3(0.15f, 0.75f, 0.75f));
    Material blue   (vec3(0.15f, 0.15f, 0.75f));
    Material purple (vec3(0.75f, 0.15f, 0.75f));
    //Material white  (vec3(0.75f, 0.75f, 0.75f));
    Material white  (vec3(1, 1, 1));

    //Material lamp = cyan;
    Material lamp(white.color, 3.0f*vec3(1.0f, 1.0f, 0.75f));

    Material mirror_2(vec3(1, 0.9, 0.9), vec3(0,0,0), true);

    Mirror mirror;
}

#endif // MATERIAL
