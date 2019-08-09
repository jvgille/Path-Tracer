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

    Material(vec3 color, vec3 emittance = vec3(0,0,0))
    : color(color), emittance(emittance) {

    }
};

namespace Materials {
    Material red    (vec3(0.75f, 0.15f, 0.15f));
    Material yellow (vec3(0.75f, 0.75f, 0.15f));
    Material green  (vec3(0.15f, 0.75f, 0.15f));
    Material cyan   (vec3(0.15f, 0.75f, 0.75f));
    Material blue   (vec3(0.15f, 0.15f, 0.75f));
    Material purple (vec3(0.75f, 0.15f, 0.75f));
    Material white  (vec3(0.75f, 0.75f, 0.75f));

    //Material lamp = cyan;
    Material lamp(vec3(0.75f, 0.75f, 0.75f), 3.0f*vec3(1.0f, 1.0f, 0.75f));
}

#endif // MATERIAL
