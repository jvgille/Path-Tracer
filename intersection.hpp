class Intersection;

#ifndef INTERSECTION
#define INTERSECTION

#include <glm/glm.hpp>
#include <functional>


#include "surface.hpp"

struct Intersection {
    glm::vec3 point;
    float distance;
    std::reference_wrapper<const Surface> surface;
};

#endif
