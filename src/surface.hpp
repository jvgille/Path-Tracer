class Surface;
class Triangle;
class Sphere;

#ifndef SURFACE
#define SURFACE

#include <glm/glm.hpp>
#include <vector>
#include <memory>

#include "material.hpp"
#include "intersection.hpp"
#include "constants.hpp"

using namespace std;
using glm::vec3;
using glm::dot, glm::cross, glm::length, glm::normalize, glm::determinant;

class Surface {
  public:
    const Material material;

	Surface(const Material & material)
	: material(material) { }

	virtual vec3 get_normal(const vec3 & intersection_point) const = 0;
	virtual optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir) const = 0;
};

class Triangle : public Surface {
  public:
	const vec3 v0;
	const vec3 v1;
	const vec3 v2;
	const vec3 normal;

	Triangle(vec3 v0, vec3 v1, vec3 v2, const Material & material)
	: v0(v0), v1(v1), v2(v2)
	, normal(normalize(cross(v2-v0, v1-v0)))
    , Surface(material) { }

	vec3 get_normal(const vec3 & _) const {
		return normal;
	}

	optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir) const {
		vec3 e1 = this->v1 - this->v0;
		vec3 e2 = this->v2 - this->v0;

		mat3 A(-ray_dir, e1, e2);
		float det_A = determinant(A);
		if (-EPSILON < det_A && det_A < EPSILON)
			return optional<Intersection>();

		vec3 b = ray_start - this->v0;
		float u = determinant(mat3(-ray_dir, b, e2))/det_A;
		if (u < -EPSILON || u >= 1)
			return optional<Intersection>();

		float v = determinant(mat3(-ray_dir, e1, b))/det_A;
		if (v < -EPSILON || u+v >= 1)
			return optional<Intersection>();

		float t = determinant(mat3(b, e1, e2))/det_A;
		if (t < EPSILON)
			return optional<Intersection>();

		vec3 intersection_point = ray_start + t*ray_dir;
		float distance = length(t*ray_dir);

		return optional<Intersection>(Intersection{intersection_point, distance, *this});
	}
};

class Sphere : public Surface {
  public:
	const vec3 position;
	const float radius;
	const float radius2;

	Sphere(vec3 position, float radius, const Material & material)
	: position(position), radius(radius), radius2(radius*radius)
    , Surface(material) { }

	vec3 get_normal(const vec3 & intersection_point) const {
		return glm::normalize(intersection_point-position);
	}

	// derived from scratchapixel
	optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir) const {
		vec3 L = this->position - ray_start;
        float tca = dot(L, ray_dir);
        if (tca < 0)
			return optional<Intersection>();

        float d2 = dot(L, L) - tca * tca;
        if (d2 > this->radius2)
			return optional<Intersection>();

        float thc = sqrt(this->radius2 - d2);

		float t = tca - thc; // try first intersection first

        if (t < EPSILON) {
            t = tca + thc; // first one behind, try second
            if (t < EPSILON)
				return optional<Intersection>(); // both behind
        }

		return optional<Intersection>(Intersection{ray_start + t*ray_dir, t, *this});
	}
};

#endif // SURFACE
