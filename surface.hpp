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
using glm::dot, glm::length, glm::normalize, glm::determinant;

class Surface {
  public:
    const Material & material;

	Surface(const Material & material)
	: material(material) { }

	virtual vec3 get_normal(const vec3 & intersection_point) const = 0;
	virtual optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir) const = 0;
};

class Triangle : public Surface {
  public:
	vec3 v0;
	vec3 v1;
	vec3 v2;
	vec3 normal;

	Triangle(vec3 v0, vec3 v1, vec3 v2, const Material & material)
	: v0(v0), v1(v1), v2(v2)
    , Surface(material) {
		recompute_normal();
	}

	void recompute_normal() {
 	   	vec3 e1 = v1-v0;
    	vec3 e2 = v2-v0;
    	normal = glm::normalize(glm::cross(e2, e1));
	}

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

template<class T>
void add_surface(vector<unique_ptr<Surface>> & surfaces, const T & surface) {
	surfaces.push_back(make_unique<T>(surface));
}

// TODO scene description should include color values, lights, camera position, dof params, bg colour, aspect ratio
// add command line args

vector<unique_ptr<Surface>> load_orange_plane() {
	using namespace Materials;
	vector<unique_ptr<Surface>> surfaces;

	add_surface(surfaces, Sphere(vec3(-1.5f, 0.0f, 0.0f), 0.75, mirror));
	add_surface(surfaces, Sphere(vec3( 1.5f, 0.0f, 0.0f), 0.75, glass));
	add_surface(surfaces, Sphere(vec3( 0.0f, 0.0f, 2.0f), 0.75, white));
	add_surface(surfaces, Sphere(vec3( 0.0f, -10.0f, 0.0f), 5, lamp));

	vec3 A = vec3(-100.0f,  0.75f, -100.0f);
	vec3 B = vec3( 100.0f,  0.75f, -100.0f);
	vec3 C = vec3(-100.0f,  0.75f,  100.0f);
	vec3 D = vec3( 100.0f,  0.75f,  100.0f);
	add_surface(surfaces, Triangle(C, B, A, orange));
	add_surface(surfaces, Triangle(C, D, B, orange));

	return surfaces;
}

vector<unique_ptr<Surface>> load_red_corner() {
	using namespace Materials;
	vector<unique_ptr<Surface>> surfaces;

	add_surface(surfaces, Sphere(vec3(0, -1, 2), 1, black));
	add_surface(surfaces, Sphere(vec3(-1.25, -1, 0), 1, mirror));
	add_surface(surfaces, Sphere(vec3(1.25, -1, 0), 1, glass));

	// Room
	vec3 A = vec3(-100.0f,  0, -10.0f);
	vec3 B = vec3( 100.0f,  0, -10.0f);
	vec3 C = vec3(-100.0f,  0,  3.0f);
	vec3 D = vec3( 100.0f,  0,  3.0f);
	vec3 E = vec3(-100.0f, -100.0f, -10.0f);
	vec3 F = vec3( 100.0f, -100.0f, -10.0f);
	vec3 G = vec3(-100.0f, -100.0f,  3.0f);
	vec3 H = vec3( 100.0f, -100.0f,  3.0f);

	// floor
	add_surface(surfaces, Triangle(C, B, A, red));
	add_surface(surfaces, Triangle(C, D, B, red));
	// back wall
	add_surface(surfaces, Triangle(G, D, C, dim_lamp));
	add_surface(surfaces, Triangle(G, H, D, dim_lamp));

    return surfaces;
}

vector<unique_ptr<Surface>> load_cornell() {
	using namespace Materials;
	vector<unique_ptr<Surface>> surfaces;

	// Room
	vec3 A = vec3(-1.0f, 1.0f, -1.0f);
	vec3 B = vec3(1.0f, 1.0f, -1.0f);
	vec3 C = vec3(-1.0f, 1.0f, 1.0f);
	vec3 D = vec3(1.0f, 1.0f, 1.0f);
	vec3 E = vec3(-1.0f, -1.0f, -1.0f);
	vec3 F = vec3(1.0f, -1.0f, -1.0f);
	vec3 G = vec3(-1.0f, -1.0f, 1.0f);
	vec3 H = vec3(1.0f, -1.0f, 1.0f);

	// Floor - green
	add_surface(surfaces, Triangle(C, B, A, green));
	add_surface(surfaces, Triangle(C, D, B, green));
	// Left wall - purple
	add_surface(surfaces, Triangle(A, E, C, purple));
	add_surface(surfaces, Triangle(C, E, G, purple));
	// Right wall - yellow
	add_surface(surfaces, Triangle(F, B, D, yellow));
	add_surface(surfaces, Triangle(H, F, D, yellow));
	// Ceiling - cyan
	add_surface(surfaces, Triangle(E, F, G, cyan));
	add_surface(surfaces, Triangle(F, H, G, cyan));
	// Back wall - white
	add_surface(surfaces, Triangle(G, D, C, dim_lamp));
	add_surface(surfaces, Triangle(G, H, D, dim_lamp));

	// Short block
	A = vec3(-0.04504504504504503f, 1.0f, -0.5891891891891892f);
	B = vec3(0.5315315315315315f, 1.0f, -0.7657657657657657f);
	C = vec3(0.1351351351351351f, 1.0f, -0.01981981981981984f);
	D = vec3(0.7045045045045045f, 1.0f, -0.18918918918918914f);
	E = vec3(-0.04504504504504503f, 0.4054054054054054f, -0.5891891891891892f);
	F = vec3(0.5315315315315315f, 0.4054054054054054f, -0.7657657657657657f);
	G = vec3(0.1351351351351351f, 0.4054054054054054f, -0.01981981981981984f);
	H = vec3(0.7045045045045045f, 0.4054054054054054f, -0.18918918918918914f);

	// Front
	add_surface(surfaces, Triangle(E, B, A, red));
	add_surface(surfaces, Triangle(E, F, B, red));
	// Front
	add_surface(surfaces, Triangle(F, D, B, red));
	add_surface(surfaces, Triangle(F, H, D, red));
	// BACK
	add_surface(surfaces, Triangle(H, C, D, red));
	add_surface(surfaces, Triangle(H, G, C, red));
	// LEFT
	add_surface(surfaces, Triangle(G, E, C, red));
	add_surface(surfaces, Triangle(E, A, C, red));
	// TOP
	add_surface(surfaces, Triangle(G, F, E, red));
	add_surface(surfaces, Triangle(G, H, F, red));

	// Tall block
	A = vec3(-0.5243243243243243f, 1.0f, -0.1099099099099099f);
	B = vec3(0.04504504504504503f, 1.0f, 0.06666666666666665f);
	C = vec3(-0.7009009009009008f, 1.0f, 0.463063063063063f);
	D = vec3(-0.13153153153153152f, 1.0f, 0.6432432432432433f);
	E = vec3(-0.5243243243243243f, -0.18918918918918926f, -0.1099099099099099f);
	F = vec3(0.04504504504504503f, -0.18918918918918926f, 0.06666666666666665f);
	G = vec3(-0.7009009009009008f, -0.18918918918918926f, 0.463063063063063f);
	H = vec3(-0.13153153153153152f, -0.18918918918918926f, 0.6432432432432433f);

	// Front
	add_surface(surfaces, Triangle(E, B, A, blue));
	add_surface(surfaces, Triangle(E, F, B, blue));
	// Front
	add_surface(surfaces, Triangle(F, D, B, blue));
	add_surface(surfaces, Triangle(F, H, D, blue));
	// BACK
	add_surface(surfaces, Triangle(H, C, D, blue));
	add_surface(surfaces, Triangle(H, G, C, blue));
	// LEFT
	add_surface(surfaces, Triangle(G, E, C, blue));
	add_surface(surfaces, Triangle(E, A, C, blue));
	// TOP
	add_surface(surfaces, Triangle(G, F, E, blue));
	add_surface(surfaces, Triangle(G, H, F, blue));

    return surfaces;
}

#endif // SURFACE
