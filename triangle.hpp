#ifndef TRIANGLE
#define TRIANGLE

#include <glm/glm.hpp>
#include <vector>

#include "material.hpp"

using glm::vec3;

class Surface {
  public:
    const Material & material;

	Surface(const Material & material)
	: material(material) { }

	virtual vec3 get_normal(const vec3 & intersection_point) const = 0;
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
};

class Sphere : public Surface {
  public:
	vec3 center;
	float radius;

	Sphere(vec3 center, float radius, const Material & material)
	: center(center), radius(radius)
    , Surface(material) { }

	vec3 get_normal(const vec3 & intersection_point) const {
		return glm::normalize(intersection_point-center);
	}
};

std::vector<Triangle> load_test() {
	using namespace Materials;
	std::vector<Triangle> triangles;

	vec3 A(-1, 1,-1);
	vec3 B( 1, 1,-1);
	vec3 C(-1, 1, 1);
	vec3 D( 1, 1, 1);
	vec3 E(-1,-1,-1);
	vec3 F( 1,-1,-1);
	vec3 G(-1,-1, 1);
	vec3 H( 1,-1, 1);

	triangles.push_back(Triangle(A,E,C,white));
	triangles.push_back(Triangle(C,E,G,white));
	triangles.push_back(Triangle(F,B,D,lamp));
	triangles.push_back(Triangle(H,F,D,lamp));

	return triangles;
}

// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1
std::vector<Triangle> load_cornell() {
	using namespace Materials;

    std::vector<Triangle> triangles;
	triangles.reserve(5*2*3); // 3 cubes with 5 faces and 2 triangles per face

	// Room
	vec3 A = vec3(-1.0f, 1.0f, -1.0f);
	vec3 B = vec3(1.0f, 1.0f, -1.0f);
	vec3 C = vec3(-1.0f, 1.0f, 1.0f);
	vec3 D = vec3(1.0f, 1.0f, 1.0f);
	vec3 E = vec3(-1.0f, -1.0f, -1.0f);
	vec3 F = vec3(1.0f, -1.0f, -1.0f);
	vec3 G = vec3(-1.0f, -1.0f, 1.0f);
	vec3 H = vec3(1.0f, -1.0f, 1.0f);

	// Floor:
	triangles.push_back( Triangle( C, B, A, green ) );
	triangles.push_back( Triangle( C, D, B, green ) );
	// Left wall
	triangles.push_back( Triangle( A, E, C, purple ) );
	triangles.push_back( Triangle( C, E, G, purple ) );
	// Right wall
	triangles.push_back( Triangle( F, B, D, yellow ) );
	triangles.push_back( Triangle( H, F, D, yellow ) );
	// Ceiling - used to be cyan
	triangles.push_back( Triangle( E, F, G, lamp ) );
	triangles.push_back( Triangle( F, H, G, lamp ) );
	// Back wall
	triangles.push_back( Triangle( G, D, C, white ) );
	triangles.push_back( Triangle( G, H, D, white ) );


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
	triangles.push_back( Triangle(E,B,A,red) );
	triangles.push_back( Triangle(E,F,B,red) );
	// Front
	triangles.push_back( Triangle(F,D,B,red) );
	triangles.push_back( Triangle(F,H,D,red) );
	// BACK
	triangles.push_back( Triangle(H,C,D,red) );
	triangles.push_back( Triangle(H,G,C,red) );
	// LEFT
	triangles.push_back( Triangle(G,E,C,red) );
	triangles.push_back( Triangle(E,A,C,red) );
	// TOP
	triangles.push_back( Triangle(G,F,E,red) );
	triangles.push_back( Triangle(G,H,F,red) );


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
	triangles.push_back( Triangle(E,B,A,blue) );
	triangles.push_back( Triangle(E,F,B,blue) );
	// Front
	triangles.push_back( Triangle(F,D,B,blue) );
	triangles.push_back( Triangle(F,H,D,blue) );
	// BACK
	triangles.push_back( Triangle(H,C,D,blue) );
	triangles.push_back( Triangle(H,G,C,blue) );
	// LEFT
	triangles.push_back( Triangle(G,E,C,blue) );
	triangles.push_back( Triangle(E,A,C,blue) );
	// TOP
	triangles.push_back( Triangle(G,F,E,blue) );
	triangles.push_back( Triangle(G,H,F,blue) );

    return triangles;
}

#endif // TRIANGLE
