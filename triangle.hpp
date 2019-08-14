#ifndef TRIANGLE
#define TRIANGLE

#include <glm/glm.hpp>
#include <vector>

#include "material.hpp"

using glm::vec3;

class Triangle {
  public:
	vec3 v0;
	vec3 v1;
	vec3 v2;
	vec3 normal;
    const Material & material;

	Triangle(vec3 v0, vec3 v1, vec3 v2, const Material & material)
	: v0(v0), v1(v1), v2(v2)
    , material(material) {
		recompute_normal();
	}

	void recompute_normal() {
 	   	vec3 e1 = v1-v0;
    	vec3 e2 = v2-v0;
    	normal = glm::normalize(glm::cross(e2, e1));
	}
};

// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1
std::vector<Triangle> load_cornell() {
	using namespace Materials;

    std::vector<Triangle> triangles;
	triangles.reserve(5*2*3); // 3 cubes with 5 faces and 2 triangles per face

	// Room
	float L = 555;			// Length of Cornell Box side.

	vec3 A(L,0,0);
	vec3 B(0,0,0);
	vec3 C(L,0,L);
	vec3 D(0,0,L);

	vec3 E(L,L,0);
	vec3 F(0,L,0);
	vec3 G(L,L,L);
	vec3 H(0,L,L);

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


	// Short block - used to be red

	A = vec3(290,0,114);
	B = vec3(130,0, 65);
	C = vec3(240,0,272);
	D = vec3( 82,0,225);

	E = vec3(290,165,114);
	F = vec3(130,165, 65);
	G = vec3(240,165,272);
	H = vec3( 82,165,225);

	// Front
	triangles.push_back( Triangle(E,B,A,mirror_2) );
	triangles.push_back( Triangle(E,F,B,mirror_2) );

	// Front
	triangles.push_back( Triangle(F,D,B,mirror_2) );
	triangles.push_back( Triangle(F,H,D,mirror_2) );

	// BACK
	triangles.push_back( Triangle(H,C,D,mirror_2) );
	triangles.push_back( Triangle(H,G,C,mirror_2) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,mirror_2) );
	triangles.push_back( Triangle(E,A,C,mirror_2) );

	// TOP
	triangles.push_back( Triangle(G,F,E,mirror_2) );
	triangles.push_back( Triangle(G,H,F,mirror_2) );

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec3(423,0,247);
	B = vec3(265,0,296);
	C = vec3(472,0,406);
	D = vec3(314,0,456);

	E = vec3(423,330,247);
	F = vec3(265,330,296);
	G = vec3(472,330,406);
	H = vec3(314,330,456);

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


	// ----------------------------------------------
	// scale to volume [-1,1]^3

	for(size_t i=0; i<triangles.size(); i++) {
		triangles[i].v0 *= 2/L;
		triangles[i].v1 *= 2/L;
		triangles[i].v2 *= 2/L;

		triangles[i].v0 -= vec3(1,1,1);
		triangles[i].v1 -= vec3(1,1,1);
		triangles[i].v2 -= vec3(1,1,1);

		triangles[i].v0.x *= -1;
		triangles[i].v1.x *= -1;
		triangles[i].v2.x *= -1;

		triangles[i].v0.y *= -1;
		triangles[i].v1.y *= -1;
		triangles[i].v2.y *= -1;

		triangles[i].recompute_normal();
	}

    return triangles;
}

#endif // TRIANGLE
