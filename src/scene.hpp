#ifndef SCENE
#define SCENE

#include <memory>

#include "surface.hpp"
#include "camera.hpp"

template<class T>
void add_surface(vector<unique_ptr<Surface>> & surfaces, const T & surface) {
	surfaces.push_back(make_unique<T>(surface));
}

// TODO aspect ratio
struct Scene {
    Camera camera;
    vec3 BG_COLOR;
	vector<unique_ptr<Surface>> surfaces;

    bool DOF_on;
    float DOF_aperture_size;
    float DOF_focal_length;

    Scene(Camera & camera, vec3 BG_COLOR, vector<unique_ptr<Surface>> & surfaces,
          bool DOF_on, float aperture_size = 0, float focal_length = 0)
    : camera(camera), BG_COLOR(BG_COLOR), surfaces(move(surfaces))
    , DOF_on(DOF_on), DOF_aperture_size(aperture_size), DOF_focal_length(focal_length) {
    }

};

namespace Scenes {

Scene test() {
    vector<unique_ptr<Surface>> surfaces;

	for (int i = 0; i <= 10; i++) {
		float v = i/10.0;
		Material color(MaterialType::DIFFUSE, vec3(1, v, 0));
		add_surface(surfaces, Sphere(vec3(i, -0.5, 0), 0.5, color));
	}

    Camera camera(vec3(5,0,-7), vec2(0,0), SCREEN_HEIGHT);
    Scene scene(camera, vec3(1,1,1), surfaces, false);
	return scene;
}

Scene orange_plane() {
    Material orange(MaterialType::DIFFUSE, vec3(1.00, 0.40, 0.00));
    Material white(MaterialType::DIFFUSE, vec3(1.00, 1.00, 1.00));
    Material lamp(MaterialType::DIFFUSE, white.color, 5.0f*vec3(1.0f, 1.0f, 1.0f));
    Material mirror(MaterialType::MIRROR);
    Material glass(MaterialType::GLASS);

    vector<unique_ptr<Surface>> surfaces;
	add_surface(surfaces, Sphere(vec3(-1.5f, 0.0f, 0.0f), 0.75, mirror));
	add_surface(surfaces, Sphere(vec3( 1.5f, 0.0f, 0.0f), 0.75, glass));
	add_surface(surfaces, Sphere(vec3( 0.0f, 0.0f, 2.0f), 0.75, white));
	add_surface(surfaces, Sphere(vec3( 0.0f, -10.0f, 0.0f), 5, lamp));
	add_surface(surfaces, Triangle(vec3(-100, 0.75,  100),
                                   vec3(100, 0.75, 100),
                                   vec3(0, 0.75, -100),
                                   orange));

    Camera camera(vec3(0,0,-3), vec2(0,0), SCREEN_HEIGHT);
    Scene scene(camera, vec3(0,0,0), surfaces, true, 0.15, 3);
	return scene;
}

Scene red_corner() {
    Material red   (MaterialType::DIFFUSE, vec3(1.00, 0.05, 0.05));
    Material black (MaterialType::DIFFUSE, vec3(0.05, 0.05, 0.05));
    Material lamp  (MaterialType::DIFFUSE, vec3(1,1,1), 1.5f*vec3(1.0f, 1.0f, 1.0f));
    Material mirror(MaterialType::MIRROR);
    Material glass (MaterialType::GLASS);

    vector<unique_ptr<Surface>> surfaces;
	add_surface(surfaces, Sphere(vec3(0, -1, 2), 1, black));
	add_surface(surfaces, Sphere(vec3(-1.25, -1, 0), 1, mirror));
	add_surface(surfaces, Sphere(vec3(1.25, -1, 0), 1, glass));
	add_surface(surfaces, Triangle(vec3(0,0,-100), vec3(-10,0,3), vec3(10,0,3), red));
	add_surface(surfaces, Triangle(vec3(0,-100,3), vec3(-10,0,3), vec3(10,0,3), lamp));

    Camera camera(vec3(0,-4.5,-3), vec2(-M_PI_4,0), SCREEN_HEIGHT);
    Scene scene(camera, vec3(0,0,0), surfaces, false);
	return scene;
}

Scene sunrise() {
	Material orange  (MaterialType::DIFFUSE, vec3(1.00, 0.40, 0.00));
    Material white   (MaterialType::DIFFUSE, vec3(0.95, 0.95, 0.95));
    Material black   (MaterialType::DIFFUSE, vec3(0.05, 0.05, 0.05));

    vector<unique_ptr<Surface>> surfaces;
	add_surface(surfaces, Triangle(vec3(-100,0,100), vec3(100,0,100), vec3(0,0,-100), black));
	add_surface(surfaces, Sphere(vec3(-2, -1, 0), 1, white));
	add_surface(surfaces, Sphere(vec3(-1, -1, 7), 1, orange));

    Camera camera(vec3(0,-1,-2), vec2(0,5.78), SCREEN_HEIGHT);
    Scene scene(camera, vec3(0.7,0.7,1), surfaces, true, 0.2, 9);
	return scene;
}

Scene cornell() {
	Material red     (MaterialType::DIFFUSE, vec3(1.00, 0.05, 0.05));
    Material yellow  (MaterialType::DIFFUSE, vec3(1.00, 1.00, 0.05));
    Material green   (MaterialType::DIFFUSE, vec3(0.05, 1.00, 0.05));
    Material cyan    (MaterialType::DIFFUSE, vec3(0.05, 1.00, 1.00));
    Material blue    (MaterialType::DIFFUSE, vec3(0.05, 0.05, 1.00));
    Material purple  (MaterialType::DIFFUSE, vec3(1.00, 0.05, 1.00));
    Material white   (MaterialType::DIFFUSE, vec3(1.00, 1.00, 1.00));
    Material black   (MaterialType::DIFFUSE, vec3(0.05, 0.05, 0.05));
    Material lamp    (MaterialType::DIFFUSE, white.color, 1.5f*vec3(1.0f, 1.0f, 1.0f));

    Material mirror(MaterialType::MIRROR);
    Material glass(MaterialType::GLASS);

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
	add_surface(surfaces, Triangle(G, D, C, white));
	add_surface(surfaces, Triangle(G, H, D, white));

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

    Camera camera(vec3(0,0,-3), vec2(0,0), SCREEN_HEIGHT);
    Scene scene(camera, vec3(1,1,1), surfaces, false);
	return scene;
}

} // namespace Scenes

#endif // SCENE
