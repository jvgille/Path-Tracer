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
    float DOF_focal_length;
    float DOF_aperture_size;

    Scene(Camera & camera, vec3 BG_COLOR, vector<unique_ptr<Surface>> & surfaces,
          bool DOF_on, float aperture_size = 0.15, float focal_length = 2)
    : camera(camera), BG_COLOR(BG_COLOR), surfaces(move(surfaces))
    , DOF_on(DOF_on), DOF_aperture_size(aperture_size), DOF_focal_length(focal_length) {
    }

};

namespace Scenes {

Scene dof_demo() {
    vector<unique_ptr<Surface>> surfaces;

	Material redish(MaterialType::DIFFUSE, vec3(1, 0, 0.5));
	Material blueish(MaterialType::DIFFUSE, vec3(0.5, 0, 1));
	Material white(MaterialType::DIFFUSE, vec3(1, 1, 1), 0.5f*vec3(1,1,1));
	Material red(MaterialType::DIFFUSE, vec3(1, 0, 0), 1.0f*vec3(1,0,0));
	Material blue(MaterialType::DIFFUSE, vec3(0, 0, 1), 1.0f*vec3(0,0,1));
	Material purple(MaterialType::DIFFUSE, vec3(1, 0, 1), 1.0f*vec3(1,0,1));

	float r = 1000;
	add_surface(surfaces, Sphere(vec3(0, r, 0), r, white)); // floor

	add_surface(surfaces, Sphere(vec3(-1, -1, 1), 1, blueish));
	add_surface(surfaces, Sphere(vec3(1.5, -1, 2), 1, redish));
	add_surface(surfaces, Sphere(vec3(-35,-20, 50), 1, red));
	add_surface(surfaces, Sphere(vec3(3,  -12, 50), 1, blue));
	add_surface(surfaces, Sphere(vec3(35, -16, 50), 1, purple));


    Camera camera(vec3(0,-1,-3), vec2(0,0), SCREEN_HEIGHT);
    Scene scene(camera, 0.1f*vec3(1,1,1), surfaces, true, 0.3, 5);
	return scene;
}

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
    Scene scene(camera, vec3(0,0,0), surfaces, true, 0.15, 5); // 3
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

Scene cornell_2() {
	Material red   (MaterialType::DIFFUSE, vec3(1.00, 0.05, 0.05));
    Material blue  (MaterialType::DIFFUSE, vec3(0.05, 0.05, 1.00));
    Material purple(MaterialType::DIFFUSE, vec3(1   ,0.05,1   ));
    Material yellow(MaterialType::DIFFUSE, vec3(1   ,1,   0.05));
    Material cyan  (MaterialType::DIFFUSE, vec3(0.05,1,   1   ));
    Material white (MaterialType::DIFFUSE, vec3(1   ,1,   1   ));
    Material light (MaterialType::DIFFUSE, vec3(1,1,1), 2.0f*vec3(1,1,1));
	Material glass(MaterialType::GLASS);
    Material mirror(MaterialType::MIRROR);

    vector<unique_ptr<Surface>> surfaces;

	add_surface(surfaces, Triangle(vec3(-2,1,-1), vec3(0,1,10), vec3(2,1,-1), red));
	add_surface(surfaces, Triangle(vec3(-1,-2,-1), vec3(-1,0,10), vec3(-1,2,-1), purple));
	add_surface(surfaces, Triangle(vec3(1,-2,-1), vec3(1,2,-1), vec3(1,0,10), yellow));
	add_surface(surfaces, Triangle(vec3(-2,-1,-1), vec3(2,-1,-1), vec3(0,-1,10), cyan));
	add_surface(surfaces, Triangle(vec3(-2,1,1), vec3(0,-10,1), vec3(2,1,1), white));

	add_surface(surfaces, Sphere(vec3(-0.32,0.6,0.26),0.4, mirror));
	add_surface(surfaces, Sphere(vec3(0.35,0.6,-0.39),0.4, glass));

	add_surface(surfaces, Sphere(vec3(0,-1.8,0),1, light));

    Camera camera(vec3(0,0,-3), vec2(0,0), SCREEN_HEIGHT);
    Scene scene(camera, 0.0f*vec3(1,1,1), surfaces, false);
	return scene;
}

Scene cornell() {
	Material red     (MaterialType::DIFFUSE, vec3(1.00, 0.05, 0.05));
    Material blue    (MaterialType::DIFFUSE, vec3(0.05, 0.05, 1.00));

    Material green   (MaterialType::DIFFUSE, vec3(0.05,1   ,0.05));
    Material purple  (MaterialType::DIFFUSE, vec3(1   ,0.05,1   ));
    Material yellow  (MaterialType::DIFFUSE, vec3(1   ,1,   0.05));
    Material cyan    (MaterialType::DIFFUSE, vec3(0.05,1,   1   ), 0.7f*vec3(1,1,1));
    Material white   (MaterialType::DIFFUSE, vec3(1   ,1,   1   ));

    vector<unique_ptr<Surface>> surfaces;

	add_surface(surfaces, Triangle(vec3(-2,1,-1), vec3(0,1,10), vec3(2,1,-1), green));
	add_surface(surfaces, Triangle(vec3(-1,-2,-1), vec3(-1,0,10), vec3(-1,2,-1), purple));
	add_surface(surfaces, Triangle(vec3(1,-2,-1), vec3(1,2,-1), vec3(1,0,10), yellow));
	add_surface(surfaces, Triangle(vec3(-2,-1,-1), vec3(2,-1,-1), vec3(0,-1,10), cyan));
	add_surface(surfaces, Triangle(vec3(-2,1,1), vec3(0,-10,1), vec3(2,1,1), white));

	add_surface(surfaces, Sphere(vec3(0.35,0.7,-0.39),0.3, red));

	vec3 A = vec3(-0.5243243243243243f, 1.0f, -0.1099099099099099f);
	vec3 B = vec3(0.04504504504504503f, 1.0f, 0.06666666666666665f);
	vec3 C = vec3(-0.7009009009009008f, 1.0f, 0.463063063063063f);
	vec3 D = vec3(-0.13153153153153152f, 1.0f, 0.6432432432432433f);
	vec3 E = vec3(-0.5243243243243243f, -0.18918918918918926f, -0.1099099099099099f);
	vec3 F = vec3(0.04504504504504503f, -0.18918918918918926f, 0.06666666666666665f);
	vec3 G = vec3(-0.7009009009009008f, -0.18918918918918926f, 0.463063063063063f);
	vec3 H = vec3(-0.13153153153153152f, -0.18918918918918926f, 0.6432432432432433f);

	add_surface(surfaces, Triangle(E, B, A, blue));
	add_surface(surfaces, Triangle(E, F, B, blue));
	add_surface(surfaces, Triangle(F, D, B, blue));
	add_surface(surfaces, Triangle(F, H, D, blue));
	add_surface(surfaces, Triangle(H, C, D, blue));
	add_surface(surfaces, Triangle(H, G, C, blue));
	add_surface(surfaces, Triangle(G, E, C, blue));
	add_surface(surfaces, Triangle(E, A, C, blue));
	add_surface(surfaces, Triangle(G, F, E, blue));
	add_surface(surfaces, Triangle(G, H, F, blue));

    Camera camera(vec3(0,0,-3), vec2(0,0), SCREEN_HEIGHT);
    Scene scene(camera, 0.1f*vec3(1,1,1), surfaces, false);
	return scene;
}

} // namespace Scenes

#endif // SCENE
