#include <iostream>
#include <optional>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp> // PI
#include <SDL.h>
#include "../SDLauxiliary.h"
#include "../TestModel.h"

#include "camera.hpp"

using namespace std;
using glm::vec3, glm::ivec2, glm::vec2, glm::mat3, glm::dvec3;
using glm::dot, glm::length, glm::normalize, glm::determinant;

const int SCREEN_WIDTH = 300;
const int SCREEN_HEIGHT = 300;

bool TRACE_RAYS = true;

SDL_Surface* screen;
int t;
vector<Triangle> triangles;

vec3 light_pos(0,0,-1.5);
vec3 light_color = 10.0f * vec3(1, 1, 1);

struct {
    bool SAVE_IMAGE = false;
    uint NEXT_TARGET = 100;
} LOGGING;

struct Intersection {
    vec3 point;
    float distance;
    reference_wrapper<const Triangle> triangle;
};

#include "raster.hpp"

typedef vector<vector<vec3>> BUFFER;
uint TOTAL_SAMPLES;

BUFFER new_buffer() {
    TOTAL_SAMPLES = 0;
    return vector<vector<vec3>>(SCREEN_WIDTH, vector<vec3>(SCREEN_HEIGHT, vec3()));
}

void print_vec(const vec3 & v) {
    printf("(%f,%f,%f)\n", v.x, v.y, v.z);
}

double luminance(const dvec3 & v) {
    return (0.299*v.x+ 0.587*v.y+ 0.114*v.z);
}

double random_real(double min, double max) {
    return min + (double(rand())/RAND_MAX)*(max-min);
}

int random_int(int min, int max) {
    return min + rand() % (max-min);
}

vec3 get_ray_direction_by_pixel(const Camera & camera, int u, int v) {
    return normalize(camera.get_rotation_matrix()*vec3(u-SCREEN_WIDTH/2,
                                                       v-SCREEN_HEIGHT/2,
                                                       camera.get_focal_length()));
}

optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir, const Triangle & triangle) {
    vec3 e1 = triangle.v1 - triangle.v0;
    vec3 e2 = triangle.v2 - triangle.v0;

    mat3 A(-ray_dir, e1, e2);
    float det_A = determinant(A);
    if (-glm::epsilon<float>() < det_A && det_A < glm::epsilon<float>())
        return optional<Intersection>();

    vec3 b = ray_start - triangle.v0;
    float u = determinant(mat3(-ray_dir, b, e2))/det_A;
    if (u < -glm::epsilon<float>() || u >= 1)
        return optional<Intersection>();

    float v = determinant(mat3(-ray_dir, e1, b))/det_A;
    if (v < -glm::epsilon<float>() || u+v >= 1)
        return optional<Intersection>();

    float t = determinant(mat3(b, e1, e2))/det_A;
    if (t < glm::epsilon<float>())
        return optional<Intersection>();

    vec3 intersection_point = ray_start + t*ray_dir;
    float distance = length(t*ray_dir);

    return optional<Intersection>(Intersection{intersection_point, distance, triangle});
}

optional<Intersection> closest_intersection(vec3 start, vec3 dir,
                                            const vector<Triangle> & triangles) {
    optional<Intersection> closest;
    for (const Triangle & triangle : triangles) {
        if (auto res = intersects(start, dir, triangle)) {
            if (!closest || res->distance < closest->distance) {
                closest = move(res);
            }
        }
    }
    return closest;
}

vec3 sample_hemisphere(const vec3 & normal) {
    // sample point on unit sphere
    float u = random_real(0, 1);
    float v = random_real(0, 1);
    float theta = glm::two_pi<float>()*u;
    float phi = acos(2*v-1);

    // convert spherical to cartesian
    vec3 direction{cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi)};

    // if dir is not in hemisphere around normal, flip it
    if (dot(direction, normal) < 0) {
        direction *= -1;
    }
    return direction;
}

const vec3 BACKGROUND_COLOR{2,2,2};
const uint MAX_DEPTH = 5;

uint num_max_depth = 0;
uint num_background = 0;

vec3 trace_ray(vec3 origin, vec3 dir, uint depth = 0) {
    // russian roulette, importance sampling
    // emissive materials, other brdf:s
    if (depth >= MAX_DEPTH) {
        num_max_depth++;
        return vec3{0,0,0};
    }

    if (auto intersection = closest_intersection(origin, dir, triangles)) {
        const Triangle & t = intersection->triangle.get();

        dir = sample_hemisphere(t.normal);
        origin = intersection->point;

        // EmittedLight + 2 * RecursiveLight * Dot(Normal, RandomHemisphereAngle) * SurfaceDiffuseColor
        return 2.0f * trace_ray(origin, dir, depth+1) * dot(dir, t.normal) * t.color;
    } else {
        return BACKGROUND_COLOR;
    }
}

void trace_rays(Camera & camera, BUFFER & buffer) {
    for (int u = 0; u < buffer.size(); u++) {
        for (int v = 0; v < buffer[u].size(); v++) {
            vec3 dir = get_ray_direction_by_pixel(camera, u, v);
            vec3 color = trace_ray(camera.get_position(), dir);
            buffer[u][v] += color;
        }
    }
    TOTAL_SAMPLES++;
}

vec3 median_filter(const BUFFER & buffer, int u, int v) {
    vector<vec3> temp;
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            if (0 < u+i && u+i < buffer.size() &&
                0 < v+j && v+j < buffer[u+i].size()) {
                temp.push_back(buffer[u+i][v+j]);
            }
        }
    }
    auto comp = [](const vec3 & a, const vec3 & b) { return luminance(dvec3(a)) < luminance(dvec3(b));};
    sort(temp.begin(), temp.end(), comp);
    return temp[temp.size()/2];
}

bool MEDIAN_FILTER = false;

void draw(const Camera & camera, const BUFFER & buffer) {
    SDL_FillRect(screen, 0, 0);
    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    if (TRACE_RAYS) {
        for (int u = 0; u < buffer.size(); u++) {
            for (int v = 0; v < buffer[u].size(); v++) {
                if (MEDIAN_FILTER) {
                    PutPixelSDL(screen, u, v, median_filter(buffer, u, v)/float(TOTAL_SAMPLES));
                } else {
                    PutPixelSDL(screen, u, v, buffer[u][v]/float(TOTAL_SAMPLES));
                }
            }
        }
    } else {
        for (uint i = 0; i < triangles.size(); i++) {
            vector<vec3> vertices = {triangles[i].v0, triangles[i].v1, triangles[i].v2};
            draw_polygon_edges(camera, vertices);
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

/*
return true if camera state has changed
todo take camera etc as arg
todo encapsulate state?
*/
bool handle_input(Camera & camera, const float dt) {
    const float MOVE_SPEED = 1 * dt/1000;
    const float ROT_SPEED = 1 * dt/1000;

    Uint8* keystate = SDL_GetKeyState(0);

    // rotate camera
    float d_rot_x = 0, d_rot_y = 0;
    if (keystate[SDLK_UP])
        d_rot_x = ROT_SPEED;
    else if (keystate[SDLK_DOWN])
        d_rot_x = -ROT_SPEED;
    if (keystate[SDLK_RIGHT])
        d_rot_y = ROT_SPEED;
    else if (keystate[SDLK_LEFT])
        d_rot_y = -ROT_SPEED;
    camera.rotate(d_rot_x, d_rot_y);

    // move camera
    float dx = 0, dy = 0, dz = 0;
    if (keystate[SDLK_d])
        dx = MOVE_SPEED;
    else if (keystate[SDLK_a])
        dx = -MOVE_SPEED;
    if (keystate[SDLK_q])
        dy = MOVE_SPEED;
    else if (keystate[SDLK_e])
        dy = -MOVE_SPEED;
    if (keystate[SDLK_w])
        dz = MOVE_SPEED;
    else if (keystate[SDLK_s])
        dz = -MOVE_SPEED;
    camera.move(dx, dy, dz);


    if (keystate[SDLK_3]) {
        cerr << "median filter on" << endl;
        MEDIAN_FILTER = true;
    } else if (keystate[SDLK_4]) {
        cerr << "median filter off" << endl;
        MEDIAN_FILTER = false;
    }
    if (keystate[SDLK_5]) {
        cerr << "image saving enabled" << endl;
        LOGGING.SAVE_IMAGE = true;
    } else if (keystate[SDLK_6]) {
        cerr << "image saving disabled" << endl;
        LOGGING.SAVE_IMAGE = false;
    }

    if (keystate[SDLK_1])
        return true;
    else if (keystate[SDLK_2])
        return false;
    else
        return TRACE_RAYS;
}

void update(Camera & camera, BUFFER & buffer) {
    int t2 = SDL_GetTicks();
    float dt = float(t2 - t);
    t = t2;

    bool clear_buffer = false;
    if (TRACE_RAYS != handle_input(camera, dt)) {
        TRACE_RAYS = !TRACE_RAYS;
        clear_buffer = true;
    }

    clear_buffer |= camera.update();

    if (TRACE_RAYS) {
        if (clear_buffer) {
            buffer = new_buffer();
        }
        trace_rays(camera, buffer);
    } else {
        // nothing to do - todo sleep to get 60 hz?
    }
}

void log() {
    if (TRACE_RAYS) {
        if (TOTAL_SAMPLES % 10 == 0)
            cerr << TOTAL_SAMPLES << endl;
        if (LOGGING.SAVE_IMAGE && TOTAL_SAMPLES >= LOGGING.NEXT_TARGET) {
            string path = "images/";
            path += to_string(TOTAL_SAMPLES);
            path += ".bmp";
            SDL_SaveBMP(screen, path.c_str());
            cerr << "saved image to " << path << endl;
            LOGGING.NEXT_TARGET *= 2;
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc > 1 && string(argv[1]) == "-log") {
        cerr << "image saving enabled" << endl;
        LOGGING.SAVE_IMAGE = true;
    }

    screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);

    Camera camera(vec3(0,0,20*glm::epsilon<float>()-3), vec2(3*glm::epsilon<float>(), 2*glm::epsilon<float>()), SCREEN_HEIGHT);

    LoadTestModel(triangles);

    const int t_0 = SDL_GetTicks();
    t = t_0;

    BUFFER buffer = new_buffer();

    while (NoQuitMessageSDL()) {
        update(camera, buffer);
        draw(camera, buffer);
        log();
    }

    float dt = SDL_GetTicks() - t_0;
}
