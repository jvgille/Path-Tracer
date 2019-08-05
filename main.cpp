#include <iostream>
#include <optional>
#include <functional>
#include <cstdlib>
#include <cmath>
#include <string>

#include <glm/glm.hpp>
#include <SDL.h>
#include "../SDLauxiliary.h"
#include "../TestModel.h"

using namespace std;
using glm::vec3, glm::ivec2, glm::vec2, glm::mat3, glm::dvec3;
using glm::sin, glm::cos, glm::tan, glm::asin, glm::acos, glm::atan;
using glm::dot, glm::length, glm::normalize, glm::determinant;

const double PI = 3.14159265;
const double EPSILON = 0.00001;

const int SCREEN_WIDTH = 300;
const int SCREEN_HEIGHT = 300;

bool TRACE_RAYS = false;

SDL_Surface* screen;
int t;
vector<Triangle> triangles;

vec3 light_pos(0,0,-1.5);
vec3 light_color = 10.0f * vec3(1, 1, 1);

struct {
    vec3 pos = vec3(0, 0, -3.001);
    vec2 rot = vec2(0, 0);
    mat3 rot_matrix = mat3(0);
    float focal_length = SCREEN_HEIGHT;
} camera;

struct Intersection {
    vec3 point;
    float distance;
    reference_wrapper<const Triangle> triangle;
};

// todo some oop up in here

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

vec3 get_ray_direction_by_pixel(int u, int v) {
    return normalize(camera.rot_matrix*vec3(u-SCREEN_WIDTH/2,
                                                 v-SCREEN_HEIGHT/2,
                                                 camera.focal_length));
}

void update_camera_rotation() {
    float cx = cos(-camera.rot.x);
    float sx = sin(-camera.rot.x);
    float cy = cos(-camera.rot.y);
    float sy = sin(-camera.rot.y);
    camera.rot_matrix = mat3(cy,0,sy,0,1,0,-sy,0,cy)* // y
                        mat3(1,0,0,0,cx,-sx,0,sx,cx); // x
}

optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir, const Triangle & triangle) {
    vec3 e1 = triangle.v1 - triangle.v0;
    vec3 e2 = triangle.v2 - triangle.v0;

    mat3 A(-ray_dir, e1, e2);
    float det_A = determinant(A);
    if (-EPSILON < det_A && det_A < EPSILON)
        return optional<Intersection>();

    vec3 b = ray_start - triangle.v0;
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

vec3 diffuse_bounce(const vec3 & normal) {
    float u = random_real(0, 1);
    float v = random_real(0, 1);
    float theta = 2*PI*u;
    float phi = acos(2*v-1); // to sample on sphere
    //float phi = acos(v); // to sample on hemisphere?

    float x = cos(theta) * sin(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(phi);
    vec3 direction(x, y, z);

    if (dot(direction, normal) < 0) {
        // if dir is not in normal's hemisphere, flip it
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

        dir = diffuse_bounce(t.normal);
        origin = intersection->point;

        // EmittedLight + 2 * RecursiveLight * Dot(Normal, RandomHemisphereAngle) * SurfaceDiffuseColor
        return 2.0f * trace_ray(origin, dir, depth+1) * dot(dir, t.normal) * t.color;
    } else {
        return BACKGROUND_COLOR;
    }
}

void trace_rays(BUFFER & buffer) {
    for (int u = 0; u < buffer.size(); u++) {
        for (int v = 0; v < buffer[u].size(); v++) {
            vec3 dir = get_ray_direction_by_pixel(u, v);
            vec3 color = trace_ray(camera.pos, dir);
            buffer[u][v] += color;
        }
    }
    TOTAL_SAMPLES++;
    cerr << TOTAL_SAMPLES << endl;
}

void draw(const BUFFER & buffer) {
    SDL_FillRect(screen, 0, 0);
    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    if (TRACE_RAYS) {
        for (int u = 0; u < buffer.size(); u++) {
            for (int v = 0; v < buffer[u].size(); v++) {
                PutPixelSDL(screen, u, v, buffer[u][v]/float(TOTAL_SAMPLES));
            }
        }
    } else {
        for (uint i = 0; i < triangles.size(); i++) {
            vector<vec3> vertices = {triangles[i].v0, triangles[i].v1, triangles[i].v2};
            draw_polygon_edges(vertices);
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
bool handle_input(const float dt) {
    const float MOVE_SPEED = 1 * dt/1000;
    const float ROT_SPEED = 1 * dt/1000;

    bool old_TRACE_RAYS = TRACE_RAYS;
    mat3 old_camera_rot_matrix = camera.rot_matrix;
    vec3 old_camera_pos = camera.pos;
    vec3 old_light_pos = light_pos;

    Uint8* keystate = SDL_GetKeyState(0);

    if (keystate[SDLK_UP])
        camera.rot.x += ROT_SPEED;
    if (keystate[SDLK_DOWN])
        camera.rot.x -= ROT_SPEED;
    if (keystate[SDLK_RIGHT])
        camera.rot.y += ROT_SPEED;
    if (keystate[SDLK_LEFT])
        camera.rot.y -= ROT_SPEED;
    update_camera_rotation();

    float dx = 0, dz = 0;
    if (keystate[SDLK_i])
        dz += MOVE_SPEED;
    if (keystate[SDLK_k])
        dz -= MOVE_SPEED;
    if (keystate[SDLK_l])
        dx += MOVE_SPEED;
    if (keystate[SDLK_j])
        dx -= MOVE_SPEED;
    camera.pos.x += dx*cos(-camera.rot.y) - dz*sin(-camera.rot.y);
    camera.pos.z += dx*sin(-camera.rot.y) + dz*cos(-camera.rot.y);

    if (keystate[SDLK_o])
        camera.pos.y -= MOVE_SPEED;
    if (keystate[SDLK_u])
        camera.pos.y += MOVE_SPEED;

    if (keystate[SDLK_w])
        light_pos.z += MOVE_SPEED;
    if (keystate[SDLK_s])
        light_pos.z -= MOVE_SPEED;
    if (keystate[SDLK_d])
        light_pos.x += MOVE_SPEED;
    if (keystate[SDLK_a])
        light_pos.x -= MOVE_SPEED;
    if (keystate[SDLK_e])
        light_pos.y -= MOVE_SPEED;
    if (keystate[SDLK_q])
        light_pos.y += MOVE_SPEED;

    if (keystate[SDLK_1])
        TRACE_RAYS = true;
    if (keystate[SDLK_2])
        TRACE_RAYS = false;

    // return true if state has changed
    return old_TRACE_RAYS != TRACE_RAYS ||
           old_camera_rot_matrix != camera.rot_matrix ||
           old_camera_pos != camera.pos ||
           old_light_pos != light_pos;
}

void update(BUFFER & buffer) {
    int t2 = SDL_GetTicks();
    float dt = float(t2 - t);
    t = t2;

    bool state_changed = handle_input(dt);

    if (TRACE_RAYS) {
        if (state_changed) {
            // clear buffer if camera moved
            buffer = new_buffer();
        }
        // trace rays
        trace_rays(buffer);
    } else {
        // nothing to do - todo sleep?
    }
}

int main(int argc, char* argv[]) {
    screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);

    LoadTestModel(triangles);

    int frames = 0;
    const int t_0 = SDL_GetTicks();
    t = t_0;

    BUFFER buffer = new_buffer();

    while (NoQuitMessageSDL()) {
        update(buffer);
        draw(buffer);
        frames++;
    }

    float dt = SDL_GetTicks() - t_0;
    cout << "average ms/frame: " << (dt/frames) << endl;
    cout << "average FPS: " << (1000*frames/dt) << endl;
}
