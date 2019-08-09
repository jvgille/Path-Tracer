#include <iostream>
#include <optional>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <string>
#include <set>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp> // PI
#include <SDL.h>
#include "../SDLauxiliary.h"

#include "triangle.hpp"
#include "camera.hpp"

using namespace std;
using glm::vec3, glm::ivec2, glm::vec2, glm::mat3, glm::dvec3;
using glm::dot, glm::length, glm::normalize, glm::determinant;

const double EPSILON = 0.00001;

const int SCREEN_WIDTH = 300;
const int SCREEN_HEIGHT = 300;

bool TRACE_RAYS = true;

vector<Triangle> triangles;

vec3 light_pos(0,0,-1.5);
vec3 light_color = 10.0f * vec3(1, 1, 1);

struct {
    bool save_image = false;
    bool exit = false;
    uint next_target = 100;
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

const vec3 BACKGROUND_COLOR{0,0,0};
const uint MAX_DEPTH = 5;

/*
point light
what part of this is lambertian?
 - lambert = dot(dir, t.normal) * t.color ?
emissive materials
importance sampling
other brdfs
 - mirror, metallic (cosine weighted around reflection?)

russian roulette https://computergraphics.stackexchange.com/questions/2316/is-russian-roulette-really-the-answer
how for recursive?
*/

vec3 trace_ray(vec3 origin, vec3 dir, uint depth = 0) {
    if (depth >= MAX_DEPTH)
        return vec3{0,0,0};

    if (auto intersection = closest_intersection(origin, dir, triangles)) {
        const Triangle & t = intersection->triangle.get();

        dir = sample_hemisphere(t.normal);
        origin = intersection->point;

        return t.material.emittance + 2.0f * trace_ray(origin, dir, depth+1) * dot(dir, t.normal) * t.material.color;
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

void draw(SDL_Surface * screen, const Camera & camera, const BUFFER & buffer) {
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
            draw_polygon_edges(screen, camera, vertices);
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

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
        LOGGING.save_image = true;
    } else if (keystate[SDLK_6]) {
        cerr << "image saving disabled" << endl;
        LOGGING.save_image = false;
    }

    if (keystate[SDLK_1])
        return true;
    else if (keystate[SDLK_2])
        return false;
    else
        return TRACE_RAYS;
}

int update(Camera & camera, BUFFER & buffer, int t) {
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
        // should sleep after drawing tho
    }

    return t;
}

bool log(SDL_Surface * screen) {
    if (TRACE_RAYS) {
        if (TOTAL_SAMPLES % 10 == 0)
            cerr << TOTAL_SAMPLES << endl;
        if (TOTAL_SAMPLES >= LOGGING.next_target) {
            if (LOGGING.save_image) {
                string path = "images/";
                path += to_string(TOTAL_SAMPLES);
                path += ".bmp";
                SDL_SaveBMP(screen, path.c_str());
                cerr << "saved image to " << path << endl;
                LOGGING.next_target *= 2;
            }
            if (LOGGING.exit) {
                return false;
            }
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    std::set<string> args;
    for (int i = 1; i < argc; i++) {
        args.insert(argv[i]);
    }
    if (args.count("-log")) {
        cerr << "image saving enabled" << endl;
        LOGGING.save_image = true;
    }
    if (args.count("-exit")) {
        cerr << "will exit at " << LOGGING.next_target << endl;
        LOGGING.exit = true;
    }

    SDL_Surface * screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);

    // slightly offset to avoid alignment lightning bugs
    Camera camera(vec3(0,0,10*EPSILON-3), vec2(2*EPSILON, EPSILON), SCREEN_HEIGHT);

    triangles = load_cornell();

    const int t_0 = SDL_GetTicks();
    int time = t_0;

    BUFFER buffer = new_buffer();

    bool should_continue = true;
    while (NoQuitMessageSDL() && should_continue) {
        time = update(camera, buffer, time);
        draw(screen, camera, buffer);
        should_continue = log(screen);
    }

    float dt = SDL_GetTicks() - t_0;
}
