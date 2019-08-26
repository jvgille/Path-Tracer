#include <iostream>
#include <optional>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <string>
#include <set>
#include <memory>

#include <glm/glm.hpp>
#include <SDL.h>

#include "sdl_helpers.hpp"
#include "camera.hpp"
#include "surface.hpp"
#include "intersection.hpp"
#include "constants.hpp"
#include "scene.hpp"

using namespace std;
using glm::vec3, glm::ivec2, glm::vec2, glm::mat3, glm::dvec3;
using glm::dot, glm::length, glm::normalize, glm::determinant;

struct {
    bool save_image = false;
    bool exit = false;
    uint next_target = 100;
} LOGGING;

typedef vector<vector<vec3>> BUFFER;
uint TOTAL_SAMPLES = 0;

void print_vec(const vec3 & v) {
    printf("(%f,%f,%f)\n", v.x, v.y, v.z);
}

float luminance(const vec3 & v) {
    return (0.299*v.x + 0.587*v.y + 0.114*v.z);
}

double random_real(double min, double max) {
    return min + (double(rand())/RAND_MAX)*(max-min);
}

int random_int(int min, int max) {
    return min + rand() % (max-min);
}

vec3 get_ray_direction_by_pixel(const Camera & camera, float u, float v) {
    return normalize(camera.get_rotation_matrix()*vec3(u-SCREEN_WIDTH/2,
                                                       v-SCREEN_HEIGHT/2,
                                                       camera.get_focal_length()));
}

optional<Intersection> closest_intersection(Scene & scene, vec3 start, vec3 dir) {
    optional<Intersection> closest;
    for (unique_ptr<Surface> & ptr : scene.surfaces) {
        const Surface & surface = *ptr;
        if (auto res = surface.intersects(start, dir)) {
            if (!closest || res->distance < closest->distance) {
                closest = move(res);
            }
        }
    }
    return closest;
}

/*
importance sampling
metallic (cosine weighted around reflection?)
atmospheric fog
bidirectional
octree
openmp
combinations of materials? also look at glowing mirror & glass
*/

vec3 trace_ray(Scene & scene, vec3 origin, vec3 dir) {
    vec3 color(0,0,0);
    vec3 throughput(1,1,1);

    for (int depth=0; ; depth++) {
        if (auto intersection = closest_intersection(scene, origin, dir)) {
            const Surface & t = intersection->surface.get();
            vec3 normal = t.get_normal(intersection->point);

            vec3 new_dir = t.material.sample_pdf(normal, dir);
            origin = intersection->point;

            color += t.material.emittance * throughput;
            throughput *= t.material.brdf(normal, new_dir);
            dir = new_dir;

            // russian roulette
            if (depth < 5)
                continue;

            float p = max(throughput.x, max(throughput.y, throughput.z)) - 0.05f; // -0.05 to prevent infinite rays
            if (random_real(0,1) > p) {
                break;
            }
            throughput *= 1/p; // add the lost energy
        } else {
            color += scene.BG_COLOR * throughput;
            break;
        }
    }
    return color;
}


void trace_rays(Scene & scene, BUFFER & buffer) {
    const Camera & camera(scene.camera);
    //#pragma omp parallel for schedule(dynamic, 1) // OpenMP
    const vec3 camera_pos = camera.get_position();
    for (int u = 0; u < buffer.size(); u++) {
        for (int v = 0; v < buffer[u].size(); v++) {
            vec3 origin = camera_pos;
            // add random_real(0,1) for anti-alias
            vec3 dir = get_ray_direction_by_pixel(camera, u+random_real(0,1), v+random_real(0,1));
            if (scene.DOF_on) {
                // use primary ray to calculate secondary ray for depth of field
                float t = scene.DOF_focal_length/dot(camera.get_forward(), dir);
                vec3 p = t*dir + camera_pos; // point of perfect focus

                // random point on circular aperture
                float r = random_real(0, scene.DOF_aperture_size);
                float phi = random_real(0, 2*M_PI);
                float dx = r*cos(phi);
                float dy = r*sin(phi);

                origin = camera_pos + dx*camera.get_right() + dy*camera.get_downward();
                dir = normalize(p - origin);
            }

            vec3 color = trace_ray(scene, origin, dir);
            buffer[u][v] += color;
        }
    }
    TOTAL_SAMPLES++;
}

// convert HDR to LDR and apply gamma correction of 2.2
float to_ldr(float x) {
    return glm::pow(glm::clamp(x, 0.0f, 1.0f), 1/2.2f)*255 + 0.5;
}
vec3 to_ldr(const vec3 & c) {
    return vec3(to_ldr(c.x), to_ldr(c.y), to_ldr(c.z));
}

void draw(SDL_Surface * screen, const Camera & camera, const BUFFER & buffer) {
    SDL_FillRect(screen, 0, 0);
    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    for (int u = 0; u < buffer.size(); u++) {
        for (int v = 0; v < buffer[u].size(); v++) {
            PutPixelSDL(screen, u, v, to_ldr(buffer[u][v]/float(TOTAL_SAMPLES)));
        }
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

bool handle_input(Scene & scene, const float dt) {
    Uint8* keystate = SDL_GetKeyState(0);
    const float MOVE_SPEED = 1 * dt/1000;
    const float ROT_SPEED = 1 * dt/1000;
    bool invalidate_buffer = false;

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
    scene.camera.rotate(d_rot_x, d_rot_y);

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
    scene.camera.move(dx, dy, dz);

    if (keystate[SDLK_1]) {
        cerr << "image saving enabled" << endl;
        LOGGING.save_image = true;
    } else if (keystate[SDLK_2]) {
        cerr << "image saving disabled" << endl;
        LOGGING.save_image = false;
    }

    if (keystate[SDLK_0]) {
        printf("Camera pos: ");
        print_vec(scene.camera.get_position());
        vec2 rot = scene.camera.get_rotation();
        printf("Camera rot: (%f, %f)", rot.x, rot.y);
        cout << endl;
    }

    if (keystate[SDLK_u]) {
        invalidate_buffer = true;
        scene.DOF_on = true;
        cerr << "depth of field enabled" << endl;
    } else if (keystate[SDLK_j]) {
        invalidate_buffer = true;
        scene.DOF_on = false;
        cerr << "depth of field disabled" << endl;
    }

    if (keystate[SDLK_i]) {
        invalidate_buffer = true;
        scene.DOF_focal_length += 0.1f;
        cerr << "DoF focal length: " << scene.DOF_focal_length << endl;
    } else if (keystate[SDLK_k]) {
        invalidate_buffer = true;
        scene.DOF_focal_length -= 0.1f;
        cerr << "DoF focal length: " << scene.DOF_focal_length << endl;
    }

    if (keystate[SDLK_o]) {
        invalidate_buffer = true;
        scene.DOF_aperture_size += 0.01f;
        cerr << "DoF aperture size: " << scene.DOF_aperture_size << endl;
    } else if (keystate[SDLK_l]) {
        scene.DOF_aperture_size -= 0.01f;
        invalidate_buffer = true;
        cerr << "DoF aperture size: " << scene.DOF_aperture_size << endl;
    }

    return invalidate_buffer;
}

int update(Scene & scene, BUFFER & buffer, int t) {
    int t2 = SDL_GetTicks();
    float dt = float(t2 - t);
    t = t2;

    bool clear_buffer = handle_input(scene, dt);
    clear_buffer |= scene.camera.update();


    if (clear_buffer) {
        TOTAL_SAMPLES = 0;
        for (vector<vec3> & col : buffer) {
            for (auto & v : col) {
                v = vec3();
            }
        }
    }
    trace_rays(scene, buffer);

    return t;
}

bool log(SDL_Surface * screen) {
    if (TOTAL_SAMPLES % 10 == 0)
        cerr << TOTAL_SAMPLES << endl;
    if (TOTAL_SAMPLES >= LOGGING.next_target) {
        if (LOGGING.save_image) {
            string path = "../images/";
            path += to_string(TOTAL_SAMPLES);
            path += ".bmp";
            SDL_SaveBMP(screen, path.c_str());
            cerr << "saved image to " << path << endl;
            while(LOGGING.next_target <= TOTAL_SAMPLES) {
                LOGGING.next_target *= 2;
            }
        }
        if (LOGGING.exit) {
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    std::set<string> args; // todo do a for loop instead
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
    //Scene scene = Scenes::sunrise();
    Scene scene = Scenes::cornell();

    const int t_0 = SDL_GetTicks();
    int time = t_0;

    BUFFER buffer = vector<vector<vec3>>(SCREEN_WIDTH, vector<vec3>(SCREEN_HEIGHT, vec3()));

    bool should_continue = true;
    while (NoQuitMessageSDL() && should_continue) {
        time = update(scene, buffer, time);
        draw(screen, scene.camera, buffer);
        should_continue = log(screen);
    }

    float dt = SDL_GetTicks() - t_0;
}
