#include <iostream>
#include <optional>
#include <functional>

#include <glm/glm.hpp>
#include <SDL.h>
#include "../SDLauxiliary.h"
#include "../TestModel.h"

using namespace std;
using glm::vec3, glm::ivec2, glm::vec2, glm::mat3;

const float PI = 3.14159265;
const float EPSILON = 0.00001;

const int SCREEN_WIDTH = 200;
const int SCREEN_HEIGHT = 200;

SDL_Surface* screen;
int t;
vector<Triangle> triangles;

vec3 light_pos(0,0,-1.5);
vec3 light_color = 10.0f * vec3(1, 1, 1);
vec3 indirect_light = 0.3f * vec3(1, 1, 1);

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

bool on_screen(int u, int v) {
    return 0 <= u && u < SCREEN_WIDTH &&
           0 <= v && v < SCREEN_HEIGHT;
}

template<class T>
vector<T> interpolate(const T & a, const T & b, const uint steps) {
    vector<T> res;
    const T diff = b - a;
    for (uint i = 0; i < steps; i++) {
        res.push_back(a + diff * (float(i) / steps));
    }
    return res;
}

vec2 vertex_shader(const vec3 & v) {
    vec2 p;
    vec3 pos = (v - camera.pos) * camera.rot_matrix;
    p.x = camera.focal_length * pos.x/pos.z + SCREEN_WIDTH/2;
    p.y = camera.focal_length * pos.y/pos.z + SCREEN_HEIGHT/2;
    return p;
}

void draw_line(const vec2 & a, const vec2 & b, const vec3 & color) {
    vec2 diff = a - b;
    ivec2 delta = glm::abs(ivec2(diff.x, diff.y));
    int num_pixels = glm::max(delta.x, delta.y) + 1;
    vector<vec2> line = interpolate(a, b, num_pixels);

    for (const vec2 & v : line) {
        PutPixelSDL(screen, v.x, v.y, color);
    }
}

void draw_polygon_edges(const vector<vec3> & vertices, const vec3 & color = vec3(1,1,1)) {
    vector<vec2> projected_vertices;
    for (const vec3 & v : vertices) {
        projected_vertices.push_back(vertex_shader(v));
    }
    for (uint i = 0; i < vertices.size(); i++) {
        int j = (i + 1) % vertices.size();
        draw_line(projected_vertices[i], projected_vertices[j], color);
    }
}

optional<Intersection> intersects(vec3 ray_start, vec3 ray_dir, const Triangle & triangle) {
    vec3 e1 = triangle.v1 - triangle.v0;
    vec3 e2 = triangle.v2 - triangle.v0;

    mat3 A(-ray_dir, e1, e2);
    float det_A = glm::determinant(A);
    if (-EPSILON < det_A && det_A < EPSILON)
        return optional<Intersection>();

    vec3 b = ray_start - triangle.v0;
    float u = glm::determinant(mat3(-ray_dir, b, e2))/det_A;
    if (u < -EPSILON || u >= 1)
        return optional<Intersection>();

    float v = glm::determinant(mat3(-ray_dir, e1, b))/det_A;
    if (v < -EPSILON || u+v >= 1)
        return optional<Intersection>();

    float t = glm::determinant(mat3(b, e1, e2))/det_A;
    if (t < -EPSILON)
        return optional<Intersection>();

    vec3 intersection_point = ray_start + t*ray_dir;
    float distance = glm::length(t*ray_dir);

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

vec3 direct_light(const Intersection & i) {
    vec3 light_direction = light_pos - i.point;

    optional<Intersection> shadow = closest_intersection(light_pos, -light_direction, triangles);
    if (shadow && shadow->distance < glm::length(light_direction) - EPSILON) {
        return vec3(0,0,0);
    }

    float res = glm::max(0.0f, glm::dot(glm::normalize(light_direction), i.triangle.get().normal));
    res /= (4 * PI * glm::length(light_direction));
    return light_color * res;
}

void draw() {
    SDL_FillRect(screen, 0, 0);

    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    for (int v = 0; v < SCREEN_HEIGHT; v++) {
        for (int u = 0; u < SCREEN_WIDTH; u++) {
            vec3 dir = camera.rot_matrix * vec3(u - SCREEN_WIDTH/2,
                                                v - SCREEN_HEIGHT/2,
                                                camera.focal_length);
            vec3 color(0,0,0);

            if (auto res = closest_intersection(camera.pos, dir, triangles)) {
                color = res->triangle.get().color * (direct_light(*res) + indirect_light);
            }

            PutPixelSDL(screen, u, v, color);
        }
    }
    for (uint i = 0; i < triangles.size(); i++) {
        vector<vec3> vertices = {triangles[i].v0, triangles[i].v1, triangles[i].v2};
        draw_polygon_edges(vertices);
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

void update_camera_rotation() {
    float cx = cos(-camera.rot.x);
    float sx = sin(-camera.rot.x);
    float cy = cos(-camera.rot.y);
    float sy = sin(-camera.rot.y);
    camera.rot_matrix = mat3(cy,0,sy,0,1,0,-sy,0,cy)* // y
                        mat3(1,0,0,0,cx,-sx,0,sx,cx); // x
}

void update() {
    int t2 = SDL_GetTicks();
    float dt = float(t2 - t);
    t = t2;

    const float MOVE_SPEED = 1 * dt/1000;
    const float ROT_SPEED = 1 * dt/1000;

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
}

struct DomainLocation {
    int x_loc, y_loc;
};

float luminance(const vec3 & v) {
    return 0.299*v.x+ 0.587*v.y+ 0.114*v.z;
}

#include <cstdlib>

float random_real(float min, float max) {
    return min + (float(rand())/RAND_MAX)*(max-min);
}

int random_int(int min, int max) {
    return min + rand() % (max-min);
}

DomainLocation initial_path() {
    return DomainLocation{0, 0};
}

DomainLocation mutate_according_to_T(const DomainLocation & X) {
    int x = random_int(0, SCREEN_WIDTH);
    int y = random_int(0, SCREEN_HEIGHT);
    return DomainLocation{x, y};
}

vec3 evaluate_light_path(const DomainLocation & d) {
    float r = 255*(float(d.x_loc)/SCREEN_WIDTH);
    float g = 255*(1-float(d.y_loc)/SCREEN_HEIGHT);
    float b = 255*(float(d.x_loc * d.y_loc)/(SCREEN_WIDTH*SCREEN_HEIGHT));
    return vec3(r, g, b);
}

float T(const DomainLocation & X, const DomainLocation & Y) {
    return 1.0f / (SCREEN_WIDTH*SCREEN_HEIGHT);
}

void mlt(uint DRAW_INTERVAL = 1, int MUTATIONS = -1) {
    vec3 histogram[SCREEN_WIDTH][SCREEN_HEIGHT]; // todo zero
    vec3 average_sample(0,0,0);

    DomainLocation X, Y;
    X = initial_path();
    vec3 color_X = evaluate_light_path(X);
    float Fx = luminance(color_X);
    for (uint i = 0; i < MUTATIONS && NoQuitMessageSDL(); i++) {
        Y = mutate_according_to_T(X);

        float Tyx = T(Y, X);
        float Txy = T(X, Y);

        vec3 color_Y = evaluate_light_path(Y); // evaluate X_f
        float Fy = luminance(color_Y);
        color_Y /= Fy;

        float Axy = glm::min(1.0f, (Fy * Txy) / (Fx * Tyx));
        if (random_real(0.0, 1.0) < Axy){
            X = Y;
            Fx = Fy;
            color_X = color_Y;
        }
        histogram[X.x_loc][X.y_loc] += color_X;
        average_sample = (color_X + float(i)*average_sample)/float(i+1);

        // TODO scale before drawing..?
        // draw to screen
        if (i % DRAW_INTERVAL == 0) {
            float samples_per_bin = float(i)/(SCREEN_WIDTH*SCREEN_HEIGHT);
            float scale = luminance(average_sample)/samples_per_bin;

            SDL_FillRect(screen, 0, 0);
            if (SDL_MUSTLOCK(screen))
                SDL_LockSurface(screen);

            for (int u = 0; u < SCREEN_WIDTH; u++) {
                for (int v = 0; v < SCREEN_HEIGHT; v++) {
                    PutPixelSDL(screen, u, v, scale*histogram[u][v]);
                }
            }
            if (SDL_MUSTLOCK(screen))
                SDL_UnlockSurface(screen);

            SDL_UpdateRect(screen, 0, 0, 0, 0);
        }
    }

}

int main(int argc, char* argv[]) {
    screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);

    mlt(1000);
    return 0;

    LoadTestModel(triangles);
    int frames = 0;
    const int t_0 = SDL_GetTicks();
    t = t_0;

    while (NoQuitMessageSDL()) {
        update();
        draw();
        frames++;
    }

    float dt = SDL_GetTicks() - t_0;
    cout << "average ms/frame: " << (dt/frames) << endl;
    cout << "average FPS: " << (1000*frames/dt) << endl;
}
