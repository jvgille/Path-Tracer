#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "../SDLauxiliary.h"
#include "../TestModel.h"

using namespace std;
using glm::vec3;
using glm::ivec2;
using glm::vec2;
using glm::mat3;

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
const float focal_length = SCREEN_HEIGHT;
SDL_Surface* screen;
int t;
vector<Triangle> triangles;
vec3 camera_pos(0, 0, -3.001);
vec2 camera_rot(0, 0);
mat3 camera_rot_matrix(0);

vec3 light_pos(0,-0.5,-1.5);
vec3 light_power = 10.0f * vec3(1, 1, 1);
vec3 indirect_light = 0.4f * vec3(1, 1, 1);

bool on_screen(int u, int v) {
    return 0 <= u && u < SCREEN_WIDTH &&
           0 <= v && v < SCREEN_HEIGHT;
}

void interpolate(const vec2 & a, const vec2 & b, vector<vec2> & result) {
    const vec2 diff = (b - a);
    float steps = glm::max(float(result.size() - 1), 1.0f);
    for (uint i = 0; i < result.size(); i++) {
        result[i] = a + diff * (i / steps);
    }
}

vec2 vertex_shader(const vec3 & v) {
    vec2 p;
    vec3 pos = (v - camera_pos) * camera_rot_matrix;
    p.x = focal_length * pos.x/pos.z + SCREEN_WIDTH/2;
    p.y = focal_length * pos.y/pos.z + SCREEN_HEIGHT/2;
    return p;
}

void draw_line(const vec2 & a, const vec2 & b, const vec3 & color) {
    vec2 diff = a - b;
    ivec2 delta = glm::abs(ivec2(diff.x, diff.y));
    int num_pixels = glm::max(delta.x, delta.y) + 1;
    vector<vec2> line(num_pixels);
    interpolate(a, b, line);

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

void draw() {
    SDL_FillRect(screen, 0, 0);

    if (SDL_MUSTLOCK(screen))
        SDL_LockSurface(screen);

    for (uint i = 0; i < triangles.size(); i++) {
        vector<vec3> vertices = {
            triangles[i].v0, triangles[i].v1, triangles[i].v2
        };

        draw_polygon_edges(vertices);
    }

    if (SDL_MUSTLOCK(screen))
        SDL_UnlockSurface(screen);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

void update_camera_rotation() {
    float cx = cos(-camera_rot.x);
    float sx = sin(-camera_rot.x);
    float cy = cos(-camera_rot.y);
    float sy = sin(-camera_rot.y);
    camera_rot_matrix = mat3(cy,0,sy,0,1,0,-sy,0,cy)* // y
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
        camera_rot.x += ROT_SPEED;
    if (keystate[SDLK_DOWN])
        camera_rot.x -= ROT_SPEED;
    if (keystate[SDLK_RIGHT])
        camera_rot.y += ROT_SPEED;
    if (keystate[SDLK_LEFT])
        camera_rot.y -= ROT_SPEED;
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
    camera_pos.x += dx*cos(-camera_rot.y) - dz*sin(-camera_rot.y);
    camera_pos.z += dx*sin(-camera_rot.y) + dz*cos(-camera_rot.y);

    if (keystate[SDLK_o])
        camera_pos.y += MOVE_SPEED;
    if (keystate[SDLK_u])
        camera_pos.y -= MOVE_SPEED;

    if (keystate[SDLK_w])
        light_pos.z += MOVE_SPEED;
    if (keystate[SDLK_s])
        light_pos.z -= MOVE_SPEED;
    if (keystate[SDLK_d])
        light_pos.x += MOVE_SPEED;
    if (keystate[SDLK_a])
        light_pos.x -= MOVE_SPEED;
    if (keystate[SDLK_e])
        light_pos.y += MOVE_SPEED;
    if (keystate[SDLK_q])
        light_pos.y -= MOVE_SPEED;
}

int main(int argc, char* argv[]) {
    LoadTestModel(triangles);
    screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);
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
