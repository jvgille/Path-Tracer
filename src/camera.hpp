#ifndef CAMERA
#define CAMERA

#include <glm/glm.hpp>

#include "constants.hpp"

using glm::mat3, glm::vec3, glm::vec2, glm::normalize;

const vec2 OFFSET{2*EPSILON, EPSILON}; // slightly offset to avoid alignment lightning bugs

class Camera {

    vec3 pos;
    vec2 rot;
    mat3 rot_matrix;
    float focal_length;

    bool dirty = true;

    void update_rotation_matrix() {
        float cx = cos(-rot.x);
        float sx = sin(-rot.x);
        float cy = cos(-rot.y);
        float sy = sin(-rot.y);
        rot_matrix = mat3(cy,0,sy,0,1,0,-sy,0,cy)* // y
                     mat3(1,0,0,0,cx,-sx,0,sx,cx); // x
    }

  public:
    Camera() : pos() , rot() , focal_length(SCREEN_HEIGHT) {
        update();
    }

    Camera(vec3 _pos, vec2 _rot, float _focal_length)
    : pos(_pos)
    , rot(_rot + OFFSET)
    , focal_length(_focal_length) {
        update();
    }

    void rotate(float dx, float dy) {
        if (dx == 0 && dy == 0)
            return;

        rot.x += dx;
        rot.y += dy;
        dirty = true;
    }

    void move(float dx, float dy, float dz) {
        if (dx == 0 && dy == 0 && dz == 0)
            return;

        pos.x += dx*cos(-rot.y) - dz*sin(-rot.y);
        pos.z += dx*sin(-rot.y) + dz*cos(-rot.y);
        pos.y += dy;
        dirty = true;
    }

    const vec3 & get_position() const {
        return pos;
    }

    const mat3 & get_rotation_matrix() const {
        return rot_matrix;
    }

    float get_focal_length() const {
        return focal_length;
    }

    bool update() {
        if (dirty) {
            dirty = false;
            update_rotation_matrix();
            rot.x = glm::clamp(rot.x, float(-M_PI_2), float(M_PI_2));
            while (rot.y > 2*M_PI)
                rot.y -= 2*M_PI;
            while (rot.y < 0)
                rot.y += 2*M_PI;
            return true;
        } else {
            return false;
        }
    }

    vec3 get_forward() const {
        return rot_matrix*vec3(0,0,1);
    }

    vec3 get_downward() const {
        return rot_matrix*vec3(0,1,0);
    }

    vec3 get_right() const {
        return rot_matrix*vec3(1,0,0);
    }
};

#endif // CAMERA
