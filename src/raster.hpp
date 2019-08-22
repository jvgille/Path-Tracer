bool on_screen(const vec2 & v) {
    return 0 <= v.x && v.x < SCREEN_WIDTH &&
           0 <= v.y && v.y < SCREEN_HEIGHT;
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

vec2 vertex_shader(const Camera & camera, const vec3 & v) {
    vec2 p;
    vec3 pos = (v - camera.get_position()) * camera.get_rotation_matrix();
    p.x = camera.get_focal_length() * pos.x/pos.z + SCREEN_WIDTH/2;
    p.y = camera.get_focal_length() * pos.y/pos.z + SCREEN_HEIGHT/2;
    return p;
}

void draw_line(SDL_Surface * screen, const vec2 & a, const vec2 & b, const vec3 & color) {
    if (!on_screen(a) || !on_screen(b)) {
        return;
    }
    vec2 diff = a - b;
    ivec2 delta = glm::abs(ivec2(diff.x, diff.y));
    int num_pixels = glm::max(delta.x, delta.y) + 1;
    vector<vec2> line = interpolate(a, b, num_pixels);

    for (const vec2 & v : line) {
        PutPixelSDL(screen, v.x, v.y, color);
    }
}

void draw_polygon_edges(SDL_Surface * screen, const Camera & camera,
                        const vector<vec3> & vertices, const vec3 & color = vec3(1,1,1)) {
    // todo draws lines behind camera
    vector<vec2> projected_vertices;
    for (const vec3 & v : vertices) {
        projected_vertices.push_back(vertex_shader(camera, v));
    }
    for (uint i = 0; i < vertices.size(); i++) {
        int j = (i + 1) % vertices.size();
        draw_line(screen, projected_vertices[i], projected_vertices[j], color);
    }
}

