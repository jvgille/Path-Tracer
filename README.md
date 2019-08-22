#Path Tracer
This is a simple, backward path tracer which as of writing supports:
- global illumination with russian roulette
- diffuse, mirror, and glass materials
- triangular and spherical surfaces
- antialias
- depth of field


#### Requirements
- SDL
- GLM
- c++17 compatible compiler
(todo versions?)

#### Build
1. Clone repo, cd into
2. In CMakeLists.txt, replace the line `~/dev/tools/glm` with your path to the glm library.
3. `mkdir build && cd build`
4. `cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/g++-7`. Substiute g++-7 for any other c++17 compatible compiler you want.
5. `make && ./MLT` builds and runs the project.


#### TODO keybinds
