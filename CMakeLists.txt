cmake_minimum_required(VERSION 2.6)
project(MLT)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SDL REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(MLT src/main.cpp)
target_compile_options(MLT PUBLIC -std=c++17 -O3 -Wno-register ${OpenMP_CXX_FLAGS})

include_directories(
	${SDL_INCLUDE_DIR}
	~/dev/tools/glm
)

target_link_libraries(MLT ${SDL_LIBRARY} ${OpenMP_CXX_FLAGS})
