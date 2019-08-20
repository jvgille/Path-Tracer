cmake_minimum_required(VERSION 2.6)
project(MLT)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SDL REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(MLT main.cpp)
target_compile_options(MLT PUBLIC -std=c++17 -O3 -Wno-register ${OpenMP_CXX_FLAGS})

include_directories(
	${SDL_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/../glm
)

# OpenMP_EXE_LINKER_FLAGS
target_link_libraries(MLT ${SDL_LIBRARY} ${OpenMP_CXX_FLAGS})
