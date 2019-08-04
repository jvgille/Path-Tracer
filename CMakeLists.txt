# Top-Level CMakeList.txt

cmake_minimum_required(VERSION 2.6)
project(MLT)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(MLT main.cpp)
target_compile_options(MLT PUBLIC -std=c++17 -O3)

find_package(SDL)

if (NOT SDL_FOUND)
   message (FATAL_ERROR "SDL not found!")
endif (NOT SDL_FOUND )

if(SDL_FOUND)
	include_directories(
		${SDL_INCLUDE_DIR}
		${PROJECT_SOURCE_DIR}/../glm
	)
	target_link_libraries(MLT ${SDL_LIBRARY})
endif(SDL_FOUND)

