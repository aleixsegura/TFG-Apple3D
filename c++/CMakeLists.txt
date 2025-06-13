cmake_minimum_required(VERSION 3.10)
project(Tfg LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD_17)
set(CMAKE_CUDA_ARCHITECTURES 86) # 86 for Ampere GPU architecture (used a RTX 3070)

include_directories(src/include)

# Search PROJ
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)
include_directories(${PROJ_INCLUDE_DIRS})
link_directories(${PROJ_LIBRARY_DIRS})


file(GLOB CPP_SOURCES "src/*.cpp")
file(GLOB CU_SOURCES "src/*.cu")

add_executable(main ${CPP_SOURCES} ${CU_SOURCES})

target_link_libraries(main ${PROJ_LIBRARIES})

target_compile_options(main PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
)