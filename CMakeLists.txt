
project(kalman)

if (WIN32)
    set(VCPKG_DIR "C:/vcpkg/installed/x64-windows")
    set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
elseif(UNIX)
    set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu")
endif(WIN32)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
else(NOT MSCV)
    set(CMAKE_CXX_STANDARD 17)
endif(MSVC)

#find_package(Eigen3 CONFIG REQUIRED)

file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(
    ${PROJECT_NAME}
    ${SRC_FILES}
    ${HEADER_FILES}
    )
add_library(echo::kalman ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE echo::linearalgebra
)
