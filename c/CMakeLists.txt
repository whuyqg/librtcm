cmake_minimum_required(VERSION 3.7)

include("cmake/VersionFromGit.cmake")
version_from_git()
message(STATUS "Building librtcm version ${VERSION}")

project(librtcm LANGUAGES C CXX VERSION ${VERSION})

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)

add_subdirectory (src)
add_subdirectory (test)
