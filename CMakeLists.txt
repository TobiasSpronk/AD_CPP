cmake_minimum_required(VERSION 3.16)

project(CppProjectTemplate VERSION 1.0.0 LANGUAGES C CXX )

set(CMAKE_CXX_STANDARD              17)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS   ON)

# Options
option(USE_CONAN            "Whether to use conan or vcpkg"         ON)
option(ENABLE_WARNINGS      "Enable to add warnings to a target."   ON)
option(ENABLE_TESTING       "Enable a Unit Testing build."          ON)
option(ENABLE_COVERAGE      "Enable a Code Coverage build."         OFF)
option(ENABLE_CLANG_TIDY    "Enable to add clang tidy."             ON)
option(ENABLE_CPPCHECK      "Enable to add cppcheck."               OFF)
option(ENABLE_LTO           "Enable to add Link Time Optimization." ON)
option(ENABLE_CCACHE        "Enable to add Ccache."                 ON)


### CMAKE MODULES
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
include(ConfigSafeGuards)
include(Warnings)
include(StaticAnalyzers)
include(LTO)
include(Ccache)
include(AddGitSubmodule)
include(FetchContent)
include(Docs)


set(EXECUTABLE_NAME Executable)
set(LIBRARY_NAME Library)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)

option(ENABLE_TESTING "Enable the unit testing build" ON)

conan_basic_setup()

add_git_submodule(external/json)


add_subdirectory(configured)
add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(app)
