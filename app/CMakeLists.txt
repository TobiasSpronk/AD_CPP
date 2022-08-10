set(EXE_SOURCES
    "main.cpp")

set(EXE_INCLUDES
    "./")

add_executable(${EXECUTABLE_NAME} ${EXE_SOURCES})

target_include_directories(${EXECUTABLE_NAME} PUBLIC
    ${EXE_INCLUDES})

target_link_libraries(${EXECUTABLE_NAME} PUBLIC
    ${LIBRARY_NAME}
    ${CONAN_CXXOPTS})

if (${ENABLE_WARNINGS})
    target_set_warnings(TARGET ${EXECUTABLE_NAME} ENABLE ON AS_ERROR OFF)
endif()

#if(${ENABLE_LTO})
#    target_enable_lto(${EXECUTABLE_NAME} optimized)
#endif()
