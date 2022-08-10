set(LIBRARY_SOURCES
    "AdFunctions.cpp")

set(LIBRARY_INCLUDES PUBLIC
    "./")


add_library(${LIBRARY_NAME} STATIC
    ${LIBRARY_SOURCES})
target_include_directories(${LIBRARY_NAME} PUBLIC
    ${LIBRARY_INCLUDES}
    "${CMAKE_BINARY_DIR}/configured_files/include")


if (${ENABLE_WARNINGS})
    target_set_warnings(TARGET ${LIBRARY_NAME} ENABLE ON AS_ERROR OFF)
endif()

#if(${ENABLE_LTO})
#    target_enable_lto(${LIBRARY_NAME} optimized)
#endif()
