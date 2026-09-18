cmake_minimum_required(VERSION 3.18)
project(ChronoMinimal LANGUAGES CXX)

find_package(Chrono CONFIG REQUIRED)
add_executable(demo_core_minimal main.cpp)
target_link_libraries(demo_core_minimal PRIVATE ${CHRONO_TARGETS})
set_property(TARGET demo_core_minimal PROPERTY MSVC_RUNTIME_LIBRARY "${CHRONO_MSVC_RUNTIME_LIBRARY}")
if(WIN32 AND NOT CHRONO_STATIC)
    add_custom_command(TARGET demo_core_minimal POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:Chrono::Chrono_core> $<TARGET_FILE_DIR:demo_core_minimal>)
endif()
enable_testing()
add_test(NAME core_free_fall COMMAND demo_core_minimal)
