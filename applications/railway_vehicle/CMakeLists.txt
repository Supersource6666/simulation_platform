cmake_minimum_required(VERSION 3.18)
project(RailwayVehicle LANGUAGES CXX)
find_package(Chrono CONFIG REQUIRED)
add_executable(railway_vehicle
    src/main.cpp
    src/Parameters.cpp
    src/VehicleModel.cpp
    src/VehicleModel3D.cpp
    src/TrackData.cpp)
target_link_libraries(railway_vehicle PRIVATE ${CHRONO_TARGETS})
target_compile_features(railway_vehicle PRIVATE cxx_std_17)
set_property(TARGET railway_vehicle PROPERTY MSVC_RUNTIME_LIBRARY "${CHRONO_MSVC_RUNTIME_LIBRARY}")
if(MSVC)
    target_compile_options(railway_vehicle PRIVATE /utf-8)
endif()
add_custom_command(TARGET railway_vehicle POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/config" "$<TARGET_FILE_DIR:railway_vehicle>/config")
add_custom_command(TARGET railway_vehicle POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/data" "$<TARGET_FILE_DIR:railway_vehicle>/data")
if(WIN32 AND NOT CHRONO_STATIC)
    add_custom_command(TARGET railway_vehicle POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:Chrono::Chrono_core> $<TARGET_FILE_DIR:railway_vehicle>)
endif()
enable_testing()

add_test(NAME railway_legacy_checks COMMAND railway_vehicle
    --config "${CMAKE_CURRENT_SOURCE_DIR}/config/example.json" --self-test)
foreach(load empty loaded)
    add_test(NAME railway_25t_yz_${load}_checks COMMAND railway_vehicle
        --config "${CMAKE_CURRENT_SOURCE_DIR}/config/25t_yz_${load}.json" --self-test)
endforeach()

add_test(NAME railway_measured_checks COMMAND railway_vehicle
    --config "${CMAKE_CURRENT_SOURCE_DIR}/config/25t_yz_loaded_measured.json" --self-test)
# Spatial 27-DOF self-test (vertical+pitch+lateral+yaw extension).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/25t_yz_loaded_3d.json")
    add_test(NAME railway_3d_yz_loaded_checks COMMAND railway_vehicle
        --config "${CMAKE_CURRENT_SOURCE_DIR}/config/25t_yz_loaded_3d.json" --self-test)
endif()
add_executable(track_input_checks tests/track_input_checks.cpp src/TrackData.cpp)
target_include_directories(track_input_checks PRIVATE src)
target_compile_features(track_input_checks PRIVATE cxx_std_17)
add_test(NAME track_input_checks COMMAND track_input_checks
    "${CMAKE_CURRENT_SOURCE_DIR}/data/origin_excitation_data.txt")

# 3-D spatial model equilibrium / dynamic response test. Runs only when the 3-D config exists.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/25t_yz_loaded_3d.json")
    add_executable(vehicle_3d_checks
        tests/vehicle_3d_checks.cpp
        src/Parameters.cpp
        src/VehicleModel3D.cpp
        src/TrackData.cpp)
    target_include_directories(vehicle_3d_checks PRIVATE src)
    target_link_libraries(vehicle_3d_checks PRIVATE ${CHRONO_TARGETS})
    target_compile_features(vehicle_3d_checks PRIVATE cxx_std_17)
    if(MSVC)
        target_compile_options(vehicle_3d_checks PRIVATE /utf-8)
    endif()
    add_test(NAME vehicle_3d_checks COMMAND vehicle_3d_checks
        "${CMAKE_CURRENT_SOURCE_DIR}/config/25t_yz_loaded_3d.json")
    add_executable(debug_3d
        tests/debug_3d.cpp
        src/Parameters.cpp
        src/VehicleModel3D.cpp
        src/TrackData.cpp)
    target_include_directories(debug_3d PRIVATE src)
    target_link_libraries(debug_3d PRIVATE ${CHRONO_TARGETS})
    target_compile_features(debug_3d PRIVATE cxx_std_17)
    if(MSVC)
        target_compile_options(debug_3d PRIVATE /utf-8)
    endif()
endif()

# Staged 10DOF vehicle / flexible-track validation baseline.
add_library(railway_coupled_model STATIC
    src/coupled/TrackDynamics.cpp
    src/coupled/Vehicle10DOF.cpp
    src/coupled/SimulationConfig.cpp
    src/coupled/CoupledSimulation.cpp
    src/Parameters.cpp
    src/TrackData.cpp)
target_include_directories(railway_coupled_model PUBLIC src)
target_link_libraries(railway_coupled_model PUBLIC ${CHRONO_TARGETS})
target_compile_features(railway_coupled_model PUBLIC cxx_std_17)
add_executable(railway_coupled src/coupled/main.cpp)
target_link_libraries(railway_coupled PRIVATE railway_coupled_model)
add_executable(coupled_checks tests/coupled_checks.cpp)
target_link_libraries(coupled_checks PRIVATE railway_coupled_model)
foreach(target railway_coupled_model railway_coupled coupled_checks)
    set_property(TARGET ${target} PROPERTY MSVC_RUNTIME_LIBRARY "${CHRONO_MSVC_RUNTIME_LIBRARY}")
    if(MSVC)
        target_compile_options(${target} PRIVATE /utf-8)
    endif()
endforeach()
add_custom_command(TARGET railway_coupled POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/config" "$<TARGET_FILE_DIR:railway_coupled>/config"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data" "$<TARGET_FILE_DIR:railway_coupled>/data")
if(WIN32 AND NOT CHRONO_STATIC)
    foreach(target railway_coupled coupled_checks)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Chrono::Chrono_core> $<TARGET_FILE_DIR:${target}>)
    endforeach()
endif()
add_test(NAME coupled_physics_checks COMMAND coupled_checks "${CMAKE_CURRENT_SOURCE_DIR}/config/coupled_vertical.json")
set_tests_properties(coupled_physics_checks PROPERTIES LABELS "railway;coupled" TIMEOUT 180)