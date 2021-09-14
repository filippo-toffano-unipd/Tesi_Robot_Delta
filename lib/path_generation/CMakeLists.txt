cmake_minimum_required( VERSION 3.10.2 )

project( path_generation_lib
        VERSION 0.1
        DESCRIPTION "Delta Robot path generation"
        )

add_library(path_generation STATIC src/path_generation.c)

target_include_directories(path_generation
        PUBLIC include
        )

target_link_libraries(path_generation
	PUBLIC DR::matrix
        PUBLIC DR::kinematics_position
        PUBLIC DR::kinematics_velocity
        PUBLIC DR::kinematics_acceleration
        PUBLIC DR::kinematics_torque
        )

add_library(DR::path_generation ALIAS path_generation)
