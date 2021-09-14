cmake_minimum_required( VERSION 3.10.2 )

project( kinematics_velocity_lib
        VERSION 0.1
        DESCRIPTION "Delta Robot velocity analysis"
        )

add_library(kinematics_velocity STATIC src/kinematics_velocity.c)

target_include_directories(kinematics_velocity
        PUBLIC include
        )

target_link_libraries(kinematics_velocity
	PUBLIC DR::matrix
        PUBLIC DR::kinematics_position
        )

add_library(DR::kinematics_velocity ALIAS kinematics_velocity)
