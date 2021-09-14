cmake_minimum_required( VERSION 3.10.2 )

project( kinematics_acceleration_lib
        VERSION 0.1
        DESCRIPTION "Delta Robot acceleration analysis"
        )

add_library(kinematics_acceleration STATIC src/kinematics_acceleration.c)

target_include_directories(kinematics_acceleration
        PUBLIC include
        )

target_link_libraries(kinematics_acceleration
	PUBLIC DR::matrix
        PUBLIC DR::kinematics_position
        PUBLIC DR::kinematics_velocity
        )

add_library(DR::kinematics_acceleration ALIAS kinematics_acceleration)
