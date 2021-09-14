cmake_minimum_required( VERSION 3.10.2 )

project( kinematics_position_lib
        VERSION 0.1
        DESCRIPTION "Delta Robot position analysis"
        )

add_library(kinematics_position STATIC src/kinematics_position.c)

target_include_directories(kinematics_position
        PUBLIC include
        )

target_link_libraries(kinematics_position
	PUBLIC DR::matrix
        )

add_library(DR::kinematics_position ALIAS kinematics_position)
