cmake_minimum_required( VERSION 3.10.2 )

project( kinematics_torque_lib
        VERSION 0.1
        DESCRIPTION "Delta Robot torque analysis"
        )

add_library(kinematics_torque STATIC src/kinematics_torque.c)

target_include_directories(kinematics_torque
        PUBLIC include
        )

target_link_libraries(kinematics_torque
	PUBLIC DR::matrix
        PUBLIC DR::kinematics_position
        PUBLIC DR::kinematics_velocity
        PUBLIC DR::kinematics_acceleration
        )

add_library(DR::kinematics_torque ALIAS kinematics_torque)
