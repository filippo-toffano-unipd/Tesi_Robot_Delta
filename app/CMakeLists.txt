cmake_minimum_required( VERSION 3.10.2)

project( Main_application
         VERSION 1.0
         DESCRIPTION "Developing main application" )

add_executable(application src/main.c)

target_link_libraries(application
    	PRIVATE DR::matrix
	PRIVATE DR::kinematics_position
        PRIVATE DR::kinematics_velocity
        PRIVATE DR::kinematics_acceleration
        PRIVATE DR::kinematics_torque
        PRIVATE DR::path_generation
)
