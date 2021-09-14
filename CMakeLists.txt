cmake_minimum_required( VERSION 3.10.2 )

project( Delta_Robot
        VERSION 0.3
        DESCRIPTION "Delta Robot kinematics analysis."
        )


if (MSVC)
        # warning level 4 and all warnings as errors
        add_compile_options(/W4 /WX)
else()
        # lots of warnings and all warnings as errors
        #add_compile_options(-Wall -g -Wextra -pedantic)
        add_compile_options(-Wall -Wextra -pedantic)
endif()

set(TARGET_GROUP production CACHE STRING "Group to build")

add_subdirectory(lib/matrix)
add_subdirectory(lib/kinematics_position)
add_subdirectory(lib/kinematics_velocity)
add_subdirectory(lib/kinematics_acceleration)
add_subdirectory(lib/kinematics_torque)
add_subdirectory(lib/path_generation)


if(TARGET_GROUP STREQUAL production)
        add_subdirectory(app)
elseif(TARGET_GROUP STREQUAL test)
        include(CTest)
else()
        message(FATAL_ERROR "Given TARGET_GROUP unknown")
endif()
