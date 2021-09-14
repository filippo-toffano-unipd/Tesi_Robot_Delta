cmake_minimum_required( VERSION 3.10.2 )

project( matrix_lib
        VERSION 0.1
        DESCRIPTION "Matrix operations"
        )

add_library(matrix STATIC src/matrix.c)

target_include_directories(matrix
        PUBLIC include
        )

target_link_libraries(matrix
	m
        )

add_library(DR::matrix ALIAS matrix)
