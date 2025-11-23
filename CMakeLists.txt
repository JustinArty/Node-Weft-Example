cmake_minimum_required(VERSION 3.23)

project(Node-Weft-Example)

add_executable(Example1 WIN32)

target_sources(Example1
	PRIVATE
	    Example1/Main.cpp
)

add_subdirectory(Node-Weft)
target_link_libraries(Example1
	PRIVATE
		Node-Weft
)

target_compile_definitions(Example1
	PRIVATE
	_WINDOWS
	WIN32_LEAN_AND_MEAN
	UNICODE 
	_UNICODE
)

add_custom_command(TARGET Example1 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/Node-Weft/resource"
        "$<TARGET_FILE_DIR:Example1>/resource"
    COMMENT "Copying node-weft resources..."
)