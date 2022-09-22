cmake_minimum_required(VERSION 3.12)

project(Hexapod)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)


set(ARIS_INSTALL_PATH "C:/aris/aris-2.3.2.220915")
# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS ${ARIS_INSTALL_PATH})
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp src/kinematics.cpp src/plan.cpp src/plan.h src/kinematics.h src/robot.cpp src/robot.h)

if(UNIX)
    target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(${PROJECT_NAME} ${aris_LIBRARIES})
endif(WIN32)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
        "${ARIS_INSTALL_PATH}/lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:Hexapod>)                 # <--this is out-file path