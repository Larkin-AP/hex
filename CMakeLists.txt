cmake_minimum_required(VERSION 3.12)

project(Hexapod_sovle)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

add_executable(Hexapod src/main.cpp src/kinematics.cpp src/plan.cpp src/plan.h src/kinematics.h src/robot.cpp src/robot.h)

if(UNIX)
    target_link_libraries(Hexapod ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(Hexapod ${aris_LIBRARIES})
endif(WIN32)



