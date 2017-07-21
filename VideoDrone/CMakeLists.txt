cmake_minimum_required(VERSION 3.0)

project( VideoDrone )
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})


# Recursively add all .cpp and .h files
file(GLOB_RECURSE HEADER_FILES ${PROJECT_SOURCE_DIR}/lib/*.h)
file(GLOB_RECURSE C_FILES ${PROJECT_SOURCE_DIR}/src/*.c)
file(GLOB_RECURSE CPP_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable( Drone ${HEADER_FILES} ${CPP_FILES} ${C_FILES})
target_link_libraries( Drone ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} )

