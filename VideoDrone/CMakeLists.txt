cmake_minimum_required(VERSION 3.0)

project( VideoDrone )
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
#find_package( Armadillo REQUIRED )

include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/camera)
include_directories(${PROJECT_SOURCE_DIR}/include/control)
include_directories(${PROJECT_SOURCE_DIR}/include/globals)
include_directories(${PROJECT_SOURCE_DIR}/include/imu)
include_directories(${PROJECT_SOURCE_DIR}/src/camera)
include_directories(${PROJECT_SOURCE_DIR}/src/control)
include_directories(${PROJECT_SOURCE_DIR}/src/imu)

# Recursively add all .cpp and .h files
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${PROJECT_SOURCE_DIR}/*.h)

add_executable( Drone ${HEADER_FILES} ${SRC_FILES} )
target_link_libraries( Drone ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} )

