# CMake entry point
cmake_minimum_required (VERSION 3.0)

project(dualQuatIcpAuto LANGUAGES CXX C)

SET(CMAKE_CXX_FLAGS "-O3 -g -std=c++11  -Wno-c++11-narrowing ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    /usr/include/
    .
)

add_definitions(
    -D_APPLE_
)

link_directories(/usr/local/lib)
link_libraries(glog)    
link_libraries(ceres)    


add_executable(dualQuatIcpAuto
    src/main.cpp
    src/GeometryModel.cpp
    include/GeometryModel.h
    include/DualQuaternion.h
        include/AnalyticalJacobian.h)

target_link_libraries(dualQuatIcpAuto
    ${OpenCV_LIBS}
)




