cmake_minimum_required(VERSION 3.0)
project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     std_msgs
#     )

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories("include")

add_library(camera_model SHARED
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(intrinsic_calib src/intrinsic_calib.cc)    
target_link_libraries(intrinsic_calib camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(stereo_calib src/stereo_calib.cc src/calib/StereoCameraCalibration.cc)
target_link_libraries(stereo_calib camera_model)

file(GLOB CALIB_HEADER_FILES include/camera_model/calib/*.h)
file(GLOB CAMERA_MODELS_HEADER_FILES include/camera_model/camera_models/*.h)
file(GLOB CHESSBOARD_HEADER_FILES include/camera_model/chessboard/*.h)
file(GLOB GPL_HEADER_FILES include/camera_model/gpl/*.h)
file(GLOB SPARSE_GRAPH_HEADER_FILES include/camera_model/sparse_graph/*.h)

install(TARGETS camera_model
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${CALIB_HEADER_FILES} DESTINATION include/camera_model/calib)
install(FILES ${CAMERA_MODELS_HEADER_FILES} DESTINATION include/camera_model/camera_models)
install(FILES ${CHESSBOARD_HEADER_FILES} DESTINATION include/camera_model/chessboard)
install(FILES ${GPL_HEADER_FILES} DESTINATION include/camera_model/gpl)
install(FILES ${SPARSE_GRAPH_HEADER_FILES} DESTINATION include/camera_model/sparse_graph)