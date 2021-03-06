cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#rosbuild_find_ros_package(actionlib_msgs)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(GRAPH_DIR ${PROJECT_SOURCE_DIR}/../graph_module)
#set(DLIB_DIR ${PROJECT_SOURCE_DIR}/../dlib-18.5)
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

set(TBB_INSTALL_DIR "/usr")
find_package(TBB QUIET)
if(TBB_FOUND)
  message("Found TBB.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_TBB")
  set(LIBS ${LIBS} ${TBB_LIBRARIES})
endif(TBB_FOUND)

FIND_PACKAGE( OpenMP QUIET)
if(OPENMP_FOUND)
  message("Found OPENMP.")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS} ${GRAPH_DIR}/include ${DLIB_DIR})
link_directories(${GRAPH_DIR}/lib)
#include(${DLIB_DIR}/dlib/cmake)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

rosbuild_add_library(feature_base src/feature_base_class.cpp)
target_link_libraries(feature_base learn_appearance)
#rosbuild_add_boost_directories()
rosbuild_add_executable(extract_features src/extract_features.cpp)
target_link_libraries(extract_features feature_base dlib)
rosbuild_add_executable(test_texton src/test_texton.cpp)
target_link_libraries(test_texton learn_appearance)
rosbuild_add_executable(test_holes src/test_hole_creation.cpp)
rosbuild_add_executable(test_shot src/test_feature_extraction.cpp)
rosbuild_add_executable(feature_extractor_test src/feature_tester_pr2.cpp)
rosbuild_add_executable(edge_extractor_test src/test_edge_extraction.cpp)
rosbuild_add_executable(svm_test src/svm_ex.cpp)
target_link_libraries(svm_test dlib)
rosbuild_add_executable(svm_multiclass_test src/svm_multiclass_ex.cpp)
target_link_libraries(svm_multiclass_test dlib)
rosbuild_add_executable(svm_pegasos_test src/svm_pegasos_ex.cpp)
target_link_libraries(svm_pegasos_test dlib)
rosbuild_add_executable(svm_sparse_test src/svm_sparse_ex.cpp)
target_link_libraries(svm_sparse_test dlib)
rosbuild_add_executable(custom_trainer_test src/custom_trainer_ex.cpp)
target_link_libraries(custom_trainer_test dlib)

# We are going to use a macro to add all the example programs.  However,
# usually you will only create one executable in your cmake projects.
# Therefore, you would just need to invoke ADD_EXECUTABLE() and
# TARGET_LINK_LIBRARIES() as shown below.  
#//MACRO(add_example name)
#//   ADD_EXECUTABLE(${name} ${name}.cpp)
#//   TARGET_LINK_LIBRARIES(${name} dlib )
#//ENDMACRO()


#here we apply our macros 
#//add_example(assignment_learning_ex)


