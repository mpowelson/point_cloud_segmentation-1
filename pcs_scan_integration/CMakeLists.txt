cmake_minimum_required(VERSION 3.5.0)
project(pcs_scan_integration)

add_compile_options(-std=c++11)

find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(tesseract_collision REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_octomap_mesh_masking src/${PROJECT_NAME}/octomap_mesh_masking.cpp)
target_link_libraries(${PROJECT_NAME}_octomap_mesh_masking tesseract::tesseract_collision_bullet console_bridge ${PCL_LIBRARIES})
target_compile_options(${PROJECT_NAME}_octomap_mesh_masking PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_octomap_mesh_masking PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_octomap_mesh_masking PRIVATE cxx_std_11)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME}_octomap_mesh_masking PUBLIC -mno-avx)
  else()
    message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
  endif()
target_include_directories(${PROJECT_NAME}_octomap_mesh_masking PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_octomap_mesh_masking SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )
#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}_octomap_mesh_masking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY config launch meshes urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(
#  DIRECTORY test
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  FILES_MATCHING PATTERN "*.test"
#)

if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)

#    add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
#    target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
#    if(CXX_FEATURE_FOUND EQUAL "-1")
#        target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE -std=c++11)
#    else()
#        target_compile_features(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE cxx_std_11)
#    endif()
#    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PUBLIC
#        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#        "$<INSTALL_INTERFACE:include>")
#    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PUBLIC
#        ${catkin_INCLUDE_DIRS}
#        ${PCL_INCLUDE_DIRS})

endif()

