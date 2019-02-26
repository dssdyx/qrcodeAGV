cmake_minimum_required(VERSION 2.8.3)
project(zbar_opencv_2)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet std_msgs geometry_msgs )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zbar_opencv_1_nodelet
  CATKIN_DEPENDS roscpp nodelet
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet src/zbar_opencv_2_nodelet.cpp src/zbar_opencv_2.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    )


install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



