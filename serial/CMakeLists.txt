cmake_minimum_required(VERSION 2.8.3)
project(serial)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet std_msgs geometry_msgs)
set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
catkin_package(
    LIBRARIES ${PROJECT_NAME}_nodelet
    INCLUDE_DIRS include
    DEPENDS rt pthread
)

set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)

add_library(${PROJECT_NAME}_nodelet src/serial_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}
    )

add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node ${PROJECT_NAME}_nodelet)

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
