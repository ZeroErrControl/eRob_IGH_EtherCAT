cmake_minimum_required(VERSION 3.5)
project(ethercat_erob)


set(CMAKE_BUILD_TYPE Debug)  # or Release, RelWithDebInfo, etc.
#find_package(ament_cmake REQUIRED)


# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

include_directories(
  ${ETHERLAB_DIR}/include
)

add_executable(
  igh_driver
  src/igh_driver.cpp)
  
target_include_directories(
  igh_driver
  PRIVATE
  ${ETHERLAB_DIR}/include
)
target_link_libraries(igh_driver ${ETHERCAT_LIB})


add_executable(
        multieRob
        src/multi_eRob.cpp)

target_include_directories(
        multieRob
        PRIVATE
        ${ETHERLAB_DIR}/include
)
target_link_libraries(multieRob ${ETHERCAT_LIB})





