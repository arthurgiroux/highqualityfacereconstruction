cmake_minimum_required(VERSION 2.6)
project( calibration )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package( OpenCV REQUIRED )
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

#add_executable( calibration src/calibration.cpp )
add_executable( camera_calibration src/camera_calibration.cpp )


#target_link_libraries( calibration ${OpenCV_LIBS} )
target_link_libraries( camera_calibration ${OpenCV_LIBS} )

add_executable( calibopencvtophotoscan src/calibopencvtophotoscan.cpp )


target_link_libraries( calibopencvtophotoscan ${OpenCV_LIBS} )

add_executable( calibopencvtosvoboda src/calibopencvtosvoboda.cpp )
target_link_libraries( calibopencvtosvoboda ${OpenCV_LIBS} )

add_executable( multcalibopencvtosvoboda src/multcalibopencvtosvoboda.cpp )
target_link_libraries( multcalibopencvtosvoboda ${OpenCV_LIBS} )

add_executable( visualize src/visualize.cpp )

target_link_libraries(visualize 
			${OpenCV_LIBS}
			)