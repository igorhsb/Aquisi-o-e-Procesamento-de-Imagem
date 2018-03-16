##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.8)
project(project_pkg)


if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp rospy std_msgs tf laser_geometry sensor_msgs cv_bridge image_transport)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml

rosbuild_prepare_qt4(QtCore QtGui QtWidget QtOpenGl) 

# Add the appropriate components to the component list here 

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/project_pkg/*.h)
file(GLOB_RECURSE INC_ALL "include/project_pkg/*.h")

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

include_directories("includes/project_pkg")

SET(SPECIAL_OS_LIBS "X11" "pthread")
 
# PointClould Libs
find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# Freenect and Libusb Libs
find_package(libfreenect REQUIRED)
include_directories("/usr/include/libusb-1.0/")

# Laser_Geometry
find_package(tf REQUIRED)
include_directories("/opt/ros/kinetic/share/tf/")

# Astra_camera
find_package(astra_camera REQUIRED)
include_directories("/opt/ros/kinetic/share/astra_camera")
find_package(cv_bridge REQUIRED)
include_directories("/opt/ros/kinetic/share/cv_bridge")
find_package(image_transport REQUIRED)
include_directories("/opt/ros/kinetic/share/image_transport")
 
#Opencv
find_package( OpenCV REQUIRED )

#Zed
find_package(ZED 2 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(Eigen3 REQUIRED)

IF(NOT WIN32)
    SET(SPECIAL_OS_LIBS "X11" "pthread")
ENDIF()

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_PATH})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories("/usr/local/zed/")
include_directories("/usr/include/GL/")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
  
add_custom_target(headers SOURCES ${INC_ALL})

add_executable(project_pkg ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

add_definitions(-std=c++0x)

target_link_libraries(project_pkg ${QT_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries (project_pkg ${PCL_LIBRARIES})
target_link_libraries(project_pkg GL glut)
target_link_libraries(project_pkg ${FREENECT_LIBRARIES})
target_link_libraries(project_pkg ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES(project_pkg ${ZED_LIBRARIES}
			          ${SPECIAL_OS_LIBS}
				  ${GLEW_LIBRARIES}
				  ${OPENGL_LIBRARIES}
				  ${GLUT_LIBRARY}
${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})


install(TARGETS project_pkg RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 
