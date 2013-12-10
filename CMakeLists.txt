## First start with some standard catkin stuff.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

cmake_minimum_required(VERSION 2.8.3)
project(adjustment_ui)
find_package(catkin REQUIRED COMPONENTS rviz roscpp roslib teleop_msgs message_generation)
catkin_package(CATKIN_DEPENDS roscpp roslib rviz teleop_msgs message_runtime)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#find_package(YamlCpp REQUIRED)
#include(${YAMLCPP_INCLUDE_DIR})
#include_directories(${catkin_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIR})
#set(LIBS ${LIBS} ${YAMLCPP_LIBRARY})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
##Adding ui file
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/AdjustmentInterface.ui)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/adjustment_interface.h
)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/adjustment_interface.cpp
  src/main.cpp
  front_view.rviz
  ${MOC_FILES}
)

## Add the "adjustment_interface" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
add_executable(adjustment_interface ${QT_SOURCES} ${SOURCE_FILES} ${QT_FORMS_HPP})

## Link the adjustment_interface executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(adjustment_interface ${QT_LIBRARIES} default_plugin ${catkin_LIBRARIES})
#target_link_libraries(adjustment_interface /usr/lib/libyaml-cpp.so.0.2) 
## END_TUTORIAL