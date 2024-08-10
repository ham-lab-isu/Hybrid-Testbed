cmake_minimum_required(VERSION 3.9)
project(
  ram_documentation
  VERSION
  0.0.4
  DESCRIPTION
  "ROS Additive Manufacturing (RAM) documentation"
)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

list(
  APPEND
  CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/
)

find_package(Sphinx 1.7.4 REQUIRED)
find_package(PdfLatex REQUIRED)

set(
  SPHINX_CACHE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/_doctrees
)
set(
  SPHINX_HTML_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/html
)
set(
  SPHINX_PDF_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/latex
)

configure_file(
   ${CMAKE_CURRENT_SOURCE_DIR}/sphinx/conf.py.in
   conf.py
   @ONLY
)

# List dependencies for the documentation
file(
  GLOB
  DOCUMENTATION_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/sphinx/*.rst
)

# Latex - PDF
add_custom_target(
  ${PROJECT_NAME}-makefile
  ${SPHINX_EXECUTABLE} -q -b latex -c ${CMAKE_CURRENT_BINARY_DIR} -d ${SPHINX_CACHE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/sphinx ${SPHINX_PDF_DIR}
)

add_custom_command(
  OUTPUT
  ${CMAKE_CURRENT_BINARY_DIR}/latex/${PROJECT_NAME}.pdf
  COMMAND
  make -C ${SPHINX_PDF_DIR}
  DEPENDS
  ${DOCUMENTATION_FILES}
)

add_custom_target(
  ${PROJECT_NAME}-pdf
  ALL
  DEPENDS
  ${PROJECT_NAME}-makefile
  ${CMAKE_CURRENT_BINARY_DIR}/latex/${PROJECT_NAME}.pdf
)

# HTML pages
add_custom_target(
  ${PROJECT_NAME}-html
  ALL
  ${SPHINX_EXECUTABLE} -q -b html -c ${CMAKE_CURRENT_BINARY_DIR} -d ${SPHINX_CACHE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/sphinx ${SPHINX_HTML_DIR}
)

#############
## Install ##
#############

if("${CMAKE_INSTALL_PREFIX}" MATCHES "^/opt/")
  # Handle release case
  set(DESKTOP_APP_DIR "/usr/share/applications")
  set(DESKTOP_ICON_DIR "/usr/share/icons")
else()
  set(DESKTOP_APP_DIR "$ENV{HOME}/.local/share/applications")
  set(DESKTOP_ICON_DIR ${DESKTOP_APP_DIR})
  set(DESKTOP_APP_SUFFIX " (local)")
endif()

# Create directory for the desktop file/icon
install(DIRECTORY DESTINATION
  ${DESKTOP_APP_DIR}
)
install(DIRECTORY DESTINATION
  ${DESKTOP_ICON_DIR}
)

# Configure / install desktop launchers
if(DEFINED DESKTOP_APP_SUFFIX)
  configure_file(
    desktop/${PROJECT_NAME}.desktop.in
    desktop/${PROJECT_NAME}_local.desktop
  )
else()
  configure_file(
    desktop/${PROJECT_NAME}.desktop.in
    desktop/${PROJECT_NAME}.desktop
  )
endif()
if(DEFINED DESKTOP_APP_SUFFIX)
  install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/desktop/${PROJECT_NAME}_local.desktop
    DESTINATION
    ${DESKTOP_APP_DIR}
  )
else()
  install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/desktop/${PROJECT_NAME}.desktop
    DESTINATION
    ${DESKTOP_APP_DIR}
  )
endif()

# Install icon
install(
  FILES
  desktop/${PROJECT_NAME}.png
  DESTINATION
  ${DESKTOP_ICON_DIR}
)

install(
  DIRECTORY
  ${SPHINX_HTML_DIR}
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
  ${SPHINX_PDF_DIR}/${PROJECT_NAME}.pdf
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
