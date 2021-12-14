message("Generating the build files for the graphics library")

# Setup Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(GRAPHICS_SOURCES
  ${PROJECT_SOURCE_DIR}/src/graphics/src/MainWindow.cpp

  # Headers using MOC need to be added to the sources for AUTOMOC to treat them
  ${PROJECT_SOURCE_DIR}/src/graphics/include/MainWindow.h
)
set(GRAPHICS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/graphics/include/  ${PROJECT_SOURCE_DIR}/src/middleware/include)

add_library(cristal-graphics STATIC ${GRAPHICS_SOURCES})
target_include_directories(cristal-graphics PUBLIC ${GRAPHICS_INCLUDE_DIRS})
target_link_libraries(cristal-graphics PUBLIC Qt5::Widgets)
