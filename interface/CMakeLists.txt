cmake_minimum_required(VERSION 3.16.0)
project(Boule_de_cristal)

# Les configurations globales du projet se font ici

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_BUILD_TYPE Debug)

# Add Qt to the prefix
if(WIN32)
  list(APPEND CMAKE_PREFIX_PATH "C:/Qt/Qt5.12.9/5.12.9/mingw73_64")
elseif(APPLE)
  list(APPEND CMAKE_PREFIX_PATH "~/Qt5.12.9/5.12.9/clang_64") 
elseif(UNIX)
  list(APPEND CMAKE_PREFIX_PATH "/opt/Qt5.12.9/5.12.9/gcc_64")
endif()


# Find Qt
find_package(Qt5 COMPONENTS Widgets REQUIRED)


# La configuration des executables/librairies se font
# avec des subdirectories.
# 
# Ils peuvent être ajoutés avec la ligne :
# 
#   add_subdirectory(source_dir)


add_subdirectory(src/graphics)
add_subdirectory(src/middleware)
